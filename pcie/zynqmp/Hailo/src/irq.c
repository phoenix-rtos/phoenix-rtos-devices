#include "../../core/pci_core.h"

#include "../phoenix/phoenix_completion.h"
#include "../phoenix/phoenix_spinlock.h"
#include "../phoenix/phoenix_list.h"

#include "../common/vdma_common.h"
#include "../phoenix/phoenix_log.h"
#include "../vdma/memory.h"
#include "../vdma/ioctl.h"
#include "nnc.h"
//#include "soc.h"

#define DEFAULT_VDMA_ENGINE_INDEX       (0)


#define IRQ_NONE 0
#define IRQ_HANDLED 1


static void firmware_notification_irq_handler(struct hailo_pcie_board *board)
{
    struct hailo_notification_wait *notif_wait_cursor = NULL;
    int err = 0;
    unsigned long irq_saved_flags = 0;

    spin_lock_irqsave(&board->nnc.notification_read_spinlock, irq_saved_flags);
    err = hailo_pcie_read_firmware_notification(&board->pcie_resources.fw_access, &board->nnc.notification_cache);
    spin_unlock_irqrestore(&board->nnc.notification_read_spinlock, irq_saved_flags);

    if (err < 0) {
        hailo_err(board, "Failed reading firmware notification");
    }
    else {
        // TODO: HRT-14502 move interrupt handling to nnc
        rcu_read_lock();
        list_for_each_entry_rcu(notif_wait_cursor, &board->nnc.notification_wait_list, notification_wait_list)
        {
            complete(&notif_wait_cursor->notification_completion);
        }
        rcu_read_unlock();
    }
}

static void boot_irq_handler(struct hailo_pcie_board *board, struct hailo_pcie_interrupt_source *irq_source)
{
    if (irq_source->sw_interrupts & HAILO_PCIE_BOOT_SOFT_RESET_IRQ) {
        hailo_dbg(board, "soft reset trigger IRQ\n");
        complete(&board->soft_reset.reset_completed);
    }
    if (irq_source->sw_interrupts & HAILO_PCIE_BOOT_IRQ) {
        hailo_dbg(board, "boot trigger IRQ\n");
        complete_all(&board->fw_boot.fw_loaded_completion);
    } else {
        board->fw_boot.boot_used_channel_bitmap &= ~irq_source->vdma_channels_bitmap;
        hailo_dbg(board, "boot vDMA data IRQ - channel_bitmap = 0x%x\n", irq_source->vdma_channels_bitmap);
        if (0 == board->fw_boot.boot_used_channel_bitmap) {
            complete_all(&board->fw_boot.vdma_boot_completion);
            hailo_dbg(board, "boot vDMA data trigger IRQ\n");
        }
    }
}

static void nnc_irq_handler(struct hailo_pcie_board *board, struct hailo_pcie_interrupt_source *irq_source)
{
    if (irq_source->sw_interrupts & HAILO_PCIE_NNC_FW_CONTROL_IRQ) {
        complete(&board->nnc.fw_control.completion);
    }

    if (irq_source->sw_interrupts & HAILO_PCIE_NNC_DRIVER_DOWN_IRQ) {
        complete(&board->driver_down.reset_completed);
    }

    if (irq_source->sw_interrupts & HAILO_PCIE_NNC_FW_NOTIFICATION_IRQ) {
        firmware_notification_irq_handler(board);
    }
}

static void soc_irq_handler(struct hailo_pcie_board *board, struct hailo_pcie_interrupt_source *irq_source)
{
    if (irq_source->sw_interrupts & HAILO_PCIE_SOC_CONTROL_IRQ) {
        complete_all(&board->soc.control_resp_ready);
    }

    if (irq_source->sw_interrupts & HAILO_PCIE_SOC_CLOSE_IRQ) {
        hailo_info(board, "soc_irq_handler - HAILO_PCIE_SOC_CLOSE_IRQ\n");
        // always use bitmap=0xFFFFFFFF - it is ok to wake all interrupts since each handler will check if the stream was aborted or not. 
        hailo_vdma_wakeup_interrupts(&board->vdma, &board->vdma.vdma_engines[DEFAULT_VDMA_ENGINE_INDEX],
            0xFFFFFFFF);
    }
}



irqreturn_t hailo_irqhandler(int irq, void *dev_id)
{
    irqreturn_t return_value = IRQ_NONE;
    struct hailo_pcie_board *board = (struct hailo_pcie_board *)dev_id;
    bool got_interrupt = false;
    struct hailo_pcie_interrupt_source irq_source = {0};

    hailo_dbg(board, "hailo_irqhandler\n");

    while (true) {
        if (!hailo_pcie_is_device_connected(&board->pcie_resources)) {
            hailo_err(board, "Device disconnected while handling irq\n");
            break;
        }

        got_interrupt = hailo_pcie_read_interrupt(&board->pcie_resources, &irq_source);
        if (!got_interrupt) {
            break;
        }

        return_value = IRQ_HANDLED;

        if (board->fw_boot.is_in_boot) {
            boot_irq_handler(board, &irq_source);
        } else {
            if (HAILO_ACCELERATOR_TYPE_NNC == board->pcie_resources.accelerator_type) {
                nnc_irq_handler(board, &irq_source);
            } else if (HAILO_ACCELERATOR_TYPE_SOC == board->pcie_resources.accelerator_type) {
                soc_irq_handler(board, &irq_source);
            } else {
                hailo_err(board, "Invalid accelerator type %d\n", board->pcie_resources.accelerator_type);
            }

            if (0 != irq_source.vdma_channels_bitmap) {
                hailo_vdma_irq_handler(&board->vdma, DEFAULT_VDMA_ENGINE_INDEX,
                    irq_source.vdma_channels_bitmap);
            }
        }
    }

    return return_value;
}