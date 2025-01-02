/*
 * Phoenix-RTOS
 *
 * ACPI server
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * %LICENSE%
 */

#include <errno.h>
#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <unistd.h>
#include <posix/utils.h>

#include <uacpi/resources.h>
#include <uacpi/sleep.h>
#include <uacpi/utilities.h>
#include <uacpi/uacpi.h>
#include <uacpi/event.h>
#include <uacpi/context.h>

#include <acpi-msg.h>

#define MSGTHR_PRIO 3

#define COL_RED    "\033[1;31m"
#define COL_NORMAL "\033[0m"
#define LOG_TAG    "acpisrv: "

#define log_error(fmt, ...) \
	do { \
		if (1) { \
			printf(COL_RED LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); \
		} \
	} while (0)

#define log_warn(fmt, ...) \
	do { \
		if (1) { \
			printf(LOG_TAG fmt "\n", ##__VA_ARGS__); \
		} \
	} while (0)

#define log_info(fmt, ...) \
	do { \
		if (1) { \
			printf(LOG_TAG fmt "\n", ##__VA_ARGS__); \
		} \
	} while (0)


struct {
	uacpi_namespace_node *root;
	uacpi_namespace_node *pciRoot;
	uint32_t port;
} acpisrv_common;


static int uacpi_init(void)
{
	uacpi_status ret;

	ret = uacpi_initialize(0);
	if (ret != UACPI_STATUS_OK) {
		log_error("uacpi_initialize error: %s", uacpi_status_to_string(ret));
		return -ENODEV;
	}

	ret = uacpi_namespace_load();
	if (ret != UACPI_STATUS_OK) {
		log_error("uacpi_namespace_load error: %s", uacpi_status_to_string(ret));
		return -ENODEV;
	}

	ret = uacpi_set_interrupt_model(UACPI_INTERRUPT_MODEL_IOAPIC);
	if (ret != UACPI_STATUS_OK) {
		log_error("uacpi_set_interrupt_model error: %s", uacpi_status_to_string(ret));
		return -ENODEV;
	}

	ret = uacpi_namespace_initialize();
	if (ret != UACPI_STATUS_OK) {
		log_error("uacpi_namespace_initialize error: %s", uacpi_status_to_string(ret));
		return -ENODEV;
	}

	ret = uacpi_finalize_gpe_initialization();
	if (ret != UACPI_STATUS_OK) {
		log_error("uACPI GPE initialization error: %s", uacpi_status_to_string(ret));
		return -ENODEV;
	}

	return 0;
}


int acpisrv_init(void)
{
	int ret;

	uacpi_status st;
	uacpi_namespace_node *root, *pciRoot;

	uacpi_pci_routing_table *prt;

	ret = uacpi_init();
	if (ret < 0) {
		return ret;
	}

	root = uacpi_namespace_root();

	st = uacpi_namespace_node_find(root, "\\_SB_.PCI0", &pciRoot);
	if (st != UACPI_STATUS_OK) {
		log_error("failed to find \\_SB_.PCI0: %s", uacpi_status_to_string(ret));
		return -ENODEV;
	}

	st = uacpi_get_pci_routing_table(pciRoot, &prt);
	if (st != UACPI_STATUS_OK) {
		log_error("failed to get _PRT: %s", uacpi_status_to_string(ret));
		return -ENODEV;
	}

	acpisrv_common.root = root;
	acpisrv_common.pciRoot = pciRoot;

	/* TODO: do some test acpi query */

	return 0;
}


int acpisrv_handleIrq(uint32_t devAddr, uint8_t *irq)
{
	int i;
	uacpi_status st;
	uacpi_pci_routing_table *prt;
	uacpi_pci_routing_table_entry *prtEntry;

	st = uacpi_get_pci_routing_table(acpisrv_common.pciRoot, &prt);
	if (st != UACPI_STATUS_OK) {
		log_error("failed to retrieve _PRT");
		return -ENODEV;
	}

	for (i = 0; i < prt->num_entries; i++) {
		prtEntry = &prt->entries[i];
		if (prtEntry->address == devAddr) {
			if (prtEntry->source == 0) {
				*irq = prtEntry->index;
				return 0;
			}
			else {
				/* TODO set and retrieve IRQ from link device */
				log_info("link device IRQ get unimplemented");
				return -ENOSYS;
			}
			return 0;
		}
	}

	return -ENODEV;
}


static void acpisrv_msgthr(void *arg)
{
	unsigned port = (int)arg;
	msg_rid_t rid;
	msg_t msg;
	acpi_msg_t *amsg, *oamsg;
	int ret;
	bool respond;

	for (;;) {
		ret = msgRecv(port, &msg, &rid);
		if (ret < 0) {
			continue;
		}

		respond = true;
		switch (msg.type) {
			case mtDevCtl:
				amsg = (acpi_msg_t *)msg.i.raw;
				oamsg = (acpi_msg_t *)msg.o.raw;
				switch (amsg->type) {
					case acpi_msg_irq:
						msg.o.err = acpisrv_handleIrq(amsg->irq.addr, &oamsg->irq.irq);
						oamsg->type = acpi_msg_irq;
						oamsg->irq.addr = amsg->irq.addr;
						break;
					default:
						msg.o.err = -EINVAL;
						log_warn("unsupported acpi_msg type: %d", amsg->type);
						break;
				}
				break;
			default:
				msg.o.err = -EINVAL;
				log_warn("unsupported msg type");
		}

		if (respond) {
			msgRespond(port, &msg, rid);
		}
	}
}


int main(void)
{
	int ret;
	oid_t oid;

	ret = acpisrv_init();
	if (ret != 0) {
		log_error("failed to init uacpi");
		return 1;
	}

	ret = portCreate(&acpisrv_common.port);
	if (ret != 0) {
		log_error("can't create port");
		return 1;
	}

	oid.port = acpisrv_common.port;
	oid.id = 0;

	ret = create_dev(&oid, "/dev/acpi");

	if (ret != 0) {
		log_error("can't create dev");
		return 1;
	}

	log_info("started");

	priority(MSGTHR_PRIO);

	acpisrv_msgthr((void *)acpisrv_common.port);
}
