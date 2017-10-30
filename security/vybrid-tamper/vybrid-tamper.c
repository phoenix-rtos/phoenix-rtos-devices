#include "vybrid-tamper.h"

#include <hal/if.h>
#include <fs/if.h>
#include <dev/if.h>
#include <dev/rtc/rtc.h>
#include <proc/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <hal/MVF50GS10MK50.h>
#include <main/std.h>
#include <lib/list.h>

#include <include/tamperctl.h>

#define LOG(msg, ...) main_printf(ATTR_FAILURE, "%s(%d): " msg "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
//#define LOG(msg, ...)
static SNVS_Type *SNVS_virt;
static SCSC_Type *SCSC_virt;

void printSSM_state(void);
void printSSM_state()
{
	u32 state = (SNVS_virt->HPSR & SNVS_HPSR_SSM_ST_MASK) >> SNVS_HPSR_SSM_ST_SHIFT;
	switch(state)
	{
		case 0:
			LOG("SSM STATE: init");
			break;
		case 0x8:
			LOG("SSM STATE: init intermediate");
			break;
		case 0x9:
			LOG("SSM STATE: check");
			break;
		case 0xB:
			LOG("SSM STATE: non-secure");
			break;
		case 0xD:
			LOG("SSM STATE: trusted");
			break;
		case 0xF:
			LOG("SSM STATE: secure");
			break;
		case 0x3:
			LOG("SSM STATE: soft-fail");
			break;
		case 0x1:
			LOG("SSM STATE: hard-fail");
			break;
		default:
			LOG("UNKNOWN STATE");
	}
	LOG("SCSC_LFSR: %x", state=SCSC_virt->LFSR_CTR);
	LOG("SNVS_LPTDCR: %x", state=SNVS_virt->LPTDCR);
	LOG("SNVS_LPSR: %x", state=SNVS_virt->LPSR);
	LOG("SNVS_LPCR: %x", state=SNVS_virt->LPCR);
	LOG("SNVS_LPSVCR: %x", state=SNVS_virt->LPSVCR);
	LOG("SNVS_HPLR: %x", state=SNVS_virt->HPLR);
	LOG("SNVS_HPSVCR: %x", state=SNVS_virt->HPSVCR);
    LOG("SNVS_HPSVSR: %x", state=SNVS_virt->HPSVSR);
    LOG("SNVS_HPSICR: %x", state=SNVS_virt->HPSICR);
    LOG("SNVS_HPCR: %x", state=SNVS_virt->HPCR);
    LOG("SNVS_HPSR: %x", state=SNVS_virt->HPSR);
	LOG("---------------------------------------\n");
}
static void tamper_reset(void);
static void tamper_reset()
{
    SNVS_virt->HPCOMR |= 1 << SNVS_HPCOMR_SSM_ST_SHIFT;
    SNVS_virt->HPCOMR |= 1 << SNVS_HPCOMR_LP_SWR_SHIFT;
    SNVS_virt->LPTDCR = (1 << SNVS_LPTDCR_ET1_EN_SHIFT) | (1 << SNVS_LPTDCR_ET2_EN_SHIFT);
}

typedef struct tamperCbInfo_t{
    tamperCb_t callback;
    void *arg;
    LIST_ENTRY(tamperCbInfo_t) entry;
} tamperCbInfo_t;

typedef struct{
    tamperInfo_t info;
    u32 mask;
    u32 invoked;
    LIST_HEAD(tamperCbInfo_t) callbacks;
} tamper_t;

thq_t tamperWq;
static tamper_t tamper_indicator[TAMPER_NUM];
semaphore_t tindLock;

int tamperCheckEnable=0;

int tamper_setCb(tamperCb_t cb, void *arg, unsigned tamper)
{
    int i = 0;
    int err = EOK;

    if(tamper == 0)
        return -EINVAL;
    if((tamper & (TAMPER_ID_1 | TAMPER_ID_2)) != tamper)
        return -EINVAL;
    if(cb == NULL)
        return -EINVAL;

    proc_semaphoreDown(&tindLock);
    //Iterate through every bit in mask
    for(i=0; i < TAMPER_NUM; ++i)
    {
        if((tamper >> i) & 0x1)
        {
            tamperCbInfo_t *ci = vm_kmalloc(sizeof(tamperCbInfo_t));
            if(ci==NULL)
                err = -ENOMEM;
            else {
                //add callback to list
                ci->callback=cb;
                ci->arg=arg;
                LIST_ADD(&tamper_indicator[i].callbacks, ci, entry);
            }
        }
    }
    proc_semaphoreUp(&tindLock);
    return err;
}

int tamper_resetCb(tamperCb_t cb, void *arg, unsigned tamper)
{
    int i = 0;

    if(tamper == 0)
        return -EINVAL;
    if((tamper & (TAMPER_ID_1 | TAMPER_ID_2)) != tamper)
        return -EINVAL;
    if(cb == NULL)
        return -EINVAL;

    proc_semaphoreDown(&tindLock);
    //Iterate through every bit in mask
    for(i=0; i < TAMPER_NUM; ++i)
    {
        if((tamper >> i) & 0x1)
        {
            tamperCbInfo_t *ci=NULL;
            //find callback on list
            LIST_FIND(&tamper_indicator[i].callbacks, ci, entry, ci->callback == cb && ci->arg == arg);

            //remove callback from list
            if(ci != NULL)
            {
                LIST_REMOVE(&tamper_indicator[i].callbacks, ci, entry);
            }
            vm_kfree(ci);
        }
    }
    proc_semaphoreUp(&tindLock);
    return 0;
}

static int vybrid_tamper_check(void *arg)
{
    int i;
    unsigned state;
    tamperCbInfo_t *inf;
    struct timespec timestamp;
    while(tamperCheckEnable)
    {

        state = (SNVS_virt->HPSR & SNVS_HPSR_SSM_ST_MASK) >> SNVS_HPSR_SSM_ST_SHIFT;
        u32 tampers = SNVS_virt->LPSR & (SNVS_LPSR_ET1D_MASK | SNVS_LPSR_ET2D_MASK);
        i = rtc_gettimeofday(&timestamp);
        if(i < 0)
        {
            //TODO - set timestamp for some invalid value
            memset(&timestamp, 0, sizeof(struct tm));
        }
        //printSSM_state();
        if(state == 0x3) {
            //soft fail
            tamper_reset();
        }

        volatile u32 tmp = SNVS_virt->LPSR;
        SNVS_virt->LPSR = tmp;

        proc_semaphoreDown(&tindLock);
        for(i=0; i < TAMPER_NUM; ++i) {
            if(!tamper_indicator[i].invoked && (tampers & tamper_indicator[i].mask))
            {
                LOG("TAMPER %d", i);
                tamper_indicator[i].info.occurances |= 0x1U;
                tamper_indicator[i].info.timestamp = timestamp;
                LIST_FOR_EACH(&tamper_indicator[i].callbacks, inf, entry)
                {
                    inf->callback(inf->arg);
                }
                tamper_indicator[i].invoked = 1;
            }
            else if(tamper_indicator[i].invoked && !(tampers & tamper_indicator[i].mask)) {
                tamper_indicator[i].invoked = 0;
                tamper_indicator[i].info.occurances <<= 1;
                tamper_indicator[i].info.occurances &= ~0x1U;
            }
        }
        if(tampers)
            proc_threadWakeAll(&tamperWq);

        proc_semaphoreUp(&tindLock);
        proc_threadSleep(1000*500);
    }
    return 0;
}

int vybrid_tamper_poll(file_t *file, ktime_t timeout, int op)
{
    vnode_t *vnode = file->vnode;
    assert(vnode->type == vnodeDevice);

    int err = EOK;

    if (MAJOR(vnode->dev) != MAJOR_TAMPER)
        return -EINVAL;

    proc_semaphoreDown(&tindLock);

    if (op == POLL_READ) { /* Wait for appearance of tamper or for timeout */
        while (!(tamper_indicator[0].info.occurances & 0x1U) && !(tamper_indicator[1].info.occurances & 0x1U)) {
            err = proc_condWait(&tamperWq, &tindLock, timeout);
            if (err == -ETIME)
                break;
            else if (err < 0)
                break;
            //do not do anything - after poll returns, ioctl should be invoked to check tamper
        }
    }
    else  /* Wait for free space in the tx sw queue or for timeout */
        err = -EINVAL;

    proc_semaphoreUp(&tindLock);
    return err;
}

int vybrid_tamper_select_poll(file_t *file, unsigned *ready)
{
    vnode_t *vnode = file->vnode;
    assert(vnode->type == vnodeDevice);

    if (MAJOR(vnode->dev) != MAJOR_TAMPER)
        return -EINVAL;

    proc_semaphoreDown(&tindLock);

    if((tamper_indicator[0].info.occurances & 0x1) || (tamper_indicator[1].info.occurances & 0x1))
        *ready |= FS_READY_READ;

    proc_semaphoreUp(&tindLock);

    return 0;
}

int vybrid_tamper_ioctl(file_t* file, unsigned int cmd, unsigned long arg)
{
    assert(file != NULL);
    vnode_t *vnode = file->vnode;
    assert(vnode->type == vnodeDevice);
    process_t *proc = proc_current()->process;
    assert(proc != NULL);

    if (MAJOR(vnode->dev) != MAJOR_TAMPER)
        return -EINVAL;
    if(arg == 0)
        return -EINVAL;

    if(cmd == TAMPER_GET_NUM) {
        *((u32 *)arg) = TAMPER_NUM;
        //returns number of tampers
    }
    else if(cmd == TAMPER_GET) {
        //returns info about tampers
        proc_semaphoreDown(&tindLock);

        if((arg > VADDR_KERNEL) || (arg + TAMPER_NUM * sizeof(tamperInfo_t)) > VADDR_KERNEL)
            return -EINVAL;

        tamperInfo_t * uti= (tamperInfo_t *) arg;
        uti[0] = tamper_indicator[0].info;
        uti[1] = tamper_indicator[1].info;
        proc_semaphoreUp(&tindLock);
    }
    else
        return -EINVAL;
    return EOK;
}

int vybrid_tamper_open(vnode_t *vnode, file_t* file)
{
    assert(vnode != NULL);
    assert(file != NULL);
    assert(vnode->type == vnodeDevice);

    if (MAJOR(vnode->dev) != MAJOR_TAMPER)
        return -EINVAL;

    return 0;
}

int vybrid_tamper_release(vnode_t *vnode)
{
    assert(vnode != NULL);
    assert(vnode->type == vnodeDevice);

    if (MAJOR(vnode->dev) != MAJOR_TAMPER)
        return -EINVAL;

    return 0;
}

void tamper_init()
{
    static const file_ops_t vybrid_tamper_ops = {
        .open = vybrid_tamper_open,
        .release = vybrid_tamper_release,
        .poll = vybrid_tamper_poll,
        .ioctl = vybrid_tamper_ioctl,
        .select_poll = vybrid_tamper_select_poll
    };



	s32 result;
	result = vm_iomap(SNVS_BASE, sizeof(SNVS_Type), PGHD_DEV_RW, (void *)&SNVS_virt);
	assert(result == EOK);
	
	result = vm_iomap(SCSC_BASE, sizeof(SCSC_Type), PGHD_DEV_RW, (void *)&SCSC_virt);
	assert(result == EOK);	


    //1. Transition SSM from check state to functional state (trusted/secure/non-secure).
    //2. Enablesecurity violations and interrupts in SNVS control and configuration registers.
    //3. Program SNVS general functions/configurations.

    unsigned state;
    while( (state = (SNVS_virt->HPSR & SNVS_HPSR_SSM_ST_MASK) >> SNVS_HPSR_SSM_ST_SHIFT) == 0x8);

    if(state == 0 || state == 0x9)
    {
        //init or check
        LOG("In init state - transition to secure or trusted");
        while(state == 0 || state == 0x9)
        {
            SNVS_virt->HPCOMR |= 1 << SNVS_HPCOMR_SSM_ST_SHIFT;
            proc_threadSleep(1000);
            state = (SNVS_virt->HPSR & SNVS_HPSR_SSM_ST_MASK) >> SNVS_HPSR_SSM_ST_SHIFT;
        }
    }
    else if(state == 0x3 || state == 0x1)
    {
        //soft fail or hard fail
        //error - we can do nothing
        LOG("Error: SSM fail");
        return;
    }
    //non-secure, secure or trusted
    LOG("SSM functional state");

	printSSM_state();

    SNVS_virt->HPLR = 0;
    SNVS_virt->LPLR = 0;
    SNVS_virt->LPSVCR = 0;

    SNVS_virt->HPCOMR |= 1 << SNVS_HPCOMR_NPSWA_EN_SHIFT;
    SNVS_virt->HPCOMR |= 1 << SNVS_HPCOMR_LP_SWR_SHIFT;


    SCSC_virt->LFSR_CTR &= ~SCSC_LFSR_CTR_TAMPER0_INV_MASK;
    SCSC_virt->LFSR_CTR &= ~SCSC_LFSR_CTR_TAMPER1_INV_MASK;

    //SCSC_virt->LFSR_CTR |= SCSC_LFSR_CTR_TAMPER0_INV_MASK;
    //SCSC_virt->LFSR_CTR |= SCSC_LFSR_CTR_TAMPER1_INV_MASK;

	printSSM_state();

    //hal_interruptsSetHandler(SNVS_Security_IRQn, vybrid_tamper_isr, NULL);
    //SNVS_virt->HPSICR |= SNVS_HPSICR_LPSVI_EN_MASK;

    SNVS_virt->LPTDCR = (1 << SNVS_LPTDCR_ET1_EN_SHIFT) | (1 << SNVS_LPTDCR_ET2_EN_SHIFT);

    memset (&tamperWq, 0, sizeof(tamperWq));

    proc_semaphoreCreate(&tindLock, 1);
    memset(tamper_indicator, 0, TAMPER_NUM * sizeof(tamper_t));
    for(result=0; result < TAMPER_NUM; ++result)
    {
        LIST_HEAD_INIT(&tamper_indicator[result].callbacks);
        if(result == 0)
            tamper_indicator[result].mask = SNVS_LPSR_ET1D_MASK;
        else if(result == 1)
            tamper_indicator[result].mask = SNVS_LPSR_ET2D_MASK;
    }
    tamperCheckEnable=1;
    proc_thread(NULL, vybrid_tamper_check, NULL, 0, NULL, ttRegular);

	if (dev_register(MAKEDEV(MAJOR_TAMPER, 0), &vybrid_tamper_ops) < 0)
        main_printf(ATTR_ERROR, "Vybrid-tamper: Can't register device for /dev/tamper!\n" );
	else
		assert(EOK==dev_mknod(MAKEDEV(MAJOR_TAMPER, 0), "tamper"));
}
