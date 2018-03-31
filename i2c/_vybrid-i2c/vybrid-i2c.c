/**
 * Vybrid-I2C kernel driver
 *
 * Synchronous functions to access an I2C bus.
 * The comfort function i2c_devRead implements the defacto standard I2C operation "read from device".
 * Asynchronous facilities started to get implemented around the i2c_pumpThread,
 * but were not finished.
 *
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Horacio Mijail Anton Quiles <horacio.anton@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <hal/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <proc/if.h>
#include <fs/if.h>
#include <dev/dev.h>
#include <lib/assert.h>
#include <stdbool.h>
#include "if.h"
#include <hal/MVF50GS10MK50.h>

/// @cond DEBUG
//#define DEBUG_BUILD 0

#ifdef DEBUG_BUILD
	#define DEBUG_BOOL 1
	#define debug_printf(...)	main_printf(__VA_ARGS__)
	#define DEBUG_FUNC
	#pragma message "*** DEBUG BUILD ***"
#else
	#define DEBUG_BOOL 0
	/* safe NOP macro*/
	#define debug_printf(...)	do {} while (0)
	#define DEBUG_FUNC			__attribute__((warning("A debug function is still being used!")))
#endif

#ifdef DEBUG_BUILD

	//show bus state changes
//	#define DEBUG_BUS_STATES	true
    #define DEBUG_BUS_STATES	false

	//show when a NACK is received
//	#define DEBUG_NACK			true
    #define DEBUG_NACK			false

	//show when no slave answers at a given address
//	#define DEBUG_MISSING		true
    #define DEBUG_MISSING		false

#else
	#define DEBUG_BUS_STATES	false
	#define DEBUG_NACK			false
	#define DEBUG_MISSING		false
#endif
/// @endcond DEBUG

struct i2c_cfg_st {
	I2C_Type *base;
	u32	irq;
};

static const struct i2c_cfg_st i2c_cfg[] ={
		{I2C0, I2C0_IRQn},
		{I2C1, I2C1_IRQn},
		{I2C2, I2C2_IRQn},
		{I2C3, I2C3_IRQn}
};


/** Result of an async transaction */
enum i2cMsg_results {
	IM_SUCCESS 		= -1,
	IM_IN_PROGRESS 	= -2,
	IM_NO_SUCH_SLAVE= -3,
	IM_NACK			= -4
	/*XXX IM_TIMEOUT if a slave keeps the NACK indefinitely?*/
};


/** Supporting infrastructure for async usage */
struct i2cMsg_st {
	u8 addr;
	bool write;
	u8 * buffer;
	u16 buffer_size;
	enum i2cMsg_results result;			/**< summary of the operation*/
	u16 bytes_done;						/**< num of successfully sent bytes*/
};


/** Data for an initialized bus instance */
struct i2c_st {
	I2C_Type *base;
	u16 irq;

	struct i2cMsg_st msg;
//	spinlock_t spinlock;
	mutex_t mutex;
//	thq_t waitq;
	event_t evt;
	u16 minor;
	char name[8];
	/*any counters, diagnostics, ...*/
	
};


/** Storage for the initialized bus instances */
struct i2c_st *i2c[I2C_NUM];



/** Prepare a bus instance so it is ready to work */
static void i2c_initRegs(struct i2c_st *i2c)
{
	I2C_Type *base = i2c->base;

	/* Set the divider for the SCL frequency
	 * 0x38 --> 64.9 KHz
	 * 0x24 --> 280 KHz?
	 */
	base->IBFD = 0x38 ; /*XXX adjust?*/

	/*Set our address as a slave - must not be used*/
	base->IBAD = 0x7Fu << 1;

	/* Clear MDIS to enable I2C*/
	base->IBCR &= ~I2C_IBCR_MDIS_MASK ;

	/* according to example code, enabling interrupt generation is
	 * required for IBIF to be set. That is not true.
	 */
//	base->IBCR |= I2C_IBCR_IBIE_MASK ;

	/* set slave mode so we don't generate a start condition accidentally*/
	base->IBCR &= ~I2C_IBCR_MSSL_MASK ;

	/* disable DMA */
	base->IBCR &= ~I2C_IBCR_DMAEN_MASK ;
}


static bool i2c_isBusIdle(u8 IBSR)	/* do not access the register itself to avoid unexpected changes */
{
	return((IBSR & I2C_IBSR_IBB_MASK) == 0);
}


static void i2c_clearInterruptFlag(I2C_Type *base)
{
	base->IBSR |= I2C_IBSR_IBIF_MASK;	/*clear IBIF flag by writing 1*/
}


void i2c_generateStart(addr_t base_addr)
{
	I2C_Type *base = (I2C_Type *) base_addr;
	while(!i2c_isBusIdle(base->IBSR)){
		if(DEBUG_BUS_STATES)
			debug_printf(ATTR_DEBUG, "b"); /*bus is busy, wait until it is idle. Can happen legally if a transaction tries to start before the last STOP has finished*/
		else
			hal_cpuReschedule();
	}

	base->IBCR |= I2C_IBCR_MSSL_MASK | I2C_IBCR_TXRX_MASK;	/*master mode (generates START) + Tx mode for address byte*/

	while(i2c_isBusIdle(base->IBSR)){
		if(DEBUG_BUS_STATES)
			debug_printf(ATTR_DEBUG, "S"); /*bus is still registering as idle; wait for START to take effect and so the bus changes to busy*/
		else
			hal_cpuReschedule();
	}

	if(DEBUG_BUS_STATES)
		debug_printf(ATTR_DEBUG, "s");
}


void i2c_generateStop(addr_t base_addr)
{
	I2C_Type *base = (I2C_Type *) base_addr;
	if(i2c_isBusIdle(base->IBSR)){ /*sanity check: we are mastering the bus and in a transaction, so the bus shouldn't be idle*/
		debug_printf(ATTR_DEBUG, "Want to STOP but bus seems idle\n");
	}
	base->IBCR &= ~I2C_IBCR_MSSL_MASK;	/*leave master mode (generates STOP)*/

	if(DEBUG_BUS_STATES)
		while(!i2c_isBusIdle(base->IBSR)){
			if(DEBUG_BUS_STATES)
				debug_printf(ATTR_DEBUG, "P"); /*bus is still registering as busy; wait for STOP to take effect and so the bus changes to idle*/
			hal_cpuReschedule();
		}

	if(DEBUG_BUS_STATES)
		debug_printf(ATTR_DEBUG, "p");
}


void i2c_generateRestart(addr_t base)
{
	((I2C_Type *)base)->IBCR |= I2C_IBCR_RSTA_MASK;
}


static bool i2c_didSlaveAck(I2C_Type *base){
	return ((base->IBSR & I2C_IBSR_RXAK_MASK) == 0);
}


/** Wait until the end of the current byte transaction
 *
 * @param base
 * @return		whether the waiting ended with no error conditions
 */
static bool i2c_waitByte(I2C_Type * base)
{
	bool sane = true;
	//XXX proc_threadCondWait() signalized by ISR instead of while
	//XXX timeout should be possible, since a slave can keep the bus in NACK indefinitely

	u8 IBSR;
	while(((IBSR = base->IBSR) & I2C_IBSR_IBIF_MASK) == 0){
		/*wait for the Interrupt Flag...*/
		hal_cpuReschedule();
	}

	/*sanity checks*/
	/* the transfer should register as completed */
	if(((IBSR & I2C_IBSR_TCF_MASK) == 0)){
		sane = false;
		debug_printf(ATTR_ERROR, "waitByte not completed! IBSR = 0x%2x\n", IBSR);
	}
	/* bus should be busy, because we are the only master and we are in the middle of a transaction*/
	if(i2c_isBusIdle(IBSR)){
		sane = false;
		debug_printf(ATTR_ERROR, "waitByte bus idle!\n");
	}
	/* we should keep being the master */
	if((IBSR & I2C_IBSR_IBAL_MASK) != 0){
		sane = false;
		debug_printf(ATTR_ERROR, "waitByte lost arbitration! IBSR = 0x%2x ,IBCR = 0x%2x\n", IBSR, base->IBCR);
		assert(false);
		base->IBSR |= I2C_IBSR_IBAL_MASK; /*clear by writing*/
	}

	i2c_clearInterruptFlag(base);
	return sane;
}


s16 i2c_busWrite(addr_t base_addr, u8 slave_addr, u8 * buf, u16 len, enum i2c_setBusCondition generate_STOP)
{
	u8 count = 0;		/*bytes that got an ACK*/
	u8 byte = (slave_addr << 1);
	bool ack;
	I2C_Type * base = (I2C_Type *)base_addr;

	if(DEBUG_BUS_STATES)
		debug_printf(ATTR_DEBUG, "W");

	i2c_generateStart(base_addr);
	do {
		i2c_clearInterruptFlag(base);
		base->IBDR = byte;	/*send the byte*/
		i2c_waitByte(base);

		ack = i2c_didSlaveAck(base);
		if(ack){
			count++;
			/*if all goes well, we'll send <len> bytes + 1 (for the address)*/
			if(count<len+1)
				byte = buf[count-1];
			else
				break;
		} else {
			if(DEBUG_MISSING && (count == 0))
				debug_printf(ATTR_DEBUG, "DevMissing ");
			if(DEBUG_NACK && (count > 0))
				debug_printf(ATTR_DEBUG, "NACK! ");
			break;
		}
	} while(true);

	if(generate_STOP)
		i2c_generateStop(base_addr);

	if(DEBUG_BUS_STATES)
		debug_printf(ATTR_DEBUG, "w");

	return count-1;

}


s16 i2c_busRead(addr_t base_addr, u8 slave_addr, u8 * buf, u16 len, enum i2c_setBusCondition generate_START)
{
	u8 count = 0;	/*num or READs done*/
	I2C_Type *base = (I2C_Type *)base_addr;

	if(DEBUG_BUS_STATES)
		debug_printf(ATTR_DEBUG, "R");

	if(generate_START)
		i2c_generateStart(base_addr);

	base->IBDR = (slave_addr << 1) | 1;					/*send address + READ flag*/
	i2c_waitByte(base);
	if(!i2c_didSlaveAck(base)){
		i2c_generateStop(base_addr);
		if(DEBUG_MISSING)
			debug_printf(ATTR_DEBUG, "DevMissing ");
		return 0;
	}
	base->IBCR &= ~(I2C_IBCR_TXRX_MASK | I2C_IBCR_NOACK_MASK);		/*switch to rx mode and send ACKs*/

	do {
		u8 byte;
		if(count == len-1)
			base->IBCR |= I2C_IBCR_NOACK_MASK;	/*the last byte request must be prepared to NOT send ACK*/
		byte = base->IBDR;						/*get last rxed byte, start rx of next one*/
		if(count>0)
			buf[count-1] = byte;				/*ignore the result of the first read, which is only needed for starting the rxor*/
		count++;
		i2c_waitByte(base);

	//} while(count<=len);
	} while(count < len);

	/*finish the reception*/
	i2c_generateStop(base_addr);						/*so the coming last read won't start a new rx*/
	buf[count-1] = base->IBDR;

	if(DEBUG_BUS_STATES)
		debug_printf(ATTR_DEBUG, "r");
	return count;

}

/** Reset the bus
 *
 * XXX This is surely not enough. A slave which gets out of clock sync (maybe because of interference) while sending
 * might be blocking the lines that would allow us to set a STOP state.
 * The only exit is then to bitbang the clock line until the slave unstucks.
 * http://www.analog.com/static/imported-files/application_notes/54305147357414AN686_0.pdf
 *
 * For SMBus devices, we can force a reset by keeping SLC low for 35 ms. Is that automatic or needs bitbanging?
 *
 * @param base
 * @return 	whether the bus seems idle after the reset
 */
bool i2c_busReset(I2C_Type *base)
{
	addr_t base_addr = (addr_t) base;
	i2c_generateStart(base_addr);
	hal_cpuReschedule();	/*wait a bit*/
	i2c_generateStop(base_addr);
	while(!i2c_isBusIdle(base->IBSR)){
		if(DEBUG_BUS_STATES)
			debug_printf(ATTR_DEBUG, "Z");
		else
			hal_cpuReschedule();	//XXX change to an explicit wait
	}
	return (i2c_isBusIdle(base->IBSR));
}


/** Thread to pump the message byte by byte as signaled by the ISR.
 *
 *	Traditional "ISR-bottom half" implementation
 *
 * @warning: UNFINISHED. The auxiliar read/write funcs are not implemented;
 * we'll probably switch to another driver model, where a set of funcs
 * 1)prepare the buffer,
 * 2)pump the contents of the buffer byte-by-byte (to be called from ISR, timer, etc)
 * 3)retrieve a result/state
 *
 */
static int i2c_pumpThread(void *arg) __attribute__((unused));
static int i2c_pumpThread(void *arg)
{
	struct i2c_st *i2c = arg;
	I2C_Type *base = i2c->base;
	addr_t base_addr = (addr_t) base;
	struct i2cMsg_st *msg = &i2c->msg;

	/*no general locking needed because there's only this thread interacting with the hw*/
	//XXX ISR does not touch regs?
	for(;;){
		/*wait for start signal*/
		//XXX threadCondWait or eventWait?
		if (proc_eventWait(&i2c->evt, 0) != EOK){
			break;
		}
		/* the I2C_msg struct is now ready */
		msg->result = IM_IN_PROGRESS;

		if(msg->write){
			msg->bytes_done = i2c_busWrite(base_addr, msg->addr, msg->buffer, msg->buffer_size, ISBC_SET);
			if(msg->bytes_done == msg->buffer_size)
				msg->result = IM_SUCCESS;
			else if(msg->bytes_done == 0)
				msg->result = IM_NO_SUCH_SLAVE;
			else msg->result = IM_NACK;
		} else {
			msg->bytes_done = i2c_busRead(base_addr, msg->addr, msg->buffer, msg->buffer_size, ISBC_SET);
			if(msg->bytes_done == 0)
				msg->result = IM_NO_SUCH_SLAVE;
			else
				msg->result = IM_SUCCESS;
		}

	}
	main_printf(ATTR_INFO, "Vybrid-I2C: pump thread exited\n");
	return 0;

}


/**Initialize I2C and supporting infrastructure
 *
 * @param i2c_regset
 * @param irq_num
 * @param minor
 * @param[OUT] serial_pp pointer to where to leave a pointer to the allocated serial private struct
 * @return error code
 */
static int i2c_initOne(I2C_Type *base, u16 irq_num, const int minor, struct i2c_st **i2c)
{
	int status;

	/* Allocate driver structures */
	if (((*i2c) = vm_kmalloc(sizeof(struct i2c_st))) == NULL)
		return -ENOMEM;

	main_memset(*i2c, 0, sizeof(struct i2c_st));

	status = vm_iomap((addr_t)base, sizeof(I2C_Type), PGHD_DEV_RW, (void **)&((*i2c)->base));
	assert(status == EOK);

	(*i2c)->irq = irq_num;
	(*i2c)->minor = minor;
	strcpy((*i2c)->name, "I2Cx");
	(*i2c)->name[3] = '0' + minor;

	i2c_initRegs(*i2c);
	proc_mutexCreate(&(*i2c)->mutex);
	main_printf(ATTR_INFO,"dev: %s at 0x%x irq=%d\n", (*i2c)->name, (addr_t)base, irq_num);

	return EOK;
}


int _i2c_init(void)
{
	int result = 0;
	s32 res;

	u8 i;
	for(i=0;i<I2C_NUM;i++){
		u16 minor = i;
		//i2c_busReset(i2c_cfg[i].base);
		if ((res = i2c_initOne(i2c_cfg[i].base, i2c_cfg[i].irq, minor, &i2c[i])) != EOK) {
			if(res != EOK){
				main_printf(ATTR_ERROR, "Vybrid-I2C[%d] initialization failed\n", i);
			}
			result++;
		}
	}

	return -result;
}

addr_t i2c_getBase(u8 num)
{
	return (addr_t)(num < I2C_NUM ? i2c[num]->base : NULL);
}


int i2c_devRead(unsigned i2c_bus_num, u8 i2c_bus_addr, u8 reg, u8 buf[], u8 n)
{
	//it is per-bus synchronized
	s16 res;
	u8 cmd[1]={reg};
	addr_t i2c_base_addr;
	assert(i2c_bus_num < I2C_NUM);
	i2c_base_addr = i2c_getBase(i2c_bus_num);
	
	proc_mutexLock(&i2c[i2c_bus_num]->mutex);
	res = i2c_busWrite(i2c_base_addr, i2c_bus_addr, cmd, 1, ISBC_SET);
//	main_printf(ATTR_INFO, "wrote to addr %d, res = %d\n", i2c_bus_addr, res);
	if(res != 1)
    {
        proc_mutexUnlock(&i2c[i2c_bus_num]->mutex);
		return -1;
    }
	res = i2c_busRead(i2c_base_addr, i2c_bus_addr, buf, n, ISBC_SET);
//	main_printf(ATTR_INFO, "read from addr %d, res = %d, buf = %x\n", i2c_bus_addr, res, buf[0]);
	proc_mutexUnlock(&i2c[i2c_bus_num]->mutex);
	if(res != n)
		return -1;
	return EOK;
}

int i2c_devWrite(unsigned i2c_bus_num, u8 i2c_bus_addr, u8 reg, u8 buf[], u8 len)
{
	//basically, this is the same as i2c_busWrite, but it writes reg in between of slave address and buf.
	//it also is per-bus synchronized

	u8 count = 0;		/*bytes that got an ACK*/
	u8 byte = (i2c_bus_addr << 1);
	bool ack;
	
	I2C_Type * base;
	assert(i2c_bus_num < I2C_NUM);
	base = (I2C_Type *) i2c_getBase(i2c_bus_num);
	proc_mutexLock(&i2c[i2c_bus_num]->mutex);
	if(DEBUG_BUS_STATES)
		debug_printf(ATTR_DEBUG, "W");
	
	i2c_generateStart((addr_t)base);
	do {
		i2c_clearInterruptFlag(base);
		base->IBDR = byte;	/*send the byte*/
		i2c_waitByte(base);

		ack = i2c_didSlaveAck(base);
		if(ack){
			count++;
			/*if all goes well, we'll send <len> bytes + 1 (for the address)*/
			if(count == 1)
			{
				byte = reg;
			}
			else if(count<len+2)
				byte = buf[count-2];
			else
				break;
		} else {
			if(DEBUG_MISSING && (count == 0))
				debug_printf(ATTR_DEBUG, "DevMissing ");
			if(DEBUG_NACK && (count > 0))
				debug_printf(ATTR_DEBUG, "NACK! ");
			break;
		}
	} while(true);

	i2c_generateStop((addr_t)base);

	if(DEBUG_BUS_STATES)
		debug_printf(ATTR_DEBUG, "w");
	proc_mutexUnlock(&i2c[i2c_bus_num]->mutex);
	if(count-2 == len)
		return EOK;
	else
		return -1;
}

bool i2c_isDevPresent(addr_t i2c_base_addr, u8 i2c_bus_addr)
{
	s16 res = i2c_busWrite(i2c_base_addr, i2c_bus_addr, NULL, 0, ISBC_SET);
	return res == 0;

}
