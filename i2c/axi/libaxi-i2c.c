/*
 * Phoenix-RTOS
 *
 * Xilinx/AMD AXI_I2C (axi_iic) driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Kamil Ber
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/mman.h>

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <i2c.h>

#if defined(__CPU_ZYNQ7000)
#include <phoenix/arch/armv7a/zynq7000/zynq7000.h>
#elif defined(__CPU_ZYNQMP)
#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>
#else
#error "Unsupported platform"
#endif

#include <board_config.h>

/* I2C registers, from PG090 Table 4 */
#define I2C_REG_GIE						(0x01C/4)
#define I2C_REG_ISR						(0x020/4)
#define I2C_REG_IER						(0x028/4)
#define I2C_REG_SOFTR					(0x040/4)
#define I2C_REG_CR						(0x100/4)
#define I2C_REG_SR						(0x104/4)
#define I2C_REG_TX_FIFO				(0x108/4)
#define I2C_REG_RX_FIFO				(0x10C/4)
#define I2C_REG_ADR						(0x110/4)
#define I2C_REG_TX_FIFO_OCY		(0x114/4)
#define I2C_REG_RX_FIFO_OCY		(0x118/4)
#define I2C_REG_TEN_ADR				(0x11C/4)
#define I2C_REG_RX_FIFO_PIRQ	(0x120/4)
#define I2C_REG_GPO						(0x124/4)
#define I2C_REG_TSUSTA				(0x128/4)
#define I2C_REG_TSUSTO				(0x12C/4)
#define I2C_REG_THDSTA				(0x130/4)
#define I2C_REG_TSUDAT				(0x134/4)
#define I2C_REG_TBUF					(0x138/4)
#define I2C_REG_THIGH					(0x13C/4)
#define I2C_REG_TLOW					(0x140/4)
#define I2C_REG_THDDAT				(0x144/4)

#define GIE_EN									31

#define ISR_ARB_LOST						0
#define ISR_TX_ERR_OR_COMPLETE	1
#define ISR_TX_FIFO_EMPY				2
#define ISR_RX_FIFO_FULL				3
#define ISR_BUS_NOT_BUSY				4
#define ISR_ADDR_AS_SLAVE				5
#define ISR_NOT_ADDR_AS_SLAVE		6
#define ISR_TX_FIFO_HALF_EMPTY	7

#define CR_EN									0
#define CR_TX_FIFO_RST				1
#define CR_TX									3
#define CR_MSMS								2
#define CR_TXAK								4
#define CR_RSTA								5

#define SR_ABGC								0
#define SR_AAS								1
#define SR_BB									2
#define SR_SRW								3
#define SR_TX_FIFO_FULL				4
#define SR_RX_FIFO_FULL				5
#define SR_RX_FIFO_EMPTY			6
#define SR_TX_FIFO_EMPTY			7


#define I2C_FIFO_DEPTH				16
/* TODO: not used in this version*/
#define I2C_TRANS_SIZE_MAX		16

typedef struct {
	unsigned int irq;	/* I2C controller IRQ */
	struct {
		int clk_freq;		/* Frequency of Clock, in Hz */
		int i2c_freq;		/* Frequency of I2C BUSm in Hz */
		int tsusta;			/* TSUSTA timing in ns (tsusta + tr) */
		int tsusto;			/* TSUSTO timing in ns (tsusto + tr) */
		int thdsta;			/* THDSTA timing in ns (thdsta + tf) */
		int tsudat;			/* TSUDAT timing in ns (tsudat + tf) */
		int tbuf;				/* TSUDAT timing in ns */
	} timings;
	addr_t paddr;			/* I2C controller base physical address */
} i2c_info_t;

/* Timings from IMX219 from IMX219 PG:
 * tr 		= 120 ns (max)
 * tf 		= 120 ns (max)
 * tsusta 	= 260 ns (min)
 * tsusto 	= 260 ns (min)
 * thdsta 	= 260 ns (min)
 * tsudat 	= 50 ns (min)
 * tbus   	= 500 ns (min)
 */

/* clang-format off */
/* Device configuration based on board_config */
/* TODO:: inject base address*/
#if defined(__CPU_ZYNQMP)
static const i2c_info_t devsInfo[] = {
	{
		.irq = 121,
		.paddr = 0xe0004000,
		.timings = {
			.clk_freq = 100000000,
			.i2c_freq = 100000,
			.tsusta = 5700,
			.tsusto = 5000,
			.thdsta = 4300,
			.tsudat = 550,
			.tbuf = 5000
		}
	},
	{
		.irq = 122,
		.paddr = 0xe0005000,
		.timings = {
			.clk_freq = 100000000,
			.i2c_freq = 100000,
			.tsusta = 5700,
			.tsusto = 5000,
			.thdsta = 4300,
			.tsudat = 550,
			.tbuf = 5000
		}
	}
};
#endif
/* clang-format on */

static struct {
	unsigned int devID;
	volatile uint32_t *base; /* I2C registers base address */
	int initialized;
	handle_t lock; 		/* I2C IRQ mutex */
	handle_t cond; 		/* I2C IRQ cond */
	handle_t inth; 		/* I2C IRQ handle */
	volatile uint32_t st;
} i2c = { 0 };

/* Not implemented:
 *  - slave mode
 *  - multi-master mode (arbitration lost status flag inspection)
 *  - dynamic control mode
 */

static int i2c_isr(unsigned int n, void *arg)
{

	uint32_t reg;

	i2c.st = *(i2c.base + I2C_REG_ISR) & *(i2c.base + I2C_REG_IER);


	/* TODO: what if more than one irq status bit is asserted???*/
	/* For some irq sources this won't work! Eg. RX Full -> Read the RX data first!*/
	/* clear status register */
	*(i2c.base + I2C_REG_ISR) = i2c.st;

	/* arbitration lost */
	if (i2c.st & (1 << ISR_ARB_LOST)) {
		/* from pg090: Firmware must respond by first clearing the Control Register (CR) MSMS bit*/
		reg = *(i2c.base + I2C_REG_CR) & ~(1 << CR_MSMS);
		
		*(i2c.base + I2C_REG_CR) = reg;
		/* From pg090: this bit must be set to flush the FIFO if either 
		 * (a) arbitration is lost or 
		 * (b) if a transmit error occurs.
		*/
		*(i2c.base + I2C_REG_CR) = reg | (1 << CR_TX_FIFO_RST); 
		*(i2c.base + I2C_REG_CR) = reg & ~(1 << CR_TX_FIFO_RST);
		
		*(i2c.base + I2C_REG_IER) &= ~(1 << ISR_ARB_LOST);
	}

	
	/* tx fifo empty (tx error or receivecompletion) */
	if (i2c.st & (1 << ISR_TX_ERR_OR_COMPLETE)) {
		*(i2c.base + I2C_REG_IER) &= ~(1 << ISR_TX_ERR_OR_COMPLETE);
	}
	/* tx fifo empty (receive underflow) */
	else if (i2c.st & (1 << ISR_RX_FIFO_FULL)) {
		/* first, need to clear the RX FULL status!!!*/
		*(i2c.base + I2C_REG_IER) &= ~(1 << ISR_RX_FIFO_FULL);
	}
	/* rx fifo full (receive underflow) */
	else if (i2c.st & (1 << ISR_TX_FIFO_EMPY)) {
		*(i2c.base + I2C_REG_IER) &= ~(1 << ISR_TX_FIFO_EMPY);
	}
	/* bus not busy */
	else if (i2c.st & (1 << ISR_BUS_NOT_BUSY)) {
		*(i2c.base + I2C_REG_IER) &= ~(1 << ISR_BUS_NOT_BUSY);
	}
	/* addressed as a slave */
	else if (i2c.st & (1 << ISR_ADDR_AS_SLAVE)) {
		*(i2c.base + I2C_REG_IER) &= ~(1 << ISR_ADDR_AS_SLAVE);
	}
	/* not addressed as a slave */
	else if (i2c.st & (1 << ISR_NOT_ADDR_AS_SLAVE)) {
		*(i2c.base + I2C_REG_IER) &= ~(1 << ISR_NOT_ADDR_AS_SLAVE);
	}  
	/* tx fifo half empty */
	else if (i2c.st & (1 << ISR_TX_FIFO_HALF_EMPTY)) {
		*(i2c.base + I2C_REG_IER) &= ~(1 << ISR_TX_FIFO_HALF_EMPTY);
	}

	return 1;
}


static inline int i2c_isBusBusy(void)
{
	return *(i2c.base + I2C_REG_SR) & (1 << SR_BB);
}


static inline void i2c_enableIrq(uint32_t mask)
{
	*(i2c.base + I2C_REG_IER) |= mask;
}

static inline void i2c_disableIrq(uint32_t mask)
{
	*(i2c.base + I2C_REG_IER) &= ~mask;

	i2c.st &= ~mask;
}


static inline void i2c_clearIrq(uint32_t mask)
{
	/* clear interrupts */
	*(i2c.base + I2C_REG_ISR) = mask;

	i2c.st &= ~mask;
}

static inline void i2c_clearIrqSt(void)
{
	uint32_t reg;

	/* clear status interrupts */
	reg = *(i2c.base + I2C_REG_ISR);
	*(i2c.base + I2C_REG_ISR) = reg;

	/* clear status */
	i2c.st = 0;
}


static int i2c_trxComplete(void)
{
	int ret = EOK;
	const time_t timeoutUs = 3000;

	mutexLock(i2c.lock);
	
	/* wait for data or completion irq */
	while ((i2c.st & ((1 << ISR_TX_FIFO_EMPY) | (1 << ISR_RX_FIFO_FULL))) == 0) {
		if (condWait(i2c.cond, i2c.lock, timeoutUs) == -ETIME) {
			ret = -ETIMEDOUT;
			break;
		}

		/* TX Error (no slave os NACK from slave), receive underflow, arbitration lost */
		if (i2c.st & ((1 << ISR_TX_ERR_OR_COMPLETE) | (1 << ISR_RX_FIFO_FULL) | (1 << ISR_ARB_LOST)) ) {
			ret = -EIO;
		}
	}
	i2c.st = 0;
	mutexUnlock(i2c.lock);

	return ret;
}


int i2c_busWrite(uint8_t dev_addr, const uint8_t *data, uint32_t len)
{
	unsigned int fifoAvail;
	int i, ret = EOK, lenDone = 0, start = 1;

	if (i2c.initialized == 0) {
		return -EIO;
	}

	if (data == NULL) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	if (i2c_isBusBusy() == 1) {
		return -EBUSY;
	}

	/* clear FIFO, enable ACK (0 = active) */
	*(i2c.base + I2C_REG_CR) = (1 < CR_TX_FIFO_RST) | (1 < CR_EN);
	*(i2c.base + I2C_REG_CR) = (1 < CR_EN);

	i2c_clearIrqSt();

	while ((len - lenDone) > 1) {

		/* Master Transmitter with a Repeated Start, addr needs 
		 * to be transmitted with each chunk 
		 */
		*(i2c.base + I2C_REG_TX_FIFO) = (dev_addr << 1);

		/* write data to FIFO */
		fifoAvail = I2C_FIFO_DEPTH - *(i2c.base + I2C_REG_TX_FIFO_OCY);
		for (i = 0; i < fifoAvail && (len - lenDone++) > 1; ++i) {
			*(i2c.base + I2C_REG_TX_FIFO) = *(data++);
		}

		/* Enable interrupts, arbitration lost, tx overflow, transfer NACK, 
		 * completion irq, bus not busy 
		 */
		i2c_clearIrq((1 << ISR_TX_FIFO_EMPY));
		 *(i2c.base + I2C_REG_IER) = (1 << ISR_RX_FIFO_FULL) | (1 << ISR_TX_FIFO_EMPY) | 
									(1 << ISR_TX_ERR_OR_COMPLETE) | (1 << ISR_ARB_LOST);
		

		/* start TX */
		if (start == 1) {
			*(i2c.base + I2C_REG_CR) = (1 << CR_MSMS) | (1 << CR_TX) | (1 < CR_EN);
			start = 0;
		}
		else {
			*(i2c.base + I2C_REG_CR) = (1 << CR_RSTA) | (1 << CR_MSMS) | (1 << CR_TX) | (1 < CR_EN);
		}

		ret = i2c_trxComplete();
		if (ret < 0) {
			break;
		}
	}
	/* last byte for which MSMS will be cleared and the STOP generated*/
	if (ret >= 0) {
		*(i2c.base + I2C_REG_CR) = (1 << CR_TX) | (1 < CR_EN);
		*(i2c.base + I2C_REG_TX_FIFO) = *(data++);
	}  
	/* arbitration lost, tx overflow, transfer NACK, completion irq, bus not busy */
	*(i2c.base + I2C_REG_IER) = (1 << ISR_RX_FIFO_FULL) | (1 << ISR_TX_FIFO_EMPY) | 
								(1 << ISR_TX_ERR_OR_COMPLETE) | (1 << ISR_ARB_LOST);
	i2c_clearIrq((1 << ISR_TX_FIFO_EMPY));

	/* disable interrupts */
	*(i2c.base + I2C_REG_IER) = 0;

	return ret;
}


static int i2c_transferRead(uint8_t dev_addr, uint8_t *buff, uint32_t len, uint8_t start, uint8_t stop)
{
	/* this function performs read in chunks of up to 16B with Repeated starts in between */
	int ret = EOK, i, size;
	uint32_t reg;

	if (len > 16) {
		return -EINVAL;
	}

	if (len > 2) {
		size = len-1;
	} 	
	else {
		size = 1;
	}
	*(i2c.base + I2C_REG_RX_FIFO_PIRQ) = size-1;
 
	/* set slave address and start transmission */
	*(i2c.base + I2C_REG_TX_FIFO) = (dev_addr << 1) | 1;
	i2c_clearIrq((1 << ISR_RX_FIFO_FULL));
	/* arbitration lost, tx overflow, transfer NACK, completion irq, bus not busy */
	*(i2c.base + I2C_REG_IER) = (1 << ISR_RX_FIFO_FULL) | (1 << ISR_TX_FIFO_EMPY) | 
								(1 << ISR_TX_ERR_OR_COMPLETE) | (1 << ISR_ARB_LOST);

	/* write address of address */
	if (start == 1) {
		reg = (1 << CR_MSMS) | (1 < CR_EN);
	} else {
		reg = (1 << CR_RSTA) | (1 << CR_MSMS) | (1 < CR_EN);
	}
	*(i2c.base + I2C_REG_CR) = reg;

	/* wait for completion of address transmission*/
	ret = i2c_trxComplete();
	if (ret < 0) {
		return ret;
	}
	i2c_clearIrq((1 << ISR_TX_FIFO_EMPY));
	*(i2c.base + I2C_REG_IER) = (1 << ISR_RX_FIFO_FULL) | (1 << ISR_TX_ERR_OR_COMPLETE) | 
								(1 << ISR_ARB_LOST);


	/* lasta byte, no-ack, len = 1 case*/
	if (len == 1) { 
		reg |= (1 << CR_TXAK);
		if (stop == 1) {
			reg &= ~(1 << CR_MSMS);
		}
	}
	
	/* wait for reception of len-1 bytes*/
	ret = i2c_trxComplete();
	if (ret < 0) {
		return ret;
	}
	

	/* lasta byte, no-ack, len > 1 case*/
	if (len > 1) {
		reg |= (1 << CR_TXAK);
		if (stop == 1) {
			reg &= ~(1 << CR_MSMS);
		}
		*(i2c.base + I2C_REG_CR) = reg;
	}
	
	/* read len-1 bytes from fifo*/
	for (i = 0; i < size; ++i) {
		*(buff++) = *(i2c.base + I2C_REG_RX_FIFO);
	}
	i2c_clearIrq((1 << ISR_RX_FIFO_FULL));
	*(i2c.base + I2C_REG_IER) = (1 << ISR_RX_FIFO_FULL) | (1 << ISR_TX_ERR_OR_COMPLETE) | 
								(1 << ISR_ARB_LOST);

	if (len > 1) {
		/* wait for reception of last byte*/
		ret = i2c_trxComplete();
		if (ret < 0) {
			return ret;
		}
		/* read last byte from fifo*/
		*(buff++) = *(i2c.base + I2C_REG_RX_FIFO);
		i2c_clearIrq((1 << ISR_RX_FIFO_FULL));
		*(i2c.base + I2C_REG_IER) = (1 << ISR_RX_FIFO_FULL) | (1 << ISR_TX_ERR_OR_COMPLETE) | 
									(1 << ISR_ARB_LOST);

	}

	return ret;
}


int i2c_busRead(uint8_t dev_addr, uint8_t *data_out, uint32_t len)
{
	int ret = EOK, size = 0, bytes2Read, start = 1, stop = 0;

	if (i2c.initialized == 0) {
		return -EIO;
	}

	if (data_out == NULL) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	if (i2c_isBusBusy() == 1) {
		return -EBUSY;
	}

	*(i2c.base + I2C_REG_CR) |= (1 << CR_EN);

	i2c_clearIrqSt();

	do {
		bytes2Read = (len - size >= I2C_FIFO_DEPTH) ? I2C_FIFO_DEPTH : (len - size);
		if (bytes2Read < I2C_FIFO_DEPTH) {
			stop = 1;
		}

		ret = i2c_transferRead(dev_addr, data_out, bytes2Read, start, stop);
		if (ret < 0) {
			break;
		}
		start = 0;

		data_out += bytes2Read;
		size += bytes2Read;
	} while (size < len);

	/* disable interrupts */
	*(i2c.base + I2C_REG_IER) = 0;

	return ret;
}


int i2c_regRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len)
{
	int ret;

	ret = i2c_busWrite(dev_addr, &reg_addr, sizeof(reg_addr));
	if (ret < 0) {
		return ret;
	}

	return i2c_busRead(dev_addr, data_out, len);
}


static void i2c_initCtrl(void)
{
	
	/* Depth of RX fifo for interrupt generation*/
	*(i2c.base + I2C_REG_RX_FIFO_PIRQ) = I2C_FIFO_DEPTH - 1;

	/* clear TX and RX fifos */
	*(i2c.base + I2C_REG_CR) = (1 < CR_TX_FIFO_RST);

	/* enable controller */
	*(i2c.base + I2C_REG_CR) = (1 < CR_EN);

	/* enable interrupts */
	*(i2c.base + I2C_REG_GIE) = (1 < GIE_EN);

	i2c_clearIrqSt();

}


static int i2c_initClk(void)
{
	int reg;
	const i2c_info_t *info = &devsInfo[i2c.devID];

	/* SCL low/high -> in number of input clock cycles*/
	reg = info->timings.clk_freq / (2 * info->timings.i2c_freq) - 7 ;
	if (reg == 0) {
		return -EINVAL;
	} 
	*(i2c.base + I2C_REG_THIGH) = reg;
	*(i2c.base + I2C_REG_TLOW) = reg;

	/* this div can lead to a significant error, however it ensures
	 * set timings are slower than the I2C spec in worst case 
	 */
	reg = 1000000000 / info->timings.clk_freq;
	
	*(i2c.base + I2C_REG_TSUSTA) = info->timings.tsusta / reg;
	*(i2c.base + I2C_REG_TSUSTO) = info->timings.tsusto / reg;
	*(i2c.base + I2C_REG_THDSTA) = info->timings.thdsta / reg;
	*(i2c.base + I2C_REG_TSUDAT) = info->timings.tsudat / reg;
	*(i2c.base + I2C_REG_TBUF) = info->timings.tbuf / reg;

	return EOK;
}


static int i2c_reset(void)
{
	/* from pg090: write 0xA to reset registers to their default states.*/
	*(i2c.base + I2C_REG_SOFTR) = 0xA;
	*(i2c.base + I2C_REG_SOFTR) = 0xA;
	
	return EOK;
}


int i2c_init(unsigned int dev_no)
{
	int err;

	if (dev_no >= sizeof(devsInfo) / sizeof(devsInfo[0])) {
		return -ENODEV;
	}

	i2c.devID = dev_no;

	/* reset controller */
	err = i2c_reset();
	if (err < 0) {
		return err;
	}

	/* initialize clock */
	err = i2c_initClk();
	if (err < 0) {
		return err;
	}


	err = mutexCreate(&i2c.lock);
	if (err < 0) {
		return err;
	}

	err = condCreate(&i2c.cond);
	if (err < 0) {
		resourceDestroy(i2c.lock);
		return err;
	}

	/* map i2c registers */
	i2c.base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, devsInfo[dev_no].paddr);
	if (i2c.base == MAP_FAILED) {
		resourceDestroy(i2c.cond);
		resourceDestroy(i2c.lock);
		return -ENOMEM;
	}

	/* controller initialization */
	i2c_initCtrl();

	interrupt(devsInfo[dev_no].irq, i2c_isr, NULL, i2c.cond, &i2c.inth);

	i2c.initialized = 1;

	return EOK;
}
