/**
 * Freescale Vybrid UART driver
 * 
 * Phoenix-RTOS
 * 
 * Operating system kernel
 * 
 * @file vybrid-uart/uart.c
 *
 * @copyright 2014 Phoenix Systems
 * 
 * @author Horacio Mijail Anton Quiles <horacio.anton@phoesys.com>
 * @author Jacek Popko <jacek.popko@phoesys.com>
 * @author Katarzyna Baranowska <katarzyna.baranowska@phoesys.com>
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
#include "if.h"
#include "ringbuff.h"
#include <hal/MVF50GS10MK50.h>
#include <include/termios.h>
#include <include/ioctl.h>


#define UART_DATA      0x03FF
#define UART_ERROR     0x1C00
#define UART_BREAK     0x0400
#define UART_PAR_ERR   0x0800
#define UART_NOISE_ERR 0x1000


static const struct {
	UART_Type *base;
	unsigned irq;
} uart_cfg[] = {
	{UART0, UART0_IRQn},
	{UART1, UART1_IRQn},
	{UART2, UART2_IRQn},
	{UART3, UART3_IRQn},
	{UART4, UART4_IRQn},
	{UART5, UART5_IRQn}	
};


struct serial_st {
	UART_Type *base;
	ringbuff_t *rx_buff;
	ringbuff_t *tx_buff;
	unsigned rbuffsz_hw;	/**< size of the hardware rx buffer */
	unsigned tbuffsz_hw;	/**< size of the hardware tx buffer */
	spinlock_t spinlock;		/**< register access locking*/
	thq_t waitqRx, waitqTx, waitqTc;
	unsigned overrun;
	unsigned overflow;
	unsigned framing;
	unsigned underrun;
	char name[8];
	struct termios termios;
} *serials[UARTS_NUM];


static int uart_open(vnode_t *vnode, file_t* file)
{
	if (MINOR(vnode->dev) >= UARTS_NUM)
		return -ENODEV;

	assert(vnode != NULL);
	vnode->flags |= VNODE_TTY;
	return 0;
}


static void ringbuff_dropIgnoredData(ringbuff_t *rb, struct termios *termios)
{
	u16 data;

	while (!ringbuff_empty(rb)) {
		data = ringbuff_next(rb);
		if ((data & UART_BREAK) && (termios->c_iflag & IGNBRK)) {
			data = ringbuff_get(rb);
			continue;
		}
		if ((data & UART_PAR_ERR) && (termios->c_iflag & PARENB) && (termios->c_iflag & IGNPAR)) {
			data = ringbuff_get(rb);
			continue;
		}
		break;
	}
}


//nibble even parity lookaside buffer
const static unsigned char parity_lb[] =
{
0, //0x0
1, //0x1
1, //0x2
0, //0x3
1, //0x4
0, //0x5
0, //0x6
1, //0x7
1, //0x8
0, //0x9
0, //0xA
1, //0xB
0, //0xC
1, //0xD
1, //0xE
0  //0xF
};

/** Put some received chars into the given buffer
 *
 * Might return less than the asked-for quantity.
 * Reads repeatedly the rx queue while new data appears in it, so might return more chars than rxbuffer's size.
 * @note If data is received by the HW and is not drained by us, the FIFO will stay filled up with old data while new data is discarded. There is no notification of data lost.
 *
 * @param vnode
 * @param offs	UNUSED
 * @param buff
 * @param len	max number of chars to read
 * @return		number of read chars, or error (negative)
 */
//TODO PARMRK, etc. Send interrupt on BRKINT.
static int uart_read(file_t* file, offs_t offs, char *buff, unsigned int len)
{
    vnode_t *vnode = file->vnode;
	struct serial_st *serial;
	int cnt = 0;
	int status;
	u16 data = 0;

	if (MINOR(vnode->dev) >= UARTS_NUM)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -ENOENT;

	proc_spinlockSet(&serial->spinlock);
	if (ringbuff_empty(serial->rx_buff)) {
		status = proc_condWait2(&serial->waitqRx, &vnode->mutex, &serial->spinlock, 0);
		if (status < 0)
			return status;
	}
	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	
	ringbuff_dropIgnoredData(serial->rx_buff, &serial->termios);
	while (cnt >= 0 && !ringbuff_empty(serial->rx_buff) && cnt < len) {
		data = ringbuff_next(serial->rx_buff);
		if ((data & UART_BREAK) && (serial->termios.c_iflag & BRKINT)) {
			if (cnt == 0) {
				data = ringbuff_get(serial->rx_buff);
				cnt = -EINTR;
			}
			break;
		}
		data = ringbuff_get(serial->rx_buff);
		if (data & UART_ERROR)
			*buff++ = (char) 0;
		else if (!(serial->termios.c_iflag & ISTRIP))
			*buff++ = (char) data;
		else if ((serial->termios.c_cflag & CSIZE) == CS7)
			*buff++ = (char) (data & 0x007F);
		else /* (serial->termios.c_cflag & CSIZE) == CS8 */
			*buff++ = (char) (data & 0x00FF);
		cnt++;
		ringbuff_dropIgnoredData(serial->rx_buff, &serial->termios);
	}

	proc_spinlockSet(&serial->spinlock);
	serial->base->C2 |= UART_C2_RE_MASK;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	return cnt;
}


/** Queue chars into the tx buffer
 *
 * Queues up to <len> chars and starts transmission
 *
 * @param vnode
 * @param offs	UNUSED
 * @param buff
 * @param len
 * @return number of chars that were queued
 */
static int uart_write(file_t* file, offs_t offs, char *buff, unsigned int len)
{
    vnode_t *vnode = file->vnode;
	struct serial_st *serial;
	unsigned cnt = 0;
	int status;

	if (MINOR(vnode->dev) >= UARTS_NUM)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -EINVAL;

	proc_spinlockSet(&serial->spinlock);
	if (ringbuff_full(serial->tx_buff)) {
		status = proc_condWait2(&serial->waitqTx, &vnode->mutex, &serial->spinlock, 0);
		if (status < 0)
			return status;
	}
	proc_spinlockClear(&serial->spinlock, sopGetCycles);


    if((serial->termios.c_cflag & CSIZE) == CS7 && serial->termios.c_cflag & PARENB)
    {

        if(serial->termios.c_cflag & PARODD)
        {
            //odd parity
            while (!ringbuff_full(serial->tx_buff) && cnt < len) {
                u16 c = *buff++;
                ringbuff_put(serial->tx_buff, (u16) ((c & 0x007F) | ((parity_lb[c & 0x000F] == parity_lb[(c & 0x00F0) >> 4]) << 7)));
                cnt++;
            }
        }
        else
        {
            //even parity
            while (!ringbuff_full(serial->tx_buff) && cnt < len) {
                u16 c = *buff++;
                ringbuff_put(serial->tx_buff, (u16) ((c & 0x007F) | ((parity_lb[c & 0x000F] != parity_lb[(c & 0x00F0) >> 4]) << 7)));
                cnt++;
            }
        }
    }
    else
    {
        while (!ringbuff_full(serial->tx_buff) && cnt < len) {
            ringbuff_put(serial->tx_buff, (u16) (*buff++));
            cnt++;
        }
    }
	proc_spinlockSet(&serial->spinlock);
	serial->base->C2 |= UART_C2_TIE_MASK;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	return cnt;
}


static int uart_poll(file_t* file, ktime_t timeout, int op)
{
    vnode_t *vnode = file->vnode;
	struct serial_st *serial;
	int err = EOK;

	if (MINOR(vnode->dev) >= UARTS_NUM)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -EINVAL;

	proc_spinlockSet(&serial->spinlock);
	if (op == POLL_READ) { /* Wait for appearance of data in rx sw queue or for timeout */
		ringbuff_dropIgnoredData(serial->rx_buff, &serial->termios);
		while (ringbuff_empty(serial->rx_buff)) {
			err = proc_condWait2(&serial->waitqRx, &vnode->mutex, &serial->spinlock, timeout);
			if (err == -ETIME)
				break;
			else if (err < 0)
				return err;
			ringbuff_dropIgnoredData(serial->rx_buff, &serial->termios);
		}
	} else if (op == POLL_WRITE) { /* Wait for free space in the tx sw queue or for timeout */
		while (ringbuff_full(serial->tx_buff)) { /*while the sending queue is full*/
			err = proc_condWait2(&serial->waitqTx, &vnode->mutex, &serial->spinlock, timeout);
			if (err == -ETIME)
				break;
			else if (err < 0)
				return err;
		}
	}
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	return err;
}


static int uart_select_poll(file_t* file, unsigned *ready)
{
    vnode_t *vnode = file->vnode;
	struct serial_st *serial;
	*ready = 0;

	if (MINOR(vnode->dev) >= UARTS_NUM)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -EINVAL;

	proc_spinlockSet(&serial->spinlock);
	ringbuff_dropIgnoredData(serial->rx_buff, &serial->termios);
	if (!ringbuff_empty(serial->rx_buff)) {
		if ((ringbuff_next(serial->rx_buff) & UART_BREAK) && (serial->termios.c_iflag & BRKINT))
			*ready |= FS_READY_HUP;
		else
			*ready |= FS_READY_READ;
	} if (!ringbuff_full(serial->tx_buff))
		*ready |= FS_READY_WRITE;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	return EOK;
}


/* If duration is zero, it transmits zero-valued bits for at least 0.25 seconds, and not more that 0.5 seconds */
static int uart_sendBreak(struct serial_st * serial, int duration)
{
	if (duration < 0)
		return EOK;

	proc_spinlockSet(&serial->spinlock);
	serial->base->C2 |= UART_C2_SBK_MASK;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	if (duration > 0) 
		duration = 1;

	proc_threadSleep(duration * 400);

	proc_spinlockSet(&serial->spinlock);
	serial->base->C2 &= ~UART_C2_SBK_MASK;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	return EOK;
}


static void uart_setBaud(UART_Type *uart_dev, unsigned baudrate)
{
	u16 sbr, brfa;
	u32 sbr64;

	/* Calculate baud settings
	* From docs: "UART baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))"
	* We need 5-bit precision for BFRA, with one extra bit for proper rounding, hence 2*32
	* DO NOT "FIX" this code, the "*4", "+1" and "/64" modifications are really meaningful for rounding *up* where appropriate.
	*/
	sbr64 = ((u32)BUS_CLK_KHZ * 1000) * 4 / baudrate; /* == 2 * 32 * bus_clock / (baud * 16) */
	sbr64 += 1;		/* round-up */
	sbr = (u16)(sbr64 / 64);
	brfa = (u16)(sbr64 / 2) & 0x1f;

	uart_dev->BDH = (uart_dev->BDH & ~UART_BDH_SBR_MASK) | UART_BDH_SBR((sbr & 0x1F00) >> 8);
	uart_dev->BDL = UART_BDL_SBR((u8)(sbr & 0x00FF));
	uart_dev->C4 &= ~UART_C4_BRFA_MASK;
	uart_dev->C4 |= UART_C4_BRFA(brfa);
}


static int uart_ioctl(file_t* file, unsigned int cmd, unsigned long arg)
{
    vnode_t *vnode = file->vnode;
	struct serial_st *serial;
	struct termios *opts = (struct termios *) arg;
	unsigned baudrate;
	int ret;

	if (MINOR(vnode->dev) >= UARTS_NUM)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -EINVAL;

	if ((cmd != TCSBRK) && (cmd != TCGETS) && (cmd != TCSETS) && (cmd != TCSETSW) && (cmd != TCSETSF) && (cmd != TCFLSH) && (cmd != TCDRAIN))
		return -EINVAL;

	if (cmd == TCFLSH) {
		if ((arg & TCIOFLUSH) != 0) {
			proc_spinlockSet(&serial->spinlock);
		}
		if ((arg & TCOFLUSH) != 0) {
			ringbuff_makeEmpty(serial->tx_buff);
			while (!(serial->base->S1 & UART_S1_TC_MASK)) {
				serial->base->C2 |= UART_C2_TCIE_MASK;
				if ((ret = proc_condWait2(&serial->waitqTc, &vnode->mutex, &serial->spinlock, 0)) != EOK)
					return ret;
			}
		}
		if ((arg & TCIFLUSH) != 0) {
			vu8 __attribute__((unused)) tmp;
			while(serial->base->RCFIFO > 0) {
				tmp = serial->base->S1;
				tmp = serial->base->D;
			}
			ringbuff_makeEmpty(serial->rx_buff);
		}
		if ((arg & TCIOFLUSH) != 0) {
			proc_spinlockClear(&serial->spinlock, sopGetCycles);
		}
		return EOK;
	}
	if (cmd == TCDRAIN) {
		proc_spinlockSet(&serial->spinlock);
		while (!ringbuff_empty(serial->tx_buff) || !(serial->base->S1 & UART_S1_TC_MASK)) {
			serial->base->C2 |= UART_C2_TCIE_MASK;
			if ((ret = proc_condWait2(&serial->waitqTc, &vnode->mutex, &serial->spinlock, 0)) != EOK)
				return ret;
		}
		proc_spinlockClear(&serial->spinlock, sopGetCycles);
		return EOK;
	}

	if (cmd == TCSBRK)
		return uart_sendBreak(serial, arg);

	if (opts == NULL)
		return -EINVAL;

	if (cmd == TCGETS)
		memcpy(opts, &serial->termios, sizeof(serial->termios));

	if ((cmd == TCSETS) || (cmd == TCSETSW) || (cmd == TCSETSF)) {
		if (opts->c_ispeed != opts->c_ospeed)
			return -EINVAL;

		switch (opts->c_ispeed) {
			case B0:
				/* This disables the baudrate generator.
				 * Should probably drop DTR to hang up, but Vybrid doesn't provide DTR */
				baudrate = 0;
				break;
			 //Will only work with a bus clock < 39.32 MHz
			case B300:
				baudrate = 300;
				break;
			case B600:
				baudrate = 600;
				break;
			case B1200:
				baudrate = 1200;
				break;
			case B1800:
				baudrate = 1800;
				break;
			case B2400:
				baudrate = 2400;
				break;
			case B4800:
				baudrate = 4800;
				break;
			case B9600:
				baudrate = 9600;
				break;
			case B19200:
				baudrate = 19200;
				break;
			case B38400:
				baudrate = 38400;
				break;
			default:
				return -EINVAL;
		}

		proc_spinlockSet(&serial->spinlock);
		if ((cmd == TCSETSW) || (cmd == TCSETSF))
			while (!ringbuff_empty(serial->tx_buff) || !(serial->base->S1 & UART_S1_TC_MASK)) {
				serial->base->C2 |= UART_C2_TCIE_MASK;
				if ((ret = proc_condWait2(&serial->waitqTc, &vnode->mutex, &serial->spinlock, 0)) != EOK)
					return ret;
			}
		
		uart_setBaud(serial->base, baudrate);

		if (((opts->c_cflag & CSIZE) == CS7) && (opts->c_cflag & PARENB)) {
			serial->base->C1 &= ~UART_C1_M_MASK;
			serial->termios.c_cflag &= ~CSIZE;
			serial->termios.c_cflag |= CS7;
		} else if (((opts->c_cflag & CSIZE) == CS8) && !(opts->c_cflag & PARENB)) {
			serial->base->C1 &= ~UART_C1_M_MASK;
			serial->termios.c_cflag &= ~CSIZE;
			serial->termios.c_cflag |= CS8;
		} else if (((opts->c_cflag & CSIZE) == CS8) && (opts->c_cflag & PARENB)) {
			serial->base->C1 |= UART_C1_M_MASK;
			serial->termios.c_cflag &= ~CSIZE;
			serial->termios.c_cflag |= CS8;
		} else
			opts->c_cflag = serial->termios.c_cflag;

		if (opts->c_cflag & PARENB) {
			serial->termios.c_cflag |= PARENB;
			serial->base->C1 |= UART_C1_PE_MASK;
			if (opts->c_cflag & PARODD) {
				serial->termios.c_cflag |= PARODD;
				serial->base->C1 |= UART_C1_PT_MASK;
			} else {
				serial->termios.c_cflag &= ~PARODD;
				serial->base->C1 &= ~UART_C1_PT_MASK;
			}
		} else {
			serial->base->C1 &= ~UART_C1_PE_MASK;
			serial->termios.c_cflag &= ~PARENB;
		}

		serial->termios.c_ispeed = serial->termios.c_ospeed = opts->c_ispeed;
		serial->termios.c_cflag &= ~CBAUD;
		serial->termios.c_cflag |= opts->c_ispeed;

		serial->termios.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | ISTRIP);
		serial->termios.c_iflag |= opts->c_iflag & (IGNBRK | BRKINT | IGNPAR | ISTRIP);

        if(opts->c_oflag & OINV)
        {
            serial->base->C3 |= UART_C3_TXINV_MASK;
            //invert tx output
        }

		if (cmd == TCSETSF) {
			vu8 __attribute__((unused)) tmp;
			while(serial->base->RCFIFO > 0) {
				tmp = serial->base->S1;
				tmp = serial->base->D;
			}
			ringbuff_makeEmpty(serial->rx_buff);
		}
		proc_spinlockClear(&serial->spinlock, sopGetCycles);
	}
	return EOK;
}


static void uart_receiveData(struct serial_st *serial)
{
	u16 data = 0;
	u8 tmp;
	
	tmp = serial->base->ED;
	if (tmp & UART_ED_NOISY_MASK)
		data |= UART_NOISE_ERR;
	if (tmp & UART_ED_PARITYE_MASK)
		data |= UART_PAR_ERR;
	data |= (serial->base->C3 & UART_C3_R8_MASK) << 1;
	data |= serial->base->D;
	ringbuff_put(serial->rx_buff, data);
}


/* NOTE: the flags in S1 can only be cleared by reading S1 and then writing or reading D.
 * That introduces a risk of clearing flags involuntarily.
 *
 * Solution: S1 is only read by the ISR, where *any and all* flags do get reacted to *in each single ISR run*.
 * Also, tx-related flags get cleared by writes to D, while rx-related flags get cleared by reads from D,
 * so both datapaths are effectively independent.
 */
#if UART_LATENCY
static void uart_kicker(void *arg)
{
	int i;

	for (i = 0; i < UARTS_NUM; i++)
		if (serials[i] != NULL) {
			UART_Type *uart_dev = serials[i]->base;

			proc_spinlockSet(&serials[i]->spinlock);
			while((uart_dev->RCFIFO > 0) && !ringbuff_full(serials[i]->rx_buff)) {
				u8 __attribute__((unused)) tmp = serials[i]->base->S1;
				uart_receiveData(serials[i]);
			}

			if (!ringbuff_empty(serials[i]->rx_buff))
				proc_threadCondSignal(&serials[i]->waitqRx);

			proc_spinlockClear(&serials[i]->spinlock, sopGetCycles);
		}
}


static int uart_isr(unsigned int n, cpu_context_t *ctx, void *arg)
{
	struct serial_st *serial;
	UART_Type *uart_dev;

	serial = (struct serial_st *)arg;
	uart_dev = serial->base;

	proc_spinlockSet(&serial->spinlock);

	if ((uart_dev->C2 & UART_C2_TCIE_MASK) && (uart_dev->S1 & UART_S1_TC_MASK)) {
		//TODO change wakeAll to WakeUp,
		//but disable UART_C2_TCIE_MASK only if no one waits on waitqTc
		proc_threadWakeAll(&serial->waitqTc);
		uart_dev->C2 &= ~UART_C2_TCIE_MASK;
	}
	if (uart_dev->S1 & UART_S1_OR_MASK) {
		vu8 __attribute__((unused)) tmp;

		serial->overrun++;
		tmp = uart_dev->D;
		uart_dev->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
	}
	else if (uart_dev->S1 & UART_S1_FE_MASK) {
		vu8 __attribute__((unused)) tmp;

		serial->framing++;
		tmp = uart_dev->D;
		uart_dev->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
	}
	else if (uart_dev->S1 & UART_S1_RDRF_MASK) {
		while (uart_dev->RCFIFO > 0) {
			if (ringbuff_full(serial->rx_buff)) {
				u8 __attribute__((unused)) data = uart_dev->D;
				serial->overflow++;
				uart_dev->C2 &= ~UART_C2_RE_MASK;
				break;
			}
			else
				uart_receiveData(serial);
		}

		if (!ringbuff_empty(serial->rx_buff))
			proc_threadCondSignal(&serial->waitqRx);
	}
	else if (uart_dev->S1 & UART_S1_TDRE_MASK) {
		while (uart_dev->TCFIFO < serial->tbuffsz_hw)
			if (ringbuff_empty(serial->tx_buff)) {
				uart_dev->C2 &= ~UART_C2_TIE_MASK;
				break;
			}
			else
				uart_dev->D = (char) ringbuff_get(serial->tx_buff);
		
		if (!ringbuff_full(serial->tx_buff))
			proc_threadCondSignal(&serial->waitqTx);	/*for uart_poll*/
	}
	/* Break Signal detection */
//TODO this is not working
// 	if (uart_dev->S2 & UART_S2_LBKDIF_MASK) {
// 		main_printf(ATTR_FAILURE, "x\n");
// 		uart_dev->S2 = UART_S2_LBKDIF_MASK;
// 		ringbuff_put(serial->rx_buff, UART_BREAK);
// 	}

	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	return IHRES_HANDLED;
}

#else

static int uart_isr(unsigned int n, cpu_context_t *ctx, void *arg)
{
	struct serial_st *serial;
	UART_Type *uart_dev;
	u8 data;

	serial = (struct serial_st *)arg;
	uart_dev = serial->base;

	proc_spinlockSet(&serial->spinlock);
	if ((uart_dev->C2 & UART_C2_TCIE_MASK) && (uart_dev->S1 & UART_S1_TC_MASK)) {
		//TODO change wakeAll to WakeUp,
		//but disable UART_C2_TCIE_MASK only if no one waits on waitqTc
		proc_threadWakeAll(&serial->waitqTc);
		uart_dev->C2 &= ~UART_C2_TCIE_MASK;
	}
	if (uart_dev->S1 & UART_S1_OR_MASK) {
		serial->overrun++;
		data = uart_dev->D;
		uart_dev->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
	}
	else if (uart_dev->S1 & UART_S1_FE_MASK) {
		serial->framing++;
		data = uart_dev->D;
		uart_dev->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
	}
	else if (uart_dev->S1 & UART_S1_RDRF_MASK) {
		if (ringbuff_full(serial->rx_buff)) {
			data = uart_dev->D;
			serial->overflow++;
			uart_dev->C2 &= ~UART_C2_RE_MASK;
		}
		else {
			uart_receiveData(serial);
			proc_threadCondSignal(&serial->waitqRx);
		}
	}
	else if (uart_dev->S1 & UART_S1_TDRE_MASK) {
		if (ringbuff_empty(serial->tx_buff))
			uart_dev->C2 &= ~UART_C2_TIE_MASK;
		else {
			uart_dev->D = ringbuff_get(serial->tx_buff);
			proc_threadCondSignal(&serial->waitqTx);
		}
	}

	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	return IHRES_HANDLED;
}

#endif


static void uart_initRegs(struct serial_st *serial, unsigned baudrate)
{
	UART_Type *uart_dev = serial->base;
	unsigned size;

	uart_dev->MODEM = 0; //Need to clear MODEM register in case BootROM sets it

	/* Disabled txor and rxor while we change settings*/
	uart_dev->C2 = 0x00;

	/*Configure the UART for 8-bit mode, no parity */
	uart_dev->C1 = 0x00;
	uart_dev->C3 = 0x00;

	uart_setBaud(uart_dev, baudrate);

#if UART_LATENCY
	uart_dev->PFIFO |= UART_PFIFO_TXFE_MASK | UART_PFIFO_RXFE_MASK;
#endif
	uart_dev->CFIFO = UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK;

	size = (uart_dev->PFIFO & UART_PFIFO_TXFIFOSIZE_MASK) >> UART_PFIFO_TXFIFOSIZE_SHIFT;
	if(size == 0)
		serial->tbuffsz_hw = 1;
	else
		serial->tbuffsz_hw = 1 << (1 + size);

	size = (uart_dev->PFIFO & UART_PFIFO_RXFIFOSIZE_MASK) >> UART_PFIFO_RXFIFOSIZE_SHIFT;
	if(size == 0)
		serial->rbuffsz_hw = 1;
	else
		serial->rbuffsz_hw = 1 << (1 + size);

#if UART_LATENCY
	uart_dev->TWFIFO = 4;
	uart_dev->RWFIFO = serial->rbuffsz_hw - 4;
#endif
	
	uart_dev->SFIFO = UART_SFIFO_RXUF_MASK | UART_SFIFO_RXOF_MASK | UART_SFIFO_TXOF_MASK;
	uart_dev->C3 |= UART_C3_ORIE_MASK | UART_C3_FEIE_MASK;
	uart_dev->C2 |= UART_C2_RE_MASK | UART_C2_TE_MASK | UART_C2_RIE_MASK; 

	/* Enable Break Signal detection */
//TODO this is not working
// 	uart_dev->S2 = UART_S2_LBKDE_MASK;
	
}


/**Initialize UART and supporting infrastructure
 *
 * @param uart_regset
 * @param irq_num
 * @param baudrate
 * @param minor
 * @param[OUT] serial_pp pointer to where to leave a pointer to the allocated serial private struct
 * @return error code
 */
static int uart_initOne(UART_Type *uart_regset, unsigned irq_num, unsigned baudrate, unsigned minor, struct serial_st **serial)
{
	int status;
	struct serial_st *uart;
	size_t size;
	
	size = sizeof(struct serial_st) + 2 * (sizeof(ringbuff_t) + SIZE_UART_BUFFER * sizeof(uart->rx_buff->data[0]));
	uart = vm_kmalloc(size);
	if (uart == NULL)
		return -ENOMEM;
	
	main_memset(uart, 0, size);

	/* place rx buffer just behind serial_st structure */
	uart->rx_buff = (ringbuff_t *)(uart + 1);
	/* verify structure alignment */
	assert(((addr_t)uart->rx_buff & 0x3) == 0);
	ringbuff_init(uart->rx_buff, SIZE_UART_BUFFER);

	/* place tx buffer just behind the rx buffer */
	uart->tx_buff = (ringbuff_t *)(&uart->rx_buff->data[SIZE_UART_BUFFER]);
	assert(((addr_t)uart->tx_buff & 0x3) == 0);
	ringbuff_init(uart->tx_buff, SIZE_UART_BUFFER);

	status = vm_iomap((addr_t)uart_regset, sizeof(UART_Type), PGHD_DEV_RW, (void **)&uart->base);
	assert(status == EOK);

	strcpy(uart->name, "UARTx");
	uart->name[4] = '0' + minor;

	proc_spinlockCreate(&(uart->spinlock), uart->name);
	proc_thqCreate(&uart->waitqTx);
	proc_thqCreate(&uart->waitqRx);
	proc_thqCreate(&uart->waitqTc);

	*serial = uart;
	hal_interruptsSetHandler(irq_num, uart_isr, uart);
	uart_initRegs(uart, baudrate);
	uart->termios.c_iflag = IGNBRK | IGNPAR;
	uart->termios.c_oflag = CR0 | NL0 | BS0 | TAB0 | VT0 | FF0;
	uart->termios.c_cflag = CS8 | CREAD | CLOCAL | B115200; //XXX should depend on baudrate!
	uart->termios.c_lflag = NOFLSH;
	uart->termios.c_ispeed = B115200;	//XXX should depend on baudrate!
	uart->termios.c_ospeed = B115200;	//XXX should depend on baudrate!

	main_printf(ATTR_INFO, "dev: %s at 0x%x irq=%d, %dB sw buffer %dB hw buffer\n",
			uart->name, (addr_t)uart_regset, irq_num, SIZE_UART_BUFFER, uart->tbuffsz_hw);

	return EOK;
}


/** Initialize serial devices
 *
 * Init all devices supported by the driver and register them with the OS
 * */
int _serial_init(u32 baudrate)
{
	static const file_ops_t serial_ops = {
		.read  = uart_read,
		.write = uart_write,
		.poll  = uart_poll,
		.ioctl = uart_ioctl,
		.open  = uart_open,
		.select_poll = uart_select_poll,
	};
#if UART_LATENCY
	static timer_t kicker_timer;
#endif
	int result = 0;
	s32 res;
	unsigned minor;
	char label[] = "serial00";

	if (dev_register(MAKEDEV(MAJOR_SERIAL, 0), &serial_ops) < 0) {
		main_printf(ATTR_ERROR, "Vybrid-UART: Can't register serial device!\n");
		return -1;
	}


	for(minor = 0; minor < UARTS_NUM; minor++) {
		if(uart_cfg[minor].base == CONSOLE_UART_PORT){
			main_printf(ATTR_INFO,"dev: UART%d skipped - in use by console\n", minor);
			assert(EOK==dev_mknod(MAKEDEV(MAJOR_SERIAL, minor), "tty"));
			continue;	/*do not touch the UART port that is being used by the console*/
		}
		if ((res = uart_initOne(uart_cfg[minor].base, uart_cfg[minor].irq, baudrate, minor, &serials[minor])) != EOK) {
			if(res != -EIO){ /*EIO is returned when the hardware is by design unavailable */
				main_printf(ATTR_ERROR, "Vybrid-UART[%d] initialization failed\n", minor);
			}
			result++;
		}
		if(result < 10)
		{
			label[6] = result + '0';
			label[7] = 0;
		}
		else
		{
			label[6] = result/10 + '0';
			label[7] = result%10 + '0';
			label[8] = 0;
		}
		assert(EOK==dev_mknod(MAKEDEV(MAJOR_SERIAL, minor), label));
	}

#if UART_LATENCY
	timesys_timerAddCyclic(&kicker_timer, UART_LATENCY * 1000, uart_kicker, NULL);
#endif

	return -result;
}
