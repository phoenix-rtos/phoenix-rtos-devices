#ifndef I2C_REGS
#define I2C_REGS

/* I2C registers, from PG090 Table 4 */
#define I2C_REG_GIE          (0x01C / 4)
#define I2C_REG_ISR          (0x020 / 4)
#define I2C_REG_IER          (0x028 / 4)
#define I2C_REG_SOFTR        (0x040 / 4)
#define I2C_REG_CR           (0x100 / 4)
#define I2C_REG_SR           (0x104 / 4)
#define I2C_REG_TX_FIFO      (0x108 / 4)
#define I2C_REG_RX_FIFO      (0x10C / 4)
#define I2C_REG_ADR          (0x110 / 4)
#define I2C_REG_TX_FIFO_OCY  (0x114 / 4)
#define I2C_REG_RX_FIFO_OCY  (0x118 / 4)
#define I2C_REG_TEN_ADR      (0x11C / 4)
#define I2C_REG_RX_FIFO_PIRQ (0x120 / 4)
#define I2C_REG_GPO          (0x124 / 4)
#define I2C_REG_TSUSTA       (0x128 / 4)
#define I2C_REG_TSUSTO       (0x12C / 4)
#define I2C_REG_THDSTA       (0x130 / 4)
#define I2C_REG_TSUDAT       (0x134 / 4)
#define I2C_REG_TBUF         (0x138 / 4)
#define I2C_REG_THIGH        (0x13C / 4)
#define I2C_REG_TLOW         (0x140 / 4)
#define I2C_REG_THDDAT       (0x144 / 4)

#define GIE_EN (1 << 31)

#define ISR_ARB_LOST           (1 << 0)
#define ISR_TX_ERR_OR_COMPLETE (1 << 1)
#define ISR_TX_FIFO_EMPTY      (1 << 2)
#define ISR_RX_FIFO_FULL       (1 << 3)
#define ISR_BUS_NOT_BUSY       (1 << 4)
#define ISR_ADDR_AS_SLAVE      (1 << 5)
#define ISR_NOT_ADDR_AS_SLAVE  (1 << 6)
#define ISR_TX_FIFO_HALF_EMPTY (1 << 7)

#define CR_EN          (1 << 0)
#define CR_TX_FIFO_RST (1 << 1)
#define CR_MSMS        (1 << 2)
#define CR_TX          (1 << 3)
#define CR_TXAK        (1 << 4)
#define CR_RSTA        (1 << 5)

#define SR_ABGC          (1 << 0)
#define SR_AAS           (1 << 1)
#define SR_BB            (1 << 2)
#define SR_SRW           (1 << 3)
#define SR_TX_FIFO_FULL  (1 << 4)
#define SR_RX_FIFO_FULL  (1 << 5)
#define SR_RX_FIFO_EMPTY (1 << 6)
#define SR_TX_FIFO_EMPTY (1 << 7)


#endif  // !DEBUG
