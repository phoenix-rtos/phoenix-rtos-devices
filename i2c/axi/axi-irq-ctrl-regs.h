#ifndef AXI_IRQ_CTRL_REGS
#define AXI_IRQ_CTRL_REGS


#define AXI_REG_ISR (0x00 / 4)  // Interrupt Status Register
#define AXI_REG_IER (0x08 / 4)  // Interrupt Enable Register
#define AXI_REG_MER (0x1C / 4)  // Master Enable Register
#define AXI_REG_ILR (0x05 / 4)  // Interrupt Level Register

#define AXI_IRQ_MIPI    (1 << 0)
#define AXI_IRQ_IIC     (1 << 1)
#define AXI_IRQ_FRM_BUF (1 << 2)
#define AXI_IRQ_ISP     (1 << 3)

#define AXI_MER_ME  (1 << 0)  // Master Enable
#define AXI_MER_HIE (1 << 1)  // Hardware Interrupt Enable

#endif /* ifndef AXI_IRQ_CTRL_REGS */
