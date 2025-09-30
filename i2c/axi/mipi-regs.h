#ifndef MIPI_REGS
#define MIPI_REGS

#define MIPI_REG_CORE_CONFIG     (0x00 / 4)
#define MIPI_REG_PROTOCOL_CONFIG (0x04 / 4)
#define MIPI_REG_CORE_STATUS     (0x10 / 4)
#define MIPI_REG_GIE             (0x20 / 4)
#define MIPI_REG_IRQ_STATUS      (0x24 / 4)
#define MIPI_REG_IRQ_ENABLE      (0x28 / 4)
#define MIPI_REG_DYN_VC_SELECT   (0x2C / 4)
#define MIPI_REG_SHORT_PACKET    (0x30 / 4)
#define MIPI_REG_VCX_FRAME_ERROR (0x34 / 4)
#define MIPI_REG_CLOCK_LANE_INFO (0x3C / 4)
#define MIPI_REG_IMG_INFO        (0x60 / 4)

#define MIPI_CORE_ENABLE     (1 << 0)
#define MIPI_CORE_SOFT_RESET (1 << 1)
#define MIPI_CORE_FULL_RESET (1 << 2)

#define MIPI_2ACTIVE_LANES (1 << 0)
#define MIPI_MAX_LANES     (1 << 3)

#define MIPI_STATUS_RESET (1 << 0)

#define MIPI_GIE (1 << 0)

#define MIPI_IRQ_FRAME_RX      (1 << 31)
#define MIPI_IRQ_VCX_FRAME_ERR (1 << 30)

#endif  // !MIPI_REGS
