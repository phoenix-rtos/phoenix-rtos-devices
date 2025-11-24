#ifndef _STM32N6_USB_REGISTERS_
#define _STM32N6_USB_REGISTERS_

/* BASE ADRESSES */
#define PHY_ADDR_OTG     0x58040000
#define PHY_ADDR_USBPHYC 0x5803FC00
#define RCC_BASE_ADDR    0x56028000

/* OTG OFFSETS */
#define USBPHYC_CR (0x000 / 4)
#define GOTGCTL    (0x000 / 4)
#define GOTGINT    (0x004 / 4)
#define GAHBCFG    (0x008 / 4)
#define GUSBCFG    (0x00C / 4)
#define GRSTCTL    (0x010 / 4)
#define GINTSTS    (0x014 / 4)
#define GINTMSK    (0x018 / 4)
#define GCCFG      (0x038 / 4)
#define CID        (0x03C / 4)
#define DCFG       (0x800 / 4)
#define DCTL       (0x804 / 4)
#define DSTS       (0x808 / 4)

#define GRXSTSP    (0x020 / 4)
#define GRXFSIZ    (0x024 / 4)
#define DIEPTXF0   (0x028 / 4)
#define DIEPEMPMSK (0x834 / 4)

#define DAINT    (0x818 / 4)
#define DAINTMSK (0x81C / 4)

#define DIEPCTL0  (0x900 / 4)
#define DIEPMSK   (0x810 / 4)
#define DIEPINT0  (0x908 / 4)
#define DIEPTSIZ0 (0x910 / 4)
#define DTXFSTS0  (0x918 / 4)

#define DOEPCTL0  (0xB00 / 4)
#define DOEPMSK   (0x814 / 4)
#define DOEPINT0  (0xB08 / 4)
#define DOEPTSIZ0 (0xB10 / 4)

#define EP_STRIDE (0x20 / 4)

/* RCC OFFSETS */
#define RCC_SR        (0x04 / 4)
#define RCC_CFGR1     (0x20 / 4)
#define RCC_CFGR2     (0x24 / 4)
#define RCC_MSICFGR   (0x44 / 4)
#define RCC_HSICFGR   (0x48 / 4)
#define RCC_PLL1CFGR1 (0x80 / 4)
#define RCC_PLL1CFGR2 (0x84 / 4)
#define RCC_PLL1CFGR3 (0x88 / 4)
#define RCC_IC2CFGR   (0xC8 / 4)
#define RCC_AHB5RSTSR (0xA20 / 4)
#define RCC_AHB5RSTCR (0x1220 / 4)

#define RCC_PLL_STRIDE (0x10 / 4)

#define RCC_AHB5ENSR (0xA60 / 4)


#endif
