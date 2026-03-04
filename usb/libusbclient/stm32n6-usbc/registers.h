#ifndef _STM32N6_USB_REGISTERS_
#define _STM32N6_USB_REGISTERS_

/* BASE ADDRESSES */
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
#define DIEPTXFx   (0x104 / 4)
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

#define PCGCCTL (0xE00 / 4)

#define EP_STRIDE (0x20 / 4)

/* RCC OFFSETS */
#define RCC_SR        (0x04 / 4)
#define RCC_CFGR1     (0x20 / 4)
#define RCC_CFGR2     (0x24 / 4)
#define RCC_MSICFGR   (0x44 / 4)
#define RCC_HSICFGR   (0x48 / 4)
#define RCC_HSECFGR   (0x54 / 4)
#define RCC_PLL1CFGR1 (0x80 / 4)
#define RCC_PLL1CFGR2 (0x84 / 4)
#define RCC_PLL1CFGR3 (0x88 / 4)
#define RCC_IC2CFGR   (0xC8 / 4)
#define RCC_AHB5RSTSR (0xA20 / 4)
#define RCC_AHB5RSTCR (0x1220 / 4)

#define RCC_PLL_STRIDE (0x10 / 4)

#define RCC_AHB5ENSR (0xA60 / 4)

/* OTG_GINST bits */
#define GINTSTS_CMOD     0
#define GINTSTS_MMIS     1
#define GINTSTS_OTGINT   2
#define GINTSTS_SOF      3
#define GINTSTS_RXFLVL   4
#define GINTSTS_NPTXFE   5
#define GINTSTS_GINAKEFF 6
#define GINTSTS_GONAKEFF 7
/* Bits 9:8 Reserved */
#define GINTSTS_ESUSP   10
#define GINTSTS_USBSUSP 11
#define GINTSTS_USBRST  12
#define GINTSTS_ENUMDNE 13
#define GINTSTS_ISOODRP 14
#define GINTSTS_EOPF    15
/* Bits 17:16 Reserved */
#define GINTSTS_IEPINT       18
#define GINTSTS_OEPINT       19
#define GINTSTS_IISOIXFR     20
#define GINTSTS_INCOMPISOOUT 21
#define GINTSTS_DATAFSUSP    22
#define GINTSTS_RSTDET       23
#define GINTSTS_HPRTINT      24
#define GINTSTS_HCINT        25
#define GINTSTS_PTXFE        26
#define GINTSTS_LPMINT       27
#define GINTSTS_CIDSCHG      28
#define GINTSTS_DISCINT      29
#define GINTSTS_SRQINT       30
#define GINTSTS_WKUPINT      31

/* GUSBCFG bits */
#define GUSBCFG_FDMOD  30
#define GUSBCFG_FHMOD  29
#define GUSBCFG_TSDPS  22
#define GUSBCFG_PHYLPC 15
#define GUSBCFG_TRDT   10

/* GOTGINT bits */
#define GOTGINT_SEDET   2
#define GOTGINT_ADTOCHG 18


/* DOEPMSK bits */
#define DOEPMSK_XFRCM     0
#define DOEPMSK_STUMP     3
#define DOEPMSK_STSPHSRXM 5
#define DOEPMSK_NAKMSK    13

/* DIEPMSK bits */
#define DIEPMSK_XFRCM 0
#define DIEPMSK_EPDM  1
#define DIEPMSK_TOM   3

/* DCFG bits */
#define DCFG_NZLSOHSK 2
#define DCFG_DAD      4
#define DCFG_PFIVL    11

/* DCTL bits */
#define DCTL_CGINAK 8
#define DCTL_CGONAK 10

/* DIEPCTL bits */
#define DIEPCTL_EPENA  31
#define DIEPCTL_EPDIS  30
#define DIEPCTL_SD1PID 29
#define DIEPCTL_SD0PID 28
#define DIEPCTL_SNAK   27
#define DIEPCTL_CNAK   26
#define DIEPCTL_TXFNUM 22
#define DIEPCTL_STALL  21
#define DIEPCTL_EPTYP  18
#define DIEPCTL_NAKSTS 17
#define DIEPCTL_DPID   16
#define DIEPCTL_USBAEP 15

/* DOEPCTL bits */
#define DOEPCTL_EPENA  31
#define DOEPCTL_EPDIS  30
#define DOEPCTL_SD1PID 29
#define DOEPCTL_SD0PID 28
#define DOEPCTL_SNAK   27
#define DOEPCTL_CNAK   26
#define DOEPCTL_STALL  21
#define DOEPCTL_SNPM   20
#define DOEPCTL_EPTYP  18
#define DOEPCTL_NAKSTS 17
#define DOEPCTL_DPID   16
#define DOEPCTL_USBAEP 15

/* DOEPINT bits */
#define DOEPINT_XFRC      0
#define DOEPINT_EPDISD    1
#define DOEPINT_AHBERR    2
#define DOEPINT_STUP      3
#define DOEPINT_OTEPDIS   4
#define DOEPINT_STSPHSRX  5
#define DOEPINT_B2BSTUP   6
#define DOEPINT_OUTPKTERR 8
#define DOEPINT_BERR      12
#define DOEPINT_NAK       13
#define DOEPINT_NYET      14
#define DOEPINT_STPKTRX   15

/* DIEPINT bits */
#define DIEPINT_XFRC       0
#define DIEPINT_EPDISD     1
#define DIEPINT_AHBERR     2
#define DIEPINT_TOC        3
#define DIEPINT_ITTXFE     4
#define DIEPINT_INEPNM     5
#define DIEPINT_INEPNE     6
#define DIEPINT_TXFE       7
#define DIEPINT_TXFIFOUDRN 8
#define DIEPINT_RESERVED9  9
#define DIEPINT_RESERVED10 10
#define DIEPINT_PKTDRPSTS  11
#define DIEPINT_RESERVED12 12
#define DIEPINT_NAK        13

/* DOEPTSIZ bits */
#define DOEPTSIZ_PKTCNT 19

/* DIEPTSIZ bits */
#define DIEPTSIZ_PKTCNT 19

#endif
