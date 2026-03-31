/*
 * Phoenix-RTOS
 *
 * libusbclient (STM32 N6)
 *
 * OTG registers
 *
 * Copyright 2026 Phoenix Systems
 * Author: Radosław Szewczyk, Rafał Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _STM32N6_USB_REGISTERS_
#define _STM32N6_USB_REGISTERS_

/* BASE ADDRESSES */
#define PHY_ADDR_OTG     0x58040000
#define PHY_ADDR_USBPHYC 0x5803FC00
#define RCC_BASE_ADDR    0x56028000

#define EP_STRIDE      8U
#define RCC_PLL_STRIDE 4U


enum {
	usbphyc_cr = 0,
	otg_gotgctl = 0,
	otg_gotgint,
	otg_gahbcfg,
	otg_gusbcfg,
	otg_grstctl,
	otg_gintsts,
	otg_gintmsk,
	otg_grxstsp = 8,
	otg_grxfsiz,
	otg_dieptxf0,
	otg_gccfg = 14,
	otg_cid,
	otg_dieptxfx = 65,
	otg_dcfg = 512,
	otg_dctl,
	otg_dsts,
	otg_diepmsk = 516,
	otg_doepmsk,
	otg_daint,
	otg_daintmsk,
	otg_diepempmsk = 525,
	otg_diepctl0 = 576,
	otg_diepint0 = 578,
	otg_dieptsiz0 = 580,
	otg_dtxfsts0 = 582,
	otg_doepctl0 = 704,
	otg_doepint0 = 706,
	otg_doeptsiz0 = 708,
	otg_pcgcctl = 896,
};


enum {
	rcc_sr = 1,
	rcc_cfgr1 = 8,
	rcc_cfgr2,
	rcc_msicfgr = 17,
	rcc_hsicfgr,
	rcc_hsecfgr = 21,
	rcc_pll1cfgr1 = 32,
	rcc_pll1cfgr2,
	rcc_pll1cfgr3,
	rcc_ic2cfgr = 50,
	rcc_ahb5rstsr = 648,
	rcc_ahb5ensr = 664,
	rcc_ahb5rstcr = 1160,
};


/* OTG_GINST bits */
#define GINTSTS_CMOD         0
#define GINTSTS_MMIS         1
#define GINTSTS_OTGINT       2
#define GINTSTS_SOF          3
#define GINTSTS_RXFLVL       4
#define GINTSTS_NPTXFE       5
#define GINTSTS_GINAKEFF     6
#define GINTSTS_GONAKEFF     7
#define GINTSTS_ESUSP        10
#define GINTSTS_USBSUSP      11
#define GINTSTS_USBRST       12
#define GINTSTS_ENUMDNE      13
#define GINTSTS_ISOODRP      14
#define GINTSTS_EOPF         15
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

/* otg_gusbcfg bits */
#define GUSBCFG_FDMOD  30
#define GUSBCFG_FHMOD  29
#define GUSBCFG_TSDPS  22
#define GUSBCFG_PHYLPC 15
#define GUSBCFG_TRDT   10

/* otg_gotgint bits */
#define GOTGINT_SEDET   2
#define GOTGINT_ADTOCHG 18


/* otg_doepmsk bits */
#define DOEPMSK_XFRCM     0
#define DOEPMSK_STUMP     3
#define DOEPMSK_STSPHSRXM 5
#define DOEPMSK_NAKMSK    13

/* otg_diepmsk bits */
#define DIEPMSK_XFRCM 0
#define DIEPMSK_EPDM  1
#define DIEPMSK_TOM   3

/* otg_dcfg bits */
#define DCFG_NZLSOHSK 2
#define DCFG_DAD      4
#define DCFG_PFIVL    11

/* otg_dctl bits */
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
