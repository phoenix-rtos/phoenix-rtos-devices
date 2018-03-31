#ifndef _USB_EHCI_DEF_H
#define _USB_EHCI_DEF_H

/* EHCI registers definitions */
typedef struct {
    volatile  uint8_t CAPLENGTH;                          /**< Capability Register Length, offset: 0x100 */
       uint8_t RESERVED_2[1];
    volatile  uint16_t HCIVERSION;                        /**< Host Controller Interface Version, offset: 0x102 */
    volatile  uint32_t HCSPARAMS;                         /**< Host Controller Structural Parameters, offset: 0x104 */
    volatile  uint32_t HCCPARAMS;                         /**< Host Controller Capability Parameters, offset: 0x108 */
} usb_ehciCapReg_t;

typedef struct {
    volatile uint32_t USBCMD;                            /**< USB Command Register, offset: 0x0 */
    volatile uint32_t USBSTS;                            /**< USB Status Register, offset: 0x4 */
    volatile uint32_t USBINTR;                           /**< Interrupt Enable Register, offset: 0x8 */
    volatile uint32_t FRINDEX;                           /**< USB Frame Index, offset: 0xC */
    volatile uint32_t CTRLDSSEGMENT;
    volatile uint32_t PERIODICLISTBASE;                  /**< Frame List Base Address, offset: 0x14 */
    volatile uint32_t ASYNCLISTADDR;                     /**< Next Asynch. Address, offset: 0x18 */
    uint32_t RESERVED[9];
    volatile uint32_t CONFIGFLAG;
    volatile uint32_t PORTSC[1];
} usb_ehciOpReg_t;

/* ----------------------------------------------------------------------------
   -- USB Register Masks
   ---------------------------------------------------------------------------- */

/**
 * @addtogroup USB_Register_Masks USB Register Masks
 * @{
 */

/* CAPLENGTH Bit Fields */
#define USB_CAPLENGTH_CAPLENGTH_MASK             0xFFu
#define USB_CAPLENGTH_CAPLENGTH_SHIFT            0
#define USB_CAPLENGTH_CAPLENGTH(x)               (((uint8_t)(((uint8_t)(x))<<USB_CAPLENGTH_CAPLENGTH_SHIFT))&USB_CAPLENGTH_CAPLENGTH_MASK)
/* HCIVERSION Bit Fields */
#define USB_HCIVERSION_HCIVERSION_MASK           0xFFFFu
#define USB_HCIVERSION_HCIVERSION_SHIFT          0
#define USB_HCIVERSION_HCIVERSION(x)             (((uint16_t)(((uint16_t)(x))<<USB_HCIVERSION_HCIVERSION_SHIFT))&USB_HCIVERSION_HCIVERSION_MASK)
/* HCSPARAMS Bit Fields */
#define USB_HCSPARAMS_N_PORTS_MASK               0xFu
#define USB_HCSPARAMS_N_PORTS_SHIFT              0
#define USB_HCSPARAMS_N_PORTS(x)                 (((uint32_t)(((uint32_t)(x))<<USB_HCSPARAMS_N_PORTS_SHIFT))&USB_HCSPARAMS_N_PORTS_MASK)
#define USB_HCSPARAMS_PPC_MASK                   0x10u
#define USB_HCSPARAMS_PPC_SHIFT                  4
#define USB_HCSPARAMS_N_PCC_MASK                 0xF00u
#define USB_HCSPARAMS_N_PCC_SHIFT                8
#define USB_HCSPARAMS_N_PCC(x)                   (((uint32_t)(((uint32_t)(x))<<USB_HCSPARAMS_N_PCC_SHIFT))&USB_HCSPARAMS_N_PCC_MASK)
#define USB_HCSPARAMS_N_CC_MASK                  0xF000u
#define USB_HCSPARAMS_N_CC_SHIFT                 12
#define USB_HCSPARAMS_N_CC(x)                    (((uint32_t)(((uint32_t)(x))<<USB_HCSPARAMS_N_CC_SHIFT))&USB_HCSPARAMS_N_CC_MASK)
#define USB_HCSPARAMS_PI_MASK                    0x10000u
#define USB_HCSPARAMS_PI_SHIFT                   16
#define USB_HCSPARAMS_N_PTT_MASK                 0xF00000u
#define USB_HCSPARAMS_N_PTT_SHIFT                20
#define USB_HCSPARAMS_N_PTT(x)                   (((uint32_t)(((uint32_t)(x))<<USB_HCSPARAMS_N_PTT_SHIFT))&USB_HCSPARAMS_N_PTT_MASK)
#define USB_HCSPARAMS_N_TT_MASK                  0xF000000u
#define USB_HCSPARAMS_N_TT_SHIFT                 24
#define USB_HCSPARAMS_N_TT(x)                    (((uint32_t)(((uint32_t)(x))<<USB_HCSPARAMS_N_TT_SHIFT))&USB_HCSPARAMS_N_TT_MASK)
/* HCCPARAMS Bit Fields */
#define USB_HCCPARAMS_ADC_MASK                   0x1u
#define USB_HCCPARAMS_ADC_SHIFT                  0
#define USB_HCCPARAMS_PFL_MASK                   0x2u
#define USB_HCCPARAMS_PFL_SHIFT                  1
#define USB_HCCPARAMS_ASP_MASK                   0x4u
#define USB_HCCPARAMS_ASP_SHIFT                  2
#define USB_HCCPARAMS_IST_MASK                   0xF0u
#define USB_HCCPARAMS_IST_SHIFT                  4
#define USB_HCCPARAMS_IST(x)                     (((uint32_t)(((uint32_t)(x))<<USB_HCCPARAMS_IST_SHIFT))&USB_HCCPARAMS_IST_MASK)
#define USB_HCCPARAMS_EECP_MASK                  0xFF00u
#define USB_HCCPARAMS_EECP_SHIFT                 8
#define USB_HCCPARAMS_EECP(x)                    (((uint32_t)(((uint32_t)(x))<<USB_HCCPARAMS_EECP_SHIFT))&USB_HCCPARAMS_EECP_MASK)

/* USBCMD Bit Fields */
#define USB_USBCMD_RS_MASK                       0x1u
#define USB_USBCMD_RS_SHIFT                      0
#define USB_USBCMD_RST_MASK                      0x2u
#define USB_USBCMD_RST_SHIFT                     1
#define USB_USBCMD_FS1_MASK                      0xCu
#define USB_USBCMD_FS1_SHIFT                     2
#define USB_USBCMD_FS1(x)                        (((uint32_t)(((uint32_t)(x))<<USB_USBCMD_FS1_SHIFT))&USB_USBCMD_FS1_MASK)
#define USB_USBCMD_PSE_MASK                      0x10u
#define USB_USBCMD_PSE_SHIFT                     4
#define USB_USBCMD_ASE_MASK                      0x20u
#define USB_USBCMD_ASE_SHIFT                     5
#define USB_USBCMD_IAA_MASK                      0x40u
#define USB_USBCMD_IAA_SHIFT                     6
#define USB_USBCMD_ASP_MASK                      0x300u
#define USB_USBCMD_ASP_SHIFT                     8
#define USB_USBCMD_ASP(x)                        (((uint32_t)(((uint32_t)(x))<<USB_USBCMD_ASP_SHIFT))&USB_USBCMD_ASP_MASK)
#define USB_USBCMD_ASPE_MASK                     0x800u
#define USB_USBCMD_ASPE_SHIFT                    11
#define USB_USBCMD_SUTW_MASK                     0x2000u
#define USB_USBCMD_SUTW_SHIFT                    13
#define USB_USBCMD_ATDTW_MASK                    0x4000u
#define USB_USBCMD_ATDTW_SHIFT                   14
#define USB_USBCMD_FS2_MASK                      0x8000u
#define USB_USBCMD_FS2_SHIFT                     15
#define USB_USBCMD_ITC_MASK                      0xFF0000u
#define USB_USBCMD_ITC_SHIFT                     16
#define USB_USBCMD_ITC(x)                        (((uint32_t)(((uint32_t)(x))<<USB_USBCMD_ITC_SHIFT))&USB_USBCMD_ITC_MASK)
/* USBSTS Bit Fields */
#define USB_USBSTS_UI_MASK                       0x1u
#define USB_USBSTS_UI_SHIFT                      0
#define USB_USBSTS_UEI_MASK                      0x2u
#define USB_USBSTS_UEI_SHIFT                     1
#define USB_USBSTS_PCI_MASK                      0x4u
#define USB_USBSTS_PCI_SHIFT                     2
#define USB_USBSTS_FRI_MASK                      0x8u
#define USB_USBSTS_FRI_SHIFT                     3
#define USB_USBSTS_SEI_MASK                      0x10u
#define USB_USBSTS_SEI_SHIFT                     4
#define USB_USBSTS_AAI_MASK                      0x20u
#define USB_USBSTS_AAI_SHIFT                     5
#define USB_USBSTS_URI_MASK                      0x40u
#define USB_USBSTS_URI_SHIFT                     6
#define USB_USBSTS_SRI_MASK                      0x80u
#define USB_USBSTS_SRI_SHIFT                     7
#define USB_USBSTS_SLI_MASK                      0x100u
#define USB_USBSTS_SLI_SHIFT                     8
#define USB_USBSTS_ULPII_MASK                    0x400u
#define USB_USBSTS_ULPII_SHIFT                   10
#define USB_USBSTS_HCH_MASK                      0x1000u
#define USB_USBSTS_HCH_SHIFT                     12
#define USB_USBSTS_RCL_MASK                      0x2000u
#define USB_USBSTS_RCL_SHIFT                     13
#define USB_USBSTS_PS_MASK                       0x4000u
#define USB_USBSTS_PS_SHIFT                      14
#define USB_USBSTS_AS_MASK                       0x8000u
#define USB_USBSTS_AS_SHIFT                      15
#define USB_USBSTS_NAKI_MASK                     0x10000u
#define USB_USBSTS_NAKI_SHIFT                    16
#define USB_USBSTS_TI0_MASK                      0x1000000u
#define USB_USBSTS_TI0_SHIFT                     24
#define USB_USBSTS_TI1_MASK                      0x2000000u
#define USB_USBSTS_TI1_SHIFT                     25
/* USBINTR Bit Fields */
#define USB_USBINTR_UE_MASK                      0x1u
#define USB_USBINTR_UE_SHIFT                     0
#define USB_USBINTR_UEE_MASK                     0x2u
#define USB_USBINTR_UEE_SHIFT                    1
#define USB_USBINTR_PCE_MASK                     0x4u
#define USB_USBINTR_PCE_SHIFT                    2
#define USB_USBINTR_FRE_MASK                     0x8u
#define USB_USBINTR_FRE_SHIFT                    3
#define USB_USBINTR_SEE_MASK                     0x10u
#define USB_USBINTR_SEE_SHIFT                    4
#define USB_USBINTR_AAE_MASK                     0x20u
#define USB_USBINTR_AAE_SHIFT                    5
#define USB_USBINTR_URE_MASK                     0x40u
#define USB_USBINTR_URE_SHIFT                    6
#define USB_USBINTR_SRE_MASK                     0x80u
#define USB_USBINTR_SRE_SHIFT                    7
#define USB_USBINTR_SLE_MASK                     0x100u
#define USB_USBINTR_SLE_SHIFT                    8
#define USB_USBINTR_ULPIE_MASK                   0x400u
#define USB_USBINTR_ULPIE_SHIFT                  10
#define USB_USBINTR_NAKE_MASK                    0x10000u
#define USB_USBINTR_NAKE_SHIFT                   16
#define USB_USBINTR_UAIE_MASK                    0x40000u
#define USB_USBINTR_UAIE_SHIFT                   18
#define USB_USBINTR_UPIE_MASK                    0x80000u
#define USB_USBINTR_UPIE_SHIFT                   19
#define USB_USBINTR_TIE0_MASK                    0x1000000u
#define USB_USBINTR_TIE0_SHIFT                   24
#define USB_USBINTR_TIE1_MASK                    0x2000000u
#define USB_USBINTR_TIE1_SHIFT                   25
/* FRINDEX Bit Fields */
#define USB_FRINDEX_FRINDEX_MASK                 0x3FFFu
#define USB_FRINDEX_FRINDEX_SHIFT                0
#define USB_FRINDEX_FRINDEX(x)                   (((uint32_t)(((uint32_t)(x))<<USB_FRINDEX_FRINDEX_SHIFT))&USB_FRINDEX_FRINDEX_MASK)
/* DEVICEADDR Bit Fields */
#define USB_DEVICEADDR_USBADRA_MASK              0x1000000u
#define USB_DEVICEADDR_USBADRA_SHIFT             24
#define USB_DEVICEADDR_USBADR_MASK               0xFE000000u
#define USB_DEVICEADDR_USBADR_SHIFT              25
#define USB_DEVICEADDR_USBADR(x)                 (((uint32_t)(((uint32_t)(x))<<USB_DEVICEADDR_USBADR_SHIFT))&USB_DEVICEADDR_USBADR_MASK)
/* PERIODICLISTBASE Bit Fields */
#define USB_PERIODICLISTBASE_BASEADR_MASK        0xFFFFF000u
#define USB_PERIODICLISTBASE_BASEADR_SHIFT       12
#define USB_PERIODICLISTBASE_BASEADR(x)          (((uint32_t)(((uint32_t)(x))<<USB_PERIODICLISTBASE_BASEADR_SHIFT))&USB_PERIODICLISTBASE_BASEADR_MASK)
/* ASYNCLISTADDR Bit Fields */
#define USB_ASYNCLISTADDR_ASYBASE_MASK           0xFFFFFFE0u
#define USB_ASYNCLISTADDR_ASYBASE_SHIFT          5
#define USB_ASYNCLISTADDR_ASYBASE(x)             (((uint32_t)(((uint32_t)(x))<<USB_ASYNCLISTADDR_ASYBASE_SHIFT))&USB_ASYNCLISTADDR_ASYBASE_MASK)
/* ENDPTLISTADDR Bit Fields */
#define USB_ENDPTLISTADDR_EPBASE_MASK            0xFFFFF800u
#define USB_ENDPTLISTADDR_EPBASE_SHIFT           11
#define USB_ENDPTLISTADDR_EPBASE(x)              (((uint32_t)(((uint32_t)(x))<<USB_ENDPTLISTADDR_EPBASE_SHIFT))&USB_ENDPTLISTADDR_EPBASE_MASK)
/* BURSTSIZE Bit Fields */
#define USB_BURSTSIZE_RXPBURST_MASK              0xFFu
#define USB_BURSTSIZE_RXPBURST_SHIFT             0
#define USB_BURSTSIZE_RXPBURST(x)                (((uint32_t)(((uint32_t)(x))<<USB_BURSTSIZE_RXPBURST_SHIFT))&USB_BURSTSIZE_RXPBURST_MASK)
#define USB_BURSTSIZE_TXPBURST_MASK              0x1FF00u
#define USB_BURSTSIZE_TXPBURST_SHIFT             8
#define USB_BURSTSIZE_TXPBURST(x)                (((uint32_t)(((uint32_t)(x))<<USB_BURSTSIZE_TXPBURST_SHIFT))&USB_BURSTSIZE_TXPBURST_MASK)
/* TXFILLTUNING Bit Fields */
#define USB_TXFILLTUNING_TXSCHOH_MASK            0xFFu
#define USB_TXFILLTUNING_TXSCHOH_SHIFT           0
#define USB_TXFILLTUNING_TXSCHOH(x)              (((uint32_t)(((uint32_t)(x))<<USB_TXFILLTUNING_TXSCHOH_SHIFT))&USB_TXFILLTUNING_TXSCHOH_MASK)
#define USB_TXFILLTUNING_TXSCHHEALTH_MASK        0x1F00u
#define USB_TXFILLTUNING_TXSCHHEALTH_SHIFT       8
#define USB_TXFILLTUNING_TXSCHHEALTH(x)          (((uint32_t)(((uint32_t)(x))<<USB_TXFILLTUNING_TXSCHHEALTH_SHIFT))&USB_TXFILLTUNING_TXSCHHEALTH_MASK)
#define USB_TXFILLTUNING_TXFIFOTHRES_MASK        0x3F0000u
#define USB_TXFILLTUNING_TXFIFOTHRES_SHIFT       16
#define USB_TXFILLTUNING_TXFIFOTHRES(x)          (((uint32_t)(((uint32_t)(x))<<USB_TXFILLTUNING_TXFIFOTHRES_SHIFT))&USB_TXFILLTUNING_TXFIFOTHRES_MASK)
/* IC_USB Bit Fields */
#define USB_IC_USB_IC_VDD1_MASK                  0x7u
#define USB_IC_USB_IC_VDD1_SHIFT                 0
#define USB_IC_USB_IC_VDD1(x)                    (((uint32_t)(((uint32_t)(x))<<USB_IC_USB_IC_VDD1_SHIFT))&USB_IC_USB_IC_VDD1_MASK)
#define USB_IC_USB_IC1_MASK                      0x8u
#define USB_IC_USB_IC1_SHIFT                     3
/* ENDPTNAK Bit Fields */
#define USB_ENDPTNAK_EPRN_MASK                   0x3Fu
#define USB_ENDPTNAK_EPRN_SHIFT                  0
#define USB_ENDPTNAK_EPRN(x)                     (((uint32_t)(((uint32_t)(x))<<USB_ENDPTNAK_EPRN_SHIFT))&USB_ENDPTNAK_EPRN_MASK)
#define USB_ENDPTNAK_EPTN_MASK                   0x3F0000u
#define USB_ENDPTNAK_EPTN_SHIFT                  16
#define USB_ENDPTNAK_EPTN(x)                     (((uint32_t)(((uint32_t)(x))<<USB_ENDPTNAK_EPTN_SHIFT))&USB_ENDPTNAK_EPTN_MASK)
/* ENDPTNAKEN Bit Fields */
#define USB_ENDPTNAKEN_EPRNE_MASK                0x3Fu
#define USB_ENDPTNAKEN_EPRNE_SHIFT               0
#define USB_ENDPTNAKEN_EPRNE(x)                  (((uint32_t)(((uint32_t)(x))<<USB_ENDPTNAKEN_EPRNE_SHIFT))&USB_ENDPTNAKEN_EPRNE_MASK)
#define USB_ENDPTNAKEN_EPTNE_MASK                0x3F0000u
#define USB_ENDPTNAKEN_EPTNE_SHIFT               16
#define USB_ENDPTNAKEN_EPTNE(x)                  (((uint32_t)(((uint32_t)(x))<<USB_ENDPTNAKEN_EPTNE_SHIFT))&USB_ENDPTNAKEN_EPTNE_MASK)
/* PORTSC Bit Fields */
#define USB_PORTSC_CCS_MASK                     0x1u
#define USB_PORTSC_CCS_SHIFT                    0
#define USB_PORTSC_CSC_MASK                     0x2u
#define USB_PORTSC_CSC_SHIFT                    1
#define USB_PORTSC_PE_MASK                      0x4u
#define USB_PORTSC_PE_SHIFT                     2
#define USB_PORTSC_PEC_MASK                     0x8u
#define USB_PORTSC_PEC_SHIFT                    3
#define USB_PORTSC_OCA_MASK                     0x10u
#define USB_PORTSC_OCA_SHIFT                    4
#define USB_PORTSC_OCC_MASK                     0x20u
#define USB_PORTSC_OCC_SHIFT                    5
#define USB_PORTSC_FPR_MASK                     0x40u
#define USB_PORTSC_FPR_SHIFT                    6
#define USB_PORTSC_SUSP_MASK                    0x80u
#define USB_PORTSC_SUSP_SHIFT                   7
#define USB_PORTSC_PR_MASK                      0x100u
#define USB_PORTSC_PR_SHIFT                     8
#define USB_PORTSC_HSP_MASK                     0x200u
#define USB_PORTSC_HSP_SHIFT                    9
#define USB_PORTSC_LS_MASK                      0xC00u
#define USB_PORTSC_LS_SHIFT                     10
#define USB_PORTSC_LS(x)                        (((uint32_t)(((uint32_t)(x))<<USB_PORTSC_LS_SHIFT))&USB_PORTSC_LS_MASK)
#define USB_PORTSC_PP_MASK                      0x1000u
#define USB_PORTSC_PP_SHIFT                     12
#define USB_PORTSC_PO_MASK                      0x2000u
#define USB_PORTSC_PO_SHIFT                     13
#define USB_PORTSC_PIC_MASK                     0xC000u
#define USB_PORTSC_PIC_SHIFT                    14
#define USB_PORTSC_PIC(x)                       (((uint32_t)(((uint32_t)(x))<<USB_PORTSC_PIC_SHIFT))&USB_PORTSC_PIC_MASK)
#define USB_PORTSC_PTC_MASK                     0xF0000u
#define USB_PORTSC_PTC_SHIFT                    16
#define USB_PORTSC_PTC(x)                       (((uint32_t)(((uint32_t)(x))<<USB_PORTSC_PTC_SHIFT))&USB_PORTSC_PTC_MASK)
#define USB_PORTSC_WKCN_MASK                    0x100000u
#define USB_PORTSC_WKCN_SHIFT                   20
#define USB_PORTSC_WKDC_MASK                    0x200000u
#define USB_PORTSC_WKDC_SHIFT                   21
#define USB_PORTSC_WKOC_MASK                    0x400000u
#define USB_PORTSC_WKOC_SHIFT                   22
#define USB_PORTSC_PHCD_MASK                    0x800000u
#define USB_PORTSC_PHCD_SHIFT                   23
#define USB_PORTSC_PFSC_MASK                    0x1000000u
#define USB_PORTSC_PFSC_SHIFT                   24
#define USB_PORTSC_PTS1_MASK                    0x2000000u
#define USB_PORTSC_PTS1_SHIFT                   25
#define USB_PORTSC_PSPD_MASK                    0xC000000u
#define USB_PORTSC_PSPD_SHIFT                   26
#define USB_PORTSC_PSPD(x)                      (((uint32_t)(((uint32_t)(x))<<USB_PORTSC_PSPD_SHIFT))&USB_PORTSC_PSPD_MASK)
#define USB_PORTSC_PTW_MASK                     0x10000000u
#define USB_PORTSC_PTW_SHIFT                    28
#define USB_PORTSC_STS_MASK                     0x20000000u
#define USB_PORTSC_STS_SHIFT                    29
#define USB_PORTSC_PTS2_MASK                    0xC0000000u
#define USB_PORTSC_PTS2_SHIFT                   30
#define USB_PORTSC_PTS2(x)                      (((uint32_t)(((uint32_t)(x))<<USB_PORTSC_PTS2_SHIFT))&USB_PORTSC_PTS2_MASK)

/* USBMODE Bit Fields */
#define USB_USBMODE_CM_MASK                      0x3u
#define USB_USBMODE_CM_SHIFT                     0
#define USB_USBMODE_CM(x)                        (((uint32_t)(((uint32_t)(x))<<USB_USBMODE_CM_SHIFT))&USB_USBMODE_CM_MASK)
#define USB_USBMODE_ES_MASK                      0x4u
#define USB_USBMODE_ES_SHIFT                     2
#define USB_USBMODE_SLOM_MASK                    0x8u
#define USB_USBMODE_SLOM_SHIFT                   3
#define USB_USBMODE_SDIS_MASK                    0x10u
#define USB_USBMODE_SDIS_SHIFT                   4


/* EHCI transfer data types */
#define EHCI_QSTRUCT_PAGES 1
#define EHCI_QSTRUCT_SIZE SIZE_PAGE*EHCI_QSTRUCT_PAGES

typedef struct usb_ehciQtd_ {
   volatile uint32_t nextQtd;     /* (5-31) Memory address of 
                                          ** next qTD to be processed
                                          ** (4..1) reserved 
                                          ** T (bit 0) indicating pointer 
                                          ** validity
                                          */
   volatile uint32_t altQtd; /* bits 31-5: alternate next 
                                          ** qTD if the above one encounters 
                                          ** a short packet
                                          ** (4..1) reserved 
                                          ** T (bit 0) indicating pointer 
                                          ** validity
                                          */
   volatile uint32_t token;            /* bits 31: data toggle 
                                          ** bits 30-16: Total bytes to transfer
                                          ** bit 15: Interrupt on Complete
                                          ** bits 14-12: Current page
                                          ** bits 11-10: Error Counter
                                          ** bits 9-8: PID code
                                          ** bits 7-0: status
                                          */
   volatile uint32_t buff[5];         /* bit 31-12: 4K-page aligned 
                                          ** physical memory address
                                          ** bit 11-0: Current Offset in b[0], reserved otherwise
                                          */
   uint32_t dmaAddr;
   LIST_ENTRY(usb_ehciQtd_) qtdList;
   usb_urb_t* urb;
} usb_ehciQtd_t;


#define EHCI_QTD_DATA_TOGGLE_MASK 0x80000000
#define EHCI_QTD_BYTES_TO_TRANSFER_MASK 0x7FFF0000
#define EHCI_QTD_BYTES_TO_TRANSFER_SHIFT 16
#define EHCI_QTD_IOC_MASK 0x00008000
#define EHCI_PID_OUT 0
#define EHCI_PID_IN 1
#define EHCI_PID_SETUP 2
#define EHCI_QTD_PID_SHIFT 8
#define EHCI_QTD_PID(x) (((x)&3)<<EHCI_QTD_PID_SHIFT)
#define EHCI_QTD_STATUS_MASK 0x000000F8
#define EHCI_QTD_ERROR_MASK 0x00000078
#define EHCI_QTD_ACTIVE_MASK 0x00000080
#define EHCI_QTD_STALL_MASK 0x00000040
#define EHCI_QTD_BUFFER_ERROR_MASK 0x00000020
#define EHCI_QTD_BUBBLE_MASK 0x00000010
#define EHCI_QTD_TRANSACTION_ERROR_MASK 0x00000008

#define EHCI_QTD_ALIGN 64


typedef struct usb_ehciQh_ {
   volatile uint32_t hlink;   /* (5-31) Memory address of 
                                          ** next data object to be processed
                                          ** (4..3) reserved 
                                          ** (2..1) type of the item
                                          ** T (bit 0) indicating pointer 
                                          ** validity
                                          */
   volatile uint32_t epCap; /* bits 31-28: NAK count reload,
                                          ** bit 27: Control endpoint flag
                                          ** bit 26-16: Maximum packet length
                                          ** bit 15: Head of reclamation 
                                          ** list flag
                                          ** bit 14: data toggle control
                                          ** bits 13-12: endpoint speed
                                          ** bit 11-8: endpoint number
                                          ** bits 7: Inactivate on next tr
                                          ** bits 6-0: Device address
                                          */
   volatile uint32_t epCap2; /* bits 31-30: High-BW pipe 
                                          ** Multiplier, 
                                          ** bit 29-23: Port number
                                          ** bit 22-16: Hub address
                                          ** bit 15-8: Split completion mask
                                          ** bit 7-0: Interrupt schedule mask
                                          */
   volatile uint32_t currQtd;/* bits 31-5: physical memory address
                                          ** of the current xaction processed
                                          */
   volatile uint32_t nextQtd;/* bits 31-5: physical memory address
                                          ** of the current xaction processed
                                          ** bit 0: Terminate bit
                                          */
   volatile uint32_t altQtd;  /* bits 31-5: physical memory address
                                                ** of the current xaction processed
                                                ** bits 4-1: NAK counter
                                                ** bit 0: Terminate bit
                                                */
   volatile uint32_t status;          /* bit 31: data-toggle
                                          ** bits 30-16: total bytes to transfer
                                          ** bit 15: Interrupt on complete
                                          ** bits 11-10: Error counter
                                          ** bit 0: Ping state/Err
                                          ** physical memory address
                                          ** bit 11-0: reserved
                                          */
   volatile uint32_t buff[5];     /* bit 31-12: 4K-page aligned 
                                          ** physical memory address
                                          ** bit 11-0: reserved
                                          */
   uint32_t dmaAddr; /* Some unused bits here */
   LIST_HEAD(usb_ehciQtd_) qtdList;
   LIST_ENTRY(usb_ehciQh_) qhList;
} usb_ehciQh_t;

#define EHCI_QH_TYPE_MASK  0x00000006
#define EHCI_QH_TYPE_SHIFT 1
#define EHCI_QH_TYPE_QH    1
#define EHCI_QH_TYPE(x) ((((u32)x)<<EHCI_QH_TYPE_SHIFT)&EHCI_QH_TYPE_MASK)
#define EHCI_LINK_INVALID  1
#define EHCI_QH_CTRL_EP_FLAG_MASK (1 << 27)
#define EHCI_QH_MAX_PACKET_LEN_MASK  0x07FF0000
#define EHCI_QH_MAX_PACKET_LEN_SHIFT 16
#define EHCI_QH_MAX_PACKET(x) ((((u32)x)<<EHCI_QH_MAX_PACKET_LEN_SHIFT)&EHCI_QH_MAX_PACKET_LEN_MASK)
#define EHCI_QH_H_MASK               0x00008000
#define EHCI_QH_DATA_TOGGLE_CTRL_MASK   0x00004000
#define EHCI_QH_EP_SPEED_MASK       0x00003000  
#define EHCI_QH_EP_SPEED_SHIFT      12
#define EHCI_QH_EP_SPEED(x) ((((u32)x)<<EHCI_QH_EP_SPEED_SHIFT)&EHCI_QH_EP_SPEED_MASK)
#define EHCI_QH_EP_HIGH_SPEED       0x00002000  
#define EHCI_QH_EP_NUM_MASK         0x00000F00
#define EHCI_QH_EP_NUM_SHIFT        8
#define EHCI_QH_EP_NUM(x) ((((u32)x)<<EHCI_QH_EP_NUM_SHIFT)&EHCI_QH_EP_NUM_MASK)
#define EHCI_QH_DEV_ADDR_MASK       0x0000003F
#define EHCI_QH_DEV_ADDR(x) (((u32)x)&EHCI_QH_DEV_ADDR_MASK)
#define EHCI_QH_MULT_MASK           0xC0000000
#define EHCI_QH_MULT_SHIFT          30
#define EHCI_QH_MULT(x) ((((u32)x)<<EHCI_QH_MULT_SHIFT)&EHCI_QH_MULT_MASK)
#define EHCI_QH_C_MASK				0x0000FF00
#define EHCI_QH_S_MASK				0x000000FF

#define EHCI_QH_ALIGN 64


typedef struct usb_ehciItd_ {
	volatile uint32_t nextLink;
	volatile uint32_t tr[8];
	volatile uint32_t buff[7];
} usb_ehciItd_t;


#endif
