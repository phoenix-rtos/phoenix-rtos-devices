
#define USBSTS_AS  (1 << 15)
#define USBSTS_PS  (1 << 14)
#define USBSTS_RCL (1 << 13)
#define USBSTS_URI (1 << 6)
#define USBSTS_SRI (1 << 7)
#define USBSTS_SLI (1 << 8)
#define USBSTS_ULPII (1 << 10)
#define USBSTS_HCH (1 << 12)
#define USBSTS_IAA (1 << 5)
#define USBSTS_SEI (1 << 4)
#define USBSTS_FRI (1 << 3)
#define USBSTS_PCI (1 << 2)
#define USBSTS_UEI (1 << 1)
#define USBSTS_UI  (1 << 0)

#define USBCMD_ASE (1 << 5)
#define USBCMD_IAA (1 << 6)

#define PORTSC_PTS_1 (3 << 30)
#define PORTSC_STS (1 << 29)
#define PORTSC_PPTW (1 << 28)
#define PORTSC_PSPD (3 << 26)
#define PORTSC_PTS_2 (1 << 25)
#define PORTSC_PFSC (1 << 24)
#define PORTSC_PHCD (1 << 23)

#define PORTSC_WKOC (1 << 22)
#define PORTSC_WKDSCNNT (1 << 21)
#define PORTSC_WKCNT (1 << 20)
#define PORTSC_PTC (0xf << 16)
#define PORTSC_PIC (3 << 14)
#define PORTSC_PO (1 << 13)
#define PORTSC_PP (1 << 12)
#define PORTSC_LS (3 << 10)
#define PORTSC_HSP (1 << 9)
#define PORTSC_PR (1 << 8)
#define PORTSC_SUSP (1 << 7)

#define PORTSC_FPR (1 << 6)
#define PORTSC_OCC (1 << 5)
#define PORTSC_OCA (1 << 4)
#define PORTSC_PEC (1 << 3)

#define PORTSC_ENA (1 << 2)
#define PORTSC_CSC (1 << 1)
#define PORTSC_CCS (1 << 0)

#define PORTSC_PSPD_HS (2 << 26)

/* 'change' bits cleared by writing 1 */
#define PORTSC_CBITS (PORTSC_CSC | PORTSC_PEC | PORTSC_OCC)


#define EHCI_PAGE_SIZE          4096
#define EHCI_MAX_QTD_BUF_SIZE   (4 * EHCI_PAGE_SIZE)

enum {
	/* identification regs */
	id = 0x0, hwgeneral, hwhost, hwdevice, hwtxbuf, hwrxbuf,

	/* operational regs */
	gptimer0ld	= 0x20, gptimer0ctrl, gptimer1ld, gptimer1ctrl, sbuscfg,

	/* capability regs */
	caplength = 0x40, hciversion = 0x40, hcsparams, hccparams,
	dciversion = 0x48, dccparams,

	/* operational regs cont. */
	usbcmd = 0x50, usbsts, usbintr, frindex,
	periodiclistbase = 0x55, deviceaddr = 0x55, asynclistaddr = 0x56,
	endpointlistaddr = 0x56, burstsize = 0x58, txfilltunning, endptnak = 0x5E,
	endptnaken, configflag, portsc1, otgsc = 0x69, usbmode, endptsetupstat,
	endptprime, endptflush, endptstat, endptcomplete, endptctrl0, endptctrl1,
	endptctrl2, endptctrl3, endptctrl4, endptctrl5, endptctrl6, endptctrl7,

	otg1_ctrl = 0x100, otg2_ctrl,
};


enum { usb_otg1_ctrl = 0x200, usb_otg2_ctrl, usb_otg1_phy_ctrl = usb_otg2_ctrl + 5, usb_otg2_phy_ctrl };

enum { ehci_item_itd = 0, ehci_item_qh, ehci_item_sitd, ehci_item_fstn };


typedef struct link_pointer {
	uint32_t terminate : 1;
	uint32_t type : 2;
	uint32_t zero : 2;
	uint32_t pointer : 27;
} __attribute__ ((__packed__)) link_pointer_t;


struct itd {
	link_pointer_t next;

	union {
		struct {
			uint32_t offset      : 12;
			uint32_t page_select :  3;
			uint32_t ioc         :  1;
			uint32_t length      : 12;
			uint32_t status      :  4;
		};
		uint32_t raw;
	} transactions[8];

	union {
		struct {
			uint32_t reserved   : 12;
			uint32_t pointer    : 20;
		} buffers[7];

		struct {
			uint32_t device_address  :  7;
			uint32_t reserved        :  1;
			uint32_t endpoint        :  4;
			uint32_t page0           : 20;
			uint32_t max_packet_size : 11;
			uint32_t direction       :  1;
			uint32_t page1           : 20;
			uint32_t mult            :  2;
		};
	};
} __attribute__ ((__packed__));


struct sitd {
	link_pointer_t next;

	uint32_t device_addr : 7;
	uint32_t reserved0   : 1;
	uint32_t endpoint    : 4;
	uint32_t reserved1   : 4;
	uint32_t hub_addr    : 7;
	uint32_t reserved2   : 1;
	uint32_t port_number : 7;
	uint32_t direction   : 1;

	uint32_t split_start_mask : 8;
	uint32_t split_completion_mask : 8;
	uint32_t reserved3 : 16;

	uint32_t status : 8;
	uint32_t split_progress_mask : 8;
	uint32_t bytes_to_transfer : 10;
	uint32_t reserved4 : 4;
	uint32_t page_select : 1;
	uint32_t ioc : 1;

	uint32_t current_offset : 12;
	uint32_t page0          : 20;

	uint32_t transaction_count    :  3;
	uint32_t transaction_position :  2;
	uint32_t reserved5            :  7;
	uint32_t page1                : 20;

	link_pointer_t back_link;
} __attribute__ ((__packed__));


struct qtd {
	link_pointer_t next; /* TODO: rename */
	link_pointer_t alt_next;

	uint32_t ping_state : 1;
	uint32_t split_state : 1;
	uint32_t missed_uframe : 1;
	uint32_t transaction_error : 1;
	uint32_t babble : 1;
	uint32_t buffer_error : 1;
	uint32_t halted : 1;
	uint32_t active : 1;

	uint32_t pid_code : 2;
	uint32_t error_counter : 2;
	uint32_t current_page : 3;
	uint32_t ioc : 1;
	uint32_t bytes_to_transfer : 15;
	uint32_t data_toggle : 1;

	union {
		struct {
			uint32_t reserved : 12;
			uint32_t page     : 20;
		} buffers[5];

		struct {
			uint32_t offset : 12;
			uint32_t page0  : 20;
		};
	};
} __attribute__ ((__packed__));


struct qh {
	link_pointer_t horizontal;

	uint32_t device_addr : 7;
	uint32_t inactivate : 1;
	uint32_t endpoint : 4;
	uint32_t endpoint_speed : 2;
	uint32_t data_toggle : 1;
	uint32_t head_of_reclamation : 1;
	uint32_t max_packet_len : 11;
	uint32_t control_endpoint : 1;
	uint32_t nak_count_reload : 4;

	uint32_t interrupt_schedule_mask : 8;
	uint32_t split_completion_mask : 8;
	uint32_t hub_addr : 7;
	uint32_t port_number : 7;
	uint32_t pipe_multiplier : 2;

	link_pointer_t current_qtd;

	struct qtd transfer_overlay;
} __attribute__ ((__packed__));

typedef struct ehci {
	link_pointer_t *periodic_list;
	volatile struct qh_node *async_head;

	handle_t irq_cond, irq_handle, irq_lock, aai_cond, async_lock;
	volatile unsigned portResetChange;
	volatile unsigned status;
	volatile unsigned port_change;
	volatile unsigned portsc;

	handle_t common_lock;
} ehci_t;


void phy_init(hcd_t *hcd);
int ehci_rootHubInit(usb_device_t *hub, int nports);