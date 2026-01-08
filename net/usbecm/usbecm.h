#ifndef _USB_ECM_H_
#define _USB_ECM_H_


typedef struct {
	/* clang-format off */
	enum { usbecm_getLinkStatusChange } type;
	/* clang-format on */

	struct {
		struct {
			/* clang-format off */
			enum { usbecm_linkDown, usbecm_linkUp } linkstatus;
			/* clang-format on */
		} irq;
	};
} usbecm_msg_t;


#endif /* _USB_ECM_H_ */
