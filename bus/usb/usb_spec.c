#include <lib/stdint.h>
#include <lib/ptr.h>
#include "usb_spec.h"

inline usb_epDesc_t* usb_cfgSkipEndpoints(usb_epDesc_t* ed,u8 epNum)
{
	for(;epNum > 0;epNum--)
		ed = PTR_ADD(ed,ed->bLength);
	return ed;
}

usb_ifDesc_t* usb_cfgIfDesc(usb_cfgDesc_t* cfg,u8 ifNum)
{
	usb_ifDesc_t* id;
	id = (usb_ifDesc_t*) PTR_ADD(cfg,cfg->bLength);
	for(;ifNum > 0;ifNum--) {
		id = (usb_ifDesc_t*)usb_cfgSkipEndpoints((usb_epDesc_t*)PTR_ADD(id,id->bLength),id->bNumEndpoints);	
	}
	return id;
}

usb_epDesc_t* usb_cfgEpDesc(usb_cfgDesc_t* cfg,u8 ifNum,u8 epNum)
{
	usb_ifDesc_t* id = usb_cfgIfDesc(cfg,ifNum);
	return usb_cfgSkipEndpoints((usb_epDesc_t*)PTR_ADD(id,id->bLength),epNum);
}
