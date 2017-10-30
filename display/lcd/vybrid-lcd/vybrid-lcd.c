/**
 * Kernel driver for Vybrid's LCD driver module

 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Horacio Mijail Anton Quiles <horacio.anton@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include "if.h"
#include <vm/if.h>
#include <main/if.h>
#include <proc/if.h>
#include <hal/MVF50GS10MK50.h>

#ifdef DEBUG_BUILD
	#define DEBUG_BOOL 1
	#define debug_printf(...)	main_printf(ATTR_DEBUG, __VA_ARGS__)
	#define DEBUG_FUNC
	#pragma message "*** DEBUG BUILD ***"
#else
	#define DEBUG_BOOL 0
	/* safe NOP macro*/
	#define debug_printf(...)	do {} while (0)
	#define DEBUG_FUNC			__attribute__((warning("A debug function is still being used!")))
#endif


#ifdef DEBUG_BUILD
	/* show correspondence segment to planes */
	#define DEBUG_SEG_TO_PLANE		true
//	#define DEBUG_SEG_TO_PLANE		false

	/* show the glass print logic */
//	#define DEBUG_GLASSPRINT	true
	#define DEBUG_GLASSPRINT	false

#else
	#define DEBUG_SEG_TO_PLANE			false
	#define DEBUG_GLASSPRINT	false
#endif



static LCD_Type * LCD_virt;

int _lcddrv_init(void)
{
	s32 status;

	status = vm_iomap(LCD_BASE, sizeof(LCD_Type), PGHD_DEV_RW, (void **)&LCD_virt);
	assert(status == EOK);

	main_printf(ATTR_INFO, "dev: LCD driver base=0x%x\n", LCD_BASE);

	return 0;
}

#define LCDRAM_MAX 		10			/*max implemented in Vybrid*/

/*clear all the segments*/
void lcddrv_clear(void)
{
	u8 i;
	for(i = 0;i < LCDRAM_MAX;i++){
		LCD_virt->Location[i]=0;
	}
}



#define EXPAND_AS_VYBRIDPLANES(glass_pin, is_backplane, vybrid_lcdxx, vybrid_plane)	vybrid_plane,
static const unsigned int vybrid_planes[] = {LCD_TABLE(EXPAND_AS_VYBRIDPLANES)};

#define EXPAND_AS_GLASSPINS(glass_pin, is_backplane, vybrid_lcdxx, vybrid_plane)	glass_pin,
static const unsigned int glass_pins[] = {LCD_TABLE(EXPAND_AS_GLASSPINS)};
static const unsigned int glass_pins_num = sizeof(glass_pins)/sizeof(glass_pins[0]);


/** Get the Vybrid plane that corresponds to the given glass pin
 *
 * Works the same for backplanes and frontplanes.
 *
 * @param glassPin
 * @return
 */
static int glassPin_to_vybridPlane(unsigned int glassPin)
{
	unsigned int i;
	for(i=0;i<glass_pins_num; i++){
		if(glass_pins[i] == glassPin)
			break;
	}
	if(i == glass_pins_num)
		return -1;
	return vybrid_planes[i];
}


void lcddrv_pinsSet(u8 glass_bpp, u8 glass_fpp)
{
	u8 driver_bp, driver_fp;
	driver_bp = glassPin_to_vybridPlane(glass_bpp);
	driver_fp = glassPin_to_vybridPlane(glass_fpp);
	LCD_Location_set(LCD_virt->Location, driver_bp, driver_fp);
//	main_printf(ATTR_DEBUG, "fp=%u, bp=%u, Loc[%d] |= 0x%X\n",frontplane, backplane, frontplane/4,1<<((8*(3-(frontplane%4))) + backplane));
}

void lcddrv_pinsClear(u8 glass_bpp, u8 glass_fpp)
{
	u8 driver_bp, driver_fp;
	driver_bp = glassPin_to_vybridPlane(glass_bpp);
	driver_fp = glassPin_to_vybridPlane(glass_fpp);
	LCD_virt->Location[driver_fp/4] &= ~(1u<<((8*(3-(driver_fp%4))) + driver_bp));
}


void lcddrv_contrastSet(unsigned int contrast)
{
	LCD_virt->LCDCCR = LCD_LCDCCR_CCEN_MASK | LCD_LCDCCR_LCC(contrast);
}

#if 0

#define EXPAND_AS_VYBRIDLCDPINS(glass_pin, is_backplane, vybrid_lcdxx, vybrid_plane)	vybrid_lcdxx,
static const unsigned int vybrid_lcdpins[] = {LCD_TABLE(EXPAND_AS_VYBRIDLCDPINS)};
static const unsigned int vybrid_lcdpins_num = sizeof(vybrid_lcdpins)/sizeof(vybrid_lcdpins[0]);

int lcddrv_convertLCDxxToPlane(unsigned int LCD_pin)
{
	unsigned int i;
	for(i=0;i<vybrid_lcdpins_num; i++){
		if(vybrid_lcdpins[i] == LCD_pin)
			break;
	}
	if(i == vybrid_lcdpins_num)		/*not found*/
		return -1;

	return vybrid_planes[i];
}
#endif
