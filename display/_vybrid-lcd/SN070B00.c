/**
 * LCD glass SN070B00 kernel driver
 *
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
#include <dev/lcdglass/SN070B00/if.h>
#include <dev/vybrid-lcddrv/if.h>


/** Mapping between segments and pins in the glass itself*/
/* Note that the numbers of pins for FP control and the number of FPs are mirrored in this glass: for example, FP0 goes to pin 17,
 * and FP17 goes to pin 0. But here we just map the pins, not the real FPs, to ease copying the table from the documentation.
 * BPs are in the range 1 to 4, FPs are 1 to 18*/
#define LCDGLASS_SEGMENT_PINS_MAP {\
/*	name,				BP		FP	*/	\
/*	SEG_T0		*/	{	3	,	15	},	\
/*	SEG_T1		*/	{	2	,	1	},	\
/*	SEG_T2		*/	{	3	,	1	},	\
/*	SEG_T3		*/	{	4	,	1	},	\
/*	SEG_T4		*/	{	1	,	14	},	\
/*	SEG_T5		*/	{	2	,	14	},	\
/*	SEG_T6		*/	{	3	,	14	},	\
/*	SEG_T7		*/	{	4	,	14	},	\
/*	SEG_T8		*/	{	4	,	15	},	\
/*	SEG_T9		*/	{	2	,	15	},	\
/*	SEG_T10		*/	{	1	,	15	},	\
/*	SEG_T11		*/	{	1	,	18	},	\
/*	SEG_T13		*/	{	4	,	5	},	\
/*	SEG_T14		*/	{	4	,	9	},	\
/*	SEG_T15		*/	{	4	,	3	},	\
/*	SEG_T16		*/	{	4	,	17	},	\
/*	SEG_COL1	*/	{   4	, 	7	},	\
/*	SEG_COL2	*/	{	4	,	11	},	\
/*	SEG_KWh		*/	{	2	,	18	},	\
/*	SEG_GJ		*/	{	3	,	18	},	\
/*	SEG_m3		*/	{	4	,	18	},	\
/*	SEG_1A		*/	{   1	,	3	},	\
/*	SEG_1B		*/	{   2	,	3	},	\
/*	SEG_1C		*/	{   3	,	3	},	\
/*	SEG_1D		*/	{   4	,	2	},	\
/*	SEG_1E		*/	{   3	,	2	},	\
/*	SEG_1F		*/	{	1	,	2	},	\
/*	SEG_1G		*/	{	2	,	2	},	\
/*	SEG_2A		*/	{   1	,	5	},	\
/*	SEG_2B		*/	{   2	,	5	},	\
/*	SEG_2C		*/	{   3	,	5	},	\
/*	SEG_2D		*/	{   4	,	4	},	\
/*	SEG_2E		*/	{   3	,	4	},	\
/*	SEG_2F		*/	{	1	,	4	},	\
/*	SEG_2G		*/	{	2	,	4	},	\
/*	SEG_3A		*/	{   1	,	7	},	\
/*	SEG_3B		*/	{   2	,	7	},	\
/*	SEG_3C		*/	{   3	,	7	},	\
/*	SEG_3D		*/	{   4	,	6	},	\
/*	SEG_3E		*/	{   3	,	6	},	\
/*	SEG_3F		*/	{	1	,	6	},	\
/*	SEG_3G		*/	{	2	,	6	},	\
/*	SEG_4A		*/	{   1	,	9	},	\
/*	SEG_4B		*/	{   2	,	9	},	\
/*	SEG_4C		*/	{   3	,	9	},	\
/*	SEG_4D		*/	{   4	,	8	},	\
/*	SEG_4E		*/	{   3	,	8	},	\
/*	SEG_4F		*/	{	1	,	8	},	\
/*	SEG_4G		*/	{	2	,	8	},	\
/*	SEG_5A		*/	{   1	,	11	},	\
/*	SEG_5B		*/	{   2	,	11	},	\
/*	SEG_5C		*/	{   3	,	11	},	\
/*	SEG_5D		*/	{   4	,	10	},	\
/*	SEG_5E		*/	{   3	,	10	},	\
/*	SEG_5F		*/	{	1	,	10	},	\
/*	SEG_5G		*/	{	2	,	10	},	\
/*	SEG_6A		*/	{   1	,	13	},	\
/*	SEG_6B		*/	{   2	,	13	},	\
/*	SEG_6C		*/	{   3	,	13	},	\
/*	SEG_6D		*/	{   4	,	12	},	\
/*	SEG_6E		*/	{   3	,	12	},	\
/*	SEG_6F		*/	{	1	,	12	},	\
/*	SEG_6G		*/	{	2	,	12	},	\
/*	SEG_7A		*/	{   1	,	17	},	\
/*	SEG_7B		*/	{   2	,	17	},	\
/*	SEG_7C		*/	{   3	,	17	},	\
/*	SEG_7D		*/	{   4	,	16	},	\
/*	SEG_7E		*/	{   3	,	16	},	\
/*	SEG_7F		*/	{	1	,	16	},	\
/*	SEG_7G	    */	{	2	,	16	},	\
}

static const struct lcdglass_segToBpFp {
	u8	BPpin;
	u8	FPpin;
} lcdglass_segToBpFpMap[] = LCDGLASS_SEGMENT_PINS_MAP;

const u8 lcdglass_seg_num = sizeof(lcdglass_segToBpFpMap)/sizeof(lcdglass_segToBpFpMap[0]);


/** get the glass backplane and frontplane that correspond to the requested segment */
/* Planes are 0-based, glass pins are 1-based. Also, fp pins are mirrored */
static void lcdglass_segmentToPins(lcdGlass_segment_t seg, u8 * bp, u8 * fp)
{
	*bp = lcdglass_segToBpFpMap[seg].BPpin;
	*fp = lcdglass_segToBpFpMap[seg].FPpin;
//	main_printf(ATTR_INFO, "seg %d = BPpin %d, FPpin %d = BPg %d, FPg %d\n", seg, lcdglass_segToBpFpMap[seg].BPpin,
//			lcdglass_segToBpFpMap[seg].FPpin, *bp, *fp);
}

void lcdglass_segmentSet(lcdGlass_segment_t seg)
{
	u8 gbp, gfp;
	lcdglass_segmentToPins(seg, &gbp, &gfp);
	lcddrv_pinsSet(gbp, gfp);
//	main_printf(ATTR_INFO, "seg %d = glass bp%d, fp%d = vybrid bp%d, fp%d\n", seg, gbp, gfp, dbp, dfp);
}

void lcdglass_segmentClear(lcdGlass_segment_t seg)
{
	u8 gbp, gfp;
	lcdglass_segmentToPins(seg, &gbp, &gfp);
	lcddrv_pinsClear(gbp, gfp);
}

/** Segments which make up each full glass digit*/
static const u8 lcdglass_digitToSegmentsMap[][7] = {
	/*digit		array of segments*/
	/*dig_1	*/	{SEG_1A, SEG_1B, SEG_1C, SEG_1D, SEG_1E, SEG_1F, SEG_1G},
	/*dig_2	*/	{SEG_2A, SEG_2B, SEG_2C, SEG_2D, SEG_2E, SEG_2F, SEG_2G},
	/*dig_3	*/	{SEG_3A, SEG_3B, SEG_3C, SEG_3D, SEG_3E, SEG_3F, SEG_3G},
	/*dig_4	*/	{SEG_4A, SEG_4B, SEG_4C, SEG_4D, SEG_4E, SEG_4F, SEG_4G},
	/*dig_5	*/	{SEG_5A, SEG_5B, SEG_5C, SEG_5D, SEG_5E, SEG_5F, SEG_5G},
	/*dig_6	*/	{SEG_6A, SEG_6B, SEG_6C, SEG_6D, SEG_6E, SEG_6F, SEG_6G},
	/*dig_7	*/	{SEG_7A, SEG_7B, SEG_7C, SEG_7D, SEG_7E, SEG_7F, SEG_7G},
	};

static const u8 lcdglass_digit_num = sizeof(lcdglass_digitToSegmentsMap)/sizeof(lcdglass_digitToSegmentsMap[0]);

/** Generic segments available in a generic digit
  * To be used as indexes in the arrays of particular segments for each particular digit*/
enum digit_segments {
	SEG_A,
	SEG_B,
	SEG_C,
	SEG_D,
	SEG_E,
	SEG_F,
	SEG_G,
	SEG_END		/*array end marker*/
};

/* BUILDING CHARS UP FROM THE DIGIT SEGMENTS */
/** Definition of char "0" from generic digit segments */
static const u8 lcdglass_char0Segs[] = {SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_END};
/** Definition of char "1" from generic digit segments */
static const u8 lcdglass_char1Segs[] = {SEG_B, SEG_C, SEG_END};
/** Definition of char "2" from generic digit segments */
static const u8 lcdglass_char2Segs[] = {SEG_A, SEG_B, SEG_G, SEG_E, SEG_D, SEG_END};
/** Definition of char "3" from generic digit segments */
static const u8 lcdglass_char3Segs[] = {SEG_A, SEG_B, SEG_G, SEG_C, SEG_D, SEG_END};
/** Definition of char "4" from generic digit segments */
static const u8 lcdglass_char4Segs[] = {SEG_F, SEG_G, SEG_B, SEG_C, SEG_END};
/** Definition of char "5" from generic digit segments */
static const u8 lcdglass_char5Segs[] = {SEG_A, SEG_F, SEG_G, SEG_C, SEG_D, SEG_END};
/** Definition of char "6" from generic digit segments */
static const u8 lcdglass_char6Segs[] = {SEG_A, SEG_F, SEG_G, SEG_E, SEG_D, SEG_C, SEG_END};
/** Definition of char "7" from generic digit segments */
static const u8 lcdglass_char7Segs[] = {SEG_A, SEG_B, SEG_C, SEG_END};
/** Definition of char "8" from generic digit segments */
static const u8 lcdglass_char8Segs[] = {SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, SEG_END};
/** Definition of char "9" from generic digit segments */
static const u8 lcdglass_char9Segs[] = {SEG_A, SEG_B, SEG_G, SEG_F, SEG_C, SEG_D, SEG_END};
/** Definition of char "A" from generic digit segments */
static const u8 lcdglass_charASegs[] = {SEG_A, SEG_B, SEG_C, SEG_E, SEG_F, SEG_G, SEG_END};
/** Definition of char "B" from generic digit segments */
static const u8 lcdglass_charBSegs[] = {SEG_F, SEG_E, SEG_D, SEG_C, SEG_G, SEG_END};
/** Definition of char "C" from generic digit segments */
static const u8 lcdglass_charCSegs[] = {SEG_A, SEG_F, SEG_E, SEG_D, SEG_END};
/** Definition of char "d" from generic digit segments */
static const u8 lcdglass_charDSegs[] = {SEG_B, SEG_C, SEG_D, SEG_E, SEG_G, SEG_END};
/** Definition of char "E" from generic digit segments */
static const u8 lcdglass_charESegs[] = {SEG_A, SEG_F, SEG_G, SEG_E, SEG_D, SEG_END};
/** Definition of char "F" from generic digit segments */
static const u8 lcdglass_charFSegs[] = {SEG_A, SEG_F, SEG_G, SEG_E, SEG_END};

/** directory of defined chars*/
static const char * lcdglass_charDirectory = "0123456789abcdef";
/** Map from character to its generic-segment definition */
static const u8 * lcdglass_charToGenericSegmentMap[] = {
	lcdglass_char0Segs,
	lcdglass_char1Segs,
	lcdglass_char2Segs,
	lcdglass_char3Segs,
	lcdglass_char4Segs,
	lcdglass_char5Segs,
	lcdglass_char6Segs,
	lcdglass_char7Segs,
	lcdglass_char8Segs,
	lcdglass_char9Segs,
	lcdglass_charASegs,
	lcdglass_charBSegs,
	lcdglass_charCSegs,
	lcdglass_charDSegs,
	lcdglass_charESegs,
	lcdglass_charFSegs
};
/** number of defined chars*/
static const u8 lcdglass_char_num = sizeof(lcdglass_charToGenericSegmentMap)/sizeof(lcdglass_charToGenericSegmentMap[0]);

/** for each char in the str, particularize the generic-segments-definition for that char into particular segments inside a concrete digit*/
/* for example, character "1" in position 3 means different segments than character "1" in position 4*/
void lcdglass_print(char * str)
{
	u8 digit;
	u8 l = min(lcdglass_digit_num,strlen(str));	/*number of glass digits that we are going to fill */

	/*for each digit to fill... */
	for(digit=0;digit<l;digit++){
		/* find it in the directory of defined chars */
		char * p = strrchr(lcdglass_charDirectory, str[digit]);
		/* if it was not there, leave the current digit and go to the next one */
		if(p == NULL)
			continue;	/*undefined chars cause a space*/
		u8 char_index = p - lcdglass_charDirectory;
		u8 j;
		/* go through the list of generic segments that form that character... */
		for(j=0;lcdglass_charToGenericSegmentMap[char_index][j]!=SEG_END;j++){
			u8 genericSegment = lcdglass_charToGenericSegmentMap[char_index][j];
			/* convert each generic segment into a particular segment for the current digit */
			lcdGlass_segment_t seg = lcdglass_digitToSegmentsMap[digit][genericSegment];
			/* turn on the particular segment */
			lcdglass_segmentSet(seg);
		}
	}
}
