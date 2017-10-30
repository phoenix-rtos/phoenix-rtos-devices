/**
 * LCD glass SML1444 kernel driver
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
#include <dev/vybrid-lcddrv/if.h>		//the glass tells the LCD controller which pins to drive
#include <dev/lcdglass/if.h>
#include <dev/dev.h>

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
//	#define DEBUG_SEG_TO_PLANE		true
	#define DEBUG_SEG_TO_PLANE		false

	/* show the glass print logic */
//	#define DEBUG_GLASSPRINT	true
	#define DEBUG_GLASSPRINT	false

#else
	#define DEBUG_SEG_TO_PLANE			false
	#define DEBUG_GLASSPRINT	false
#endif

#ifdef DEBUG_BUILD
	#define EXPAND_AS_NAME_ARRAY(segment)	#segment ,
	/** array[segment_number] = segment name as string */
	static char *segment_names[]={
		SEGMENT_TABLE(EXPAND_AS_NAME_ARRAY)
	};
#endif


/** array[index] = Printing position in the glass where the <index> row starts */
static const unsigned int lcdglass_rows[]={
	0,
	8
};
static const unsigned int lcdglass_rows_num = sizeof(lcdglass_rows)/sizeof(lcdglass_rows[0]);


/** Mapping between segments and pins in the glass itself*/
/* There are 8 BackPlane pins (1 to 8) and 28 Front Plane pins (9 to 36).
 * We assume 0-based plane numbers corresponding to each set of plane pins to ease encoding.
 * A segment in the enum gets a number which encodes a plane combination as FP*8 + BP.
 * So for example SEG_1B gets number 2, meaning FP 0, BP 2, which means pin 9 for FP and pin 3 for BP*/
static void lcdglass_segmentToPins(lcdGlass_segment_t seg, u8 * bp, u8 * fp)
{
	*bp = (seg % 8)+1;
	*fp = seg / 8 +9;
}

void lcdglass_segmentSet(lcdGlass_segment_t seg)
{
	u8 gbp, gfp;
	lcdglass_segmentToPins(seg, &gbp, &gfp);
	lcddrv_pinsSet(gbp, gfp);
	if(DEBUG_SEG_TO_PLANE) debug_printf("%s(%u)= glass bp%d, fp%d\n", segment_names[seg], seg, gbp, gfp);
}


void lcdglass_segmentClear(lcdGlass_segment_t seg)
{
	u8 gbp, gfp;
	lcdglass_segmentToPins(seg, &gbp, &gfp);
	lcddrv_pinsClear(gbp, gfp);
}


/** General Kind Of a Digit.
 *
 * The KOD value allows to recognize what segment patterns can be applied onto the digit: the KOD defines
 * the expected order and number of segments in the digit.
 * The actual definition of a digit is always an array containing KOD, segments and sentinel. So any combination
 * of segments can be treated usefully as forming a digit and taking a position in a "print", even if the segments were non-standard */
enum kinds_of_digits {
	KOD_NONE		= 0,	/* this set of segments will consume one char from the printed string but print nothing */
	KOD_D7SEGS		= 1,	/* 7 segments */
	KOD_D14SEGS		= 2,	/* 14 segments */
	KOD_DOT			= 4,	/* 1 segment = decimal point */
	KOD_MIDDOT		= 8,	/* "·", as for "multiplying". 2 segments, but only the 2nd one is used (to allow for reuse in KOD_COLON) */
	KOD_COLON		= KOD_DOT | KOD_MIDDOT,
	KOD_COLON_SS	= 16,	/* both dots in a Single Segment */
};


/** List of segments that shape a full 7-segment digit, defined in the standard segment order (clockwise from top)*/
/* pretty much standard */
#define D7SEGS(segment_set)	{\
	KOD_D7SEGS,	/* digit type */ 	\
	SEG_##segment_set##A,	/* top */			\
	SEG_##segment_set##B,	/* top right*/		\
	SEG_##segment_set##C,	/* bottom right*/	\
	SEG_##segment_set##D,	/* bottom */		\
	SEG_##segment_set##E,	/* bottom left */	\
	SEG_##segment_set##F,	/* top left */		\
	SEG_##segment_set##G	/* middle */		\
}


/** List of segments that shape a full 14-segment digit, defined in clockwise order */
/* 14-segment digits are not as standardized as 7-segment ones, so make sure that the segment names exist and are in the correct order */
#define D14SEGS(segment_set)	{\
	KOD_D14SEGS,	/* digit type */ 	\
	SEG_##segment_set##A,	/* top */			\
	SEG_##segment_set##B,	/* top right*/		\
	SEG_##segment_set##C,	/* bottom right*/	\
	SEG_##segment_set##D,	/* bottom */		\
	SEG_##segment_set##E,	/* bottom left */	\
	SEG_##segment_set##F,	/* top left */		\
	SEG_##segment_set##G,	/* middle left*/	\
	SEG_##segment_set##J,	/* inner top left*/	\
	SEG_##segment_set##K,	/* inner top */		\
	SEG_##segment_set##L,	/* inner top right*/\
	SEG_##segment_set##M,	/* mid right */		\
	SEG_##segment_set##Q,	/* inner low right*/\
	SEG_##segment_set##P,	/* inner bottom*/	\
	SEG_##segment_set##N	/* inner low left*/	\
}


/* Definition of each available print position. The order of the segments is important. */
/*first line*/
static const unsigned int lg_p0[] = D7SEGS(1);
static const unsigned int lg_p1[] = D7SEGS(2);
static const unsigned int lg_p2[] = {KOD_COLON, SEG_H2, SEG_DP5};
static const unsigned int lg_p3[] = D7SEGS(3);
static const unsigned int lg_p4[] = {KOD_COLON, SEG_H3, SEG_DP6};
static const unsigned int lg_p5[] = D7SEGS(4);
static const unsigned int lg_p6[] = {KOD_DOT, SEG_H4};
static const unsigned int lg_p7[] = D7SEGS(5);
/*second line*/
static const unsigned int lg_p8[] = D14SEGS(6);
static const unsigned int lg_p9[] = {KOD_COLON, SEG_H6, SEG_DP1};
static const unsigned int lg_p10[] = D14SEGS(7);
static const unsigned int lg_p11[] = {KOD_DOT, SEG_H7};
static const unsigned int lg_p12[] = D14SEGS(8);
static const unsigned int lg_p13[] = {KOD_COLON, SEG_H8, SEG_DP2};
static const unsigned int lg_p14[] = D14SEGS(9);
static const unsigned int lg_p15[] = {KOD_DOT, SEG_H9};
static const unsigned int lg_p16[] = D14SEGS(10);
static const unsigned int lg_p17[] = {KOD_COLON, SEG_H10, SEG_DP3};
static const unsigned int lg_p18[] = D14SEGS(11);
static const unsigned int lg_p19[] = {KOD_DOT, SEG_H11};
static const unsigned int lg_p20[] = D14SEGS(12);
static const unsigned int lg_p21[] = {KOD_COLON, SEG_H12, SEG_DP4};
static const unsigned int lg_p22[] = D14SEGS(13);
static const unsigned int lg_p23[] = {KOD_DOT, SEG_H13};
static const unsigned int lg_p24[] = D14SEGS(14);


/** array[print_position] = information to use that printable position = KOD for the digit + array of segments that shape the full digit in that position*/
/* By using this array, the segments are laid out in a standardized way, so we can use our set of standardized character definitions */
static const unsigned int * lcdglass_printposToSegmentsMap[] = {
	lg_p0	,
	lg_p1	,
	lg_p2	,
	lg_p3	,
	lg_p4	,
	lg_p5	,
	lg_p6	,
	lg_p7	,
	lg_p8	,
	lg_p9	,
	lg_p10	,
	lg_p11	,
	lg_p12	,
	lg_p13	,
	lg_p14	,
	lg_p15	,
	lg_p16	,
	lg_p17	,
	lg_p18	,
	lg_p19	,
	lg_p20	,
	lg_p21	,
	lg_p22	,
	lg_p23	,
	lg_p24
};
/** Number of print positions */
static const u8 lcdglass_printpos_num = sizeof(lcdglass_printposToSegmentsMap)/sizeof(lcdglass_printposToSegmentsMap[0]);


/** Generic segments available in a generic 7-segment digit
  * To be used as indexes in the arrays of segments for each print position*/
/* the first 7 segments are equivalent for 7-segment and 14-segment digits */
typedef enum generic_segments {
	GSEG_KOD,		/* position used by the KOD */
	GSEG_A	,
	GSEG_B	,
	GSEG_C	,
	GSEG_D	,
	GSEG_E	,
	GSEG_F	,
	GSEG_G	,
	GSEG_J	,
	GSEG_K	,
	GSEG_L	,
	GSEG_M	,
	GSEG_Q	,
	GSEG_P	,
	GSEG_N	,
	GSEG_END		/*array end marker*/
} gseg_t;


/* BUILDING CHARS UP FROM 7-SEGMENT DIGIT SEGMENTS */
/* the order of the segments is not important, apart from the GSEG_END at the end :P */
static const gseg_t lg_D7S_charSpace[] = {GSEG_END};
static const gseg_t lg_D7S_char0[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_END};
static const gseg_t lg_D7S_char1[] = {GSEG_B, GSEG_C, GSEG_END};
static const gseg_t lg_D7S_char2[] = {GSEG_A, GSEG_B, GSEG_G, GSEG_E, GSEG_D, GSEG_END};
static const gseg_t lg_D7S_char3[] = {GSEG_A, GSEG_B, GSEG_G, GSEG_C, GSEG_D, GSEG_END};
static const gseg_t lg_D7S_char4[] = {GSEG_F, GSEG_G, GSEG_B, GSEG_C, GSEG_END};
static const gseg_t lg_D7S_char5[] = {GSEG_A, GSEG_F, GSEG_G, GSEG_C, GSEG_D, GSEG_END};
static const gseg_t lg_D7S_char6[] = {GSEG_A, GSEG_F, GSEG_G, GSEG_E, GSEG_D, GSEG_C, GSEG_END};
static const gseg_t lg_D7S_char7[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_END};
static const gseg_t lg_D7S_char8[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_G, GSEG_END};
static const gseg_t lg_D7S_char9[] = {GSEG_A, GSEG_B, GSEG_G, GSEG_F, GSEG_C, GSEG_D, GSEG_END};
static const gseg_t lg_D7S_chara[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_F, GSEG_E, GSEG_G, GSEG_END};
static const gseg_t lg_D7S_charb[] = {GSEG_F, GSEG_E, GSEG_D, GSEG_C, GSEG_G, GSEG_END};
static const gseg_t lg_D7S_charc[] = {GSEG_G, GSEG_E, GSEG_D, GSEG_END};
static const gseg_t lg_D7S_chard[] = {GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_G, GSEG_END};
static const gseg_t lg_D7S_chare[] = {GSEG_A, GSEG_B, GSEG_F, GSEG_G, GSEG_E, GSEG_D, GSEG_END};
static const gseg_t lg_D7S_charf[] = {GSEG_A, GSEG_F, GSEG_G, GSEG_E, GSEG_END};
static const gseg_t lg_D7S_charh[] = {GSEG_B, GSEG_C, GSEG_E, GSEG_F, GSEG_G, GSEG_END};
static const gseg_t lg_D7S_chari[] = {GSEG_F,  GSEG_E, GSEG_END};
static const gseg_t lg_D7S_charj[] = {GSEG_B,  GSEG_C, GSEG_D, GSEG_END};
static const gseg_t lg_D7S_charl[] = {GSEG_D,  GSEG_E, GSEG_F, GSEG_END};
static const gseg_t lg_D7S_charo[] = {GSEG_G, GSEG_C, GSEG_D, GSEG_E, GSEG_END};
static const gseg_t lg_D7S_charp[] = {GSEG_A, GSEG_F, GSEG_G, GSEG_E, GSEG_B, GSEG_END};


/** directory of defined chars for 7-segment digits*/
/* a char position in the directory gives its index in the char-to-genericSegment map */
static const char * lcdglass_D7S_charDirectory = " 0123456789ABCDEFHIJLOP";
/** Map from characters to their generic-segment 7-segment-digit definitions */
static const u8 * lcdglass_D7S_charToGenericSegmentMap[] = {
	lg_D7S_charSpace,
	lg_D7S_char0,
	lg_D7S_char1,
	lg_D7S_char2,
	lg_D7S_char3,
	lg_D7S_char4,
	lg_D7S_char5,
	lg_D7S_char6,
	lg_D7S_char7,
	lg_D7S_char8,
	lg_D7S_char9,
	lg_D7S_chara,
	lg_D7S_charb,
	lg_D7S_charc,
	lg_D7S_chard,
	lg_D7S_chare,
	lg_D7S_charf,
	lg_D7S_charh,
	lg_D7S_chari,
	lg_D7S_charj,
	lg_D7S_charl,
	lg_D7S_charo,
	lg_D7S_charp
};


/* BUILDING CHARS UP FROM 14-SEGMENT DIGIT SEGMENTS */
/* the order of the segments is not important, apart from the GSEG_END at the end :P */
static const gseg_t lg_D14S_charSpace[] = {GSEG_END};

static const gseg_t lg_D14S_charStar[] = {GSEG_J, GSEG_K, GSEG_L, GSEG_M, GSEG_P, GSEG_Q, GSEG_N, GSEG_G, GSEG_END};

static const gseg_t lg_D14S_charPlus[] = {GSEG_G, GSEG_M, GSEG_P, GSEG_K, GSEG_END};
static const gseg_t lg_D14S_charMinus[] = {GSEG_G, GSEG_M, GSEG_END};

static const gseg_t lg_D14S_charSlash[] = {GSEG_N, GSEG_L, GSEG_END};
static const gseg_t lg_D14S_charBackslash[] = {GSEG_J, GSEG_Q, GSEG_END};

static const gseg_t lg_D14S_charVtab[] = {GSEG_K, GSEG_P, GSEG_END};

static const gseg_t lg_D14S_charDash[] = {GSEG_F, GSEG_A, GSEG_L, GSEG_END};

static const gseg_t lg_D14S_charEq[] = {GSEG_A, GSEG_G, GSEG_M, GSEG_END};

static const gseg_t lg_D14S_charGt[] = {GSEG_J, GSEG_N, GSEG_END};
static const gseg_t lg_D14S_charLt[] = {GSEG_L, GSEG_Q, GSEG_END};

static const gseg_t lg_D14S_charQm[] = {GSEG_A, GSEG_B, GSEG_G, GSEG_M, GSEG_E, GSEG_END};

static const gseg_t lg_D14S_charDollar[] = {GSEG_A, GSEG_B, GSEG_G, GSEG_M, GSEG_E, GSEG_D, GSEG_K, GSEG_P, GSEG_END};
static const gseg_t lg_D14S_charHash[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_G, GSEG_M, GSEG_K, GSEG_P, GSEG_END};


static const gseg_t lg_D14S_char0[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_END};
static const gseg_t lg_D14S_char1[] = {GSEG_B, GSEG_C, GSEG_L, GSEG_END};
static const gseg_t lg_D14S_char2[] = {GSEG_A, GSEG_B, GSEG_G, GSEG_M, GSEG_E, GSEG_D, GSEG_END};
static const gseg_t lg_D14S_char3[] = {GSEG_A, GSEG_B, GSEG_M, GSEG_C, GSEG_D, GSEG_END};
static const gseg_t lg_D14S_char4[] = {GSEG_F, GSEG_G, GSEG_M, GSEG_B, GSEG_C, GSEG_END};
static const gseg_t lg_D14S_char5[] = {GSEG_A, GSEG_F, GSEG_G, GSEG_M, GSEG_C, GSEG_D, GSEG_END};
static const gseg_t lg_D14S_char6[] = {GSEG_A, GSEG_F, GSEG_G, GSEG_M, GSEG_E, GSEG_D, GSEG_C, GSEG_END};
static const gseg_t lg_D14S_char7[] = {GSEG_A, GSEG_L, GSEG_P, GSEG_END};
static const gseg_t lg_D14S_char8[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_G, GSEG_M, GSEG_END};
static const gseg_t lg_D14S_char9[] = {GSEG_A, GSEG_B, GSEG_G, GSEG_M, GSEG_F, GSEG_C, GSEG_END};
static const gseg_t lg_D14S_charA[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_E, GSEG_F, GSEG_G, GSEG_M, GSEG_END};
static const gseg_t lg_D14S_charB[] = {GSEG_A, GSEG_B, GSEG_K, GSEG_P, GSEG_D, GSEG_C, GSEG_M, GSEG_END};
static const gseg_t lg_D14S_charC[] = {GSEG_A, GSEG_F, GSEG_E, GSEG_D, GSEG_END};
static const gseg_t lg_D14S_charD[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_D, GSEG_K, GSEG_P, GSEG_END};
static const gseg_t lg_D14S_charE[] = {GSEG_A, GSEG_F, GSEG_G, GSEG_M, GSEG_E, GSEG_D, GSEG_END};
static const gseg_t lg_D14S_charF[] = {GSEG_A, GSEG_F, GSEG_G, GSEG_M, GSEG_E, GSEG_END};
static const gseg_t lg_D14S_charG[] = {GSEG_A, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_M, GSEG_END};
static const gseg_t lg_D14S_charH[] = {GSEG_B, GSEG_C, GSEG_E, GSEG_F, GSEG_G, GSEG_M, GSEG_END};
static const gseg_t lg_D14S_charI[] = {GSEG_A, GSEG_K, GSEG_P, GSEG_D, GSEG_END};
static const gseg_t lg_D14S_charJ[] = {GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_END};
static const gseg_t lg_D14S_charK[] = {GSEG_E, GSEG_F, GSEG_G, GSEG_L, GSEG_Q, GSEG_END};
static const gseg_t lg_D14S_charL[] = {GSEG_D, GSEG_E, GSEG_F, GSEG_END};
static const gseg_t lg_D14S_charM[] = {GSEG_B, GSEG_C, GSEG_E, GSEG_F, GSEG_J, GSEG_L, GSEG_END};
static const gseg_t lg_D14S_charN[] = {GSEG_B, GSEG_C, GSEG_Q, GSEG_J, GSEG_E, GSEG_F, GSEG_END};
static const gseg_t lg_D14S_charO[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_END};
static const gseg_t lg_D14S_charP[] = {GSEG_A, GSEG_B, GSEG_E, GSEG_F, GSEG_G, GSEG_M, GSEG_END};
static const gseg_t lg_D14S_charQ[] = {GSEG_A, GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_Q, GSEG_END};
static const gseg_t lg_D14S_charR[] = {GSEG_A, GSEG_B, GSEG_E, GSEG_F, GSEG_G, GSEG_M, GSEG_Q, GSEG_END};
static const gseg_t lg_D14S_charS[] = {GSEG_A, GSEG_C, GSEG_D, GSEG_J, GSEG_M, GSEG_END};
static const gseg_t lg_D14S_charT[] = {GSEG_A, GSEG_K, GSEG_P, GSEG_END};
static const gseg_t lg_D14S_charU[] = {GSEG_B, GSEG_C, GSEG_D, GSEG_E, GSEG_F, GSEG_END};
static const gseg_t lg_D14S_charV[] = {GSEG_E, GSEG_F, GSEG_N, GSEG_L, GSEG_END};
static const gseg_t lg_D14S_charW[] = {GSEG_B, GSEG_C, GSEG_Q, GSEG_N, GSEG_E, GSEG_F, GSEG_END};
static const gseg_t lg_D14S_charX[] = {GSEG_J, GSEG_L, GSEG_Q, GSEG_N, GSEG_END};
static const gseg_t lg_D14S_charY[] = {GSEG_J, GSEG_L, GSEG_P, GSEG_END};
static const gseg_t lg_D14S_charZ[] = {GSEG_A, GSEG_L, GSEG_N, GSEG_D, GSEG_END};
static const gseg_t lg_D14S_charPercent[] = {GSEG_F, GSEG_C, GSEG_L, GSEG_N, GSEG_END};



/** directory of defined chars for 14-segment digits*/
/* a char position in the directory gives its index in the char-to-genericSegment map */
static const char * lcdglass_D14S_charDirectory = " *+-/\\|^=><?$#0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ%";
/** Map from characters to their generic-segment 7-segment-digit definitions */
static const u8 * lcdglass_D14S_charToGenericSegmentMap[] = {
	lg_D14S_charSpace,
	lg_D14S_charStar,
	lg_D14S_charPlus,
	lg_D14S_charMinus,
	lg_D14S_charSlash,
	lg_D14S_charBackslash,
	lg_D14S_charVtab,
	lg_D14S_charDash,
	lg_D14S_charEq,
	lg_D14S_charGt,
	lg_D14S_charLt,
	lg_D14S_charQm,
	lg_D14S_charDollar,
	lg_D14S_charHash,
	lg_D14S_char0,
	lg_D14S_char1,
	lg_D14S_char2,
	lg_D14S_char3,
	lg_D14S_char4,
	lg_D14S_char5,
	lg_D14S_char6,
	lg_D14S_char7,
	lg_D14S_char8,
	lg_D14S_char9,
	lg_D14S_charA,
	lg_D14S_charB,
	lg_D14S_charC,
	lg_D14S_charD,
	lg_D14S_charE,
	lg_D14S_charF,
	lg_D14S_charG,
	lg_D14S_charH,
	lg_D14S_charI,
	lg_D14S_charJ,
	lg_D14S_charK,
	lg_D14S_charL,
	lg_D14S_charM,
	lg_D14S_charN,
	lg_D14S_charO,
	lg_D14S_charP,
	lg_D14S_charQ,
	lg_D14S_charR,
	lg_D14S_charS,
	lg_D14S_charT,
	lg_D14S_charU,
	lg_D14S_charV,
	lg_D14S_charW,
	lg_D14S_charX,
	lg_D14S_charY,
	lg_D14S_charZ,
	lg_D14S_charPercent
};

#define UPPER(c) ((c <= 'z' && c >= 'a') ? (c +('A'-'a')) : c)

/** for each char in the str, particularize the generic-segments-definition for that char into particular segments inside a concrete printing position*/
/* for example, character "1" in position 3 means different segments than character "1" in position 4*/
int lcdglass_print(char * str, unsigned int print_pos)
{
	unsigned int c;
	char const * charDirectory;
	u8 const * const * charToGenericSegmentMap;
	bool correct = true;

	/*iterate over the chars in the string until they run out or there is no more printing space on the glass */
	for(c=0 ; (str[c] != 0) && (print_pos<lcdglass_printpos_num) ; c++){
		unsigned int const * printpos_segments = lcdglass_printposToSegmentsMap[print_pos];
		u8 kind_of_digit = printpos_segments[0];
		if(DEBUG_GLASSPRINT)
			debug_printf("print char %c in glass pos %u (type %u):", str[c], print_pos, kind_of_digit);
		/*is the current char supported on the current print position? */
		/* the loop iterates the char to print; each print "routine" must do everything else, like advancing the print_pos */
		switch(str[c]){
			case '.':
				if(kind_of_digit & KOD_DOT){
					lcdglass_segmentSet(printpos_segments[1]);
					print_pos++;
					if(DEBUG_GLASSPRINT) debug_printf("DONE\n");
				} else {
					if(DEBUG_GLASSPRINT)
						debug_printf("BAD TYPE\n");
					/* if not printed, the dot is lost, but we stay on the same print position */
					correct = false;
				}
				break;
			case ':':
				switch(kind_of_digit){
					case KOD_COLON_SS:
						lcdglass_segmentSet(printpos_segments[1]);
						print_pos++;
						if(DEBUG_GLASSPRINT)
							debug_printf("DONE\n");
						break;
					case KOD_COLON:
						lcdglass_segmentSet(printpos_segments[1]);	/* dot */
						lcdglass_segmentSet(printpos_segments[2]);	/* middot */
						print_pos++;
						if(DEBUG_GLASSPRINT)
							debug_printf("DONE\n");
						break;
					default:
						if(DEBUG_GLASSPRINT)
							debug_printf("BAD TYPE\n");
						/* if not printed, the colon is lost, but we stay on the same print position */
						correct = false;
						break;
				}
				break;
			case '*':
				if(kind_of_digit & KOD_MIDDOT){
					lcdglass_segmentSet(printpos_segments[2]);
					print_pos++;
					if(DEBUG_GLASSPRINT)
						debug_printf("DONE\n");
					break;
				}
			default:
				/* the char is not one of the recognized specialist chars. Must be a normal char */
				if(DEBUG_GLASSPRINT)
					debug_printf(" normal char:");
				/* go to a non-specialist printing position before trying to print */
				while(((kind_of_digit = lcdglass_printposToSegmentsMap[print_pos][0]) & (KOD_D7SEGS | KOD_D14SEGS)) == 0){
					if(DEBUG_GLASSPRINT)
						debug_printf("(skip printpos %u)", print_pos);
					print_pos++;
					if(print_pos>=lcdglass_printpos_num){
						/* at least one char was pending to be written */
						return false;
					}
				}
				switch(kind_of_digit){
					case KOD_D7SEGS:
					case KOD_D14SEGS:
						if(kind_of_digit == KOD_D7SEGS){
							if(DEBUG_GLASSPRINT)
								debug_printf("7seg digit: ");
							charDirectory = lcdglass_D7S_charDirectory;
							charToGenericSegmentMap = lcdglass_D7S_charToGenericSegmentMap;
						} else {
							if(DEBUG_GLASSPRINT)
								debug_printf("14seg digit: ");
							charDirectory = lcdglass_D14S_charDirectory;
							charToGenericSegmentMap = lcdglass_D14S_charToGenericSegmentMap;
						}
						/* find the char in the directory of defined chars */
						char * p = strrchr(charDirectory, UPPER(str[c]));
						/* if it was not there, leave the current print_pos and go to the next one */
						if(p == NULL){
							if(DEBUG_GLASSPRINT)
								debug_printf("not in directory, LEFT EMTPY\n");
							print_pos++;
							correct = false;
							break;	/*undefined chars cause a space*/
						}
						u8 char_index = p - charDirectory;
						u8 j;
						/* iterate through the list of generic segments that form that character... */
						u8 genericSegment;
						for(j=0; (genericSegment = charToGenericSegmentMap[char_index][j])!=GSEG_END;j++){
							/* convert each generic segment into a particular segment for the current print_pos */
							lcdGlass_segment_t seg = lcdglass_printposToSegmentsMap[print_pos][genericSegment];
							/* turn on the particular segment */
							lcdglass_segmentSet(seg);
						}
						if(DEBUG_GLASSPRINT)
							debug_printf("DONE\n");
						print_pos++;
						break;
					default:
						/* can not happen */
						if(DEBUG_GLASSPRINT) debug_printf("lcdglass_print: wat?\n");
						correct = false;
						print_pos++;
						break;
				} 	/*end of switch*/
			} /* end of switch */
		} /* end of for */
	return correct;
}


int lcdglass_printRow(char * str, unsigned int row)
{
	if(row>=lcdglass_rows_num)
		return false;
	return lcdglass_print(str, lcdglass_rows[row]);
}

int sml1444_init() {
	static lcdglass_device dev = {
		.clear = lcddrv_clear,
		.setSegment = (void (*)(int segment))lcdglass_segmentSet,
		.clearSegment = (void (*)(int segment))lcdglass_segmentClear,
		.printRow = lcdglass_printRow,
		.print = lcdglass_print
	};

	if(subdev_register(MAKEDEV(MAJOR_LCDGLASS, 0), (void *)&dev) < 0)
	{
		//registration failed
		return -1;
	}
	dev_mknod(MAKEDEV(MAJOR_LCDGLASS, 0), "lcd");
	return 0;
}
