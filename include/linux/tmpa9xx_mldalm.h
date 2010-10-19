/*
 *	Generic watchdog defines. Derived from..
 *
 * Berkshire PC Watchdog Defines
 * by Ken Hollis <khollis@bitgate.com>
 *
 */

#ifndef _TMPA9XX_MLDALM_H
#define _TMPA9XX_MLDALM_H

#include <linux/ioctl.h>
#include <linux/types.h>

struct mldalm_type {
	unsigned char type;	/* Select type of output */
	unsigned char invert;	/* invert output */
        unsigned int  data;     /* Alarm type or output frequency */
};

#define	MLDALM_TYPE_SELECT	_IOW('m', 0, struct mldalm_type)
#define	MLDALM_START_STOP	_IOW('m', 1, unsigned char)

#define MLDALM_MELODY 0x1 /* Select Melody as Output */
#define MLDALM_ALARM  0x2 /* Select Alarm as Output */

#define MLDALM_START 1
#define MLDALM_STOP  0

#define MLDALM_INVERT 0x1 /* Invert the output signal */

#define ALMPTSEL_AL0 0x00 /* Fixed at 0 */
#define ALMPTSEL_AL1 0x01 /* AL1 pattern */
#define ALMPTSEL_AL2 0x02 /* AL2 pattern */
#define ALMPTSEL_AL3 0x04 /* AL3 pattern */
#define ALMPTSEL_AL4 0x08 /* AL4 pattern */
#define ALMPTSEL_AL5 0x10 /* AL5 pattern */
#define ALMPTSEL_AL6 0x20 /* AL6 pattern */
#define ALMPTSEL_AL7 0x40 /* AL7 pattern */
#define ALMPTSEL_AL8 0x80 /* AL8 pattern */

#endif  /* _TMPA9XX_MLDALM_H */
