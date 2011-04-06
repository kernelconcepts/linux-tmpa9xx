/*
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License Version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#ifndef __TMPA9XX_TIMER_H__
#define __TMPA9XX_TIMER_H__

#define TIMER_LOAD		(0x00)
#define TIMER_VALUE		(0x04)
#define TIMER_CONTROL		(0x08)
#define TIMER_INTCLR		(0x0c)
#define TIMER_RIS		(0x10)
#define TIMER_MIS		(0x14)
#define TIMER_BGLOAD		(0x18)
#define TIMER_MODE		(0x1c)
#define TIMER_COMPARE_1		(0xa0)
#define TIMER_CMPINTCLR_1	(0xc0)
#define TIMER_CMPEN		(0xe0)
#define TIMER_CMP_RIS		(0xe4)
#define TIMER_CMP_MIS		(0xe8)
#define TIMER_BGCMP		(0xec)

#define TIMxEN			(1<<7) /* Enable Timer */
#define TIMxMOD_PER		(1<<6) /* Periodic Timer */
#define TIMxINTE		(1<<5) /* Timer Interrupt Enable */
#define TIMxSIZE_16B		(1<<1) /* 16 Bit Timer */
#define TIMxOSCTL_NORESTART	(1<<0) /* Timer Wrapping operation */

#define TIMxPRS_1 		(0x0<<2) /* Timer prescaler   1 */
#define TIMxPRS_16 		(0x1<<2) /* Timer prescaler  16 */
#define TIMxPRS_256		(0x2<<2) /* Timer prescaler 256 */

#endif
