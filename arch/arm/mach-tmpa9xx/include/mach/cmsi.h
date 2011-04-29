/*
 * CMSI driver for Toshiba TMPA9xx processors.
 *
 *  Copyright (C) 2011 Michael Hunold <michael@mihu.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __TMPA9XX_CMSI__
#define __TMPA9XX_CMSI__

enum cmsi_process { CMSI_NONE, CMSI_SCALE_1_2, CMSI_SCALE_1_4, CMSI_SCALE_1_8, CMSI_TRIM };

struct cmsi_config
{
	enum cmsi_process p;
};

#define STATUS_OK		0x0
#define STATUS_FRAME_LOST	0x1

struct cmsi_buf
{
	void *ptr;
	size_t len;
	uint8_t status;
};

#define CMSI_CONFIG	_IOW('C', 0x01, struct cmsi_config)
#define CMSI_QBUF	_IOW('C', 0x02, struct cmsi_buf)
#define CMSI_DQBUF	_IOR('C', 0x03, struct cmsi_buf)

#endif

