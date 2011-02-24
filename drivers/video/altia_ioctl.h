/*
 *  drivers/video/altia_ioctl.h
 *
 * Copyright (C) 2010,2011 Altia Inc.
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
 * 
 */

#ifndef ALTIA_IOCTL_H
#define ALTIA_IOCTL_H

#include <linux/ioctl.h>

/***************************************************************************
** types
***************************************************************************/

typedef struct
{
    int size;
    unsigned long physical;
} ALTIA_IO_MEMORY_T;

typedef struct 
{
    unsigned long status;
    unsigned short ga;
    unsigned short err;
}
ALTIA_IO_IST_T;

typedef struct
{
    unsigned long reg;
    unsigned long value;
    char size;
} ALTIA_IO_REG_T;

typedef struct
{
    unsigned long cr0;
    unsigned long dr0;
    unsigned long dr1;
    unsigned long fcp;
    unsigned long efcp;
    unsigned long dv0;
    unsigned long dv1;
    unsigned long cr2;
    unsigned long dxdst;
    unsigned long dydst;
    unsigned long ssize;
    unsigned long dsize;
    unsigned long s0adr;
    unsigned long dadr;
    unsigned long cr1;
} ALTIA_IO_UPDATE_T;

typedef struct
{
    int operation : 5;
    int pending : 1;
} ALTIA_IO_STATUS_T;


/***************************************************************************
** definitions 
***************************************************************************/

#define ALTIA_IOC_MAGIC     0xAA

#define ALTIA_IOVER     _IO(ALTIA_IOC_MAGIC, 1)
#define ALTIA_IOALLOC   _IOWR(ALTIA_IOC_MAGIC, 2, ALTIA_IO_MEMORY_T)
#define ALTIA_IOFREE    _IOWR(ALTIA_IOC_MAGIC, 3, ALTIA_IO_MEMORY_T)
#define ALTIA_IOIST     _IOWR(ALTIA_IOC_MAGIC, 4, ALTIA_IO_IST_T)
#define ALTIA_IOREGR    _IOWR(ALTIA_IOC_MAGIC, 5, ALTIA_IO_REG_T)
#define ALTIA_IOREGW    _IOWR(ALTIA_IOC_MAGIC, 6, ALTIA_IO_REG_T)
#define ALTIA_IOUPDATE  _IOWR(ALTIA_IOC_MAGIC, 8, ALTIA_IO_UPDATE_T)
#define ALTIA_IOSTATUS  _IOWR(ALTIA_IOC_MAGIC, 9, ALTIA_IO_STATUS_T)
#define ALTIA_IORESET   _IO(ALTIA_IOC_MAGIC, 10)


#endif /* ALTIA_IOCTL_H */
