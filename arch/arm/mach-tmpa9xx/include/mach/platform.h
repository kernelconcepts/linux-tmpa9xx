/*
 * Platform specific stuff shared between the drivers
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

#ifndef __TMPA9XX_PLATFORM_H__
#define __TMPA9XX_PLATFORM_H__

struct tmpa9xx_panel_ts_info
{
	struct clcd_panel *panel;
	int fuzz;
	int rate;
	int lcd_power;
	int lcd_reset;
};

extern int tmpa9xx_ts_fuzz;
extern int tmpa9xx_ts_rate;

struct tmpa9xx_i2s_cfg
{
	bool is_master_tx;
	bool is_master_rx;
	bool common_rx_tx_clock;
};

int tmpa9xx_clcd_register_ioctl(int (*ioctl)(void *priv, unsigned int cmd, unsigned long arg), void *priv);
int tmpa9xx_clcd_unregister_ioctl(void *priv);

#endif
