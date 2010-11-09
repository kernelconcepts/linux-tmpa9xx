/*
 * drivers/mmc/host/tmpa9xx-sdhc-user.c
 *
 * Copyright (C) 2010 
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#include "tmpa9xx-sdhc.h"

//#define SDHC_USER_DEBUG

#ifdef SDHC_USER_DEBUG
#define sdprintk(...)	do {printk("%s(): ", __func__); printk(__VA_ARGS__);} while(0)
#else
#define sdprintk(...)
#endif

struct sdhc_userland_data
{
	/* sdhc device handling */
	dev_t dev_t;
	struct device *dev;
	struct cdev cdev;

	struct semaphore sem;
	struct completion c_req;
	struct completion c_rsp;
	unsigned char buf[PAGE_SIZE];
	int len;

	int once;
} sdhc_userland;

static int open(struct inode *inode, struct file *filp)
{
	struct sdhc_userland_data *d = &sdhc_userland;

	if (!d->once) {
		d->once = 1;
		sdhc_functions_connect();
	}

	return 0;
}

static int release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t read(struct file *filp, char __user *buf, size_t count, loff_t * ppos)
{
	struct sdhc_userland_data *d = &sdhc_userland;
	int ret;

	ret = wait_for_completion_interruptible(&d->c_req);
	if (ret)
		return -EFAULT;

	BUG_ON(d->len >= sizeof(d->buf));

	if (copy_to_user(buf, &d->buf[0], d->len))
		return -EFAULT;

	return d->len;
}

static ssize_t write(struct file *filp, const char __user *buf, size_t count, loff_t * ppos)
{
	struct sdhc_userland_data *d = &sdhc_userland;

	if (copy_from_user(&d->buf[0], buf, count))
		return -EFAULT;

	init_completion(&d->c_req);

	complete(&d->c_rsp);

	return count;
}

static struct file_operations fops = {
	.open = open,
	.release = release,
	.read = read,
	.write = write,
	.owner = THIS_MODULE,
};

static void host_classdev_release(struct device *dev)
{
	/* anything todo here? */
}

static struct class host_class = {
        .name           = "sdhc_userland_host",
        .dev_release    = host_classdev_release,
};

static int register_host_class(void)
{
	return class_register(&host_class);
}

static void unregister_host_class(void)
{
	class_unregister(&host_class);
}

int schedule_request_to_userland(unsigned char cmd, void *ptr, int len)
{
	struct sdhc_userland_data *d = &sdhc_userland;
	int *b;
	int ret;

	/* serialise the access so that only one access is active at a time */
	ret = down_interruptible(&d->sem);
	if (ret)
		return ret;

	b = (int *)&d->buf[0];
	/* store command in first 4 bytes */
	*b++ = cmd;
	/* now copy the remaining structure bytes if any */
	memcpy(b, ptr, len);
	d->len = len + sizeof(unsigned int);

	init_completion(&d->c_rsp);

	complete(&d->c_req);

	ret = wait_for_completion_interruptible(&d->c_rsp);
	if (ret) {
		up(&d->sem);
		return -EIO;
	}

	/* the return code of the function is stored in the first 4 bytes */
	b = (int *)&d->buf[0];
	ret = *b++;

	/* if a data structure was provided, make sure to copy it back just
	in case there were any changes */
	if(len)
		memcpy(ptr, b, len);

	up(&d->sem);

	return ret;
}

#define USERLAND_REQUEST(num, func) \
int func(struct func ## _struct *s) { \
	return schedule_request_to_userland(num, s, sizeof(struct func ## _struct)); \
};

#define USERLAND_REQUEST_NOARGS(num, func) \
int func(void) { \
	return schedule_request_to_userland(num, NULL, 0); \
};

#define USERLAND_REQUEST_NORET(num, func) \
void func(struct func ## _struct *s) { \
	schedule_request_to_userland(num, s, sizeof(struct func ## _struct)); \
};

#define USERLAND_REQUEST_NOARGS_NORET(num, func) \
void func(void) { \
	schedule_request_to_userland(num, NULL, 0); \
};

USERLAND_REQUEST_NOARGS		( 0, sdhc_is_card_present);
USERLAND_REQUEST_NOARGS		( 1, sdhc_is_readonly);
USERLAND_REQUEST_NOARGS_NORET	( 2, sdhc_wait_data_ready);
USERLAND_REQUEST_NORET		( 3, sdhc_setup_ios);
USERLAND_REQUEST_NORET		( 4, sdhc_response_get_16b);
USERLAND_REQUEST_NORET		( 5, sdhc_response_get_8b);
USERLAND_REQUEST_NORET		( 6, sdhc_prepare_data_transfer);
USERLAND_REQUEST_NOARGS_NORET	( 7, sdhc_mrq_stop);
USERLAND_REQUEST_NOARGS_NORET	( 8, sdhc_reset);
USERLAND_REQUEST_NORET		( 9, sdhc_start_cmd_execution);
USERLAND_REQUEST_NOARGS		(10, sdhc_irq_handler);
USERLAND_REQUEST		(11, sdhc_connect);
USERLAND_REQUEST_NOARGS_NORET	(12, sdhc_disconnect);

static int connect_internal(void)
{
	struct sdhc_userland_data *d = &sdhc_userland;
	int ret;

	sema_init(&d->sem, 1);

	init_completion(&d->c_req);
	init_completion(&d->c_rsp);

	ret = register_host_class();
	if(ret) {
		sdprintk("register_host_class() failed\n");
		return -1;
	}

	ret = alloc_chrdev_region(&d->dev_t, 0, 1, "sdhc_userland");
	if (ret < 0) {
		unregister_host_class();
		sdprintk("alloc_chrdev_region() failed\n");
		return -1;
	}

	d->dev = device_create(&host_class, NULL, d->dev_t, NULL, "sdhc%d", 0);
	if (!d->dev) {
		sdprintk("device_create() failed\n");
		unregister_chrdev_region(d->dev_t, 1);
		unregister_host_class();
		return -1;
	}

	cdev_init(&d->cdev, &fops);
	ret = cdev_add(&d->cdev, d->dev_t, 1);
	if(ret < 0) {
		sdprintk("cdev_add() failed\n");
		device_del(d->dev);
		unregister_chrdev_region(d->dev_t, 1);
		unregister_host_class();
		return -1;
	}

	return 0;
}

static void disconnect_internal(struct sdhc_userland_data *d)
{
	sdprintk("\n");
	cdev_del(&d->cdev);
	device_del(d->dev);
	unregister_chrdev_region(d->dev_t, 1);
	unregister_host_class();
}

int sdhc_init(void)
{
	int ret;

	ret = connect_internal();
	if (ret) {
		sdprintk("connect_internal() failed\n");
		return -1;
	}

	sdprintk("\n");
	return 0;
}

void sdhc_deinit(void)
{
	struct sdhc_userland_data *d = &sdhc_userland;
	sdprintk("\n");

	sdhc_disconnect();
	disconnect_internal(d);
}

