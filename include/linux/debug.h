/*
 * Copyright 2008, bplan GmbH. All Rights Reserved.
 *
 * Debug header
*/

#ifndef __DEBUG_H__
#define __DEBUG_H__

#if 1 && defined(__DEBUG__)
#include <linux/kernel.h>
		static int NPRINTK_(char *fmt, ...)
		{
			uint32_t buffer_32[256/4];
			char *buffer = (char *) buffer_32;
			int buffer_size;
			int ret;
			va_list ap;

			buffer_size  = sizeof(buffer_32);

			va_start(ap, fmt);
			ret = vsnprintf(buffer, buffer_size, fmt, ap);
			va_end(ap);	

			if(ret>=0)
			{
				printk(buffer);
			}

			return ret;
		}

		#define NPRINTK(fmt, args...) \
		{                                                               \
        NPRINTK_("%s %s()/%d: " fmt, __FILE__, __func__, __LINE__, ## args);     \
		}
	#ifdef pr_debug
	#undef pr_debug
	#endif
	#define pr_debug NPRINTK
#else
#define NPRINTK(fmt, args...)
#endif

#endif
