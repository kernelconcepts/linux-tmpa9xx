#
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

# Object file lists.

obj-y				        += irq.o time.o gpio.o dma.o clock.o ofd.o tmpa9xx-generic.o
obj-$(CONFIG_HAVE_PWM)		        += pwm.o
obj-$(CONFIG_MACH_TOPAS910)	        += topas910.o baseboard-topas910.o led-topasbb.o 
obj-$(CONFIG_MACH_TOPASA900)	        += topasa900.o baseboard-topasa900.o led-topasbb.o 
obj-$(CONFIG_MACH_TONGA)	        += tonga.o baseboard-glyn.o
obj-$(CONFIG_MACH_TONGA2_TFTTIMER)	+= tfttimer.o baseboard-tfttimer.o
