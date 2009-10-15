#
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

# Object file lists.

obj-y                   += irq.o time.o gpio.o led-topas910.o dma.o
obj-$(CONFIG_MACH_TOPAS910)    += topas910.o
obj-$(CONFIG_MACH_TOPASA900)   += topasa900.o
