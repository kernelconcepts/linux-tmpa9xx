#
# Makefile for ALSA
#
obj-$(CONFIG_SND_TMPA9XX_PCM1773) += snd-tmpa9xx-pcm1773.o 
snd-tmpa9xx-pcm1773-objs	  := pcm1773.o

obj-$(CONFIG_SND_TMPA9XX_WM89XX) += snd-tmpa9xx-wm89xx.o 
snd-tmpa9xx-wm89xx-objs		 := wm89xx.o

obj-$(CONFIG_SND_TMPA9XX_I2S)	+= snd-tmpa9xx-i2s.o 

obj-$(CONFIG_SND_ARMAACI)	+= snd-aaci.o
snd-aaci-objs			:= aaci.o

obj-$(CONFIG_SND_PXA2XX_PCM)	+= snd-pxa2xx-pcm.o
snd-pxa2xx-pcm-objs		:= pxa2xx-pcm.o

obj-$(CONFIG_SND_PXA2XX_LIB)	+= snd-pxa2xx-lib.o
snd-pxa2xx-lib-y		:= pxa2xx-pcm-lib.o
snd-pxa2xx-lib-$(CONFIG_SND_PXA2XX_LIB_AC97)	+= pxa2xx-ac97-lib.o

obj-$(CONFIG_SND_PXA2XX_AC97)	+= snd-pxa2xx-ac97.o
snd-pxa2xx-ac97-objs		:= pxa2xx-ac97.o
