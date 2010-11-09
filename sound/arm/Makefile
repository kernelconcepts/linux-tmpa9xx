#
# Makefile for ALSA
#

obj-$(CONFIG_SND_TMPA9XX_WM8976) += snd-tmpa9xx-wm8976.o 
snd-tmpa9xx-wm8976-objs		:= tmpa9xx_i2s.o wm8976.o

obj-$(CONFIG_SND_TMPA9XX_WM8983) += snd-tmpa9xx-wm8983.o 
snd-tmpa9xx-wm8983-objs		:= tmpa9xx_i2s.o wm8983.o

obj-$(CONFIG_SND_TMPA9XX_PCM1773) += snd-tmpa9xx-pcm1773.o 
snd-tmpa9xx-pcm1773-objs	:= tmpa9xx_i2s.o pcm1773.o

obj-$(CONFIG_SND_ARMAACI)	+= snd-aaci.o
snd-aaci-objs			:= aaci.o

obj-$(CONFIG_SND_PXA2XX_PCM)	+= snd-pxa2xx-pcm.o
snd-pxa2xx-pcm-objs		:= pxa2xx-pcm.o

obj-$(CONFIG_SND_PXA2XX_LIB)	+= snd-pxa2xx-lib.o
snd-pxa2xx-lib-y		:= pxa2xx-pcm-lib.o
snd-pxa2xx-lib-$(CONFIG_SND_PXA2XX_LIB_AC97)	+= pxa2xx-ac97-lib.o

obj-$(CONFIG_SND_PXA2XX_AC97)	+= snd-pxa2xx-ac97.o
snd-pxa2xx-ac97-objs		:= pxa2xx-ac97.o
