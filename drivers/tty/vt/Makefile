# SPDX-License-Identifier: GPL-2.0
#
# This file contains the font map for the default (hardware) font
#
ifeq ($(CONFIG_FONT_PRAGMATAPRO),y)
FONTMAPFILE = $(obj)/pragmatapro.uni
else
FONTMAPFILE = $(src)/cp437.uni
endif

obj-$(CONFIG_VT)			+= vt_ioctl.o vc_screen.o \
					   selection.o keyboard.o
obj-$(CONFIG_CONSOLE_TRANSLATIONS)	+= consolemap.o consolemap_deftbl.o
obj-$(CONFIG_HW_CONSOLE)		+= vt.o defkeymap.o

# Files generated that shall be removed upon make clean
clean-files := consolemap_deftbl.c defkeymap.c pragmatapro.uni

hostprogs += conmakehash

quiet_cmd_conmk = CONMK   $@
      cmd_conmk = $(obj)/conmakehash $< > $@

$(obj)/consolemap_deftbl.c: $(FONTMAPFILE) $(obj)/conmakehash
	$(call cmd,conmk)

$(obj)/defkeymap.o:  $(obj)/defkeymap.c

# Uncomment if you're changing the keymap and have an appropriate
# loadkeys version for the map. By default, we'll use the shipped
# versions.
# GENERATE_KEYMAP := 1

ifdef GENERATE_KEYMAP

$(obj)/defkeymap.c: $(obj)/%.c: $(src)/%.map
	loadkeys --mktable --unicode $< > $@

endif
