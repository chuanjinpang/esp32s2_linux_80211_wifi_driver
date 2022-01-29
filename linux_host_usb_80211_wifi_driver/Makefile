######################################
#
# usb wifi drv for esp
#
######################################


DRIVER_NAME := xfz_usb_wifi
KERNEL_SOURCE_DIR ?= /lib/modules/`uname -r`/build

EXTRA_CFLAGS +=-g -I$(PWD)/src -I$(PWD)/../common

obj-m := $(DRIVER_NAME).o

DRIVER_FILES := main.o cfg80211.o  \
                usb_link.o msg.o link_glue.o
CFLAGS_main.o = -O0

$(DRIVER_NAME)-objs:= $(DRIVER_FILES)

modules:
	$(MAKE) -C $(KERNEL_SOURCE_DIR) KCPPFLAGS="$(EXTRA_CFLAGS)" M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_SOURCE_DIR) M=$(PWD) modules_install

install: modules_install

clean:
	$(MAKE) -C $(KERNEL_SOURCE_DIR) M=$(PWD) clean
