LINUX_DIR=/lib/modules/$(shell uname -r)/build
GADGET_DIR=$(LINUX_DIR)/drivers/usb/gadget

g_vgadget-objs := vgadget.o usbstring.o config.o epautoconf.o
obj-m += g_vgadget.o

EXTRA_CFLAGS += -I$(GADGET_DIR)
EXTRA_CFLAGS += $(OTHER_CFLAGS)

all: usbstring.c config.c epautoconf.c
	make -C $(LINUX_DIR) M=$(PWD) modules

epautoconf.c:
	cp $(GADGET_DIR)/epautoconf.c $(PWD)/

usbstring.c:
	cp $(GADGET_DIR)/usbstring.c $(PWD)/

config.c:
	cp $(GADGET_DIR)/config.c $(PWD)/

clean:
	make -C $(LINUX_DIR) M=$(PWD) clean
	[ ! -f epautoconf.c ] || rm epautoconf.c
	[ ! -f usbstring.c ] || rm usbstring.c
	[ ! -f config.c ] || rm config.c
	[ ! -f Module.symvers ] || rm Module.symvers

install:
	make -C $(LINUX_DIR) M=$(PWD) modules_install
