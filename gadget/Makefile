LINUX_DIR=/lib/modules/$(shell uname -r)/build
#GADGET_DIR=$(LINUX_DIR)/drivers/usb/gadget

obj-m += vgadget.o
#vgadget-objs := usbstring.o config.o epautoconf.o


all:
	make -C $(LINUX_DIR) M=$(PWD) modules

clean:
	make -C $(LINUX_DIR) M=$(PWD) clean
