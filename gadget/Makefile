obj-m += vgadget.o

LINUX_BUILD_DIR=/lib/modules/$(shell uname -r)/build

all:
	make -C $(LINUX_BUILD_DIR) M=$(PWD) modules

clean:
	make -C $(LINUX_BUILD_DIR) M=$(PWD) clean
