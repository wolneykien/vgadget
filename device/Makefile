LINUX_DIR=/usr/src/linux-$(shell uname -r)

obj-m += vdevice.o

all:
	make -C $(LINUX_DIR) M=$(PWD) modules

clean:
	make -C $(LINUX_DIR) M=$(PWD) clean
	[ ! -f Module.symvers ] || rm Module.symvers
	[ ! -f modules.order ] || rm modules.order

install:
	make -C $(LINUX_DIR) M=$(PWD) modules_install
