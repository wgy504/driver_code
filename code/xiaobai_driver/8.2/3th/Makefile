obj-m += device.o device1.o device2.o driver.o driver1.o bus.o

KDIR := /root/Desktop/drives/nfsroot-29/linux-2.6.29
all:
	make -C $(KDIR) M=`pwd` modules
clean:
	make -C $(KDIR) M=`pwd` modules clean
	rm -f modules.order
