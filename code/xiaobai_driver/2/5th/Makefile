obj-m += test_haha.o
test_haha-objs += haha.o test.o

KDIR:=/root/Desktop/drives/nfsroot-29/linux-2.6.29
all:
	make -C $(KDIR) M=`pwd` modules
clean:
	make -C $(KDIR) M=`pwd` modules clean
	rm -f modules.order

