
obj-m := plat_dev1.o plat_dev2.o raw_drv.o

#KERNEL := /lib/modules/`uname -r`/build
KERNEL := /mnt/hgfs/system/linux-3.2-g90b_mini
all:
	make -C $(KERNEL) M=`pwd` modules
install:
	make -C $(KERNEL) M=`pwd` modules_install
	depmod -A
clean:
	make -C $(KERNEL) M=`pwd` clean
