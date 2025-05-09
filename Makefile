obj-m += Driver_BMP180.o
KDIR = /lib/modules/$(shell uname -r)/build

all: 
	make -C $(KDIR) M=$(shell pwd) modules
	sudo insmod Driver_BMP180.ko
	gcc Test_Driver_BMP180.c -o run
	@for i in $$(seq 1 10); do \
		echo "⏱️  Number: $$i:"; \
		sudo ./run; \
		sleep 1; \
	done



clean: 
	rm run
	sudo rmmod Driver_BMP180
	make -C $(KDIR) M=$(shell pwd) clean

