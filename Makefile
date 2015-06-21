cflags-y +=-I/mnt/linux/include
obj-m += dscmod.o

ifdef CREAD
    ccflags-y += -DCREAD=${CREAD}
endif

all: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules 

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
