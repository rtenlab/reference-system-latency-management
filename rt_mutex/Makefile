# Kernel build system paths
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# Additional compiler flags
EXTRA_CFLAGS := -DMODULE -D__KERNEL__ -fno-pic -O2

# Target module
obj-m := rt_be_mutex.o

# Build rules
all:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean