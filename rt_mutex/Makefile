# # Kernel build system paths
# KDIR := /lib/modules/$(shell uname -r)/build
# PWD := $(shell pwd)

# # Additional compiler flags
# EXTRA_CFLAGS := -DMODULE -D__KERNEL__ -fno-pic -O2

# # Target module
# obj-m := rt_be_mutex.o

# # Build rules
# all:
# 	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" modules

# clean:
# 	$(MAKE) -C $(KDIR) M=$(PWD) clean
obj-m += rt_be_mutex.o

# This is the key line: Add your module's source dir to include path for this object
CFLAGS_rt_be_mutex.o := -I$(src)

# Your existing EXTRA_CFLAGS can stay, but they're not related to this issue
EXTRA_CFLAGS += -DMODULE -D__KERNEL__ -fno-pic -O2

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean