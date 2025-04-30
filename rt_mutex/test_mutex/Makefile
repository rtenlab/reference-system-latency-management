CC = gcc
CFLAGS = -Wall -Wextra -O2 -pthread
TARGETS = test_mutex basic_test

all: $(TARGETS)

test_mutex: test_mutex.o
	$(CC) $(CFLAGS) -o test_mutex test_mutex.o

basic_test: basic_test.o
	$(CC) $(CFLAGS) -o basic_test basic_test.o

test_mutex.o: test_mutex.c
	$(CC) $(CFLAGS) -c test_mutex.c

basic_test.o: basic_test.c
	$(CC) $(CFLAGS) -c basic_test.c

clean:
	rm -f $(TARGETS) test_mutex.o basic_test.o

.PHONY: all clean