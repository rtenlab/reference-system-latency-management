CC = gcc
CFLAGS = -Wall -Wextra -O2
TARGET = test_mutex

all: $(TARGET)

$(TARGET): test_mutex.o
	$(CC) $(CFLAGS) -o $(TARGET) test_mutex.o

test_mutex.o: test_mutex.c
	$(CC) $(CFLAGS) -c test_mutex.c

clean:
	rm -f $(TARGET) test_mutex.o

.PHONY: all clean