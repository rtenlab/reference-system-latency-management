# Makefile for the rt_be_mutex test program

CC = gcc
CFLAGS = -Wall -O2 -g
LDFLAGS = -lpthread

TARGET = test
SOURCES = test.c

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)