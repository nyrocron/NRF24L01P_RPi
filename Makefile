CC = gcc --std=gnu11
CFLAGS = -Wall -g

OBJECTS = nrf24l01.o gpio.o
HEADERS = nrf24l01.h gpio.h

BINARIES = sender receiver filesend filerecv

default: $(BINARIES)

clean:
	rm -rf *.o $(BINARIES)

sender: sender.c $(OBJECTS) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $< $(OBJECTS)

receiver: receiver.c $(OBJECTS) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $< $(OBJECTS)

filesend: filesend.c $(OBJECTS) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $< $(OBJECTS)

filerecv: filerecv.c $(OBJECTS) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $< $(OBJECTS)

nrf24l01.o: nrf24l01.c nrf24l01.h gpio.h
	$(CC) $(CFLAGS) -o $@ -c $<

gpio.o: gpio.c gpio.h
	$(CC) $(CFLAGS) -o $@ -c $<


.PHONY: clean
