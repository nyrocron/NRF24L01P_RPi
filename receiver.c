/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "nrf24l01.h"

volatile sig_atomic_t flag = 0;
void sig_handler(int sig) {
	flag = 1;
}

int main(int argc, char **argv)
{
	signal(SIGINT, sig_handler);

	struct nrf24l01 dev = {
		.spi = "/dev/spidev0.0",
		.ce = 22,
		.cs = 25,
	};

	if (rf_init(&dev) < 0) {
		fprintf(stderr, "rf_init fail\n");
		goto out;
	}
	
	if (rf_rx_addr(&dev, 0x0a0b0c0d0e) < 0) {
		fprintf(stderr, "rx_addr fail\n");
		goto out;
	}

	if (rf_speed(&dev, SPEED_250K) < 0) {
		fprintf(stderr, "rf_speed fail\n");
		goto out;
	}
	
	struct rf_packet packet;

	while (!flag) {
		if (rf_packet_recv(&dev, &packet, 2000))
			printf("timeout\n");
		else {
			printf("received packet");
			for (int i = 0; i < PACKET_LEN; i++) {
				if (i % 16 == 0)
					printf("\n  ");
				printf("%02x", ((uint8_t*)&packet)[i]);
			}
			printf("\n  FLAGS=%02x DLEN=%d\n", packet.flags,
					packet.dlen);
		}
	}

out:
	rf_shutdown(&dev);
	return 0;
}
