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
	
	struct rf_packet packet;

	while (!flag) {
		if (rf_packet_recv(&dev, &packet, 2000))
			printf("timeout\n");
		else {
			printf("received packet\n");
			
			for (int ln = 0; ln < 4; ln++) {
				printf("  ");
				for (int i = 0; i < 8; i++) {
					printf("%02x",
						((uint8_t*)&packet)[ln*8+i]);
				}
				printf("\n");
			}

			printf("  FLAGS=%02x DLEN=%d SEQ=%d\n", packet.flags,
					packet.dlen, packet.seq);
		}
	}

out:
	rf_shutdown(&dev);
	return 0;
}
