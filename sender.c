/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#include <stdio.h>
#include <string.h>
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
	
	if (rf_tx_addr(&dev, 0x0a0b0c0d0e) < 0) {
		fprintf(stderr, "tx_addr fail\n");
		goto out;
	}

	if (rf_speed(&dev, SPEED_250K) < 0) {
		fprintf(stderr, "rf_speed fail\n");
		goto out;
	}

	/*
	char *msg = "Hello";
	struct rf_packet packet = {
		.flags = FT_DATA,
		.dlen = strlen(msg),
	};
	memset(&packet.data, 0, sizeof(packet.data));
	strcpy(&packet.data, msg);
	*/

	struct rf_packet pack1 = {
		.flags = FT_DATA,
		.dlen = 14,
		.data = "Hello, this is",
	};
	struct rf_packet pack2 = {
		.flags = FT_DATA | FT_END,
		.dlen = 12,
		.data = " a test msg.",
	};

	int ret;
	while (!flag) {
		usleep(500 * 1000);

		ret = rf_packet_send(&dev, &pack1);
		if (ret) {
			printf("pack1 failed\n");
			continue;
		}

		ret = rf_packet_send(&dev, &pack2);
		if (ret) {
			printf("pack2 failed\n");
			continue;
		}

		printf("ok\n");
	}

out:
	rf_shutdown(&dev);
	return 0;
}
