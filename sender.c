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
	
	if (rf_tx_addr(&dev, 0x0a0b0c0d0e) < 0) {
		fprintf(stderr, "tx_addr fail\n");
		goto out;
	}

	while (!flag) {
		if (rf_send(&dev, "Hello", 5)) {
			printf("send failed\n");
		} else {
			printf("send ok\n");
		}

		usleep(1000 * 1000);
	}

out:
	rf_shutdown(&dev);
	return 0;
}
