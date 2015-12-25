/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#include <stdio.h>

#include "nrf24l01.h"

int main(int argc, char **argv)
{
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
	
	printf("CONFIG: %02x\n", rf_reg_read(&dev, REG_CONFIG));
	
	char msg[32];
	if (rf_receive(&dev, msg, sizeof(msg), 3 * 1000)) {
		fprintf(stderr, "receive failed\n");
		goto out;
	}

	printf("received message: %s\n", msg);

out:
	rf_shutdown(&dev);
	return 0;
}
