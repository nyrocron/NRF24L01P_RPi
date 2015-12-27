/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include "nrf24l01.h"

#define RX_TIMEOUT 5000
#define RXADDR 0x0a0b0c0d0e

volatile sig_atomic_t flag = 0;
void sig_handler(int sig) {
	flag = 1;
}

int main(int argc, char **argv)
{
	int ret;

	FILE *outfile;
	if (argc > 1) {
		outfile = fopen(argv[1], "w");
		if (outfile == NULL) {
			perror("filesend outfile fopen()");
			ret = errno;
			goto out;
		}
	} else {
		outfile = stdout;
	}

	/* register signal handler for keyboard interrupt */
	signal(SIGINT, sig_handler);

	struct nrf24l01 dev = {
		.spi = "/dev/spidev0.0",
		.ce = 22,
		.cs = 25,
	};

	if (rf_init(&dev) < 0) {
		fprintf(stderr, "rf_init fail\n");
		ret = 1;
		goto out_fclose;
	}
	
	if (rf_rx_addr(&dev, RXADDR) < 0) {
		fprintf(stderr, "rx_addr fail\n");
		ret = 2;
		goto out_rfclose;
	}

	ret = rf_file_recv(&dev, outfile, RX_TIMEOUT);

out_rfclose:
	rf_shutdown(&dev);

out_fclose:
	/* only close outfile if we are not using stdout */
	if (argc > 1)
		fclose(outfile);

out:
	return ret;
}
