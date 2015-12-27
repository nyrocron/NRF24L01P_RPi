/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

#include "nrf24l01.h"

#define TXADDR 0x0a0b0c0d0e

volatile sig_atomic_t flag = 0;
void sig_handler(int sig) {
	flag = 1;
}

void timespec_diff(struct timespec *result,
		struct timespec *start, struct timespec *stop)
{
	if ((stop->tv_nsec - start->tv_nsec) < 0) {
		result->tv_sec = stop->tv_sec - start->tv_sec - 1;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
	} else {
		result->tv_sec = stop->tv_sec - start->tv_sec;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec;
	}

	return;
}

int main(int argc, char **argv)
{
	int ret;

	FILE *infile;
	if (argc > 1) {
		infile = fopen(argv[1], "r");
		if (infile == NULL) {
			perror("filesend infile fopen()");
			ret = errno;
			goto out;
		}
	} else {
		infile = stdin;
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
	
	if (rf_tx_addr(&dev, TXADDR) < 0) {
		fprintf(stderr, "tx_addr fail\n");
		ret = 2;
		goto out_rfclose;
	}

	struct timespec start, end, delta;
	clock_gettime(CLOCK_MONOTONIC, &start);
	ret = rf_file_send(&dev, infile);
	clock_gettime(CLOCK_MONOTONIC, &end);
	timespec_diff(&delta, &start, &end);

	printf("done sending after %0ld.%09lds\n",
			delta.tv_sec, delta.tv_nsec);

out_rfclose:
	rf_shutdown(&dev);

out_fclose:
	/* only close infile if we are not using stdin */
	if (argc > 1)
		fclose(infile);

out:
	return ret;
}
