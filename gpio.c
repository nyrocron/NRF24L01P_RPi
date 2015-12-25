/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <stdint.h>

#include "gpio.h"

#define TAG "GPIO "

int write_int_to_file(const char *filename, int value)
{
	int ret;

	int fd = open(filename, O_WRONLY);
	if (fd < 0) {
		ret = -errno;
		goto out;
	}

	ret = dprintf(fd, "%d", value);
	if (ret < 0)
		goto out_close;
	
out_close:
	if (close(fd) < 0)
		ret = -errno;
out:
	return ret;
}

int gpio_export(int gpio)
{
	return write_int_to_file("/sys/class/gpio/export", gpio);
}

int gpio_unexport(int gpio)
{
	return write_int_to_file("/sys/class/gpio/unexport", gpio);
}

int gpio_set_mode(int gpio, char mode)
{
	int ret;
	char filename[128];

	ret = snprintf(filename, sizeof(filename),
			"/sys/class/gpio/gpio%d/direction", gpio);
	if (ret < 0)
		goto out;

	int fd = open(filename, O_WRONLY);
	if (fd < 0) {
		ret = -errno;
		goto out;
	}

	switch (mode) {
	case MODE_IN:
		if (write(fd, "in", 2) != 2) {
			ret = -errno;
			goto out_close;
		}
		break;
	case MODE_OUT:
		if (write(fd, "out", 3) != 3) {
			ret = -errno;
			goto out_close;
		}
		break;
	
	default:
		ret = -EINVAL;
	}

out_close:
	if (close(fd) < 0)
		ret = -errno;
out:
	return ret;
}

int gpio_set_irq(int gpio, char mode)
{
	int ret;
	char filename[128];

	ret = snprintf(filename, sizeof(filename),
			"/sys/class/gpio/gpio%d/edge", gpio);
	if (ret < 0)
		goto out;

	int fd = open(filename, O_WRONLY);
	if (fd < 0) {
		ret = -errno;
		goto out;
	}

	switch (mode) {
	case IRQ_NONE:
		if (write(fd, "none", 4) != 4) {
			ret = -errno;
			goto out_close;
		}
		break;
	case IRQ_RISING:
		if (write(fd, "rising", 6) != 6) {
			ret = -errno;
			goto out_close;
		}
		break;
	case IRQ_FALLING:
		if (write(fd, "falling", 7) != 7) {
			ret = -errno;
			goto out_close;
		}
		break;
	case IRQ_BOTH:
		if (write(fd, "both", 4) != 4) {
			ret = -errno;
			goto out_close;
		}
		break;
	
	default:
		ret = -EINVAL;
	}

out_close:
	if (close(fd) < 0)
		ret = -errno;
out:
	return ret;
}

int gpio_read(int gpio)
{
	int ret;
	char filename[128];

	ret = snprintf(filename, sizeof(filename),
			"/sys/class/gpio/gpio%d/value", gpio);
	if (ret < 0)
		goto out;
	
	int fd = open(filename, O_RDONLY);
	if (fd < 0) {
		ret = -errno;
		goto out;
	}

	char value;
	if (read(fd, &value, sizeof(value)) != sizeof(value)) {
		ret = -errno;
		goto out_close;
	}

	ret = (value == '1' ? 1 : 0);

out_close:
	if (close(fd) < 0)
		ret = -errno;
out:
	return ret;
}

int gpio_write(int gpio, int value)
{
	int ret;
	char filename[128];

	ret = snprintf(filename, sizeof(filename),
			"/sys/class/gpio/gpio%d/value", gpio);
	if (ret < 0)
		goto out;

	ret = write_int_to_file(filename, value);

out:
	return ret;
}

int gpio_poll(int gpio, char targetstate, int timeout)
{
	int ret;
	char filename[128];
	char pinstate;

	ret = snprintf(filename, sizeof(filename),
			"/sys/class/gpio/gpio%d/value", gpio);
	if (ret < 0)
		goto out;
	
	int fd = open(filename, O_RDONLY);
	if (fd < 0) {
		ret = -errno;
		perror(TAG "poll_open");
		goto out;
	}
	read(fd, &pinstate, sizeof(pinstate));
	if (pinstate == targetstate) {
#ifdef DEBUG
		fprintf(stderr, TAG "irq already happened\n");
#endif
		ret = pinstate;
		goto out_close;
	}

#ifdef DEBUG
	fprintf(stderr, TAG "irq pinstate %c\n", pinstate);
#endif

	struct pollfd pollfd = {
		.fd = fd,
		.events = POLLPRI,
		.revents = 0,
	};
	ret = poll(&pollfd, 1, timeout);
	if (ret <= 0) {
#ifdef DEBUG
		perror(TAG "poll()");
		fprintf(stderr, TAG "poll fail or timeout (ret=%d)\n", ret);
#endif
		ret = -1;
		goto out_close;
	}
#ifdef DEBUG
	fprintf(stderr, TAG "poll ret %d\n", ret);
	fprintf(stderr, TAG "poll events %x (POLLPRI=%x)\n",
			pollfd.revents, POLLPRI);
#endif

#ifdef DEBUG
	if (pollfd.revents & POLLIN)
		fprintf(stderr, TAG "POLLIN\n");
	if (pollfd.revents & POLLPRI)
		fprintf(stderr, TAG "POLLPRI\n");
	if (pollfd.revents & POLLOUT)
		fprintf(stderr, TAG "POLLOUT\n");
	if (pollfd.revents & POLLERR)
		fprintf(stderr, TAG "POLLERR\n");
	if (pollfd.revents & POLLHUP)
		fprintf(stderr, TAG "POLLHUP\n");
	if (pollfd.revents & POLLNVAL)
		fprintf(stderr, TAG "POLLNVAL\n");
#endif

	if (pollfd.revents & POLLPRI) {
		read(fd, &pinstate, sizeof(pinstate));
#ifdef DEBUG
		fprintf(stderr, TAG "pollpri pinstate %c\n", pinstate);
#endif
		ret = pinstate;
	} else {
		ret = -1;
#ifdef DEBUG
		fprintf(stderr, TAG "poll events %x\n", pollfd.revents);
#endif
	}

out_close:
	close(fd);
out:
	return ret;
}
