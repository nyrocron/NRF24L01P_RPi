/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "nrf24l01.h"
#include "gpio.h"

int rf_command(struct nrf24l01 *dev, uint8_t cmd, void *buf, size_t len,
		int read)
{
	int ret;

	if (len > CMD_MAXLEN)
		len = CMD_MAXLEN;

	size_t buffer_len = len + 1;
	uint8_t txbuf[buffer_len];
	uint8_t rxbuf[buffer_len];
	if (!read && len > 0)
		memcpy(&txbuf[1], buf, len);
	
	txbuf[0] = cmd;

	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = (unsigned long)txbuf;
	xfer.rx_buf = (unsigned long)rxbuf;
	xfer.len = buffer_len;
	xfer.speed_hz = SPI_SPEED;
	xfer.bits_per_word = 8;
	xfer.cs_change = 1;

	int fd = open(dev->spi, O_RDWR);
	if (fd < 0) {
		perror(TAG "cmd open");
		ret = -1;
		goto out;
	}

	gpio_write(dev->cs, 0);
	usleep(10);

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
	if (ret < 0) {
		perror(TAG "cmd ioctl");
		goto out_close;
	}

	usleep(10);
	gpio_write(dev->cs, 1);

	if (read && len > 0)
		memcpy(buf, &rxbuf[1], len);

	ret = rxbuf[0];

out_close:
	close(fd);
out:
	return ret;
}

uint8_t rf_reg_read(struct nrf24l01 *dev, uint8_t addr)
{
	uint8_t value;
	if (rf_command(dev, R_REGISTER | addr, &value, sizeof(value), 1) < 0)
		return 0xff;
	return value;
}

int rf_reg_write(struct nrf24l01 *dev, uint8_t addr, uint8_t value)
{
	return rf_command(dev, W_REGISTER | addr, &value, sizeof(value), 0);
}

int rf_receive(struct nrf24l01 *dev, void *buf, size_t len,
		unsigned int timeout)
{
	int ret;

	uint8_t conf = rf_reg_read(dev, REG_CONFIG);
	if (conf == 0xff) {
		ret = -1;
		goto out;
	}

	conf |= CONF_PRIM_RX;

	ret = rf_reg_write(dev, REG_CONFIG, conf);
	if (ret < 0)
		goto out;

#ifdef DEBUG
	/* verify config */
	conf = rf_reg_read(dev, REG_CONFIG);
	if (!(conf & CONF_PRIM_RX)) {
		fprintf(stderr, TAG "RX CONF invalid %02x\n", conf);
		ret = -1;
		goto out;
	}
#endif
	
	struct timespec start, now;
	uint64_t millis = 0;
	clock_gettime(CLOCK_MONOTONIC, &start);

	uint8_t status = 0;
	gpio_write(dev->ce, 1);
	while (millis < timeout) {
		ret = rf_command(dev, NOP, NULL, 0, 0);
		if (ret < 0)
			goto out;
		status = ret & 0xff;
		if (status & RX_DR)
			break;

		usleep(1000);
		clock_gettime(CLOCK_MONOTONIC, &now);
		millis = (now.tv_sec - start.tv_sec) * SEC_MILLIS +
				(now.tv_nsec - start.tv_nsec) / MILLI_NSECS;
	}
	gpio_write(dev->ce, 0);

	/* handle successful receive */
	if (status & RX_DR) {
		ret = rf_command(dev, R_RX_PAYLOAD, buf, len, 1);
		if (ret < 0)
			goto out;

		ret = 0;
		goto out;
	}

	ret = 1; /* timeout */

out:
	return ret;
}

static int rf_pinsetup(struct nrf24l01 *dev)
{
	int ret;

	ret = gpio_export(dev->ce);
	if (ret < 0)
		goto out;

	ret = gpio_set_mode(dev->ce, MODE_OUT);
	if (ret < 0)
		goto out_unex_ce;

	ret = gpio_export(dev->cs);
	if (ret < 0)
		goto out_reset_ce;

	ret = gpio_set_mode(dev->cs, MODE_OUT);
	if (ret < 0)
		goto out_unex_cs;

	return ret;

out_unex_cs:
	gpio_unexport(dev->cs);
out_reset_ce:
	gpio_set_mode(dev->ce, MODE_IN);
out_unex_ce:
	gpio_unexport(dev->ce);
out:
	return ret;
}

int rf_init(struct nrf24l01 *dev)
{
	int ret;

	ret = rf_pinsetup(dev);
	if (ret < 0)
		goto out;

	usleep(5 * 1000);

	ret = rf_reg_write(dev, REG_CONFIG, 0b00001100);
	if (ret < 0)
		goto out;

	ret = rf_reg_write(dev, REG_SETUP_RETR, 0b01011010);
	if (ret < 0)
		goto out;

	ret = rf_reg_write(dev, REG_RF_SETUP, 0b00000000);
	if (ret < 0)
		goto out;

	ret = rf_reg_write(dev, REG_FEATURE, 0b00000000);
	if (ret < 0)
		goto out;

	ret = rf_reg_write(dev, REG_DYNPD, 0b00000000);
	if (ret < 0)
		goto out;

	ret = rf_reg_write(dev, REG_STATUS, IRQ_MASK);
	if (ret < 0)
		goto out;

	ret = rf_reg_write(dev, REG_RF_CH, 76);
	if (ret < 0)
		goto out;

	ret = rf_command(dev, FLUSH_RX, NULL, 0, 0);
	if (ret < 0)
		goto out;

	ret = rf_command(dev, FLUSH_TX, NULL, 0, 0);
	if (ret < 0)
		goto out;

	/* power up and wait */
	ret = rf_reg_write(dev, REG_CONFIG, 0b00001110);
	if (ret < 0)
		goto out;
	usleep(5 * 1000);

out:
	return ret;
}

void rf_shutdown(struct nrf24l01 *dev)
{
	rf_reg_write(dev, REG_CONFIG, 0b00001100); /* power down */
	gpio_unexport(dev->ce);
	gpio_unexport(dev->cs);
}

int rf_rx_addr(struct nrf24l01 *dev, uint64_t addr)
{
	int ret;

	ret = rf_reg_write(dev, REG_EN_RXADDR, 0b00000001);
	if (ret < 0)
		goto out;

	ret = rf_reg_write(dev, REG_EN_AA, 0b00000001);
	if (ret < 0)
		goto out;

	ret = rf_reg_write(dev, REG_RX_PW_P0, PACKET_LEN);
	if (ret < 0)
		goto out;

	ret = rf_reg_writelong(dev, REG_RX_ADDR_P0, &addr, 5);
	if (ret < 0)
		goto out;
	
	return 0;

out:
	return ret;
}

int rf_reg_writelong(struct nrf24l01 *dev, uint8_t addr,
		void *buf, size_t len)
{
	return rf_command(dev, W_REGISTER | addr, buf, len, 0);
}
