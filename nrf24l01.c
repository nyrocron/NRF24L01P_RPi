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
		perror(TAG "cmd open()");
		ret = -1;
		goto out;
	}

	gpio_write(dev->cs, 0);
	usleep(CSDELAY);

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
	if (ret < 0) {
		perror(TAG "cmd ioctl()");
		goto out_close;
	}

	usleep(CSDELAY);
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

	/* first, check if there already is data in RX FIFO */
	/* TODO */
/*
	uint8_t fifostat = rf_reg_read(dev, REG_FIFO_STATUS);
	if (!(fifostat & RX_EMPTY)) {
	}
*/

	uint8_t conf = rf_reg_read(dev, REG_CONFIG);
	if (conf == 0xff) {
#ifdef DEBUG
		fprintf(stderr, TAG "recv cfg read error\n");
#endif
		ret = -1;
		goto out;
	}

	ret = rf_reg_write(dev, REG_CONFIG, conf | CONF_PRIM_RX);
	if (ret < 0)
		goto out;

	/* start receiving and wait for the data ready flag */
	gpio_write(dev->ce, 1);
	ret = rf_wait_status(dev, RX_DR, timeout);
	gpio_write(dev->ce, 0);
	if (ret < 0) {
#ifdef DEBUG
		fprintf(stderr, TAG "recv wait_status error\n");
#endif
		goto out; /* timeout */
	}
	
	/* handle successful receive */
	ret = rf_command(dev, R_RX_PAYLOAD, buf, len, 1);
	if (ret < 0) {
#ifdef DEBUG
		fprintf(stderr, TAG "recv R_RX_PAYLOAD error\n");
#endif
		goto out;
	}

	/* clear interrupt flag */
	ret = rf_reg_write(dev, REG_STATUS, RX_DR);
	if (ret < 0)
		goto out;

	ret = 0;

out:
	/* unset RX mode */
	rf_reg_write(dev, REG_CONFIG, conf & ~CONF_PRIM_RX);
	return ret;
}

uint8_t txbuffer[PACKET_LEN];

int rf_send(struct nrf24l01 *dev, void *buf, size_t len)
{
	int ret;

	/* ensure we're in TX mode */
	uint8_t conf;
	conf = rf_reg_read(dev, REG_CONFIG);
	if (conf == 0xff) {
#ifdef DEBUG
		fprintf(stderr, TAG "rf_send cfg read failed\n");
#endif
		ret = -1;
		goto out;
	}
	ret = rf_reg_write(dev, REG_CONFIG, conf & ~CONF_PRIM_RX);
	if (ret < 0) {
#ifdef DEBUG
		fprintf(stderr, TAG "rf_send cfg write failed\n");
#endif
		goto out;
	}
	
	/* apply padding and write data to FIFO */
	uint8_t *txbuf;
	if (len < PACKET_LEN) {
		memcpy(txbuffer, buf, len);
		memset(&txbuffer[len], 0, sizeof(txbuffer) - len);
		txbuf = txbuffer;
	} else {
		txbuf = buf;
	}
	ret = rf_command(dev, W_TX_PAYLOAD, txbuf, PACKET_LEN, 0);
	if (ret < 0) {
#ifdef DEBUG
		fprintf(stderr, TAG "rf_send txpayload write failed\n");
#endif
		goto out;
	}
	
	/* initiate transmission */
	gpio_write(dev->ce, 1);
	usleep(10);
	gpio_write(dev->ce, 0);

	/* wait for transmission to complete */
	ret = rf_wait_status(dev, TX_DS | MAX_RT, TX_TIMEOUT);
	if (ret < 0) {
#ifdef DEBUG
		fprintf(stderr, TAG "rf_send timeout\n");
#endif
		goto out;
	}
	uint8_t status = ret;
	/* clear interrupt flags */
	ret = rf_reg_write(dev, REG_STATUS, TX_DS | MAX_RT);
	if (ret < 0)
		goto out;

	if (status & MAX_RT) {
#ifdef DEBUG
		fprintf(stderr, TAG "rf_send MAX_RT\n");
#endif
		ret = -1;
		goto out;
	}
	if (status & TX_DS) {
		ret = 0;
		goto out;
	}
#ifdef DEBUG
	fprintf(stderr, TAG "rf_send INVALID STATE\n");
#endif
	ret = -1;

out:
	return ret;
}

/* Helper for RF module GPIO setup */
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

	ret = rf_reg_write(dev, REG_RF_SETUP, 0b00001000);
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
	
	ret = 0;

out:
	return ret;
}

int rf_tx_addr(struct nrf24l01 *dev, uint64_t addr)
{
	int ret;

	ret = rf_reg_writelong(dev, REG_TX_ADDR, &addr, 5);
	if (ret < 0)
		goto out;

	ret = rf_reg_writelong(dev, REG_RX_ADDR_P0, &addr, 5);
	if (ret < 0)
		goto out;
	
	ret = 0;

out:
	return ret;
}

int rf_reg_writelong(struct nrf24l01 *dev, uint8_t addr,
		void *buf, size_t len)
{
	return rf_command(dev, W_REGISTER | addr, buf, len, 0);
}

/* difference between two timespecs in milliseconds */
static uint64_t timespec_diff(struct timespec *ts, struct timespec *te)
{
	struct timespec delta;
	if (te->tv_nsec < ts->tv_nsec) {
		delta.tv_sec = te->tv_sec - ts->tv_sec - 1;
		delta.tv_nsec = te->tv_nsec - ts->tv_nsec + 1000000000;
	} else {
		delta.tv_sec = te->tv_sec - ts->tv_sec;
		delta.tv_nsec = te->tv_nsec - ts->tv_nsec;
	}
	return delta.tv_sec * 1000 + delta.tv_nsec / 1000000;
}

int rf_wait_status(struct nrf24l01 *dev, uint8_t mask, unsigned int timeout)
{
	int ret;

	struct timespec start, now;
	uint64_t millis = 0;
	clock_gettime(CLOCK_MONOTONIC, &start);

	uint8_t status = 0;
	while (millis < timeout) {
		ret = rf_command(dev, NOP, NULL, 0, 0);
		if (ret < 0) {
#ifdef DEBUG
			fprintf(stderr, TAG "wait_status read status error\n");
#endif
			goto out;
		}
		status = ret & 0xff;
		if (status & mask) {
			/* return status */
			ret = status;
			goto out;
		}

		usleep(WAIT_INTERVAL);
		clock_gettime(CLOCK_MONOTONIC, &now);
		millis = timespec_diff(&start, &now);
	}

#ifdef DEBUG
	fprintf(stderr, TAG "wait_status timeout\n");
#endif
	ret = -1;

out:
	return ret;
}

int rf_packet_recv(struct nrf24l01 *dev, struct rf_packet *packet,
		unsigned int timeout)
{
	return rf_receive(dev, packet, sizeof(*packet), timeout);
}

int rf_packet_send(struct nrf24l01 *dev, struct rf_packet *packet)
{
	return rf_send(dev, packet, sizeof(*packet));
}

int rf_file_recv(struct nrf24l01 *dev, FILE *fp, unsigned int timeout)
{
	int ret;
	struct rf_packet packet;

	while (1) {
		ret = rf_packet_recv(dev, &packet, timeout);
		if (ret) {
#ifdef DEBUG
			fprintf(stderr, TAG "file_recv packet error %d\n",
					ret);
#endif
			goto out;
		}

		if (packet.flags & FT_ERR) {
#ifdef DEBUG
			fprintf(stderr, TAG "file_recv FT_ERR\n");
#endif
			ret = -1;
			goto out;
		}

		if (packet.flags & FT_DATA) {
			size_t written = 0;
			while (written < packet.dlen) {
				written += fwrite(&packet.data[written], 1,
						packet.dlen - written, fp);
				ret = ferror(fp);
				if (ret)
					goto out;
			}
		}

		if (packet.flags & FT_END) {
#ifdef DEBUG
			fprintf(stderr, TAG "file_recv FT_END\n");
#endif
			break;
		}
	}

	ret = 0;

out:
	return ret;
}

int rf_file_send(struct nrf24l01 *dev, FILE *fp)
{
	int ret;
	struct rf_packet packet;

	int err = 0;
	while (!err && !feof(fp)) {
		packet.dlen = fread(&packet.data, 1, sizeof(packet.data), fp);

		if (packet.dlen > 0) {
			packet.flags = FT_DATA;
			if (feof(fp))
				packet.flags |= FT_END;
		} else {
			if (ferror(fp)) {
				packet.flags = FT_ERR;
			}
			if (feof(fp)) {
				packet.flags = FT_END;
			}
		}
		
		ret = rf_packet_send(dev, &packet);
		if (ret) {
#ifdef DEBUG
			fprintf(stderr, TAG "file_send send error\n");
#endif
			goto out;
		}
	}

out:
	return ret;
}
