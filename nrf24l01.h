/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include <stddef.h>
#include <stdint.h>

#define TAG "NRF24L01"

#define DEBUG

#define CMD_MAXLEN 32
#define SPI_SPEED 1000000
#define PACKET_LEN 32

#define SEC_MILLIS 1000
#define MILLI_NSECS 1000000

/* Commands */
#define R_REGISTER 0b00000000
#define W_REGISTER 0b00100000
#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000
#define FLUSH_TX 0b11100001
#define FLUSH_RX 0b11100010
#define REUSE_TX_PL 0b11100011
#define R_RX_PL_WID 0b01100000
#define W_ACK_PAYLOAD 0b10101000
#define W_TX_PAYLOAD_NOACK 0b10110000
#define NOP 0b11111111

/* Registers */
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_OBSERVE_TX 0x08
#define REG_RPD 0x09
#define REG_RX_ADDR_P0 0x0a
#define REG_RX_ADDR_P1 0x0b
#define REG_RX_ADDR_P2 0x0c
#define REG_RX_ADDR_P3 0x0d
#define REG_RX_ADDR_P4 0x0e
#define REG_RX_ADDR_P5 0x0f
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_RX_PW_P1 0x12
#define REG_RX_PW_P2 0x13
#define REG_RX_PW_P3 0x14
#define REG_RX_PW_P4 0x15
#define REG_RX_PW_P5 0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD 0x1c
#define REG_FEATURE 0x1d

/* Configuration options */
#define CONF_PWR_UP 0b00000010
#define CONF_PRIM_RX 0b00000001

/* Status flags */
#define IRQ_MASK 0b01110000
#define RX_DR 0b01000000
#define TX_DS 0b00100000
#define MAX_RT 0b00010000

struct nrf24l01 {
	char *spi;
	int ce;
	int cs;
};

int rf_init(struct nrf24l01 *dev);
void rf_shutdown(struct nrf24l01 *dev);

int rf_receive(struct nrf24l01 *dev, void *buf, size_t len,
		unsigned int timeout);

int rf_rx_addr(struct nrf24l01 *dev, uint64_t addr);

uint8_t rf_reg_read(struct nrf24l01 *dev, uint8_t addr);
int rf_reg_write(struct nrf24l01 *dev, uint8_t addr, uint8_t value);
int rf_reg_writelong(struct nrf24l01 *dev, uint8_t addr,
		void *buf, size_t len);

int rf_command(struct nrf24l01 *dev, uint8_t cmd, void *buf, size_t len,
		int read);

#endif /* __NRF24L01_H__ */