/*
 * Copyright (c) 2015 Florian Tautz 
 * Licensed under The MIT License, see LICENSE for licensing details.
 */

#ifndef __GPIO_H__
#define __GPIO_H__

#define MODE_IN     0
#define MODE_OUT    1

#define IRQ_NONE    0
#define IRQ_RISING  1
#define IRQ_FALLING 2
#define IRQ_BOTH    3

int write_int_to_file(const char *filename, int value);
int gpio_export(int gpio);
int gpio_unexport(int gpio);

int gpio_set_mode(int gpio, char mode);
int gpio_set_irq(int gpio, char mode);

int gpio_read(int gpio);
int gpio_write(int gpio, int value);
int gpio_poll(int gpio, char targetstate, int timeout);

#endif /* __GPIO_H__ */
