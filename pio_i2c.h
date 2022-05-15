/**
 * Modifications for I2C High Speed (c) 2022 Tyrel M. McQueen
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PIO_I2C_H
#define _PIO_I2C_H

#include "i2c.pio.h"

// ----------------------------------------------------------------------------
// Low-level functions

void pio_i2c_start(PIO pio, uint sm, uint pin_hs);
void pio_i2c_stop(PIO pio, uint sm, uint pin_hs);

bool pio_i2c_check_error(PIO pio, uint sm);
void pio_i2c_resume_after_error(PIO pio, uint sm);

// If I2C is ok, block and push data. Otherwise fall straight through.
void pio_i2c_put_or_err(PIO pio, uint sm, uint16_t data);
uint8_t pio_i2c_get(PIO pio, uint sm);

// Each I2C byte is transmitted with additional information
// as a 16-bit half-word to the PIO FIFO
// Conversions to a 2-byte half-word for the PIO state machine
#define PIO_I2C_ICOUNT_LSB 10
#define PIO_I2C_FINAL_LSB 9
#define PIO_I2C_DATA_LSB 1
#define PIO_I2C_NAK_LSB 0
#define pio_i2c_prepare_byte(txbyte, final) (uint16_t)(((txbyte) << PIO_I2C_DATA_LSB) | (((final) == true) << PIO_I2C_FINAL_LSB) | (1u << PIO_I2C_NAK_LSB))
#define pio_i2c_prepare_instr(num_instr) (uint16_t)(((num_instr-1)<<PIO_I2C_ICOUNT_LSB))

// ----------------------------------------------------------------------------
// Transaction-level functions
bool pio_i2c_dma_ongoing_any();
bool pio_i2c_dma_ongoing(PIO pio, uint sm);
bool pio_i2c_dma_check_error(PIO pio, uint sm);
int pio_i2c_dma_stop(PIO pio, uint sm, uint pin_hs, uint dma_chan);
int pio_i2c_write_dma_start(PIO pio, uint sm, uint pin_hs, uint8_t addr, uint dma_chan);
int pio_i2c_write_blocking(PIO pio, uint sm, uint pin_hs, uint8_t addr, uint8_t *txbuf, uint len);
int pio_i2c_read_blocking(PIO pio, uint sm, uint pin_hs, uint8_t addr, uint8_t *rxbuf, uint len);

#endif
