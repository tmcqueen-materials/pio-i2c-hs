/**
 * Modifications for I2C High Speed (c) 2022 Tyrel M. McQueen
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pio_i2c.h"

// DMA transfers do not block, but we need to surface
// PIO errors to stop the DMA machinery and let
// callers check on completion.
struct pio_i2c_irq_info_struct {
  PIO pio;
  uint sm;
  uint pin_hs;
  uint dma_chan;
  bool dma_ongoing;
  bool dma_error;
} pio_i2c_irq_info = { .dma_ongoing = false, .dma_error = false };

bool pio_i2c_check_error(PIO pio, uint sm) {
    return pio_interrupt_get(pio, sm);
}

void pio_i2c_resume_after_error(PIO pio, uint sm) {
    pio_sm_drain_tx_fifo(pio, sm);
    pio_sm_exec(pio, sm, (pio->sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    pio_interrupt_clear(pio, sm);
}

void pio_i2c_rx_enable(PIO pio, uint sm, bool en) {
    if (en)
        hw_set_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    else
        hw_clear_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
}

static inline void pio_i2c_put16(PIO pio, uint sm, uint16_t data) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        ;
    // some versions of GCC dislike this
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    *(io_rw_16 *)&pio->txf[sm] = data;
#pragma GCC diagnostic pop
}

// If I2C is ok, block and push data. Otherwise fall straight through.
void pio_i2c_put_or_err(PIO pio, uint sm, uint16_t data) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        if (pio_i2c_check_error(pio, sm))
            return;
    if (pio_i2c_check_error(pio, sm))
        return;
    // some versions of GCC dislike this
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    *(io_rw_16 *)&pio->txf[sm] = data;
#pragma GCC diagnostic pop
}

uint8_t pio_i2c_get(PIO pio, uint sm) {
    return (uint8_t)pio_sm_get(pio, sm);
}

void pio_i2c_start(PIO pio, uint sm, uint pin_hs) {
    // Do a "slow" (= I2C fast, 400 kb/s) start, master code 00001111, then nack to
    // switch to high speed mode. Do all this with manual codes since the PIO
    // state machine is designed for only the HS transmissions.
    // Then do a high speed start.

    // Slow start
    pio_i2c_put_or_err(pio, sm, pio_i2c_prepare_instr(7)); // Escape code for 7 instruction sequence
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S1_10]);     // We are already idle state, just pull SDA low
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_W1P1_10]);    // Allow clock stretch
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S1_10]);     // Still SDA low
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S1_10]);     // Still SDA low
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);      // Pull clock low so we can present data
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);      // Keep clock low
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);      // Keep clock low

    // first four bits of master code
    for (int i = 0; i < 4; ++i) {
      pio_i2c_put_or_err(pio, sm, pio_i2c_prepare_instr(8)); // Escape code for 8 instruction sequence
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);     // Keep clock low
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);     // Keep clock low, set data low
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S1_10]);    // Set clock high
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_W1P1_4]);    // Allow clock stretch
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S1_9]);    // Keep clock high
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S1_9]);    // Keep clock high
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);     // Set clock low, keep data low
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);     // Keep clock low
    }
    // last four bits of master code
    for (int i = 0; i < 4; ++i) {
      pio_i2c_put_or_err(pio, sm, pio_i2c_prepare_instr(8)); // Escape code for 8 instruction sequence
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);     // Keep clock low
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S0_8]);     // Keep clock low, set data high
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S1_10]);    // Set clock high
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_W1P1_4]);    // Allow clock stretch
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S1_9]);    // Keep clock high
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S1_9]);    // Keep clock high
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S0_8]);     // Set clock low, keep data high
      pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_0S0_8]);     // Keep clock low
    }

    // Slow nack
    // I2C-HS Spec says if an ACK is received here, the switch to HS mode should not succeed.
    // Right now this code does not check for an ACK, instead assuming it is a NACK.
    pio_i2c_put_or_err(pio, sm, pio_i2c_prepare_instr(8)); // Escape code for 8 instruction sequence
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S0_8]);     // Keep clock low, let data high
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S0_8]);     // Keep clock low
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S1_9]);     // Set clock high
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_W1P1_4]);    // Allow clock stretch
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S1_9]);     // Keep clock high
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S1_9]);     // Keep clock high
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S0_9]);     // Set clock low, keep data high
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_1S1_8]);     // Bring to high speed idle

    // High speed start
    // At this point, the master (us) should disconnect low speed devices from the bus, and
    // enable SCL to be current sourcing when set high. Toggle GPIO for that.
    if (pin_hs < NUM_BANK0_GPIOS)
      gpio_put(pin_hs, true);
    pio_i2c_put_or_err(pio, sm, pio_i2c_prepare_instr(2));                            // Escape code for 2 instruction sequence
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0_HS]);    // We are already in idle state, just pull SDA low
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0_HS]);    // Also pull clock low so we can present data
}

void pio_i2c_stop(PIO pio, uint sm, uint pin_hs) {
    // High speed stop
    pio_i2c_put_or_err(pio, sm, pio_i2c_prepare_instr(3));                            // Escape code for 3 instruction sequence
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0_HS]);    // SDA is unknown; pull it down
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0_HS]);    // Release clock
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1_HS]);    // Release SDA to return to idle state
    // At this point, the master (us) should resconnect low speed devices from the bus, and
    // disable SCL to be current sourcing when set high. Toggle GPIO for that. The circuit
    // should be designed to *not* glitch the bus.
    if (pin_hs < NUM_BANK0_GPIOS)
      gpio_put(pin_hs, false);
};

static void pio_i2c_wait_idle(PIO pio, uint sm) {
    // Finished when TX runs dry or SM hits an IRQ
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    while (!(pio->fdebug & 1u << (PIO_FDEBUG_TXSTALL_LSB + sm) || pio_i2c_check_error(pio, sm)))
        tight_loop_contents();
}

int pio_i2c_write_blocking(PIO pio, uint sm, uint pin_hs, uint8_t addr, uint8_t *txbuf, uint len) {
    int err = 0;
    if (pio_i2c_dma_ongoing(pio, sm))
      return -1; // A DMA transfer is ongoing
    pio_i2c_start(pio, sm, pin_hs);
    pio_i2c_rx_enable(pio, sm, false);
    pio_i2c_put16(pio, sm, (addr << 2) | 1u);
    while (len && !pio_i2c_check_error(pio, sm)) {
        if (!pio_sm_is_tx_fifo_full(pio, sm)) {
            --len;
            pio_i2c_put_or_err(pio, sm, pio_i2c_prepare_byte(*txbuf++, len == 0));
        }
    }
    pio_i2c_stop(pio, sm, pin_hs);
    pio_i2c_wait_idle(pio, sm);
    if (pio_i2c_check_error(pio, sm)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm);
        pio_i2c_stop(pio, sm, pin_hs);
    }
    return err;
}

void i2c_irq_handler_internal(PIO pio, uint sm, uint pin_hs, uint dma_chan) {
    pio_i2c_irq_info.dma_error = true;
    (void)pio_i2c_dma_stop(pio, sm, pin_hs, dma_chan);
    pio_interrupt_clear(pio, sm);
    return;
}

void i2c_irq_handler() {
    i2c_irq_handler_internal(pio_i2c_irq_info.pio, pio_i2c_irq_info.sm, pio_i2c_irq_info.pin_hs, pio_i2c_irq_info.dma_chan);
    return;
}

int pio_i2c_disable_sys_irq(PIO pio, uint sm) {
    uint pio_irq = PIO0_IRQ_0 + ((pio_get_index(pio)<<1) & 0x2);
    if (irq_is_enabled(pio_irq)) {
        pio_set_irq0_source_enabled(pio, pis_interrupt0 + sm, false);

        irq_set_enabled(pio_irq, false);
        irq_remove_handler(pio_irq, i2c_irq_handler);
    }
    // we might have missed a PIO call for help between disabling system IRQ and now,
    // so check for that for returning
    return pio_i2c_check_error(pio, sm);
}

int pio_i2c_enable_sys_irq(PIO pio, uint sm, uint pin_hs, uint dma_chan) {
    uint pio_irq = PIO0_IRQ_0 + ((pio_get_index(pio)<<1) & 0x2);
    if (irq_is_enabled(pio_irq)) {
      assert(pio_i2c_irq_info.pio == pio);
      assert(pio_i2c_irq_info.sm == sm);
      return 0;
    }
    pio_i2c_irq_info.pio = pio;
    pio_i2c_irq_info.sm = sm;
    pio_i2c_irq_info.pin_hs = pin_hs;
    pio_i2c_irq_info.dma_chan = dma_chan;
    pio_i2c_irq_info.dma_ongoing = true;
    pio_i2c_irq_info.dma_error = false;
    irq_set_exclusive_handler(pio_irq, i2c_irq_handler);
    irq_set_enabled(pio_irq, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt0 + sm, true);
    // we might have missed a PIO call for help between setting and enabling sys IRQ and now,
    // so check for that for returning
    return pio_i2c_check_error(pio, sm);
}

int pio_i2c_write_dma_start(PIO pio, uint sm, uint pin_hs, uint8_t addr, uint dma_chan) {
    int err = 0;
    if (pio_i2c_dma_ongoing(pio, sm))
      return -1; // Another DMA transfer is ongoing
    if (pio_i2c_disable_sys_irq(pio, sm))
      return -1;
    pio_i2c_start(pio, sm, pin_hs);
    pio_i2c_rx_enable(pio, sm, false);
    pio_i2c_put16(pio, sm, (addr << 2) | 1u);
    err = pio_i2c_check_error(pio, sm);
    if (!err) {
      dma_channel_start(dma_chan);
      err = pio_i2c_enable_sys_irq(pio, sm, pin_hs, dma_chan);
    }
    if (err) {
      pio_i2c_dma_stop(pio, sm, pin_hs, dma_chan);
      return -1;
    } else
      return 0;
}

bool pio_i2c_dma_ongoing_any() {
    return pio_i2c_irq_info.dma_ongoing;
}

bool pio_i2c_dma_ongoing(PIO pio, uint sm) {
    if (pio_i2c_irq_info.pio == pio && pio_i2c_irq_info.sm == sm)
      return pio_i2c_irq_info.dma_ongoing;
    else
      return false;
}

bool pio_i2c_dma_check_error(PIO pio, uint sm) {
    if (pio_i2c_irq_info.pio == pio && pio_i2c_irq_info.sm == sm)
      return pio_i2c_irq_info.dma_error;
    else
      return false;
}

int pio_i2c_dma_stop(PIO pio, uint sm, uint pin_hs, uint dma_chan) {
    int err = 0;
    dma_channel_abort(dma_chan); // This probably needs refinement in light of https://github.com/raspberrypi/pico-feedback/issues/224
                                 // and the possibility of DMA chains.
    err = pio_i2c_disable_sys_irq(pio, sm);
    if (!err) {
      pio_i2c_stop(pio, sm, pin_hs);
      pio_i2c_wait_idle(pio, sm);
    }
    if (err || pio_i2c_check_error(pio, sm)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm);
        pio_i2c_stop(pio, sm, pin_hs);
        pio_i2c_irq_info.dma_error = true;
    }
    pio_i2c_irq_info.dma_ongoing = false;
    return err;
}

int pio_i2c_read_blocking(PIO pio, uint sm, uint pin_hs, uint8_t addr, uint8_t *rxbuf, uint len) {
    int err = 0;
    if (pio_i2c_dma_ongoing(pio, sm))
      return -1; // A DMA transfer is ongoing
    pio_i2c_start(pio, sm, pin_hs);
    pio_i2c_rx_enable(pio, sm, true);
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_i2c_get(pio, sm);
    pio_i2c_put16(pio, sm, (addr << 2) | 3u);
    uint32_t tx_remain = len; // Need to stuff 0xff bytes in to get clocks

    bool first = true;

    while ((tx_remain || len) && !pio_i2c_check_error(pio, sm)) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, sm)) {
            --tx_remain;
            pio_i2c_put16(pio, sm, (0xffu << 1) | (tx_remain ? 0 : (1u << PIO_I2C_FINAL_LSB) | (1u << PIO_I2C_NAK_LSB)));
        }
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            if (first) {
                // Ignore returned address byte
                (void)pio_i2c_get(pio, sm);
                first = false;
            }
            else {
                --len;
                *rxbuf++ = pio_i2c_get(pio, sm);
            }
        }
    }
    pio_i2c_stop(pio, sm, pin_hs);
    pio_i2c_wait_idle(pio, sm);
    if (pio_i2c_check_error(pio, sm)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm);
        pio_i2c_stop(pio, sm, pin_hs);
    }
    return err;
}

