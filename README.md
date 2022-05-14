# RP2040 PIO I2C High Speed (3.4 Mbps) Implementation

The Raspberry Pi [RP2040](https://www.raspberrypi.com/products/rp2040/) is a high performance, dual-core, 32-bit microcontroller. One of the more versatile subsystems are the Programmable I/O (PIO) blocks, instruction driven machines with predictable, cycle-precise timing. The Raspberry-provided [examples](https://github.com/raspberrypi/pico-examples/tree/master/pio) demonstrate how to implement a number of bus interfaces, including SPI and I2C. 

Missing from both these examples, and the available on-package peripherals, is support for I2C High-speed (I2C-HS). The I2C specification defines several speed tiers, including standard (100 kbit/sec), Fast (400 kbit/sec), Fast-plus (1 Mbit/sec), and High-speed (1.7 or 3.4 Mbit/sec). Despite being rarely supported by microcontrollers, I2C-HS can be extremely useful in driving ADCs DACs or other components at high frequencies for tight control loop or sampling applications.

This repo builds on the Raspberry Pi Pico [PIO I2C example](https://github.com/raspberrypi/pico-examples/tree/master/pio/i2c) to provide a I2C-HS implementation. To minimize jitter, it is recommended that the RP2040 be clocked at a multiple of `8*3.4 MHz = 27.2 MHz`, for example 136 MHz via `set_sys_clock_khz(136000, true);`. To use in`1.7 Mbps` HS mode  (less useful due to only a marginal improvement over Fast-plus) , change `3400000` to `1700000` in `i2c.pio`.

It deviates from the I2C-HS spec in three ways:
1. It does not check for a NACK condition after sending the master HS enable code. In practice this should not be a problem since bus arbitration will have happened before the expected non-ACK.
2. It allows for clock stretching mid-byte in hish speed transfers, even though formally disallowed by spec, so that the implementation fits within 16 PIO instructions.
3. It does not provide current-source pull-up on the SCL line, so no current boosting is available to slave devices.

The implementation is designed such that with additional circuitry, the current-source pull-up during HS operation can be provided. The GPIO pin `pin_hs` is set high when HS mode is active (set low otherwise), and can be used to switch a transistor network that disconnects low-speed devices and enables current-sourcing when HS mode is active. Without this additional circuitry, some I2C-HS devices may be unable to keep up reliable communications, though in practice this is only likely to occur when wires are very long ("meters"), or if there are many devices sharing the bus. Since there are 8 state machines, the latter can be avoided by connecting each device to its own I2C-HS bus (and connecting all non-HS devices to one of the hardware I2C buses).

Use in the same manner as demonstrated in the Raspberry Pi Pico [PIO I2C example](https://github.com/raspberrypi/pico-examples/tree/master/pio/i2c), but pass an additional GPIO pin number for the high-speed circuitry enable pin. To disable this functionality, set `pin_hs` to a non-existant GPIO pin number.

This implementation also provides an (optional) DMA-driven I2C-HS write functionality. In the current implementation, this is limited to a single I2C-HS state machine.
