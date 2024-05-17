/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if 1

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include <stdio.h>

// ----------------------------------------------------------------------------
// Initialize our little LED for control puposes
const uint led_pin = PICO_DEFAULT_LED_PIN;
bool led_state = false;

void led_init() {
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
}

void led_toggle() {
    led_state = ~led_state;
    gpio_put(led_pin, led_state);
}

// ----------------------------------------------------------------------------
// Define all pins for LCD input via flat ribbon cable
const uint lcd_pin_vsync = 2;   // 75Hz
const uint lcd_pin_hsync = 3;   // 24kHz
const uint lcd_pin_pclk = 4;    // 1.5Mhz
const uint lcd_pin_d0 = 6;      // 1.5MHz, 2x 16 shades of gray
const uint lcd_data_pins = 8;   // on 8 data pins

const uint lcd_line_bytes = 120;
const uint lcd_num_lines = 160;
// LCD related variables
uint8_t lcd_line[32][lcd_line_bytes];          // store one line of LCD pixels

void lcd_init() {
    gpio_init(lcd_pin_vsync);
    gpio_set_dir(lcd_pin_vsync, GPIO_IN);
    gpio_init(lcd_pin_hsync);
    gpio_set_dir(lcd_pin_hsync, GPIO_IN);
    gpio_init(lcd_pin_pclk);
    gpio_set_dir(lcd_pin_pclk, GPIO_IN);
    for (int i=0; i<lcd_data_pins; i++) {
        gpio_init(lcd_pin_d0 + i);
        gpio_set_dir(lcd_pin_d0 + i, GPIO_IN);
    }
}

void lcd_wait_vsync() {
    // wait util vsync goes H which is the start of the vertical sync pulse
    while (gpio_get(lcd_pin_vsync) == 1) { }
    while (gpio_get(lcd_pin_vsync) == 0) { }
}

volatile uint32_t delay_dummy = 0;

void lcd_scan_line(int y) {
    uint8_t *dst = lcd_line[y];
    int i;
    // wait until hsync goes H
    while (gpio_get(lcd_pin_hsync) == 0) { }
    // wait until hsync goes L, which is the end of the hsync flank
    while (gpio_get(lcd_pin_hsync) == 1) { }
    for (i=20; i>0; --i) delay_dummy = i;
    for (i=lcd_line_bytes; i>0; --i) {
#if 1
        while (gpio_get(lcd_pin_pclk) == 0) { }
        while (gpio_get(lcd_pin_pclk) == 1) { }
        *dst++ = (((*(uint32_t*)(SIO_BASE + 0x004)) >> lcd_pin_d0));
        // for (int j=4; j>0; --j) delay_dummy = j;
        // if (gpio_get(lcd_pin_d0))
        //         *dst++ = 0xffff;
        //     else
        //         *dst++ = 0x0000;
        //     sleep_us(1);
#else
        *dst++ =
            (gpio_get(lcd_pin_d0)<<7) |
            (gpio_get(lcd_pin_d0+1)<<6) |
            (gpio_get(lcd_pin_d0+2)<<5) |
            (gpio_get(lcd_pin_d0+3)<<4) |
            (gpio_get(lcd_pin_d0+4)<<3) |
            (gpio_get(lcd_pin_d0+5)<<2) |
            (gpio_get(lcd_pin_d0+6)<<1) |
            (gpio_get(lcd_pin_d0+7)<<0);
#endif
        //sleep_us(1);
    }
}
// ----------------------------------------------------------------------------
// Define all pins for the TFT screen output via SPI
const uint tft_pin_cs = 17;
const uint tft_pin_sck = 18;
const uint tft_pin_mosi = 19;
const uint tft_pin_miso = 16;
const uint tft_pin_reset = 14;
const uint tft_pin_dc = 15;     // select data or control

// TFT related variables
spi_inst_t *tft_spi = spi0;     // use this SPI port
uint16_t tft_line[480];         // store one line of pixels for TFT


// ST7796s 480x320
// https://www.displayfuture.com/Display/datasheet/controller/ST7796s.pdf

// Send a command a number of bytes the TFT controller
void tft_send(uint8_t cmd, size_t size=0, const uint8_t *data=nullptr) {
    gpio_put(tft_pin_dc, 0);
    gpio_put(tft_pin_cs, 0);
    sleep_us(1);
    spi_write_blocking(tft_spi, &cmd, 1);
    if (size) {
        sleep_us(1);
        gpio_put(tft_pin_dc, 1);
        sleep_us(1);
        spi_write_blocking(tft_spi, data, size);
    }
    sleep_us(1);
    gpio_put(tft_pin_cs, 1);
}

// Configure a GPIO pin for the TFT diplay port
void tft_pin_config_out(uint pin, bool value) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, value);
}

// Initialize SPI and boot the display controller
void tft_init() {
    // define the pins we use directly
    tft_pin_config_out(tft_pin_cs, 1);
    tft_pin_config_out(tft_pin_dc, 1);
    tft_pin_config_out(tft_pin_reset, 1);

    // initialize the SPI port
    spi_init(tft_spi, 62.5 * 1000 * 1000);  // 62.5MHz, can probably be more if needed
    spi_set_format( spi0,           // SPI instance
                    8,              // Number of bits per transfer
                    SPI_CPOL_1,     // Polarity (CPOL)
                    SPI_CPHA_1,     // Phase (CPHA)
                    SPI_MSB_FIRST);
    gpio_set_function(tft_pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(tft_pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(tft_pin_miso, GPIO_FUNC_SPI);

    // reset the TFT controller
    gpio_put(tft_pin_reset, 0);
    sleep_ms(100);
    gpio_put(tft_pin_reset, 1);
    sleep_ms(100);

    tft_send(0xf0, 1, (uint8_t*)"\xc3");    // Command Set Control
    tft_send(0xf0, 1, (uint8_t*)"\x96");    // Command Set Control
    tft_send(0xc5, 1, (uint8_t*)"\x1c");
    tft_send(0x36, 1, (uint8_t*)"\xe8");    // 
    tft_send(0x3a, 1, (uint8_t*)"\x55");    // 
    tft_send(0xb0, 1, (uint8_t*)"\x80");    // 
    tft_send(0xb4, 1, (uint8_t*)"\x01");    // 
    tft_send(0xb6, 3, (uint8_t*)"\x80\x02\x3b");
    tft_send(0xb7, 1, (uint8_t*)"\xc6");    // 
    tft_send(0xf0, 1, (uint8_t*)"\x69");    // 
    tft_send(0xf0, 1, (uint8_t*)"\x3c");    // 
    tft_send(0x11);
    sleep_ms(150);
    tft_send(0x29);
    sleep_ms(150);
}

void tft_send_line(int y, int rpt) {
    static uint8_t cmd_col[] = {0, 0, 479 >> 8, 479 & 0xff};
    tft_send(0x2a, 4, cmd_col);
    uint8_t cmd_row[] = {(uint8_t)(y>>8), (uint8_t)y, (uint8_t)((y+rpt)>>8), (uint8_t)(y+rpt)};
    tft_send(0x2b, 4, cmd_row);
    static uint8_t cmd12[100] = {0};
    tft_send(0x2c, 480 * 2, (uint8_t *)tft_line);
    for (int i = 0; i < rpt; i++) {
        tft_send(0x3c, 480 * 2, (uint8_t *)tft_line);
    }
}

// PIO pio = pio0;
// uint sm = 0;
// uint dma_chan = 0;

#if 0
void init_n2_dma() {
    // Grant high bus priority to the DMA, so it can shove the processors out
    // of the way. This should only be needed if you are pushing things up to
    // >16bits/clk here, i.e. if you need to saturate the bus completely.
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    pio = pio0;
    sm = 0;
    dma_chan = 0;

    uint16_t capture_prog_instr = pio_encode_in(pio_pins, 8);
    struct pio_program capture_prog = {
            .instructions = &capture_prog_instr,
            .length = 1,
            .origin = -1
    };
    uint offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, lcd_pin_d0);
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_clkdiv(&c, 10.0f);
    // Note that we may push at a < 32 bit threshold if pin_count does not
    // divide 32. We are using shift-to-right, so the sample data ends up
    // left-justified in the FIFO in this case, with some zeroes at the LSBs.
    sm_config_set_in_shift(&c, true, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}
#endif

#define TFT_GRAY(a) ( (a<<4) | ((a>>1)|((a&0x01)<<15)) | (a<<9) )
uint16_t lcd_convert(uint8_t px) {
    #if 0 // inverse
    static const uint16_t px_lut[16] = {
        TFT_GRAY(0x00), TFT_GRAY(0x01), TFT_GRAY(0x02), TFT_GRAY(0x03),
        TFT_GRAY(0x04), TFT_GRAY(0x05), TFT_GRAY(0x06), TFT_GRAY(0x07),
        TFT_GRAY(0x08), TFT_GRAY(0x09), TFT_GRAY(0x0a), TFT_GRAY(0x0b),
        TFT_GRAY(0x0c), TFT_GRAY(0x0d), TFT_GRAY(0x0e), TFT_GRAY(0x0f),
    };
    #elif 1 // original
    static const uint16_t px_lut[16] = {
        TFT_GRAY(0x0f), TFT_GRAY(0x0e), TFT_GRAY(0x0d), TFT_GRAY(0x0c),
        TFT_GRAY(0x0b), TFT_GRAY(0x0a), TFT_GRAY(0x09), TFT_GRAY(0x08),
        TFT_GRAY(0x07), TFT_GRAY(0x06), TFT_GRAY(0x05), TFT_GRAY(0x04),
        TFT_GRAY(0x03), TFT_GRAY(0x02), TFT_GRAY(0x01), TFT_GRAY(0x00),
    };
    #else // bit swapped
    static const uint16_t px_lut[16] = {
        TFT_GRAY(0x00), TFT_GRAY(0x08), TFT_GRAY(0x04), TFT_GRAY(0x0c),
        TFT_GRAY(0x02), TFT_GRAY(0x0a), TFT_GRAY(0x06), TFT_GRAY(0x0e),
        TFT_GRAY(0x01), TFT_GRAY(0x09), TFT_GRAY(0x05), TFT_GRAY(0x0d),
        TFT_GRAY(0x03), TFT_GRAY(0x0b), TFT_GRAY(0x07), TFT_GRAY(0x0f),
    };
    #endif
    return px_lut[px&0x0f];
}

void lcd_line_to_tft(int y, bool odd) {
    uint8_t *src = lcd_line[y];
    if (odd) src += lcd_line_bytes/2;
    uint16_t *dst = tft_line;
    for (int i=0; i<lcd_line_bytes/2; i++) {
        uint8_t p1 = *src++;
        *dst++ = lcd_convert((p1>>4)&0x08);
        *dst++ = lcd_convert((p1>>3)&0x08);
        *dst++ = lcd_convert((p1>>2)&0x08);
        *dst++ = lcd_convert((p1>>1)&0x08);
        *dst++ = lcd_convert((p1   )&0x08);
        *dst++ = lcd_convert((p1<<1)&0x08);
        *dst++ = lcd_convert((p1<<2)&0x08);
        *dst++ = lcd_convert((p1<<3)&0x08);
    }
}

int main() {
    stdio_init_all();
    led_init();
    lcd_init();
    tft_init();

    for (;;) {
        int i;

        //sleep_ms(100);
        led_toggle();

        lcd_wait_vsync();
        for (int i=0; i<lcd_num_lines; i++) {
            lcd_scan_line(i);
        }
        for (int i=0; i<lcd_num_lines; i++) {
            lcd_line_to_tft(i, false);
            tft_send_line(i*2, 1);
            lcd_line_to_tft(i, true);
            tft_send_line(i*2+1, 1);
        }
    }
#if 0
    uint8_t data[32];

      gpio_put(tft_pin_dc, 0);
      gpio_put(tft_pin_cs, 0);
      sleep_us(1);
      data[0] = 0x04; // read display ID
      spi_write_blocking(tft_spi, data, 1);
      sleep_us(1);
      gpio_put(tft_pin_dc, 1);
      sleep_us(1);
      spi_read_blocking(tft_spi, 0, data, 4);
      sleep_us(1);
      gpio_put(tft_pin_cs, 1);
      printf("%02x %02x %02x %02x\n", data[0], data[1], data[2], data[3]);


    for (;;) {
      gpio_put(led_pin, 1);

      gpio_put(tft_pin_dc, 0);
      gpio_put(tft_pin_cs, 0);
      sleep_us(1);
      data[0] = 0x28;
      spi_write_blocking(tft_spi, data, 1);
      sleep_us(1);
      gpio_put(tft_pin_cs, 1);
      sleep_ms(1000);

      gpio_put(led_pin, 0);

      gpio_put(tft_pin_dc, 0);
      gpio_put(tft_pin_cs, 0);
      sleep_us(1);
      data[0] = 0x29;
      spi_write_blocking(tft_spi, data, 1);
      sleep_us(1);
      gpio_put(tft_pin_cs, 1);
      sleep_ms(1000);
    }
#endif
}


#else

/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// PIO logic analyser example
//
// This program captures samples from a group of pins, at a fixed rate, once a
// trigger condition is detected (level condition on one pin). The samples are
// transferred to a capture buffer using the system DMA.
//
// 1 to 32 pins can be captured, at a sample rate no greater than system clock
// frequency.

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"

// Some logic to analyse:
#include "hardware/structs/pwm.h"

const uint CAPTURE_PIN_BASE = 6;
const uint CAPTURE_PIN_COUNT = 8;
const uint CAPTURE_N_SAMPLES = 96;

static inline uint bits_packed_per_word(uint pin_count) {
    // If the number of pins to be sampled divides the shift register size, we
    // can use the full SR and FIFO width, and push when the input shift count
    // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
    // a little less efficiently.
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div) {
    // Load a program to capture n pins. This is just a single `in pins, n`
    // instruction with a wrap.
    uint16_t capture_prog_instr[] = {
        //pio_encode_wait_gpio(1, 2),
        pio_encode_wait_gpio(0, 2),
        pio_encode_in(pio_pins, pin_count),
        pio_encode_jmp(1)
    };
    struct pio_program capture_prog = {
            .instructions = capture_prog_instr,
            .length = 3,
            .origin = -1
    };
    uint offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_clkdiv(&c, div);
    // Note that we may push at a < 32 bit threshold if pin_count does not
    // divide 32. We are using shift-to-right, so the sample data ends up
    // left-justified in the FIFO in this case, with some zeroes at the LSBs.
    sm_config_set_in_shift(&c, true, true, bits_packed_per_word(pin_count));
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}

void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level) {
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_words, // Number of transfers
        true                // Start immediately
    );

    //pio_sm_exec(pio, sm, pio_encode_wait_gpio(1, 2));
    //pio_sm_exec(pio, sm, pio_encode_wait_gpio(0, 2));
    pio_sm_set_enabled(pio, sm, true);
}

void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    printf("Capture:\n");
    // Each FIFO record may be only partially filled with bits, depending on
    // whether pin_count is a factor of 32.
    uint record_size_bits = bits_packed_per_word(pin_count);
    for (int pin = 0; pin < pin_count; ++pin) {
        printf("%02d: ", pin + pin_base);
        for (int sample = 0; sample < n_samples; ++sample) {
            uint bit_index = pin + sample * pin_count;
            uint word_index = bit_index / record_size_bits;
            // Data is left-justified in each FIFO entry, hence the (32 - record_size_bits) offset
            uint word_mask = 1u << (bit_index % record_size_bits + 32 - record_size_bits);
            printf(buf[word_index] & word_mask ? "-" : "_");
        }
        printf("\n");
    }
    fflush(stdout);
}

int main() {
    stdio_init_all();
    printf("PIO logic analyser example\n");

    // We're going to capture into a u32 buffer, for best DMA efficiency. Need
    // to be careful of rounding in case the number of pins being sampled
    // isn't a power of 2.
    uint total_sample_bits = CAPTURE_N_SAMPLES * CAPTURE_PIN_COUNT;
    total_sample_bits += bits_packed_per_word(CAPTURE_PIN_COUNT) - 1;
    uint buf_size_words = total_sample_bits / bits_packed_per_word(CAPTURE_PIN_COUNT);
    uint32_t *capture_buf = (uint32_t*)malloc(buf_size_words * sizeof(uint32_t));
    hard_assert(capture_buf);

    // Grant high bus priority to the DMA, so it can shove the processors out
    // of the way. This should only be needed if you are pushing things up to
    // >16bits/clk here, i.e. if you need to saturate the bus completely.
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    PIO pio = pio0;
    uint sm = 0;
    uint dma_chan = 0;

    logic_analyser_init(pio, sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, 500.f);


    printf("Arming trigger\n");
    logic_analyser_arm(pio, sm, dma_chan, capture_buf, buf_size_words, CAPTURE_PIN_BASE, true);

    printf("Starting PWM example\n");
    // PWM example: -----------------------------------------------------------
    gpio_set_function(CAPTURE_PIN_BASE, GPIO_FUNC_PWM);
    gpio_set_function(CAPTURE_PIN_BASE + 1, GPIO_FUNC_PWM);
    // Topmost value of 3: count from 0 to 3 and then wrap, so period is 4 cycles
    pwm_hw->slice[0].top = 3;
    // Divide frequency by two to slow things down a little
    pwm_hw->slice[0].div = 4 << PWM_CH0_DIV_INT_LSB;
    // Set channel A to be high for 1 cycle each period (duty cycle 1/4) and
    // channel B for 3 cycles (duty cycle 3/4)
    pwm_hw->slice[0].cc =
            (1 << PWM_CH0_CC_A_LSB) |
            (3 << PWM_CH0_CC_B_LSB);
    // Enable this PWM slice
    pwm_hw->slice[0].csr = PWM_CH0_CSR_EN_BITS;
    // ------------------------------------------------------------------------

    // The logic analyser should have started capturing as soon as it saw the
    // first transition. Wait until the last sample comes in from the DMA.
    dma_channel_wait_for_finish_blocking(dma_chan);

    print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);
    for (;;) sleep_ms(100);
}


#endif

#if 0
void main() {
    stdio_init_all();
    printf("MessagePad LCD to TFT V0.1\n");

    tft_init();
    lcd_init();

    /// check for vsync

    printf("Done.\n");
    fflush(stdout);
    for (;;) sleep_ms(100);
}
#endif

#if 0

int main() {
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    stdio_init_all();
    const uint led_pin = PICO_DEFAULT_LED_PIN;
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    while (true) {
        printf("Hello World\n");
        gpio_put(led_pin, 1);
        sleep_ms(250);
        gpio_put(led_pin, 0);
        sleep_ms(250);
    }
#endif
}

#endif