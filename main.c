#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pwm_in.pio.h" // Generated automatically from pwm_in.pio

// --- CONFIGURATION ---
#define UART_ID uart0
#define BAUD_RATE 100000
#define UART_TX_PIN 16
#define UART_RX_PIN 17 // Not used, but required for init

#define PWM_PIN_START 0 // Channels will be GP0 to GP7
#define NUM_CHANNELS 8

// SBUS defines
#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

// Globals to store current channel values (in SBUS range 0-2047)
uint16_t channels[16] = {0}; 

// --- HELPER: Map microseconds to SBUS ---
// PWM: 1000us - 2000us
// SBUS: 172 - 1811 (Standard FrSky/Futaba scaling)
uint16_t map_pwm_to_sbus(uint32_t pwm_us) {
    if (pwm_us < 800) pwm_us = 800;   // Safety clamping
    if (pwm_us > 2200) pwm_us = 2200;

    // Linear mapping formula
    // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    int32_t val = (int32_t)(pwm_us - 1000) * (1811 - 172) / (2000 - 1000) + 172;
    
    if (val < 0) val = 0;
    if (val > 2047) val = 2047;
    return (uint16_t)val;
}

// --- HELPER: Pack SBUS Frame ---
void send_sbus_frame() {
    uint8_t frame[SBUS_FRAME_SIZE];
    memset(frame, 0, SBUS_FRAME_SIZE);

    frame[0] = SBUS_HEADER;

    // SBUS Bit Packing (11 bits per channel into bytes)
    // This is the "Horror" part of SBUS. 
    // We only map the first 8 channels. Channels 9-16 stay 0.
    frame[1]  = (uint8_t) ((channels[0] & 0x07FF));
    frame[2]  = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
    frame[3]  = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
    frame[4]  = (uint8_t) ((channels[2] & 0x07FF)>>2);
    frame[5]  = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
    frame[6]  = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
    frame[7]  = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
    frame[8]  = (uint8_t) ((channels[5] & 0x07FF)>>1);
    frame[9]  = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
    frame[10] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
    frame[11] = (uint8_t) ((channels[7] & 0x07FF)>>3);

    // Bytes 12-22 would be Channels 9-16 (We leave them 0)

    frame[23] = 0x00; // Flags (Failsafe, Frame Lost, etc)
    frame[24] = SBUS_FOOTER;

    // Send via UART
    uart_write_blocking(UART_ID, frame, SBUS_FRAME_SIZE);
}

int main() {
    stdio_init_all();

    // --- 1. SETUP UART FOR SBUS ---
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    
    // SBUS is 8E2 (8 data, Even parity, 2 stop bits)
    uart_set_format(UART_ID, 8, 2, UART_PARITY_EVEN);

    // CRITICAL: SBUS Logic is Inverted. 
    // We tell the GPIO hardware to invert the signal leaving the pin.
    gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT);

    // --- 2. SETUP PIO FOR PWM INPUT ---
    // Load the PIO program into both PIO0 and PIO1
    // We need 8 state machines. PIO0 has 4, PIO1 has 4.
    uint offset0 = pio_add_program(pio0, &pwm_in_program);
    uint offset1 = pio_add_program(pio1, &pwm_in_program);

    // Loop through 8 channels to configure State Machines
    for (int i = 0; i < NUM_CHANNELS; i++) {
        PIO pio = (i < 4) ? pio0 : pio1;
        uint sm = (i < 4) ? i : (i - 4);
        uint offset = (i < 4) ? offset0 : offset1;
        uint pin = PWM_PIN_START + i;

        pio_sm_config c = pwm_in_program_get_default_config(offset);
        sm_config_set_in_pins(&c, pin); // Set the 'wait' pin
        sm_config_set_jmp_pin(&c, pin); // Set the 'jmp pin' pin
        
        // Connect GPIO to PIO
        pio_gpio_init(pio, pin);
        gpio_pull_down(pin);

        // CLOCK DIVIDER CALCULATION
        // We want the PIO loop (2 cycles) to run at 1MHz (1us per loop)
        // System clock is 125MHz. 
        // 125MHz / 2MHz = 62.5 divider.
        // This makes the State Machine run at 2MHz.
        // Since the loop is 2 instructions, the loop frequency is 1MHz.
        // Result: The counter 'x' decreases by 1 every microsecond.
        float div = 125000000.0f / 2000000.0f;
        sm_config_set_clkdiv(&c, div);

        // Init and enable
        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }

    // --- 3. MAIN LOOP ---
    // SBUS standard frame rate is usually every 7ms - 14ms.
    // We will update whenever we can, but throttle sending to ~10ms.
    absolute_time_t next_send = get_absolute_time();

    while (true) {
        // A. Poll all 8 PIO State Machines for new data
        for (int i = 0; i < NUM_CHANNELS; i++) {
            PIO pio = (i < 4) ? pio0 : pio1;
            uint sm = (i < 4) ? i : (i - 4);

            // Check if FIFO is not empty
            if (pio_sm_get_rx_fifo_level(pio, sm) > 0) {
                // Read raw value.
                // Value is 0 minus the duration (because we decremented X)
                uint32_t raw = pio_sm_get(pio, sm);
                
                // Convert 2's complement negative number to positive duration
                uint32_t duration_us = (0xFFFFFFFF - raw);

                // Update the channel array
                channels[i] = map_pwm_to_sbus(duration_us);
            }
        }

        // B. Send SBUS Frame every 10ms (100Hz update rate)
        if (absolute_time_diff_us(get_absolute_time(), next_send) < 0) {
            send_sbus_frame();
            next_send = make_timeout_time_ms(10);
        }
    }
}