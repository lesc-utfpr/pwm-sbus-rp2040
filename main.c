#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pwm_in.pio.h" 

// TinyUSB headers for HID Device
#include "bsp/board.h"
#include "tusb.h"

// --- CONFIGURATION ---
#define PWM_PIN_START 0 
#define NUM_CHANNELS 8

// Globals to store raw microseconds
uint32_t raw_pwm[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000}; 

// --- HELPER: Map microseconds to Joystick Axis (-127 to 127) ---
// Gamepad axes in TinyUSB use signed 8-bit integers.
int8_t map_pwm_to_axis(uint32_t pwm_us) {
    if (pwm_us < 1000) pwm_us = 1000;
    if (pwm_us > 2000) pwm_us = 2000;
    
    // Center at 1500. Scale +/- 500us to +/- 127
    int32_t val = (int32_t)(pwm_us - 1500) * 127 / 500;
    
    if (val < -127) val = -127;
    if (val > 127) val = 127;
    
    return (int8_t)val;
}

// --- HELPER: Send the HID Report ---
void send_hid_report() {
    // Skip if USB HID is not ready/plugged in
    if (!tud_hid_ready()) return;

    // 1. Mapped exactly to QGroundControl's default expectations
    int8_t x  = map_pwm_to_axis(raw_pwm[0]);  // QGC Yaw      (CH 1)
    int8_t y  = map_pwm_to_axis(raw_pwm[1]);  // QGC Pitch    (CH 2)
    
    // --> THROTTLE IS NOW INVERTED USING A MINUS SIGN <--
    int8_t ry = -map_pwm_to_axis(raw_pwm[2]); // QGC Throttle (CH 3) 
    
    int8_t rx = map_pwm_to_axis(raw_pwm[3]);  // QGC Roll     (CH 4)

    // 2. Move the switches to the leftover axes so they don't overlap
    int8_t z  = map_pwm_to_axis(raw_pwm[4]); // AUX Switch (CH 5)
    int8_t rz = map_pwm_to_axis(raw_pwm[5]); // AUX Switch (CH 6)

    // 3. Map channels 7-8 to buttons (threshold at 1500us)
    uint32_t buttons = 0;
    if (raw_pwm[6] > 1500) buttons |= (1 << 0); // Button 1 (CH 7)
    if (raw_pwm[7] > 1500) buttons |= (1 << 1); // Button 2 (CH 8)

    // Send the gamepad report
    tud_hid_gamepad_report(0, x, y, z, rz, rx, ry, 0, buttons);
}

int main() {
    // 1. Initialize Board and TinyUSB stack
    board_init();
    tusb_init();

    // 2. Setup PIO
    uint offset0 = pio_add_program(pio0, &pwm_in_program);
    uint offset1 = pio_add_program(pio1, &pwm_in_program);

    for (int i = 0; i < NUM_CHANNELS; i++) {
        PIO pio = (i < 4) ? pio0 : pio1;
        uint sm = (i < 4) ? i : (i - 4);
        uint offset = (i < 4) ? offset0 : offset1;
        uint pin = PWM_PIN_START + i;

        pio_sm_config c = pwm_in_program_get_default_config(offset);
        sm_config_set_in_pins(&c, pin);
        sm_config_set_jmp_pin(&c, pin);
        
        pio_gpio_init(pio, pin);
        gpio_pull_down(pin); 

        float div = 125000000.0f / 2000000.0f;
        sm_config_set_clkdiv(&c, div);

        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }

    uint32_t next_send = board_millis();

    while (true) {
        // A. Process TinyUSB tasks
        tud_task();

        // B. Poll PIO for RC Channels
        for (int i = 0; i < NUM_CHANNELS; i++) {
            PIO pio = (i < 4) ? pio0 : pio1;
            uint sm = (i < 4) ? i : (i - 4);

            if (pio_sm_get_rx_fifo_level(pio, sm) > 0) {
                uint32_t raw = pio_sm_get(pio, sm);
                uint32_t duration_us = (0xFFFFFFFF - raw);

                // Noise Filter
                if (duration_us > 800 && duration_us < 2200) {
                    raw_pwm[i] = duration_us;
                }
            }
        }

        // C. Send USB HID Report (~50Hz)
        if (board_millis() - next_send >= 20) {
            send_hid_report();
            next_send += 20;
        }
    }
}

// --- TinyUSB Required Callbacks ---

// Invoked when the host requests a report (GET_REPORT)
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    (void) instance; (void) report_id; (void) report_type; (void) buffer; (void) reqlen;
    return 0;
}

// Invoked when the host sends a report (SET_REPORT)
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    (void) instance; (void) report_id; (void) report_type; (void) buffer; (void) bufsize;
}