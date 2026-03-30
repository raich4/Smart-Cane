#include <stdlib.h>
#include <stdbool.h>

#define JP2_BASE        0xFF200070
#define HEX3_HEX0_BASE  0xFF200020
#define HEX5_HEX4_BASE  0xFF200030

typedef struct {
    volatile unsigned int DATA;
    volatile unsigned int DDR;
} GPIO_t;

typedef enum {
    MODE_NORMAL,
    MODE_ASSIST
} SystemMode;

// ================= GLOBALS =================
volatile int *hex   = (int *)HEX3_HEX0_BASE;
volatile int *hex54 = (int *)HEX5_HEX4_BASE;

// ================= GPIO =================
void pinMode(GPIO_t *gpio, int pin, int mode) {
    if(mode) gpio->DDR |= (1 << pin);
    else     gpio->DDR &= ~(1 << pin);
}

int digitalRead(GPIO_t *gpio, int pin) {
    return (gpio->DATA >> pin) & 1;
}

// ================= DISPLAY =================
void show_assist() {
    *hex54 = 0;
    *hex = (0x77<<24)|(0x6D<<16)|(0x6D<<8)|0x78;
}

void show_disengage() {
    *hex54 = 0;
    *hex = (0x5E<<24)|(0x06<<16)|(0x6D<<8)|0x79;
}

// ================= MAIN =================
int main() {

    GPIO_t *jp2 = (GPIO_t *)JP2_BASE;

    pinMode(jp2,2,0); // vibration sensor

    SystemMode mode = MODE_NORMAL;

    // ===== TAP SYSTEM =====
    int tap_count = 0;
    int tap_timer = 0;

    int vib_state = 0;
    int armed = 1;  // 👈 IMPORTANT (re-arm flag)

    while(1) {

        int curr = digitalRead(jp2,2);

        // ===== TAP DETECTION WITH RE-ARM =====
        if(curr && armed) {
            tap_count++;
            tap_timer = 0;
            armed = 0;  // disarm until released
        }

        // re-arm only when signal goes LOW
        if(!curr) {
            armed = 1;
        }

        // ===== TAP WINDOW =====
        if(tap_count > 0) {
            tap_timer++;

            if(tap_timer > 5000) {

                // SINGLE TAP
                if(tap_count == 1 && mode == MODE_NORMAL) {
                    mode = MODE_ASSIST;
                    show_assist();
                }

                // TRIPLE TAP
                else if(tap_count >= 3 && mode == MODE_ASSIST) {
                    mode = MODE_NORMAL;
                    show_disengage();
                }

                tap_count = 0;
            }
        }
    }
}
