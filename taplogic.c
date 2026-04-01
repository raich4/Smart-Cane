#include <stdlib.h>

#define JP2_BASE        0xFF200070
#define HEX3_HEX0_BASE  0xFF200020

typedef struct {
    volatile unsigned int DATA;
    volatile unsigned int DDR;
} GPIO_t;

volatile int *hex = (int *)HEX3_HEX0_BASE;

int digitalRead(GPIO_t *gpio, int pin) {
    return (gpio->DATA >> pin) & 1;
}

void pinMode(GPIO_t *gpio, int pin, int mode) {
    if(mode == 1) gpio->DDR |=  (1 << pin);
    else          gpio->DDR &= ~(1 << pin);
}

void delay(int us) {
    volatile int i;
    for(i = 0; i < us * 40; i++);
}

void show_assist() {
    *hex = (0x77 << 24) | (0x6D << 16) | (0x6D << 8) | 0x78;
}

void show_disengage() {
    *hex = (0x5E << 24) | (0x06 << 16) | (0x6D << 8) | 0x79;
}

int countTaps(GPIO_t *gpio, int pin) {
    int count = 1;
    int timeout = 0;

    while(digitalRead(gpio, pin));
    delay(10000);  // shorter debounce

    while(timeout < 2000) {  // short window
        if(digitalRead(gpio, pin)) {
            count++;
            while(digitalRead(gpio, pin));
            delay(5000);
            timeout = 0;
        }
        delay(100);
        timeout++;
    }
    return count;
}

// in main:

int main() {
    GPIO_t *jp2 = (GPIO_t *)JP2_BASE;

    pinMode(jp2, 2, 0);

    int assist_mode = 0;
    *hex = 0x00000000;  // blank on startup

    while(1) {
    if(digitalRead(jp2, 2)) {
        int taps = countTaps(jp2, 2);

        if(!assist_mode && taps <= 2) {  // 1 or 2 taps = assist on
            assist_mode = 1;
            show_assist();
        }
        else if(assist_mode && taps >= 3) {  // 3+ taps = assist off
            assist_mode = 0;
            show_disengage();
            delay(1000000);
            *hex = 0x00000000;
        }
    }
}
}