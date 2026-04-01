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

int main() {
    GPIO_t *jp2 = (GPIO_t *)JP2_BASE;

    pinMode(jp2, 2, 0);  // bit 2, pin 5 = input

    while(1) {
        if(digitalRead(jp2, 2)) {
            *hex = 0x3F;  // shows 0 when vibration detected
        } else {
            *hex = 0x06;  // shows 1 when nothing
        }
    }
}