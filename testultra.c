#include <stdio.h>

// Corrected Base Addresses for DE1-SoC Computer System
#define HEX3_HEX0_BASE  0xFF200020
#define JP2_BASE        0xFF200070 

typedef struct {
    volatile unsigned int DATA; // Offset 0 
    volatile unsigned int DDR;  // Offset 4 
} GPIO_t;

typedef struct {
    GPIO_t *gpio;
    int trig_pin;
    int echo_pin;
} HCSR04_t;

volatile int *hex = (int *)HEX3_HEX0_BASE;

const unsigned char seg7[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// Mode 1 = Output, Mode 0 = Input 
void pinMode(GPIO_t *gpio, int pin, int mode){
    if(mode == 1) gpio->DDR |= (1 << pin);
    else          gpio->DDR &= ~(1 << pin);
}

void digitalWrite(GPIO_t *gpio, int pin, int val){
    if(val != 0) gpio->DATA |= (1 << pin);
    else         gpio->DATA &= ~(1 << pin);
}

int digitalRead(GPIO_t *gpio, int pin){
    return (gpio->DATA >> pin) & 1;
}

// Simple software delay
void delay(int us){
    volatile int i;
    for(i = 0; i < us * 40; i++); 
}

void trigger(HCSR04_t *sensor){
    digitalWrite(sensor->gpio, sensor->trig_pin, 0);
    delay(2);
    digitalWrite(sensor->gpio, sensor->trig_pin, 1);
    delay(15); 
    digitalWrite(sensor->gpio, sensor->trig_pin, 0);
}

float distance(HCSR04_t *sensor){
    trigger(sensor);
    
    int timeout = 1000000;
    while(digitalRead(sensor->gpio, sensor->echo_pin) == 0 && timeout > 0) timeout--;
    
    if (timeout <= 0) return -1.0; 

    volatile int count = 0;
    while(digitalRead(sensor->gpio, sensor->echo_pin) == 1) count++;
    
    return (float)count / 150.0; 
}

void display_distance(float dist){
    int d = (int)dist;
    if(d < 0) { 
        *hex = 0x40404040; // Display "----"
        return;
    }
    if(d > 999) d = 999;
    
    // Write segments to HEX3-0 [cite: 408, 410]
    *hex = (seg7[(d/100)%10] << 16) |
           (seg7[(d/10) %10] <<  8) |
            seg7[ d     %10];
}

int main(){
    GPIO_t *jp2 = (GPIO_t *)JP2_BASE;
    
    // Mapping for Physical Pins 7 and 8
    // Pin 7 = D5 (Bit 5), Pin 8 = D6 (Bit 6)
    HCSR04_t sensor1 = { jp2, 5, 6 }; 

    pinMode(sensor1.gpio, sensor1.trig_pin, 1); 
    pinMode(sensor1.gpio, sensor1.echo_pin, 0); 

    while(1){
        float dist = distance(&sensor1);
        display_distance(dist);
        delay(60000); 
    }
    return 0;
}