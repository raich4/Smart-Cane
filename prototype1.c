#include <stdlib.h>
#include <stdbool.h>

#define JP2_BASE        0xFF200070
#define HEX3_HEX0_BASE  0xFF200020
#define HEX5_HEX4_BASE  0xFF200030
#define AUDIO_BASE      0xFF203040
#define SAMPLE_RATE     8000

typedef struct {
    volatile unsigned int CONTROL;
    volatile unsigned int FIFOSPACE;
    volatile unsigned int LEFTDATA;
    volatile unsigned int RIGHTDATA;
} AUDIO_t;

typedef struct {
    volatile unsigned int DATA;
    volatile unsigned int DDR;
} GPIO_t;

typedef struct {
    GPIO_t *gpio;
    int trig_pin;
    int echo_pin;
} HCSR04_t;

typedef struct {
    GPIO_t *gpio;
    int pin;
} VIBRATION_t;

AUDIO_t *audio  = (AUDIO_t *)AUDIO_BASE;
volatile int *hex   = (int *)HEX3_HEX0_BASE;
volatile int *hex54 = (int *)HEX5_HEX4_BASE;

const unsigned char seg7[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// GPIO FUNCTIONS
void pinMode(GPIO_t *gpio, int pin, int mode) {
    if(mode == 1) gpio->DDR |=  (1 << pin);
    else          gpio->DDR &= ~(1 << pin);
}

void digitalWrite(GPIO_t *gpio, int pin, int val) {
    if(val != 0) gpio->DATA |=  (1 << pin);
    else         gpio->DATA &= ~(1 << pin);
}

int digitalRead(GPIO_t *gpio, int pin) {
    return (gpio->DATA >> pin) & 1;
}

void delay(int us) {
    volatile int i;
    for(i = 0; i < us * 40; i++);
}

// HEX DISPLAY
void display_distances(float d1, float d2) {
    int s1 = (int)d1;
    int s2 = (int)d2;

    if(s1 > 999) s1 = 999;
    if(s2 > 999) s2 = 999;
    if(s1 < 0)   s1 = 0;
    if(s2 < 0)   s2 = 0;

    // sensor 1 on HEX5-HEX4-HEX3
    *hex54 = (seg7[(s1/100)%10] << 8) | seg7[(s1/10)%10];
    *hex   = (seg7[s1%10]       << 24) |
             (seg7[(s2/100)%10] << 16) |
             (seg7[(s2/10) %10] <<  8) |
              seg7[ s2     %10];
}

void show_assist() {
    // A S S t on HEX3-HEX0
    *hex54 = 0x00000000;
    *hex   = (0x77 << 24) | (0x6D << 16) | (0x6D << 8) | 0x78;
}

void show_disengage() {
    // d I S E on HEX3-HEX0
    *hex54 = 0x00000000;
    *hex   = (0x5E << 24) | (0x06 << 16) | (0x6D << 8) | 0x79;
}

// SENSOR FUNCTIONS
void trigger(HCSR04_t *sensor) {
    digitalWrite(sensor->gpio, sensor->trig_pin, 0);
    delay(2);
    digitalWrite(sensor->gpio, sensor->trig_pin, 1);
    delay(15);
    digitalWrite(sensor->gpio, sensor->trig_pin, 0);
}

float distance(HCSR04_t *sensor) {
    trigger(sensor);
    int timeout = 1000000;
    while(digitalRead(sensor->gpio, sensor->echo_pin) == 0 && timeout > 0) timeout--;
    if(timeout <= 0) return -1.0;
    volatile int count = 0;
    while(digitalRead(sensor->gpio, sensor->echo_pin) == 1) count++;
    return (float)count / 150.0;
}

// AUDIO FUNCTIONS
bool FIFOspace() {
    unsigned int fifospace = audio->FIFOSPACE;
    unsigned int wslc = (fifospace & 0xFF000000) >> 24;
    unsigned int wsrc = (fifospace & 0x00FF0000) >> 16;
    return (wslc > 0 && wsrc > 0);
}

void play_sample(int sample) {
    while(!FIFOspace());
    audio->LEFTDATA  = sample;
    audio->RIGHTDATA = sample;
}

void play_tone(int frequency, int duration_ms, int VOLUME) {
    if(frequency == 0) {
        int total_samples = (SAMPLE_RATE * duration_ms) / 1000;
        for(int i = 0; i < total_samples; i++) play_sample(0);
        return;
    }
    int half_period_samples = SAMPLE_RATE / frequency / 2;
    int total_samples = (SAMPLE_RATE * duration_ms) / 1000;
    int sample_count = 0;
    bool is_high = true;
    for(int i = 0; i < total_samples; i++) {
        play_sample(is_high ? VOLUME : -VOLUME);
        sample_count++;
        if(sample_count >= half_period_samples) {
            is_high = !is_high;
            sample_count = 0;
        }
    }
}

void play_obstacle_alert(int VOLUME) {
    play_tone(1000, 100, VOLUME);
    play_tone(0, 100, VOLUME);
}

void play_ledge_alert(int VOLUME) {
    play_tone(300, 400, VOLUME);
    play_tone(0, 100, VOLUME);
}

void play_emergency_siren(int VOLUME) {
    play_tone(600, 300, VOLUME);
    play_tone(800, 300, VOLUME);
}

// VIBRATION FUNCTIONS
int vibrationRead(VIBRATION_t *vib) {
    return digitalRead(vib->gpio, vib->pin);
}

int countTaps(GPIO_t *gpio, int pin) {
    int count = 1;
    int timeout = 0;

    while(digitalRead(gpio, pin));
    delay(10000);

    while(timeout < 8000) {
        if(digitalRead(gpio, pin)) {
            count++;
            while(digitalRead(gpio, pin));
            delay(10000);
            timeout = 0;
        }
        delay(100);
        timeout++;
    }
    return count;
}

// FALL DETECTION
bool detectFall(float curr_d1, float curr_d2,
                float prev_d1, float prev_d2) {
    float change1 = curr_d1 - prev_d1;
    float change2 = curr_d2 - prev_d2;

    if(change1 < 0) change1 = -change1;
    if(change2 < 0) change2 = -change2;

    return (change1 > 20.0 || change2 > 20.0);
}

int main(void) {
    audio->CONTROL = 0xC;
    audio->CONTROL = 0x0;

    unsigned int volume = 0;
    int assist_mode = 0;
    int idle_count  = 0;
    float prev_d1   = 0.0;
    float prev_d2   = 0.0;

    GPIO_t *jp2 = (GPIO_t *)JP2_BASE;

    // sensor 1: pin 8 trig (D5 bit5), pin 9 echo (D6 bit6)
    HCSR04_t sensor1 = { jp2, 5, 6 };
    pinMode(sensor1.gpio, sensor1.trig_pin, 1);
    pinMode(sensor1.gpio, sensor1.echo_pin, 0);

    // sensor 2: pin 6 trig (D3 bit3), pin 7 echo (D4 bit4)
    HCSR04_t sensor2 = { jp2, 3, 4 };
    pinMode(sensor2.gpio, sensor2.trig_pin, 1);
    pinMode(sensor2.gpio, sensor2.echo_pin, 0);

    // vibration: pin 5 (D2 bit2)
    VIBRATION_t vib = { jp2, 2 };
    pinMode(vib.gpio, vib.pin, 0);

    *hex   = 0x00000000;
    *hex54 = 0x00000000;

    while(1) {
        float dist1 = distance(&sensor1);
        delay(60000);
        float dist2 = distance(&sensor2);
        delay(60000);

        // --- VIBRATION CHECK ---
        if(vibrationRead(&vib)) {

            // check for quick fall first
            if(detectFall(dist1, dist2, prev_d1, prev_d2)) {
                assist_mode = 1;
                show_assist();
                play_emergency_siren(0x3FFFFFFF);
            } else {
                // normal tap — count taps
                int taps = countTaps(jp2, 2);

                if(taps == 1 && !assist_mode) {
                    assist_mode = 1;
                    show_assist();
                    play_emergency_siren(0x3FFFFFFF);
                }
                else if(taps >= 3 && assist_mode) {
                    assist_mode = 0;
                    show_disengage();
                    delay(1000000);
                    *hex   = 0x00000000;
                    *hex54 = 0x00000000;
                }
            }
        }

        // --- SLOW FALL DETECTION ---
        float change1 = dist1 - prev_d1;
        float change2 = dist2 - prev_d2;
        if(change1 < 0) change1 = -change1;
        if(change2 < 0) change2 = -change2;

        if(change1 < 2.0 && change2 < 2.0) {
            idle_count++;
        } else {
            idle_count = 0;
        }

        if(idle_count >= 50 && !assist_mode) {
            assist_mode = 1;
            show_assist();
            play_emergency_siren(0x3FFFFFFF);
            idle_count = 0;
        }

        // update previous distances
        prev_d1 = dist1;
        prev_d2 = dist2;

        // --- SENSOR LOGIC (only when not in assist mode) ---
        if(!assist_mode) {

            // obstacle detection sensor 1
            if(dist1 > 0 && dist1 <= 30) {
                if(dist1 > 20)      volume = 0x3FFFFFFF / 4;
                else if(dist1 > 10) volume = 0x3FFFFFFF / 2;
                else                volume = 0x3FFFFFFF;
                play_obstacle_alert(volume);
            }

            // ledge detection sensor 2
            if(dist2 >= 18.0)      play_emergency_siren(0x3FFFFFFF);
            else if(dist2 >= 10.0) play_ledge_alert(0x3FFFFFFF);
            else if(dist2 >= 5.0)  play_ledge_alert(0x3FFFFFFF / 2);
            else if(dist2 >= 3.0)  play_ledge_alert(0x3FFFFFFF / 4);

            // show distances on HEX
            display_distances(dist1, dist2);
        }
    }

    return 0;
}
