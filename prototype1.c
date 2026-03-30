#include <stdlib.h>
#include <stdbool.h>

#define JP2_BASE        0xFF200070
#define HEX3_HEX0_BASE  0xFF200020
#define HEX5_HEX4_BASE  0xFF200030
#define AUDIO_BASE      0xFF203040
#define SAMPLE_RATE     8000

// ================= STRUCTS =================
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
    int last_state;
    int tap_count;
    int timer;
    int event; // 1 = single tap, 3 = triple tap
} TapState;

typedef struct {
    int frequency;
    int volume;
    int total_samples;
    int sample_index;
    int half_period;
    int counter;
    int is_high;
    int active;
} AudioState;

typedef enum {
    MODE_NORMAL,
    MODE_ASSIST
} SystemMode;

// ================= GLOBALS =================
AUDIO_t *audio = (AUDIO_t *)AUDIO_BASE;
volatile int *hex   = (int *)HEX3_HEX0_BASE;
volatile int *hex54 = (int *)HEX5_HEX4_BASE;

AudioState audio_state = {0};

// ================= SEG7 =================
const unsigned char seg7[] = {
    0x3F,0x06,0x5B,0x4F,0x66,
    0x6D,0x7D,0x07,0x7F,0x6F
};

// ================= GPIO =================
void pinMode(GPIO_t *gpio, int pin, int mode) {
    if(mode) gpio->DDR |= (1 << pin);
    else     gpio->DDR &= ~(1 << pin);
}

void digitalWrite(GPIO_t *gpio, int pin, int val) {
    if(val) gpio->DATA |= (1 << pin);
    else    gpio->DATA &= ~(1 << pin);
}

int digitalRead(GPIO_t *gpio, int pin) {
    return (gpio->DATA >> pin) & 1;
}

void delay(int us) {
    volatile int i;
    for(i = 0; i < us * 40; i++);
}

// ================= DISPLAY =================
void display_distance1(float d1) {
    int s = (int)d1;
    if(s < 0) s = 0;
    if(s > 999) s = 999;

    *hex54 = (seg7[(s/100)%10] << 8) | seg7[(s/10)%10];
    *hex = (*hex & 0x00FFFFFF) | (seg7[s%10] << 24);
}

void display_distance2(float d2) {
    int s = (int)d2;
    if(s < 0) s = 0;
    if(s > 999) s = 999;

    *hex = (*hex & 0xFF000000) |
           (seg7[(s/100)%10] << 16) |
           (seg7[(s/10)%10] << 8) |
           seg7[s%10];
}

void show_assist() {
    *hex54 = 0;
    *hex = (0x77<<24)|(0x6D<<16)|(0x6D<<8)|0x78;
}

void show_disengage() {
    *hex54 = 0;
    *hex = (0x5E<<24)|(0x06<<16)|(0x6D<<8)|0x79;
}

// ================= SENSOR =================
void trigger(HCSR04_t *s) {
    digitalWrite(s->gpio, s->trig_pin, 0);
    delay(2);
    digitalWrite(s->gpio, s->trig_pin, 1);
    delay(15);
    digitalWrite(s->gpio, s->trig_pin, 0);
}

float distance(HCSR04_t *s) {
    trigger(s);

    int timeout = 100000;
    while(!digitalRead(s->gpio, s->echo_pin) && timeout--);
    if(timeout <= 0) return -1;

    int count = 0;
    while(digitalRead(s->gpio, s->echo_pin)) count++;

    return (float)count / 150.0;
}

// ================= AUDIO =================
bool FIFOspace() {
    unsigned int f = audio->FIFOSPACE;
    return ((f>>24)&0xFF) && ((f>>16)&0xFF);
}

void startTone(int freq, int duration_ms, int volume) {
    audio_state.frequency = freq;
    audio_state.volume = volume;
    audio_state.sample_index = 0;
    audio_state.total_samples = (SAMPLE_RATE * duration_ms)/1000;
    audio_state.half_period = (freq==0)?1:SAMPLE_RATE/freq/2;
    audio_state.counter = 0;
    audio_state.is_high = 1;
    audio_state.active = 1;
}

void updateAudio() {
    if(!audio_state.active) return;
    if(!FIFOspace()) return;

    int sample = 0;

    if(audio_state.frequency != 0) {
        sample = audio_state.is_high ? audio_state.volume : -audio_state.volume;

        if(++audio_state.counter >= audio_state.half_period) {
            audio_state.counter = 0;
            audio_state.is_high ^= 1;
        }
    }

    audio->LEFTDATA = sample;
    audio->RIGHTDATA = sample;

    if(++audio_state.sample_index >= audio_state.total_samples)
        audio_state.active = 0;
}

// ================= TAP FSM =================
void updateTap(TapState *t, GPIO_t *gpio, int pin) {
    int curr = digitalRead(gpio, pin);

    if(curr && !t->last_state) {
        t->tap_count++;
        t->timer = 0;
    }

    t->last_state = curr;

    if(t->tap_count > 0) {
        t->timer++;

        if(t->timer > 4000) {
            if(t->tap_count == 1) t->event = 1;
            else if(t->tap_count >= 3) t->event = 3;

            t->tap_count = 0;
            t->timer = 0;
        }
    }
}

// ================= FALL DETECTION =================
bool detectFall(float d1, float d2, float p1, float p2) {
    if(d1 <= 0 || d2 <= 0 || p1 <= 0 || p2 <= 0) return false;

    float c1 = d1 - p1;
    float c2 = d2 - p2;

    if(c1 < 0) c1 = -c1;
    if(c2 < 0) c2 = -c2;

    return (c1 > 20.0 || c2 > 20.0);
}

// ================= MAIN =================
int main() {

    audio->CONTROL = 0xC;
    audio->CONTROL = 0x0;

    GPIO_t *jp2 = (GPIO_t *)JP2_BASE;

    HCSR04_t s1 = {jp2,5,6};
    HCSR04_t s2 = {jp2,3,4};

    pinMode(jp2,5,1);
    pinMode(jp2,6,0);
    pinMode(jp2,3,1);
    pinMode(jp2,4,0);
    pinMode(jp2,2,0); // vibration

    TapState tap = {0};
    SystemMode mode = MODE_NORMAL;

    float d1=0,d2=0,prev1=0,prev2=0;

    int sensor_timer = 0;
    int idle_counter = 0;
    int fall_cooldown = 0;
    int system_ready = 0;

    while(1) {

        // --- ALWAYS RUN ---
        updateTap(&tap, jp2, 2);
        updateAudio();

        if(fall_cooldown > 0) fall_cooldown--;

        // --- SENSOR UPDATE ---
        if(++sensor_timer > 20000) {
            d1 = distance(&s1);
            d2 = distance(&s2);
            sensor_timer = 0;

            if(d1 > 0 && d2 > 0) system_ready = 1;
        }

        // --- TAP EVENTS ---
        if(tap.event == 1) {
            if(mode == MODE_NORMAL) {
                mode = MODE_ASSIST;
                show_assist();
                startTone(800,300,0x3FFFFFFF);
                fall_cooldown = 200000;
            }
            tap.event = 0;
        }

        if(tap.event == 3) {
            if(mode == MODE_ASSIST) {
                mode = MODE_NORMAL;
                show_disengage();
                fall_cooldown = 200000;
            }
            tap.event = 0;
        }

        // --- FALL DETECTION ---
        if(system_ready && fall_cooldown == 0 && mode == MODE_NORMAL) {
            if(detectFall(d1,d2,prev1,prev2)) {
                mode = MODE_ASSIST;
                show_assist();
                startTone(600,500,0x3FFFFFFF);
                fall_cooldown = 200000;
            }
        }

        // --- SLOW FALL ---
        float c1 = d1 - prev1;
        float c2 = d2 - prev2;
        if(c1<0) c1=-c1;
        if(c2<0) c2=-c2;

        if(c1<2 && c2<2) idle_counter++;
        else idle_counter=0;

        if(idle_counter > 50 && mode == MODE_NORMAL && fall_cooldown == 0) {
            mode = MODE_ASSIST;
            show_assist();
            startTone(600,500,0x3FFFFFFF);
            fall_cooldown = 200000;
            idle_counter = 0;
        }

        prev1 = d1;
        prev2 = d2;

        // --- NORMAL MODE ---
        if(mode == MODE_NORMAL) {

            if(d1>0 && d1<=30 && !audio_state.active)
                startTone(1000,100,0x3FFFFFFF);

            if(d2>=18 && !audio_state.active)
                startTone(600,300,0x3FFFFFFF);

            display_distance1(d1);
            display_distance2(d2);
        }
    }
}
