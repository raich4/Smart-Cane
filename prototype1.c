/*
 * ============================================================
 *  Smart Assistive Mobility Cane — Fully Integrated System
 *  DE1-SoC (RISC-V / Nios II)
 *
 *  Combines:
 *    - HC-SR04 dual ultrasonic sensors  (forward + downward)
 *    - Vibration-tap emergency trigger
 *    - Fall detection state machine      (fast + slow fall)
 *    - .h PCM audio playback             (object / edge / assistance)
 *    - VGA split-screen companion UI     (Apple Watch + Mobile App)
 *    - Live sensor distance on VGA
 *    - HEX LED status display
 *
 *  Changes from doc9:
 *    1. distance_read() replaced with distance_read_vib() which polls
 *       the vibration pin inside both echo-wait loops and returns -2
 *       immediately when a tap is detected, so sensor reads never block
 *       tap detection even when sensor 1 sees open air.
 *    2. countTaps() debounce 10ms→50ms (kills sensor ringing that was
 *       inflating tap counts), window 8000→2000 iters (800ms→200ms).
 *    3. Perpetual siren moved to end of loop (step 6) so vibration
 *       check always runs first on every iteration.
 *    4. Blocking for(i<3) siren on fast fall replaced with single cycle;
 *       step 6 handles repetition without blocking.
 * ============================================================
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* ── .h PCM SOUND FILES ─────────────────────────────────────────────────── */
#include "assistancesound.h"
#include "edgesound.h"
#include "objectsound.h"

/* ── MEMORY-MAP ADDRESSES ───────────────────────────────────────────────── */
#define JP2_BASE          0xFF200070
#define HEX3_HEX0_BASE    0xFF200020
#define HEX5_HEX4_BASE    0xFF200030
#define AUDIO_BASE        0xFF203040
#define PIXEL_CTRL_BASE   0xFF203020
#define CHAR_BUFFER_BASE  0x09000000

/* ── SENSOR / FALL TUNING ───────────────────────────────────────────────── */
#define SAMPLE_RATE         8000

#define MOUNT_HEIGHT        12.0f
#define LEDGE_WARN_LOW      (MOUNT_HEIGHT + 3.0f)
#define LEDGE_WARN_MID      (MOUNT_HEIGHT + 6.0f)
#define LEDGE_WARN_HIGH     (MOUNT_HEIGHT + 10.0f)
#define LEDGE_DANGER        (MOUNT_HEIGHT + 15.0f)
#define S2_OPEN_AIR         (MOUNT_HEIGHT + 25.0f)
#define FAST_FALL_VELOCITY  15.0f
#define SLOW_FALL_SUSTAIN   12
#define BASELINE_MARGIN     (MOUNT_HEIGHT + 4.0f)
#define LEDGE_RETURN_WINDOW 8

/* ── VGA COLOUR PALETTE (RGB565) ────────────────────────────────────────── */
#define BLACK       0x0000
#define WHITE       0xFFFF
#define RED         0xF800
#define CYAN        0x07FF
#define LIGHT_BG    0xDF1E
#define DARK_GREY   0x3186
#define MED_GREY    0x8410
#define STRAP_COLOR 0x18E3
#define SAFE_BG     0x2124

/* ── PERIPHERAL STRUCTS ─────────────────────────────────────────────────── */
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

/* ── PERIPHERAL POINTERS ────────────────────────────────────────────────── */
AUDIO_t          *audio   = (AUDIO_t *)          AUDIO_BASE;
volatile int     *hex     = (volatile int *)      HEX3_HEX0_BASE;
volatile int     *hex54   = (volatile int *)      HEX5_HEX4_BASE;
GPIO_t           *jp2     = (GPIO_t *)            JP2_BASE;
volatile int      pixel_buffer_start;

/* ── 7-SEGMENT LUT ──────────────────────────────────────────────────────── */
const unsigned char seg7[] = {
    0x3F,0x06,0x5B,0x4F,0x66,
    0x6D,0x7D,0x07,0x7F,0x6F
};

/* ══════════════════════════════════════════════════════════════════════════
 *  GPIO HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */
void pinMode(GPIO_t *gpio, int pin, int mode) {
    if (mode == 1) gpio->DDR |=  (1 << pin);
    else           gpio->DDR &= ~(1 << pin);
}
void digitalWrite(GPIO_t *gpio, int pin, int val) {
    if (val) gpio->DATA |=  (1 << pin);
    else     gpio->DATA &= ~(1 << pin);
}
int digitalRead(GPIO_t *gpio, int pin) { return (gpio->DATA >> pin) & 1; }

void delay_us(int us) {
    volatile int i;
    for (i = 0; i < us * 40; i++);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  HEX DISPLAY
 * ══════════════════════════════════════════════════════════════════════════ */
void display_distance1(int cm) {
    if (cm > 999) cm = 999;
    if (cm < 0)   cm = 0;
    *hex54 = (seg7[(cm/100)%10] << 8) | seg7[(cm/10)%10];
    *hex   = (*hex & 0x00FFFFFF) | (seg7[cm%10] << 24);
}
void display_distance2(int cm) {
    if (cm > 999) cm = 999;
    if (cm < 0)   cm = 0;
    *hex = (*hex & 0xFF000000)
         | (seg7[(cm/100)%10] << 16)
         | (seg7[(cm/10)%10]  <<  8)
         |  seg7[cm%10];
}
void display_error1(void) {
    *hex54 = (0x79 << 8) | 0x50;
    *hex   = (*hex & 0x00FFFFFF) | (0x50 << 24);
}
void display_error2(void) {
    *hex = (*hex & 0xFF000000) | (0x79 << 16) | (0x50 << 8) | 0x50;
}
void show_fall(void) {
    *hex54 = (0x71 << 8) | 0x77;
    *hex   = (0x38 << 24) | (0x38 << 16);
}
void show_slow_fall(void) {
    *hex54 = (0x6D << 8) | 0x38;
    *hex   = (*hex & 0x00FFFFFF) | (0x3F << 24);
}
void show_assist(void) {
    *hex54 = 0x00000000;
    *hex   = (0x77 << 24) | (0x6D << 16) | (0x6D << 8) | 0x78;
}
void show_disengage(void) {
    *hex54 = 0x00000000;
    *hex   = (0x5E << 24) | (0x06 << 16) | (0x6D << 8) | 0x79;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  AUDIO — PCM PLAYBACK FROM .h ARRAYS
 * ══════════════════════════════════════════════════════════════════════════ */
static inline bool fifo_has_space(void) {
    unsigned int fs = audio->FIFOSPACE;
    return ((fs >> 24) & 0xFF) > 0 && ((fs >> 16) & 0xFF) > 0;
}
void play_pcm(const int *array, int length, int vol_shift) {
    for (int i = 0; i < length; i++) {
        while (!fifo_has_space());
        int sample = array[i] >> vol_shift;
        audio->LEFTDATA  = sample;
        audio->RIGHTDATA = sample;
    }
}
void play_obstacle_alert(int vol_shift) {
    play_pcm(objectsound_array, OBJECTSOUND_ARRAY_LENGTH, vol_shift);
}
void play_ledge_alert(int vol_shift) {
    play_pcm(edgesound_array, EDGESOUND_ARRAY_LENGTH, vol_shift);
}
void play_emergency_siren(void) {
    play_pcm(assistancesound_array, ASSISTANCESOUND_ARRAY_LENGTH, 0);
}
void play_slow_fall_alert(void) {
    play_pcm(assistancesound_array, ASSISTANCESOUND_ARRAY_LENGTH, 0);
    play_pcm(assistancesound_array, ASSISTANCESOUND_ARRAY_LENGTH, 0);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  VIBRATION / TAP SENSOR
 *
 *  countTaps changes:
 *    - debounce 10ms → 50ms: vibration sensors ring for ~20ms after
 *      impact; 10ms wasn't enough, so ringing was counted as extra taps.
 *    - window 8000 → 2000 iters (800ms → 200ms): shorter window means
 *      less time for noise to accumulate between taps.
 * ══════════════════════════════════════════════════════════════════════════ */
int vibrationRead(VIBRATION_t *vib) { return digitalRead(vib->gpio, vib->pin); }

int countTaps(GPIO_t *gpio, int pin) {
    int count   = 1;
    int timeout = 0;
    while (digitalRead(gpio, pin));
    delay_us(50000);                  // 50ms debounce — kills all sensor ringing
    while (timeout < 2000) {          // 200ms inter-tap window
        if (digitalRead(gpio, pin)) {
            count++;
            while (digitalRead(gpio, pin));
            delay_us(50000);
            timeout = 0;
        }
        delay_us(100);
        timeout++;
    }
    return count;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  HC-SR04 — VIBRATION-AWARE DISTANCE READ
 *
 *  Polls the vibration pin (bit 2) inside both echo-wait loops.
 *  If a tap fires mid-read, sets *vib_triggered = 1 and returns -2
 *  immediately from the pre-echo wait (sensor 1 open-air timeout was
 *  the main culprit blocking tap detection while the cane is moving).
 *  During the echo-count phase the tap is flagged but counting finishes
 *  so the sensor resets cleanly.
 *
 *  Returns:
 *    >= 0  normal distance in cm
 *    -1    sensor timeout
 *    -2    tap detected during pre-echo wait (aborted)
 * ══════════════════════════════════════════════════════════════════════════ */
float distance_read_vib(HCSR04_t *s, int vib_pin, int *vib_triggered) {
    digitalWrite(s->gpio, s->trig_pin, 0);
    delay_us(2);
    digitalWrite(s->gpio, s->trig_pin, 1);
    delay_us(10);
    digitalWrite(s->gpio, s->trig_pin, 0);

    int timeout = 1200000;
    while (digitalRead(s->gpio, s->echo_pin) == 0 && timeout-- > 0) {
        if (digitalRead(s->gpio, vib_pin)) {
            *vib_triggered = 1;
            return -2.0f;   // abort immediately — tap is more urgent
        }
    }
    if (timeout <= 0) return -1.0f;

    volatile int count = 0;
    int echo_timeout   = 1200000;
    while (digitalRead(s->gpio, s->echo_pin) == 1 && echo_timeout-- > 0) {
        count++;
        if (digitalRead(s->gpio, vib_pin))
            *vib_triggered = 1;   // flag but finish echo so sensor resets
    }

    return (float)count / 150.0f;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  FALL DETECTION STATE MACHINE  (unchanged)
 * ══════════════════════════════════════════════════════════════════════════ */
typedef enum { STATE_NORMAL, STATE_ELEVATED, STATE_OPEN_AIR } FallState;

int detect_fall_state(float d_down) {
    static FallState state        = STATE_NORMAL;
    static float     prev_d       = -1.0f;
    static int       sustain_count = 0;
    static int       return_timer  = 0;

    if (prev_d < 0.0f) {
        prev_d = (d_down >= 0.0f) ? d_down : MOUNT_HEIGHT;
        return 0;
    }

    float d        = (d_down < 0.0f) ? (S2_OPEN_AIR + 10.0f) : d_down;
    float velocity = d - prev_d;
    prev_d = d;

    switch (state) {
        case STATE_NORMAL:
            if (d > S2_OPEN_AIR) {
                sustain_count = 1; return_timer = 0;
                state = STATE_OPEN_AIR;
                if (velocity >= FAST_FALL_VELOCITY) {
                    state = STATE_NORMAL; sustain_count = 0; prev_d = MOUNT_HEIGHT;
                    return 1;
                }
            } else if (d > LEDGE_WARN_LOW) {
                state = STATE_ELEVATED; return_timer = 0;
            }
            break;

        case STATE_ELEVATED:
            if (d <= BASELINE_MARGIN) {
                state = STATE_NORMAL; return_timer = 0;
            } else if (d > S2_OPEN_AIR) {
                sustain_count = 1; return_timer = 0;
                state = STATE_OPEN_AIR;
                if (velocity >= FAST_FALL_VELOCITY) {
                    state = STATE_NORMAL; sustain_count = 0; prev_d = MOUNT_HEIGHT;
                    return 1;
                }
            } else {
                return_timer++;
                if (return_timer > LEDGE_RETURN_WINDOW) {
                    state = STATE_OPEN_AIR;
                    sustain_count = return_timer;
                    return_timer  = 0;
                }
            }
            break;

        case STATE_OPEN_AIR:
            if (d <= BASELINE_MARGIN) {
                state = STATE_NORMAL; sustain_count = 0; return_timer = 0;
            } else if (d > S2_OPEN_AIR) {
                sustain_count++;
                if (velocity >= FAST_FALL_VELOCITY) {
                    state = STATE_NORMAL; sustain_count = 0; prev_d = MOUNT_HEIGHT;
                    return 1;
                }
                if (sustain_count >= SLOW_FALL_SUSTAIN) {
                    state = STATE_NORMAL; sustain_count = 0; return_timer = 0;
                    return 2;
                }
            } else {
                return_timer++;
                if (return_timer > LEDGE_RETURN_WINDOW) {
                    state = STATE_NORMAL; sustain_count = 0; return_timer = 0;
                    return 2;
                }
            }
            break;
    }
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  VGA DRIVERS  (unchanged)
 * ══════════════════════════════════════════════════════════════════════════ */
void plot_pixel(int x, int y, short int color) {
    if (x >= 0 && x < 320 && y >= 0 && y < 240) {
        volatile short int *addr =
            (short int *)(pixel_buffer_start + (y << 10) + (x << 1));
        *addr = color;
    }
}
void write_char(int x, int y, char c) {
    if (x >= 0 && x < 80 && y >= 0 && y < 60) {
        volatile char *character_buffer =
            (char *)(CHAR_BUFFER_BASE + (y << 7) + x);
        *character_buffer = c;
    }
}
void write_string(int x, int y, char *str) {
    for (int i = 0; str[i] != '\0'; i++)
        write_char(x + i, y, str[i]);
}
void draw_rect(int x1, int y1, int x2, int y2, short int color) {
    for (int y = y1; y <= y2; y++)
        for (int x = x1; x <= x2; x++)
            plot_pixel(x, y, color);
}
void clear_text(void) {
    for (int y = 0; y < 60; y++)
        for (int x = 0; x < 80; x++)
            write_char(x, y, ' ');
}
void draw_static_chassis(void) {
    draw_rect(0,   0, 319, 239, LIGHT_BG);
    draw_rect(0,   0, 319,  18, DARK_GREY);
    draw_rect(158, 0, 161, 239, CYAN);
    draw_rect( 55,  25, 105, 239, STRAP_COLOR);
    draw_rect( 35,  70, 125, 180, MED_GREY);
    draw_rect( 38,  73, 122, 177, BLACK);
    draw_rect(180,  25, 300, 235, WHITE);
    draw_rect(185,  40, 295, 220, BLACK);
    draw_rect(220,  30, 260,  35, MED_GREY);
    draw_rect(230, 225, 250, 230, MED_GREY);
    write_string(14,  1, "APPLE WATCH");
    write_string(54,  1, "COMPANION APP");
}
void update_screens(int alert_type) {
    for (int y = 18; y < 45; y++)
        for (int x = 9;  x < 32; x++) write_char(x, y, ' ');
    for (int y = 12; y < 32; y++)
        for (int x = 47; x < 74; x++) write_char(x, y, ' ');
    for (int y = 39; y < 52; y++)
        for (int x = 47; x < 74; x++) write_char(x, y, ' ');
    draw_rect( 40,  75, 120, 175, SAFE_BG);
    draw_rect(187,  42, 293, 218, SAFE_BG);

    if (alert_type > 0) {
        draw_rect( 40, 108, 120, 142, WHITE);
        draw_rect( 42, 110, 118, 140, RED);
        draw_rect(190,  92, 290, 124, WHITE);
        draw_rect(192,  94, 288, 122, RED);
        write_string(14, 22, "EMERGENCY");
        write_string(13, 40, "Calling SOS");
        write_string(53, 14, "CRITICAL ALERT");
        write_string(53, 20, "Calling SOS...");
        write_string(52, 45, "Assistance mode");
        write_string(55, 48, "ACTIVATED");
        if (alert_type == 1) {
            write_string(15, 31, "Manual SOS");
            write_string(56, 24, "Reason:");
            write_string(53, 27, "User Manual SOS");
        } else if (alert_type == 2) {
            write_string(15, 31, "Drop >18cm");
            write_string(56, 24, "Reason:");
            write_string(53, 27, "Dangerous Drop");
        } else if (alert_type == 3) {
            write_string(15, 31, "Slow Fall");
            write_string(56, 24, "Reason:");
            write_string(51, 27, "Slow Fall Detected");
        } else if (alert_type == 4) {
            write_string(15, 31, "FAST FALL!");
            write_string(56, 24, "Reason:");
            write_string(51, 27, "FAST FALL Detected");
        }
    } else {
        write_string(16, 24, "12:00 PM");
        write_string(15, 29, "Status: OK");
        write_string(16, 33, "BPM: 72");
        write_string(55, 14, "App Active");
        write_string(53, 22, "User is safe.");
        write_string(52, 24, "Sensors normal.");
    }
    write_string(54, 50, "Battery: 85%");
}
void update_distance_display(int dist1, int dist2) {
    char buf[20];
    for (int row = 32; row <= 38; row++)
        for (int x = 47; x < 74; x++) write_char(x, row, ' ');
    write_string(52, 33, "=== SENSORS ===");
    if (dist1 < 0)
        write_string(51, 35, "Obstacle:  --- cm");
    else {
        sprintf(buf, "Obstacle: %3d cm", dist1);
        write_string(51, 35, buf);
    }
    if (dist2 < 0)
        write_string(51, 37, "    Drop:  --- cm");
    else {
        sprintf(buf, "    Drop: %3d cm", dist2);
        write_string(51, 37, buf);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  MAIN
 * ══════════════════════════════════════════════════════════════════════════ */
int main(void) {

    /* VGA init */
    volatile int *pixel_ctrl_ptr = (int *)PIXEL_CTRL_BASE;
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_text();
    draw_static_chassis();
    update_screens(0);
    update_distance_display(0, 0);

    /* Audio init */
    audio->CONTROL = 0xC;
    audio->CONTROL = 0x0;

    /* HEX startup indicator */
    *hex   = 0x3F3F3F3F;
    *hex54 = 0x00003F3F;

    /* Sensor 1 — forward  (trig=bit5, echo=bit6) */
    HCSR04_t sensor1 = {jp2, 5, 6};
    pinMode(jp2, sensor1.trig_pin, 1);
    pinMode(jp2, sensor1.echo_pin, 0);

    /* Sensor 2 — downward (trig=bit3, echo=bit4) */
    HCSR04_t sensor2 = {jp2, 3, 4};
    pinMode(jp2, sensor2.trig_pin, 1);
    pinMode(jp2, sensor2.echo_pin, 0);

    /* Vibration tap — bit2 */
    VIBRATION_t vib = {jp2, 2};
    pinMode(jp2, vib.pin, 0);

    digitalWrite(jp2, sensor1.trig_pin, 0);
    digitalWrite(jp2, sensor2.trig_pin, 0);

    delay_us(500000);

    *hex   = 0x00000000;
    *hex54 = 0x00000000;

    int assist_mode         = 0;
    int last_alert_state    = 0;
    int current_alert_state = 0;
    int vib_triggered       = 0;   // set by distance_read_vib on mid-read tap

    while (1) {

        /* ── 1. Read sensors, bailing immediately on tap ─────────────── */
        vib_triggered = 0;

        float dist_fwd  = distance_read_vib(&sensor1, 2, &vib_triggered);
        if (!vib_triggered) {
            float dist_down = distance_read_vib(&sensor2, 2, &vib_triggered);
            delay_us(500);

            int d_fwd  = (dist_fwd  < 0.0f) ? -1 : (int)dist_fwd;
            int d_down = (dist_down < 0.0f) ? -1 : (int)dist_down;

            /* ── 2. Vibration tap check ──────────────────────────────── */
            if (!vib_triggered && vibrationRead(&vib))
                vib_triggered = 1;

            if (vib_triggered) {
                int taps = countTaps(jp2, 2);
                if (taps <= 2 && !assist_mode) {
                    assist_mode         = 1;
                    current_alert_state = 1;
                    show_assist();
                    update_screens(1);
                    update_distance_display(d_fwd, d_down);
                    last_alert_state = 1;
                    play_emergency_siren();
                } else if (taps >= 3 && assist_mode) {
                    assist_mode         = 0;
                    current_alert_state = 0;
                    show_disengage();
                    update_screens(0);
                    update_distance_display(d_fwd, d_down);
                    last_alert_state = 0;
                    delay_us(500000);
                    *hex   = 0x00000000;
                    *hex54 = 0x00000000;
                }
            }

            /* ── 3. Fall detection — only when not in assist mode ─────── */
            if (!assist_mode) {
                int fall_state = detect_fall_state(dist_down);

                if (fall_state == 1) {
                    assist_mode         = 1;
                    current_alert_state = 4;
                    show_fall();
                    update_screens(4);
                    update_distance_display(d_fwd, d_down);
                    last_alert_state = 4;
                    play_emergency_siren();   // single cycle; step 6 repeats

                } else if (fall_state == 2) {
                    assist_mode         = 1;
                    current_alert_state = 3;
                    show_slow_fall();
                    update_screens(3);
                    update_distance_display(d_fwd, d_down);
                    last_alert_state = 3;
                    play_slow_fall_alert();
                }
            }

            /* ── 4. Normal sensor logic ──────────────────────────────── */
            if (!assist_mode) {

                if (dist_fwd  < 0) display_error1(); else display_distance1(d_fwd);
                if (dist_down < 0) display_error2(); else display_distance2(d_down);

                if (dist_down >= 0.0f) {
                    if (dist_down >= LEDGE_DANGER) {
                        assist_mode         = 1;
                        current_alert_state = 2;
                        show_assist();
                        update_screens(2);
                        update_distance_display(d_fwd, d_down);
                        last_alert_state = 2;
                        play_emergency_siren();
                    } else if (dist_down >= LEDGE_WARN_HIGH) {
                        play_ledge_alert(0);
                    } else if (dist_down >= LEDGE_WARN_MID) {
                        play_ledge_alert(1);
                    } else if (dist_down >= LEDGE_WARN_LOW) {
                        play_ledge_alert(2);
                    }
                }

                if (dist_fwd > 0.0f && dist_fwd <= 100.0f) {
                    int vol_shift;
                    if      (dist_fwd > 60) vol_shift = 2;
                    else if (dist_fwd > 30) vol_shift = 1;
                    else                    vol_shift = 0;
                    play_obstacle_alert(vol_shift);
                }

                current_alert_state = 0;
                if (current_alert_state != last_alert_state) {
                    update_screens(0);
                    last_alert_state = 0;
                }
            }

            /* ── 5. Always refresh VGA sensor distances ──────────────── */
            update_distance_display(d_fwd, d_down);

        } else {
            /* ── Tap fired during sensor 1 read — handle immediately ─── */
            int taps = countTaps(jp2, 2);
            if (taps <= 2 && !assist_mode) {
                assist_mode         = 1;
                current_alert_state = 1;
                show_assist();
                update_screens(1);
                last_alert_state = 1;
                play_emergency_siren();
            } else if (taps >= 3 && assist_mode) {
                assist_mode         = 0;
                current_alert_state = 0;
                show_disengage();
                update_screens(0);
                last_alert_state = 0;
                delay_us(500000);
                *hex   = 0x00000000;
                *hex54 = 0x00000000;
            }
        }

        /* ── 6. Perpetual siren — LAST so vibration always runs first ───
         *    One cycle plays, loop restarts at step 1.
         *    3-tap disengage is catchable on every iteration.          */
        if (assist_mode) {
            play_emergency_siren();
        }

    } /* while(1) */

    return 0;
}
