/*******************************************************************************
 * Assistive walking aid — DE1-SoC bare-metal (ARM Cortex-A9)
 *
 * Vibration tap detection uses the A9 GIC + JP2 PIO edge-capture interrupt.
 * No Nios II HAL headers required.
 *
 * JP2 pin mapping (bit numbers in the PIO DATA/DDR registers):
 *   bit 2 — vibration sensor (input)
 *   bit 3 — sensor 2 TRIG   (output)
 *   bit 4 — sensor 2 ECHO   (input)
 *   bit 5 — sensor 1 TRIG   (output)
 *   bit 6 — sensor 1 ECHO   (input)
 ******************************************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

// ─────────────────────────────────────────
// BASE ADDRESSES  (from system_info.h)
// ─────────────────────────────────────────
#define JP2_BASE        0xFF200070
#define HEX3_HEX0_BASE  0xFF200020
#define HEX5_HEX4_BASE  0xFF200030
#define AUDIO_BASE      0xFF203040

// ARM A9 GIC base addresses on the DE1-SoC
#define MPCORE_BASE         0xFFFEC000
#define GIC_CPUIF_BASE      (MPCORE_BASE + 0x0100)   // CPU interface
#define GIC_DIST_BASE       (MPCORE_BASE + 0x1000)   // Distributor

// JP2 PIO IRQ number in the GIC (FPGA soft-IP IRQ 13 → GIC ID 45 on DE1-SoC)
// JP2 is mapped to HPS IRQ 45.  Verify with your Qsys/Platform Designer
// address map if this ever doesn't work — it is 45 on the standard DE1-SoC.
#define JP2_IRQ_ID          45

#define SAMPLE_RATE         8000

// ─────────────────────────────────────────
// PERIPHERAL STRUCTS
// ─────────────────────────────────────────
typedef struct {
    volatile uint32_t CONTROL;
    volatile uint32_t FIFOSPACE;
    volatile uint32_t LEFTDATA;
    volatile uint32_t RIGHTDATA;
} AUDIO_t;

// Altera PIO core register map (each field is one 32-bit word)
typedef struct {
    volatile uint32_t DATA;      // +0x00  read inputs / write outputs
    volatile uint32_t DDR;       // +0x04  direction: 1=output, 0=input
    volatile uint32_t _pad0;     // +0x08  (port B — unused)
    volatile uint32_t _pad1;     // +0x0C  (port B DDR — unused)
    volatile uint32_t INTMASK;   // +0x10  interrupt enable mask
    volatile uint32_t EDGECAP;   // +0x14  edge capture (write 1 to clear)
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

// ─────────────────────────────────────────
// PERIPHERAL POINTERS
// ─────────────────────────────────────────
AUDIO_t      *audio = (AUDIO_t *)AUDIO_BASE;
volatile int *hex   = (volatile int *)HEX3_HEX0_BASE;
volatile int *hex54 = (volatile int *)HEX5_HEX4_BASE;
GPIO_t       *jp2   = (GPIO_t *)JP2_BASE;

// ─────────────────────────────────────────
// TAP DETECTION GLOBALS
// Written by the ISR, read by main — must be volatile.
// ─────────────────────────────────────────
volatile int g_tap_count = 0;  // taps accumulated in current window
volatile int g_tap_armed = 0;  // 1 = window open
volatile int g_tap_ready = 0;  // 1 = window expired, main loop must act
volatile int g_tap_timer = 0;  // ticked by main loop each iteration

// Number of main-loop iterations before the tap window closes.
// Each iteration ≈ 20-40 ms (two sensor reads + delays, no audio).
// 40 ticks ≈ 0.8–1.6 s — plenty of time for a triple-tap.
// If audio is playing, iterations are longer; window stays open longer,
// which is fine — the ISR still catches every tap instantly.
#define TAP_TIMER_MAX   40

// ─────────────────────────────────────────
// 7-SEGMENT FONT
// ─────────────────────────────────────────
const unsigned char seg7[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// ─────────────────────────────────────────
// GIC HELPERS  (bare-metal A9)
// ─────────────────────────────────────────

// Write to a GIC Distributor register
static inline void gic_dist_write(uint32_t offset, uint32_t val) {
    *(volatile uint32_t *)(GIC_DIST_BASE + offset) = val;
}
// Read from a GIC Distributor register
static inline uint32_t gic_dist_read(uint32_t offset) {
    return *(volatile uint32_t *)(GIC_DIST_BASE + offset);
}
// Write to a GIC CPU Interface register
static inline void gic_cpu_write(uint32_t offset, uint32_t val) {
    *(volatile uint32_t *)(GIC_CPUIF_BASE + offset) = val;
}

// Configure and enable a single SPI interrupt in the GIC.
// irq_id : GIC interrupt ID (e.g. 45 for JP2)
// priority: 0x00 (highest) – 0xF8 (lowest), must be even
void gic_enable_irq(uint32_t irq_id, uint8_t priority) {
    // 1. Set priority (8 bits per IRQ, packed 4 per word)
    uint32_t reg   = (irq_id / 4) * 4;   // byte offset into IPRIORITYR array
    uint32_t shift = (irq_id % 4) * 8;
    uint32_t val   = gic_dist_read(0x400 + reg);
    val &= ~(0xFFu << shift);
    val |=  ((uint32_t)priority << shift);
    gic_dist_write(0x400 + reg, val);

    // 2. Set target CPU (route to CPU0 — bit 0)
    reg   = (irq_id / 4) * 4;
    shift = (irq_id % 4) * 8;
    val   = gic_dist_read(0x800 + reg);
    val &= ~(0xFFu << shift);
    val |=  (0x01u << shift);
    gic_dist_write(0x800 + reg, val);

    // 3. Set edge-triggered (bit pair: 10b) in ICFGR
    reg   = (irq_id / 16) * 4;
    shift = (irq_id % 16) * 2;
    val   = gic_dist_read(0xC00 + reg);
    val |=  (0x2u << shift);   // edge-sensitive
    gic_dist_write(0xC00 + reg, val);

    // 4. Enable (set bit in ISENABLER)
    gic_dist_write(0x100 + (irq_id / 32) * 4, (1u << (irq_id % 32)));
}

// Initialise the GIC distributor and CPU interface.
// Call once before enabling any individual IRQ.
void gic_init(void) {
    // Distributor: enable group 0
    gic_dist_write(0x000, 0x1);

    // CPU interface: set priority mask to allow all priorities,
    // then enable signalling to this CPU.
    gic_cpu_write(0x004, 0xF0);   // priority mask (0xF0 = allow prio < 0xF0)
    gic_cpu_write(0x000, 0x1);    // enable CPU interface
}

// Call at the end of every ISR to acknowledge and deactivate the interrupt.
// Returns the IAR value (contains the IRQ ID).
uint32_t gic_ack(void) {
    return *(volatile uint32_t *)(GIC_CPUIF_BASE + 0x00C);  // ICCIAR
}

void gic_eoi(uint32_t iar) {
    *(volatile uint32_t *)(GIC_CPUIF_BASE + 0x010) = iar;   // ICCEOIR
}

// ─────────────────────────────────────────
// A9 EXCEPTION VECTOR TABLE SETUP
// The A9 requires you to place a branch to your IRQ handler at
// vector address 0x18 (IRQ exception).  On the DE1-SoC the
// exception vector table lives in on-chip RAM starting at 0xFFFF0000.
// The simplest approach: write a branch instruction directly there.
// ─────────────────────────────────────────

// Forward declaration — defined below
void __attribute__((interrupt("IRQ"))) irq_handler(void);

void setup_irq_vector(void) {
    // ARM branch instruction: B <target>
    // Opcode: 0xEA000000 | ((offset - 8) >> 2)
    // Vector table IRQ slot is at 0xFFFF0018.
    // We place a branch from 0xFFFF0018 to irq_handler.
    volatile uint32_t *vec_irq = (volatile uint32_t *)0xFFFF0018;
    uint32_t handler_addr = (uint32_t)irq_handler;
    uint32_t vec_addr     = 0xFFFF0018;
    // Branch offset: (target - (vec + 8)) >> 2
    int32_t offset = ((int32_t)handler_addr - (int32_t)(vec_addr + 8)) >> 2;
    *vec_irq = 0xEA000000u | (uint32_t)(offset & 0x00FFFFFF);
}

// Enable IRQ exceptions on the A9 (clear the I bit in CPSR)
void enable_irq(void) {
    __asm__ volatile (
        "MRS r0, CPSR\n\t"
        "BIC r0, r0, #0x80\n\t"   // clear I bit
        "MSR CPSR, r0\n\t"
        ::: "r0"
    );
}

// ─────────────────────────────────────────
// VIBRATION ISR  (called from irq_handler)
// ─────────────────────────────────────────
void vibration_isr(void) {
    // Clear the PIO edge-capture bit first so the IRQ de-asserts.
    jp2->EDGECAP = (1u << 2);

    if (!g_tap_armed) {
        g_tap_count = 1;
        g_tap_armed = 1;
        g_tap_timer = 0;
        g_tap_ready = 0;
    } else {
        g_tap_count++;
        g_tap_timer = 0;   // extend window on each new tap
    }
}

// ─────────────────────────────────────────
// TOP-LEVEL IRQ HANDLER
// The A9 jumps here on every IRQ exception.
// Read the GIC IAR to find which interrupt fired, dispatch, then EOI.
// ─────────────────────────────────────────
void __attribute__((interrupt("IRQ"))) irq_handler(void) {
    uint32_t iar    = gic_ack();        // acknowledge; get IRQ ID
    uint32_t irq_id = iar & 0x3FF;     // bits [9:0] = interrupt ID

    if (irq_id == JP2_IRQ_ID) {
        vibration_isr();
    }
    // Add else-if branches here for any other IRQs you add later.

    gic_eoi(iar);                       // signal end-of-interrupt
}

// ─────────────────────────────────────────
// TAP WINDOW TICK  (called once per main loop iteration)
// ─────────────────────────────────────────
void tick_tap_timer(void) {
    if (!g_tap_armed) return;
    g_tap_timer++;
    if (g_tap_timer >= TAP_TIMER_MAX) {
        g_tap_ready = 1;
        g_tap_armed = 0;
        g_tap_timer = 0;
    }
}

// ─────────────────────────────────────────
// GPIO HELPERS
// ─────────────────────────────────────────
void pinMode(GPIO_t *gpio, int pin, int mode) {
    if (mode == 1) gpio->DDR |=  (1u << pin);
    else           gpio->DDR &= ~(1u << pin);
}

void digitalWrite(GPIO_t *gpio, int pin, int val) {
    if (val) gpio->DATA |=  (1u << pin);
    else     gpio->DATA &= ~(1u << pin);
}

int digitalRead(GPIO_t *gpio, int pin) {
    return (gpio->DATA >> pin) & 1;
}

void delay(int us) {
    volatile int i;
    for (i = 0; i < us * 40; i++);
}

// ─────────────────────────────────────────
// HEX DISPLAY
// ─────────────────────────────────────────
void display_distance1(float d1) {
    int s1 = (int)d1;
    if (s1 > 999) s1 = 999;
    if (s1 < 0)   s1 = 0;
    *hex54 = (seg7[(s1 / 100) % 10] << 8) | seg7[(s1 / 10) % 10];
    *hex   = (*hex & 0x00FFFFFF) | (seg7[s1 % 10] << 24);
}

void display_distance2(float d2) {
    int s2 = (int)d2;
    if (s2 > 999) s2 = 999;
    if (s2 < 0)   s2 = 0;
    *hex = (*hex & 0xFF000000)
         | (seg7[(s2 / 100) % 10] << 16)
         | (seg7[(s2 / 10)  % 10] <<  8)
         |  seg7[s2 % 10];
}

void show_assist(void) {
    *hex54 = 0x00000000;
    *hex   = (0x77 << 24) | (0x6D << 16) | (0x6D << 8) | 0x78;
}

void show_disengage(void) {
    *hex54 = 0x00000000;
    *hex   = (0x5E << 24) | (0x06 << 16) | (0x6D << 8) | 0x79;
}

// ─────────────────────────────────────────
// ULTRASONIC SENSOR
// ─────────────────────────────────────────
void trigger(HCSR04_t *sensor) {
    digitalWrite(sensor->gpio, sensor->trig_pin, 0);
    delay(2);
    digitalWrite(sensor->gpio, sensor->trig_pin, 1);
    delay(15);
    digitalWrite(sensor->gpio, sensor->trig_pin, 0);
}

float distance_read(HCSR04_t *sensor) {
    trigger(sensor);
    int timeout = 1000000;
    while (digitalRead(sensor->gpio, sensor->echo_pin) == 0 && timeout > 0) timeout--;
    if (timeout <= 0) return -1.0f;
    volatile int count = 0;
    while (digitalRead(sensor->gpio, sensor->echo_pin) == 1) count++;
    return (float)count / 150.0f;
}

// ─────────────────────────────────────────
// AUDIO
// ─────────────────────────────────────────
static bool FIFOspace(void) {
    uint32_t fifospace = audio->FIFOSPACE;
    return ((fifospace >> 24) & 0xFF) > 0 && ((fifospace >> 16) & 0xFF) > 0;
}

void play_sample(int sample) {
    while (!FIFOspace());
    audio->LEFTDATA  = sample;
    audio->RIGHTDATA = sample;
}

void play_tone(int frequency, int duration_ms, int VOLUME) {
    if (frequency == 0) {
        int total = (SAMPLE_RATE * duration_ms) / 1000;
        for (int i = 0; i < total; i++) play_sample(0);
        return;
    }
    int half = SAMPLE_RATE / frequency / 2;
    int total = (SAMPLE_RATE * duration_ms) / 1000;
    int cnt = 0;
    bool hi = true;
    for (int i = 0; i < total; i++) {
        play_sample(hi ? VOLUME : -VOLUME);
        if (++cnt >= half) { hi = !hi; cnt = 0; }
    }
}

void play_obstacle_alert(int V) { play_tone(1000, 100, V); play_tone(0, 100, V); }
void play_ledge_alert(int V)    { play_tone(300,  400, V); play_tone(0, 100, V); }
void play_emergency_siren(int V){ play_tone(600,  300, V); play_tone(800, 300, V); }

// ─────────────────────────────────────────
// FALL DETECTION
// ─────────────────────────────────────────
bool detectFall(float c1, float c2, float p1, float p2) {
    float d1 = c1 - p1; if (d1 < 0) d1 = -d1;
    float d2 = c2 - p2; if (d2 < 0) d2 = -d2;
    return (d1 > 20.0f || d2 > 20.0f);
}

// ─────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────
int main(void) {
    audio->CONTROL = 0xC;
    audio->CONTROL = 0x0;

    unsigned int volume = 0;
    int  assist_mode    = 0;
    int  idle_count     = 0;
    float prev_d1       = 0.0f;
    float prev_d2       = 0.0f;

    // ── Sensor pin setup ────────────────────────────────────────────────
    HCSR04_t sensor1 = { jp2, 5, 6 };
    pinMode(jp2, sensor1.trig_pin, 1);
    pinMode(jp2, sensor1.echo_pin, 0);

    HCSR04_t sensor2 = { jp2, 3, 4 };
    pinMode(jp2, sensor2.trig_pin, 1);
    pinMode(jp2, sensor2.echo_pin, 0);

    VIBRATION_t vib = { jp2, 2 };
    pinMode(jp2, vib.pin, 0);

    *hex   = 0x00000000;
    *hex54 = 0x00000000;

    // ── Interrupt setup ─────────────────────────────────────────────────
    // 1. Install the IRQ vector into the A9 exception table
    setup_irq_vector();

    // 2. Initialise the GIC distributor and CPU interface
    gic_init();

    // 3. Enable the JP2 PIO interrupt (GIC ID 45, priority 0xA0)
    gic_enable_irq(JP2_IRQ_ID, 0xA0);

    // 4. Configure JP2 PIO to generate an interrupt on rising edge of bit 2
    jp2->EDGECAP = (1u << 2);   // clear any stale edge first
    jp2->INTMASK = (1u << 2);   // unmask bit 2 → PIO drives its IRQ line

    // 5. Enable IRQ exceptions on the A9 core
    enable_irq();

    // ────────────────────────────────────────────────────────────────────
    while (1) {
        // ── Read sensors ────────────────────────────────────────────────
        float dist1 = distance_read(&sensor1);
        delay(10000);
        float dist2 = distance_read(&sensor2);
        delay(10000);

        // ── Advance tap window clock ─────────────────────────────────────
        tick_tap_timer();

        // ── Act when tap window closes ───────────────────────────────────
        if (g_tap_ready) {
            int taps    = g_tap_count;
            g_tap_ready = 0;
            g_tap_count = 0;

            if (detectFall(dist1, dist2, prev_d1, prev_d2)) {
                assist_mode = 1;
                show_assist();
                play_emergency_siren(0x3FFFFFFF);
            }
            else if (taps <= 2 && !assist_mode) {
                assist_mode = 1;
                show_assist();
                play_emergency_siren(0x3FFFFFFF);
            }
            else if (taps >= 3 && assist_mode) {
                assist_mode = 0;
                show_disengage();
                delay(1000000);
                *hex   = 0x00000000;
                *hex54 = 0x00000000;
            }
        }

        // ── Slow / idle fall detection ───────────────────────────────────
        float c1 = dist1 - prev_d1; if (c1 < 0) c1 = -c1;
        float c2 = dist2 - prev_d2; if (c2 < 0) c2 = -c2;

        if (c1 < 2.0f && c2 < 2.0f) idle_count++;
        else                          idle_count = 0;

        if (idle_count >= 50 && !assist_mode) {
            assist_mode = 1;
            show_assist();
            play_emergency_siren(0x3FFFFFFF);
            idle_count = 0;
        }

        prev_d1 = dist1;
        prev_d2 = dist2;

        // ── Obstacle / ledge alerts (only outside assist mode) ───────────
        if (!assist_mode) {
            if (dist1 > 0 && dist1 <= 30) {
                if      (dist1 > 20) volume = 0x3FFFFFFF / 4;
                else if (dist1 > 10) volume = 0x3FFFFFFF / 2;
                else                 volume = 0x3FFFFFFF;
                play_obstacle_alert(volume);
            }

            if      (dist2 >= 18.0f) play_emergency_siren(0x3FFFFFFF);
            else if (dist2 >= 10.0f) play_ledge_alert(0x3FFFFFFF);
            else if (dist2 >=  5.0f) play_ledge_alert(0x3FFFFFFF / 2);
            else if (dist2 >=  3.0f) play_ledge_alert(0x3FFFFFFF / 4);

            display_distance1(dist1);
            display_distance2(dist2);
        }
    }

    return 0;
}
