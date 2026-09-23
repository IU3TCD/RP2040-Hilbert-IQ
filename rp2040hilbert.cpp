#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "ws2812.pio.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Ingresso microfono -----------------------------------------------
// Mic analogico gia' polarizzato a meta' alimentazione (bias esterno).
#define MIC_ADC_GPIO    28
#define MIC_ADC_CHANNEL 2       // GPIO28 = ADC2
#define ADC_MIDPOINT    2048    // 2^12 / 2, centro scala dell'ADC 12 bit

// --- Uscite in quadratura -----------------------------------------------
// 0 e 90 gradi sono generati direttamente dal filtro di Hilbert (I/Q),
// 180 e 270 sono semplicemente le stesse uscite invertite di segno.
#define PIN_PHASE_0     9
#define PIN_PHASE_90    8
#define PIN_PHASE_180   7
#define PIN_PHASE_270   6

#define PWM_WRAP        1023   // risoluzione uscita ~10 bit, portante PWM a ~195 kHz (clk_sys/1024)

// --- Tasto CW (GPIO10, a massa quando premuto, pull-up interno) --------
#define PIN_CW_KEY      10
#define CW_TONE_HZ      700

// --- Interfaccia I2C slave (GPIO0=SDA, GPIO1=SCL) -----------------------
// Registro 0x00 (scrittura/lettura): modo   (0=AM, 1=USB, 2=LSB, 3=CW)
// Registro 0x01 (scrittura/lettura): sorgente (0=MIC, 1=TONO TEST, 2=DOPPIO TONO)
#define I2C_PORT         i2c0
#define PIN_I2C_SDA      0
#define PIN_I2C_SCL      1
#define I2C_SLAVE_ADDR   0x42
#define I2C_BAUDRATE     100000
#define I2C_REG_MODE     0x00
#define I2C_REG_SOURCE   0x01
#define I2C_REG_CARRIER  0x02   // 0-100, solo modo AM
#define I2C_REG_DEPTH    0x03   // 0-100, solo modo AM

// --- LED di stato RGB (WS2812 su GPIO16) ---------------------------------
#define PIN_STATUS_LED   16
static PIO status_led_pio = pio0;
static uint status_led_sm = 0;

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b; // ordine GRB
}

static inline void status_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    pio_sm_put_blocking(status_led_pio, status_led_sm, urgb_u32(r, g, b) << 8u);
}

// --- Overclock: 225 MHz (default RP2040 e' 125 MHz, 133 MHz e' il massimo
// garantito da datasheet) per avere piu' margine di CPU con NTAPS alto a 32 kHz.
// 250 MHz si e' rivelato instabile su questa scheda, sceso a 225 MHz.
#define SYS_CLOCK_KHZ   225000

// --- Campionamento / filtro Hilbert -------------------------------------
#define SAMPLE_RATE_HZ  32000
#define NTAPS           383                  // dispari, piu' tap = miglior comportamento alle basse frequenze audio
#define CENTER          ((NTAPS - 1) / 2)    // ritardo di gruppo del filtro, in campioni

// --- Tono di test (calibrazione con oscilloscopio, senza microfono) -----
#define TEST_TONE_HZ      1000
#define SINE_TABLE_BITS   8
#define SINE_TABLE_SIZE   (1 << SINE_TABLE_BITS)
static int16_t sine_table[SINE_TABLE_SIZE];

// --- Doppio tono (two-tone test per linearita'/IMD della catena SSB) ----
#define TWO_TONE_HZ_1     700
#define TWO_TONE_HZ_2     1800

// Rampa di salita/discesa del tono CW (raised cosine) per evitare i "click"
// di commutazione quando il tasto viene premuto/rilasciato di scatto.
#define CW_RAMP_MS      5
#define CW_RAMP_LEN     ((SAMPLE_RATE_HZ * CW_RAMP_MS) / 1000)
static int32_t cw_ramp_table[CW_RAMP_LEN + 1]; // 0 = silenzio, CW_RAMP_LEN = piena scala, formato Q15
static int cw_ramp_index = 0;

// Nel trasformatore di Hilbert meta' dei tap sono esattamente zero (tutti gli
// offset pari dal centro): si conservano solo i coefficienti non nulli, che
// per costruzione sono equispaziati di 2 posizioni a partire da nz_start.
#define MAX_NZ_TAPS     ((NTAPS + 1) / 2)
static int32_t nz_coef[MAX_NZ_TAPS];   // coefficienti non nulli, formato Q15
static int nz_count = 0;
static int nz_start = 0;               // indice (0..NTAPS-1) del primo tap non nullo

static int32_t delay[NTAPS];   // linea di ritardo circolare dei campioni centrati
static int dpos = 0;

// --- Stato condiviso fra i due core --------------------------------------
typedef enum { SRC_MIC = 0, SRC_TEST_TONE = 1, SRC_TWO_TONE = 2 } input_source_t;
static volatile input_source_t g_input_source = SRC_MIC;
static volatile bool g_debug_monitor = false;

// Modo operativo: determina come le uscite I/Q vengono combinate sulle 4 fasi,
// per pilotare un modulatore/mixer RF esterno in quadratura.
//  USB: uscite normali (0=I, 90=Q, 180=-I, 270=-Q).
//  LSB: quadratura invertita (0=I, 90=-Q, 180=-I, 270=Q) -> banda laterale opposta nel mixer esterno.
//  AM : bypass del filtro di Hilbert, tutte le uscite portano carrier_level + modulation_depth*I
//       (nessuno sfasamento) -- vedi g_carrier_level/g_modulation_depth piu' sotto.
//  CW : silenzio finche' il tasto (GPIO10, a massa) non viene premuto, poi tutte
//       le uscite portano il tono laterale a CW_TONE_HZ (nessuno sfasamento).
typedef enum { MODE_AM = 0, MODE_USB, MODE_LSB, MODE_CW } radio_mode_t;
static volatile radio_mode_t g_mode = MODE_USB;

// Parametri del modo AM: carrier_level (0-100%) e' l'offset costante (la
// "portante") aggiunto su tutte e 4 le fasi -- in un mixer a commutazione
// come un FST3253, un valore costante identico sui 4 canali viene
// ripresentato inalterato in uscita RF indipendentemente dalla fase
// selezionata, cioe' diventa portante. modulation_depth (0-100%) scala
// l'ampiezza dell'audio sommato sopra la portante.
static volatile uint8_t g_carrier_level = 0;      // 0% = DSB pura (comportamento AM precedente)
static volatile uint8_t g_modulation_depth = 100; // 100% = piena ampiezza audio

static const char *mode_name(radio_mode_t m)
{
    switch (m) {
        case MODE_AM:  return "AM";
        case MODE_USB: return "USB";
        case MODE_LSB: return "LSB";
        case MODE_CW:  return "CW";
        default:       return "?";
    }
}

// Colore del LED di stato associato al modo attivo.
static void mode_color(radio_mode_t m, uint8_t *r, uint8_t *g, uint8_t *b)
{
    switch (m) {
        case MODE_AM:  *r = 40; *g = 40; *b = 0;  break; // giallo
        case MODE_USB: *r = 0;  *g = 40; *b = 0;  break; // verde
        case MODE_LSB: *r = 0;  *g = 0;  *b = 40; break; // blu
        case MODE_CW:  *r = 40; *g = 0;  *b = 0;  break; // rosso
        default:       *r = 0;  *g = 0;  *b = 0;  break;
    }
}

typedef struct {
    int32_t raw_in;
    int32_t out0, out90, out180, out270;
} telemetry_t;
static volatile telemetry_t g_telemetry;

// Cattura di campioni CONSECUTIVI di I/Q (uscita grezza del filtro di Hilbert,
// prima della combinazione per modo) per verificare il verso reale dello
// sfasamento a 90 gradi. -1 = inattiva, 0..CAPTURE_LEN-1 = in corso,
// CAPTURE_LEN = pronta per la stampa da parte del core 0.
#define CAPTURE_LEN 64
static volatile int32_t capture_i[CAPTURE_LEN];
static volatile int32_t capture_q[CAPTURE_LEN];
static volatile int capture_index = -1;

// Trasformatore di Hilbert (tipo III, antisimmetrico) con finestra di Hamming:
// h[n] = 0 per n-CENTER pari, h[n] = 2/(pi*(n-CENTER)) per n-CENTER dispari.
// Si scartano subito i coefficienti nulli: non serve calcolarli a runtime.
static void generate_hilbert_coeffs(void)
{
    nz_count = 0;
    bool start_set = false;
    for (int n = 0; n < NTAPS; n++) {
        int k = n - CENTER;
        double h = (k == 0 || (k % 2) == 0) ? 0.0 : 2.0 / (M_PI * k);
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / (NTAPS - 1));
        int32_t coeff = (int32_t)lround(h * w * 32768.0);
        if (coeff != 0) {
            if (!start_set) { nz_start = n; start_set = true; }
            nz_coef[nz_count++] = coeff;
        }
    }
}

static void generate_sine_table(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        double phase = 2.0 * M_PI * i / SINE_TABLE_SIZE;
        sine_table[i] = (int16_t)lround(sin(phase) * 1500.0); // ampiezza paragonabile al segnale mic
    }
}

static void generate_cw_ramp(void)
{
    for (int i = 0; i <= CW_RAMP_LEN; i++) {
        double x = (double)i / CW_RAMP_LEN;
        double g = 0.5 * (1.0 - cos(M_PI * x)); // raised cosine, 0..1
        cw_ramp_table[i] = (int32_t)lround(g * 32768.0);
    }
}

// Inserisce un nuovo campione e calcola in-phase (0 gradi, semplice ritardo
// allineato al ritardo di gruppo del filtro) e quadrature (90 gradi, uscita FIR).
// Il ciclo scorre solo i tap non nulli (passo 2 in indice) e non contiene
// branch condizionali: il wrap della linea circolare e' una somma sola.
static inline void hilbert_step(int32_t sample, int32_t *i_out, int32_t *q_out)
{
    delay[dpos] = sample;

    int idx = dpos - nz_start;
    if (idx < 0) idx += NTAPS;

    int64_t acc = 0;
    for (int i = 0; i < nz_count; i++) {
        acc += (int64_t)nz_coef[i] * delay[idx];
        idx -= 2;
        if (idx < 0) idx += NTAPS;
    }
    *q_out = (int32_t)(acc >> 15);

    int center_idx = dpos - CENTER;
    if (center_idx < 0) center_idx += NTAPS;
    *i_out = delay[center_idx];

    dpos = (dpos + 1 == NTAPS) ? 0 : (dpos + 1);
}

// Dithering con error-feedback (noise shaping del primo ordine): l'errore di
// arrotondamento introdotto da >>2 (2 bit di dinamica ADC persi nel passaggio
// a PWM_WRAP=1023) non viene scartato ma riportato al campione successivo
// dello stesso canale, cosi' il rumore di quantizzazione medio si riduce
// invece di accumularsi come semplice troncamento.
static int32_t pwm_dither_error[4] = {0, 0, 0, 0}; // 0=fase0, 1=fase90, 2=fase180, 3=fase270

static inline uint16_t sample_to_duty(int32_t s, int channel)
{
    int32_t target_q2 = ((PWM_WRAP / 2) << 2) + s + pwm_dither_error[channel];
    int32_t duty = target_q2 >> 2;
    int32_t remainder = target_q2 - (duty << 2);

    if (duty < 0) { duty = 0; remainder = 0; }
    else if (duty > PWM_WRAP) { duty = PWM_WRAP; remainder = 0; }

    pwm_dither_error[channel] = remainder;
    return (uint16_t)duty;
}

static void setup_pwm_pin(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_enabled(slice, true);
}

// Somma di due toni di pari ampiezza (meta' ciascuno, cosi' il picco in fase
// resta paragonabile al tono singolo) per il two-tone test.
static inline int32_t two_tone_sample(void)
{
    static uint32_t phase1 = 0, phase2 = 0;
    static const uint32_t inc1 = (uint32_t)(((uint64_t)TWO_TONE_HZ_1 << 32) / SAMPLE_RATE_HZ);
    static const uint32_t inc2 = (uint32_t)(((uint64_t)TWO_TONE_HZ_2 << 32) / SAMPLE_RATE_HZ);
    phase1 += inc1;
    phase2 += inc2;
    int32_t s1 = sine_table[phase1 >> (32 - SINE_TABLE_BITS)];
    int32_t s2 = sine_table[phase2 >> (32 - SINE_TABLE_BITS)];
    return (s1 + s2) / 2;
}

// Chiamata solo dal core 1: legge il microfono oppure genera il tono di test
// (singolo o doppio), a seconda della sorgente selezionata da core 0.
static int32_t read_input_sample(void)
{
    if (g_input_source == SRC_TEST_TONE) {
        static uint32_t phase = 0;
        static const uint32_t increment = (uint32_t)(((uint64_t)TEST_TONE_HZ << 32) / SAMPLE_RATE_HZ);
        phase += increment;
        return sine_table[phase >> (32 - SINE_TABLE_BITS)];
    }
    if (g_input_source == SRC_TWO_TONE) {
        return two_tone_sample();
    }
    // Oversampling 2x: due letture ADC mediate per ridurre il rumore di
    // conversione (~0,5 bit effettivo in piu' a fronte di un costo CPU minimo).
    uint16_t raw1 = adc_read();
    uint16_t raw2 = adc_read();
    uint16_t raw = (uint16_t)((raw1 + raw2) >> 1);
    return (int32_t)raw - ADC_MIDPOINT;
}

// Nota di lato (sidetone) del tasto CW: stessa tabella seno del tono di
// test, incrementata a CW_TONE_HZ invece che a TEST_TONE_HZ.
static inline int32_t cw_tone_sample(void)
{
    static uint32_t phase = 0;
    static const uint32_t increment = (uint32_t)(((uint64_t)CW_TONE_HZ << 32) / SAMPLE_RATE_HZ);
    phase += increment;
    return sine_table[phase >> (32 - SINE_TABLE_BITS)];
}

// --- Core 1: campionamento in tempo reale, filtro di Hilbert, uscite PWM --
static void core1_entry(void)
{
    adc_init();
    adc_gpio_init(MIC_ADC_GPIO);
    adc_select_input(MIC_ADC_CHANNEL);

    gpio_init(PIN_CW_KEY);
    gpio_set_dir(PIN_CW_KEY, GPIO_IN);
    gpio_pull_up(PIN_CW_KEY);

    setup_pwm_pin(PIN_PHASE_0);
    setup_pwm_pin(PIN_PHASE_90);
    setup_pwm_pin(PIN_PHASE_180);
    setup_pwm_pin(PIN_PHASE_270);

    const int64_t period_us = 1000000 / SAMPLE_RATE_HZ;
    absolute_time_t next = get_absolute_time();

    while (true) {
        sleep_until(next);
        next = delayed_by_us(next, period_us);

        int32_t in = read_input_sample();

        int32_t i_out, q_out;
        hilbert_step(in, &i_out, &q_out);

        if (capture_index >= 0 && capture_index < CAPTURE_LEN) {
            capture_i[capture_index] = i_out;
            capture_q[capture_index] = q_out;
            capture_index++;
        }

        int32_t out0, out90, out180, out270;
        switch (g_mode) {
            case MODE_LSB:
                out0 = i_out; out90 = -q_out; out180 = -i_out; out270 = q_out;
                break;
            case MODE_AM: {
                int32_t carrier = ((int32_t)2047 * g_carrier_level) / 100;
                int32_t modulated = (i_out * (int32_t)g_modulation_depth) / 100;
                out0 = out90 = out180 = out270 = carrier + modulated;
                break;
            }
            case MODE_CW: {
                bool keyed = !gpio_get(PIN_CW_KEY); // premuto = chiude a massa, pull-up interno
                if (keyed && cw_ramp_index < CW_RAMP_LEN) cw_ramp_index++;
                else if (!keyed && cw_ramp_index > 0) cw_ramp_index--;

                int32_t tone = cw_tone_sample();
                int32_t gain = cw_ramp_table[cw_ramp_index];
                int32_t shaped = (int32_t)(((int64_t)tone * gain) >> 15);
                out0 = out90 = out180 = out270 = shaped;
                break;
            }
            case MODE_USB:
            default:
                out0 = i_out; out90 = q_out; out180 = -i_out; out270 = -q_out;
                break;
        }

        pwm_set_gpio_level(PIN_PHASE_0,   sample_to_duty(out0, 0));
        pwm_set_gpio_level(PIN_PHASE_90,  sample_to_duty(out90, 1));
        pwm_set_gpio_level(PIN_PHASE_180, sample_to_duty(out180, 2));
        pwm_set_gpio_level(PIN_PHASE_270, sample_to_duty(out270, 3));

        g_telemetry.raw_in = in;
        g_telemetry.out0   = out0;
        g_telemetry.out90  = out90;
        g_telemetry.out180 = out180;
        g_telemetry.out270 = out270;
    }
}

// --- Core 0: interfaccia comandi su seriale/USB --------------------------
static void print_menu(void)
{
    printf("\r\n=== rp2040hilbert ===\r\n"
           " m : sorgente MICROFONO (ADC su GPIO%d)\r\n"
           " t : sorgente TONO DI TEST (%d Hz)\r\n"
           " 2 : sorgente DOPPIO TONO (%d + %d Hz, two-tone test)\r\n"
           " a : modo AM\r\n"
           " u : modo USB\r\n"
           " l : modo LSB\r\n"
           " c : modo CW\r\n"
           " p/P : (modo AM) portante -10%%/+10%%\r\n"
           " i/I : (modo AM) profondita' modulazione -10%%/+10%%\r\n"
           " d : attiva/disattiva monitor debug livelli\r\n"
           " r : cattura %d campioni I/Q consecutivi (verifica verso quadratura)\r\n"
           " h : mostra questo menu\r\n"
           " I2C: slave addr 0x%02X su GPIO%d(SDA)/GPIO%d(SCL) -- reg 0x00=modo, 0x01=sorgente,\r\n"
           "      0x02=portante%%, 0x03=profondita' modulazione%%\r\n",
           MIC_ADC_GPIO, TEST_TONE_HZ, TWO_TONE_HZ_1, TWO_TONE_HZ_2, CAPTURE_LEN,
           I2C_SLAVE_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);
}

static void print_source_status(input_source_t src)
{
    const char *name = "?";
    switch (src) {
        case SRC_MIC:       name = "MICROFONO";   break;
        case SRC_TEST_TONE: name = "TONO DI TEST"; break;
        case SRC_TWO_TONE:  name = "DOPPIO TONO";  break;
    }
    printf("-> sorgente: %s\r\n", name);
}

static void print_mode_status(radio_mode_t m)
{
    printf("-> modo: %s\r\n", mode_name(m));
}

// --- Interfaccia I2C slave: registro 0x00=modo, 0x01=sorgente. Il primo
// byte scritto dal master seleziona il registro, i successivi lo leggono o
// scrivono con auto-incremento (stile registro classico). Gira in IRQ,
// deve restare breve: tocca solo le stesse variabili volatile del comando seriale.
static uint8_t i2c_reg_addr = 0;
static bool i2c_addr_received = false;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            uint8_t byte = i2c_read_byte_raw(i2c);
            if (!i2c_addr_received) {
                i2c_reg_addr = byte;
                i2c_addr_received = true;
            } else {
                if (i2c_reg_addr == I2C_REG_MODE && byte <= MODE_CW) {
                    g_mode = (radio_mode_t)byte;
                } else if (i2c_reg_addr == I2C_REG_SOURCE && byte <= SRC_TWO_TONE) {
                    g_input_source = (input_source_t)byte;
                } else if (i2c_reg_addr == I2C_REG_CARRIER && byte <= 100 && g_mode == MODE_AM) {
                    g_carrier_level = byte;
                } else if (i2c_reg_addr == I2C_REG_DEPTH && byte <= 100 && g_mode == MODE_AM) {
                    g_modulation_depth = byte;
                }
                i2c_reg_addr++;
            }
            break;
        }
        case I2C_SLAVE_REQUEST: {
            uint8_t value = 0xFF;
            if (i2c_reg_addr == I2C_REG_MODE) value = (uint8_t)g_mode;
            else if (i2c_reg_addr == I2C_REG_SOURCE) value = (uint8_t)g_input_source;
            else if (i2c_reg_addr == I2C_REG_CARRIER) value = g_carrier_level;
            else if (i2c_reg_addr == I2C_REG_DEPTH) value = g_modulation_depth;
            i2c_write_byte_raw(i2c, value);
            i2c_reg_addr++;
            break;
        }
        case I2C_SLAVE_FINISH:
            i2c_addr_received = false;
            break;
        default:
            break;
    }
}

int main()
{
    // Overclock: alza prima la tensione core, poi la frequenza di sistema.
    // Va fatto prima di qualunque periferica che derivi i propri divisori da
    // clk_sys (qui: il PIO del LED di stato) per avere i timing corretti.
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    // LED di stato (WS2812 su GPIO16): lampeggia per segnalare che il firmware
    // e' vivo, con il colore del modo attivo (vedi mode_color()).
    uint status_led_offset = pio_add_program(status_led_pio, &ws2812_program);
    ws2812_program_init(status_led_pio, status_led_sm, status_led_offset, PIN_STATUS_LED, 800000, false);
    status_led_set(0, 0, 0);

    stdio_init_all();

    generate_hilbert_coeffs();
    generate_sine_table();
    generate_cw_ramp();

    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    i2c_slave_init(I2C_PORT, I2C_SLAVE_ADDR, &i2c_slave_handler);

    multicore_launch_core1(core1_entry);

    print_menu();
    print_source_status(g_input_source);
    print_mode_status(g_mode);

    absolute_time_t next_debug_print = get_absolute_time();
    absolute_time_t next_blink = get_absolute_time();
    bool led_state = true;

    while (true) {
        if (absolute_time_diff_us(get_absolute_time(), next_blink) <= 0) {
            led_state = !led_state;
            if (led_state) {
                uint8_t r, g, b;
                mode_color(g_mode, &r, &g, &b);
                status_led_set(r, g, b);
            } else {
                status_led_set(0, 0, 0);
            }
            next_blink = delayed_by_ms(get_absolute_time(), 500);
        }

        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            switch (c) {
                case 'm': case 'M':
                    g_input_source = SRC_MIC;
                    print_source_status(SRC_MIC);
                    break;
                case 't': case 'T':
                    g_input_source = SRC_TEST_TONE;
                    print_source_status(SRC_TEST_TONE);
                    break;
                case '2':
                    g_input_source = SRC_TWO_TONE;
                    print_source_status(SRC_TWO_TONE);
                    break;
                case 'a': case 'A':
                    g_mode = MODE_AM;
                    print_mode_status(MODE_AM);
                    break;
                case 'u': case 'U':
                    g_mode = MODE_USB;
                    print_mode_status(MODE_USB);
                    break;
                case 'l': case 'L':
                    g_mode = MODE_LSB;
                    print_mode_status(MODE_LSB);
                    break;
                case 'c': case 'C':
                    g_mode = MODE_CW;
                    print_mode_status(MODE_CW);
                    break;
                case 'd': case 'D':
                    g_debug_monitor = !g_debug_monitor;
                    printf("-> monitor debug: %s\r\n", g_debug_monitor ? "ON" : "OFF");
                    break;
                case 'r': case 'R':
                    printf("-> cattura in corso...\r\n");
                    capture_index = 0;
                    break;
                case 'p':
                    if (g_mode != MODE_AM) { printf("-> comando valido solo in modo AM\r\n"); break; }
                    g_carrier_level = (g_carrier_level >= 10) ? (g_carrier_level - 10) : 0;
                    printf("-> portante: %u%%\r\n", g_carrier_level);
                    break;
                case 'P':
                    if (g_mode != MODE_AM) { printf("-> comando valido solo in modo AM\r\n"); break; }
                    g_carrier_level = (g_carrier_level <= 90) ? (g_carrier_level + 10) : 100;
                    printf("-> portante: %u%%\r\n", g_carrier_level);
                    break;
                case 'i':
                    if (g_mode != MODE_AM) { printf("-> comando valido solo in modo AM\r\n"); break; }
                    g_modulation_depth = (g_modulation_depth >= 10) ? (g_modulation_depth - 10) : 0;
                    printf("-> profondita' modulazione: %u%%\r\n", g_modulation_depth);
                    break;
                case 'I':
                    if (g_mode != MODE_AM) { printf("-> comando valido solo in modo AM\r\n"); break; }
                    g_modulation_depth = (g_modulation_depth <= 90) ? (g_modulation_depth + 10) : 100;
                    printf("-> profondita' modulazione: %u%%\r\n", g_modulation_depth);
                    break;
                case 'h': case 'H': case '?':
                    print_menu();
                    break;
                default:
                    break;
            }
        }

        if (g_debug_monitor && absolute_time_diff_us(get_absolute_time(), next_debug_print) <= 0) {
            telemetry_t t;
            t.raw_in = g_telemetry.raw_in;
            t.out0   = g_telemetry.out0;
            t.out90  = g_telemetry.out90;
            t.out180 = g_telemetry.out180;
            t.out270 = g_telemetry.out270;
            printf("in=%5ld  0=%5ld  90=%5ld  180=%5ld  270=%5ld\r\n",
                   (long)t.raw_in, (long)t.out0, (long)t.out90, (long)t.out180, (long)t.out270);
            next_debug_print = delayed_by_ms(get_absolute_time(), 200);
        }

        if (capture_index == CAPTURE_LEN) {
            printf("n\tI\tQ\r\n");
            for (int i = 0; i < CAPTURE_LEN; i++) {
                printf("%d\t%ld\t%ld\r\n", i, (long)capture_i[i], (long)capture_q[i]);
            }
            printf("-> fine cattura\r\n");
            capture_index = -1;
        }

        sleep_ms(5);
    }
}
