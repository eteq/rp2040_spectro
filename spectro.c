#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "kissfft/kiss_fftr.h"

#include "font8x8_basic.h"
#define FONT_WIDTH 8


#define LED_GPIO 13
#define IMPULSE_GPIO 0

#define SDA_PIN 2
#define SCL_PIN 3
// assuming here that SCL is consistent
#if ((SDA_PIN/2) % 2)
#define WHICH_I2C i2c1
#else
#define WHICH_I2C i2c0
#endif
#define I2C_KHZ 400

#define DISPLAY_ADDR 0x3c
#define WIDTH 128
#define HEIGHT 64

#define ADC_CHANNEL 0 // Channel 0 is GPIO26
#define N_SAMPLES 8192  // 8192 -> ~20 ms

#define WAIT_TIME_MS 10

#define BUTTON_HOLD_MS 1000


uint8_t samples[N_SAMPLES];
uint dma_chan;
dma_channel_config dma_cfg;
int display_spacing = 1;

bool should_capture=false;
bool should_draw=false;
bool should_print=false;
bool draw_frequency=false;
bool continuous_mode=false;

alarm_id_t alarm_id_9 = -2;
alarm_id_t alarm_id_8 = -2;
alarm_id_t alarm_id_7 = -2;

bool display_buffer[WIDTH][HEIGHT];

void setup_adc() {
    bi_decl(bi_1pin_with_name(26 + ADC_CHANNEL, "ADC pin for capturing"));

    adc_gpio_init(26 + ADC_CHANNEL);

    adc_init();
    adc_select_input(ADC_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    adc_set_clkdiv(0);

}

int write_display_buffer() {
    int thisret, ret = 0;

    uint8_t byte_buffer[HEIGHT+1];
    byte_buffer[0] = 0x40;  //control byte, all follow data
    for (int i=0;i<HEIGHT;i++) {
        byte_buffer[i+1] = 0;
    }

    uint8_t reset_pointer_cmds[3] = {0x0, 0x10, 0xb0};

    for (int i=0; i < (WIDTH/8); i++) {
        // reset the byte buffer
        for (int j=0; j<HEIGHT; j++) { byte_buffer[j+1] = 0; }

        reset_pointer_cmds[2] = 0xb0 + i;
        if (i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, reset_pointer_cmds, 3, false) == PICO_ERROR_GENERIC) {return PICO_ERROR_GENERIC;}

        for (int j=0; j < HEIGHT; j++) {
            for (int k=0; k < 8; k++) {
                byte_buffer[j+1] += display_buffer[i*8+k][j] << k;
            }
        }

        thisret = i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, byte_buffer, HEIGHT+1, false);
        if (ret == PICO_ERROR_GENERIC) {
            return ret;
        } else {
            ret += thisret;
        }
    }
    return ret;
}

void clear_buffer() {
    for (int i=0;i<WIDTH;i++) {
        for (int j=0;j<HEIGHT;j++) {
            display_buffer[i][j] = false;
        }
    }
}

int char_to_buffer(char chr, uint x, uint y) {
    char * bmp  = font8x8_basic[chr];
    for (int i=0; i < 8; i++) {
        for (int j=0; j < 8; j++) {
            display_buffer[i+x][j+y] = (bmp[(7-j)] >> i) & 1;
        }
    }
}

int plot_to_buffer(uint8_t * samplearr, int nsamp) {
    assert(nsamp >= WIDTH);
    int sample_idx = 0;
    float avgval;

    clear_buffer();

    for (int i=0; i < WIDTH; i++) {
        avgval = 0;
        for (int j=0; j < display_spacing; j++) {
            avgval += samplearr[sample_idx++];
            if (sample_idx >= nsamp) {
                return -1;
            }
        }
        avgval /= display_spacing;

        display_buffer[i][(int)round(avgval * ((HEIGHT-1.)/255.))] = true;
    }
    return 0;
}
int plot_around_to_buffer(uint8_t * samplearr, int nsamp, int around_idx) {
    assert(nsamp >= WIDTH);

    int start_idx;

    clear_buffer();

    if (around_idx < WIDTH / 2) {
        start_idx = 0;
    }  else if (around_idx > (nsamp - WIDTH/2)) {
        start_idx = nsamp - 128;
    } else {
        start_idx = around_idx - WIDTH/2;
    }

    for (int i=0; i < WIDTH; i++) {
        int valint = (int)round(samplearr[i+start_idx] * ((HEIGHT-1.)/255.));
        display_buffer[i][valint] = true;
    }

    return 0;
}

void setup_display() {
    const uint8_t display_on[2] = {0x0, 0xaf};
    const uint8_t display_init_bytes[20] = {0x0,  //control byte - many command follow
                        0xae, // display off
                        0xdc, 0, // start line 0 - default
                        0x81, 0x4f, //contrast
                        0x20, // vertical addressing - default?
                        0xa0,  // down rotation/segment remap=0
                        0xc0, // scan direction - default
                        0xa8, 0x3f, // multiplex=64
                        0xd3, 0x60, // display offset - 0x60 according to featherwing/adafruit sh1107 driver docs?
                        0xd9, 0x22, // pre-charge/dis-charge period mode: 2 DCLKs/2 DCLKs - default
                        0xdb, 0x35, // VCOM deselect level = 0.770 - default
                        0xa4, // normal/disp off - default
                        0xa6 // normal (not reversed) display - default
    };
    const size_t n_init_bytes = 20;

    bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

    i2c_init(WHICH_I2C, I2C_KHZ * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, display_init_bytes, n_init_bytes, false);
    clear_buffer();
    i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, display_on, 2, false);
}

void setup_dma() {
    dma_chan = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);
}

void capture_dma() {
    dma_channel_configure(dma_chan, &dma_cfg,
        samples,    // dst
        &adc_hw->fifo,  // src
        N_SAMPLES,  // transfer count
        true            // start immediately
    );

    printf("Starting capture\n");
    gpio_put(IMPULSE_GPIO, !gpio_get(IMPULSE_GPIO));
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
    adc_run(false);
    adc_fifo_drain();
    gpio_put(IMPULSE_GPIO, !gpio_get(IMPULSE_GPIO));

}

void print_samples() {
    printf("Results: [\n");

    for (int i = 0; i < (N_SAMPLES-1); i++) {
        printf("%-3d, ", samples[i]);
    }
    printf("%-3d\n]\n", samples[N_SAMPLES-1]);
}

int64_t button_hold_callback(alarm_id_t id, void *user_data) {
    int gpio_num = -1;
    if (id == alarm_id_9) {
        // DO NOTHING FOR A HOLD
        return BUTTON_HOLD_MS * 1000;
    } else if (id == alarm_id_8) {
        // DO NOTHING FOR B HOLD
        return BUTTON_HOLD_MS * 1000;
    } else if (id == alarm_id_7) {
        // DO NOTHING FOR C HOLD
        return BUTTON_HOLD_MS * 1000;
    }

    return 0;
}

void buttons_callback(uint gpio, uint32_t events) {
    bool press = false;
    if (events & GPIO_IRQ_EDGE_FALL) {
        // button down
        switch (gpio) {
            case 9:
                alarm_id_9 = add_alarm_in_ms(BUTTON_HOLD_MS, button_hold_callback, NULL, false);
                break;
            case 8:
                alarm_id_8 = add_alarm_in_ms(BUTTON_HOLD_MS, button_hold_callback, NULL, false);
                break;
            case 7:
                alarm_id_7 = add_alarm_in_ms(BUTTON_HOLD_MS, button_hold_callback, NULL, false);
                break;
        }
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        // button up
        switch (gpio) {
            // cancel_alarm returns true if the alarm was canceled, which means
            // it was not yet fired (I think?), so we interpret that as press
            case 9:
                if (alarm_id_9 > -1) {
                    press = cancel_alarm(alarm_id_9);
                }
                break;
            case 8:
                if (alarm_id_8 > -1) {
                    press = cancel_alarm(alarm_id_8);
                }
                break;
            case 7:
                if (alarm_id_7 > -1) {
                    press = cancel_alarm(alarm_id_7);
                }
                break;
        }
    }

    if (press) {
        switch (gpio) {
            case 9: //A
                should_capture = true;
                should_draw = true;
                continuous_mode = ! continuous_mode;
                break;
            case 8: //B
                if (display_spacing == -1) {
                    display_spacing = 1;
                } else {
                    display_spacing *= 2;
                    if (display_spacing > (N_SAMPLES/128)) {
                        if (draw_frequency) {
                            display_spacing = -1;  // means do the peak-zoom
                        } else {
                            display_spacing = 1;
                        }
                    }
                }
                should_draw = true;
                break;
            case 7: //C
                //bool toggled_gpio = ! gpio_get(IMPULSE_GPIO);
                //gpio_put(IMPULSE_GPIO, toggled_gpio);
                //printf("Reset impulse GPIO to %d\n", toggled_gpio);
                if (draw_frequency) {
                    printf("Switching to time plot\n");
                    draw_frequency = false;
                } else {
                    printf("Switching to freqency plot\n");
                    draw_frequency = true;
                }
                should_draw = true;
                break;
        }
    }
}

int main() {
    bi_decl(bi_program_description("This is an in-progress spectrometer binary."));
    bi_decl(bi_1pin_with_name(LED_GPIO, "On-board LED"));

    stdio_init_all();

    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_put(LED_GPIO, 1);
    gpio_init(IMPULSE_GPIO);
    gpio_set_dir(IMPULSE_GPIO, GPIO_OUT);
    gpio_put(IMPULSE_GPIO, 0);

    // featherwing buttons
    for (int pinnum=7; pinnum<10; pinnum++) {
        gpio_init(pinnum);
        gpio_set_dir(pinnum, GPIO_IN);
        gpio_pull_up(pinnum);
        gpio_set_irq_enabled_with_callback(pinnum, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &buttons_callback);
    }

    printf("Getting display Ready\n");
    setup_display();

    setup_adc();
    printf("ADC raw result: %d\n", adc_read());
    printf("Getting DMA Ready\n");
    setup_dma();

    // this indicates startup but also ensures the cap has ample time to charge
    for (int i=0; i < 5; i++) {
        gpio_put(LED_GPIO, 1);
        sleep_ms(250);
        gpio_put(LED_GPIO, 0);
        sleep_ms(250);
    }

    while (true) {
        int maxfftidx;
        double maxfftsq;

        if (should_capture | continuous_mode) {
            gpio_put(LED_GPIO, 1);
            capture_dma();
            printf("Capture complete.\n");
            if (should_print) { print_samples(); }

            gpio_put(LED_GPIO, 0);
            should_capture = false;
        }

        if (should_draw | continuous_mode) {
            if (draw_frequency) {
                kiss_fft_scalar samples_fft_t[N_SAMPLES];
                kiss_fft_cpx fft_cpx[N_SAMPLES];
                double fftabssq[N_SAMPLES/2 + 1];
                uint8_t fftabs[N_SAMPLES/2 + 1];
                kiss_fftr_cfg fftrcfg = kiss_fftr_alloc(N_SAMPLES, false, 0, 0);
                maxfftsq=0;
                maxfftidx=0;

                uint64_t sum = 0;
                for (int i=0;i < N_SAMPLES;i++) {sum += samples[i];}
                float avg = (float)sum/N_SAMPLES;
                for (int i=0;i < N_SAMPLES;i++) {samples_fft_t[i] = (float)samples[i] - avg;}

                kiss_fftr(fftrcfg, samples_fft_t, fft_cpx);
                for (int i=0;i<N_SAMPLES/2 + 1;i++) {
                    fftabssq[i] = fft_cpx[i].r*fft_cpx[i].r + fft_cpx[i].i*fft_cpx[i].i;
                    if (fftabssq[i] > maxfftsq) {
                        maxfftsq = fftabssq[i];
                        maxfftidx = i;
                    }
                }
                kiss_fft_free(fftrcfg);

                for (int i=0;i<N_SAMPLES/2 + 1;i++) {
                    fftabs[i] = round(255*sqrt(fftabssq[i]/maxfftsq));
                }
                if (display_spacing == -1) {
                    // zoom in on peak
                    plot_around_to_buffer(fftabs, N_SAMPLES/2 + 1, maxfftidx);
                } else {
                    plot_to_buffer(fftabs, N_SAMPLES/2 + 1);
                }
            } else {
                plot_to_buffer(samples, N_SAMPLES);
            }

            char toprint[16];
            int n, offset;

            if (draw_frequency) {
                float fdisp;
                char prefix[2];
                if (display_spacing == -1) {
                    // tell the user where the peak is
                    fdisp = 500000. * maxfftidx / N_SAMPLES;
                    strcpy(prefix, "p");
                } else {
                    fdisp = 500000. * display_spacing * 128. / N_SAMPLES;
                    strcpy(prefix, "");
                }
                printf("%g Hz\n", fdisp);
                if (fdisp > 1e3) {
                    n = sprintf(toprint, "%s%.2fkHz", prefix, fdisp/1e3);
                } else {
                    n = sprintf(toprint, "%s%.2gHz", prefix, fdisp);
                }
            } else {
                float tdisp = 128./500000. * display_spacing;
                printf("%g sec\n", tdisp);
                if ((1e-3 > tdisp) && (tdisp > 1e-6)) {
                    n = sprintf(toprint, "%.1fus", tdisp*1e6);
                } else if (tdisp < 1) {
                    n = sprintf(toprint, "%.1fms", tdisp*1e3);
                } else {
                    n = sprintf(toprint, "%.1gs", tdisp);
                }
            }
            offset = 127 - 8*n; if (n < 0) { offset = 0; }
            for (int i=0; i < n; i++) {
                if (offset + 8*i + 7 >= 128) { break; } // this should only be if the string < 16...
                char_to_buffer(toprint[i], offset + 8*i, 56);
            }

            write_display_buffer();
            should_draw = false;
        }

        sleep_ms(WAIT_TIME_MS);

    }

    return 0;
}