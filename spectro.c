#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"


#define LED_GPIO 13
#define IMPULSE_GPIO 6

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
#define N_SAMPLES 1024

#define WAIT_TIME_MS 10


uint8_t samples[N_SAMPLES];
uint dma_chan;

bool should_capture = false, should_draw = false;

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
    for (int i=0;i<WIDTH;i++) {
        for (int j=0;j<HEIGHT;j++) {
            display_buffer[i][j] = false;
        }
    }
    write_display_buffer();
    i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, display_on, 2, false);
}

void setup_dma() {
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        samples,    // dst
        &adc_hw->fifo,  // src
        N_SAMPLES,  // transfer count
        true            // start immediately
    );
}

void capture_dma() {
    printf("Starting capture\n");
    gpio_put(IMPULSE_GPIO, !gpio_get(IMPULSE_GPIO));
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
    adc_run(false);
    adc_fifo_drain();
    printf("Capture finished. Results: [\n");

    for (int i = 0; i < (N_SAMPLES-1); i++) {
        printf("%-3d, ", samples[i]);
    }
    printf("%-3d\n]\n", samples[N_SAMPLES-1]);
}

void buttons_callback(uint gpio, uint32_t events) {
    bool toggled_gpio;

    if (events & GPIO_IRQ_EDGE_FALL) {
        switch (gpio) {
            case 9: //A
                should_capture = true;
                //printf("A pressed\n");
                break;
            case 8: //B
                should_draw = true;
                display_buffer[4][4] = true;
                display_buffer[115][55] = true;
                //printf("B pressed\n");
                break;
            case 7: //C
                toggled_gpio = ! gpio_get(IMPULSE_GPIO);
                gpio_put(IMPULSE_GPIO, toggled_gpio);
                printf("Reset impulse GPIO to %d\n", toggled_gpio);
                //printf("C pressed\n");
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
        gpio_set_irq_enabled_with_callback(pinnum, GPIO_IRQ_EDGE_FALL, true, &buttons_callback);
    }

    printf("Getting display Ready\n");
    setup_display();

    setup_adc();
    printf("ADC raw result: %d\n", adc_read());
    printf("Getting DMA Ready\n");
    setup_dma();


    gpio_put(IMPULSE_GPIO, 1); // start charging
    // this indicates startup but also ensures the cap has ample time to charge
    for (int i=0; i < 5; i++) {
        gpio_put(LED_GPIO, 1);
        sleep_ms(250);
        gpio_put(LED_GPIO, 0);
        sleep_ms(250);
    }

    while (true) {
        if (should_capture) {
            gpio_put(LED_GPIO, 1);
            capture_dma();
            gpio_put(LED_GPIO, 0);
            should_capture = false;
        }

        if (should_draw) {
            printf("pre");
            write_display_buffer();
            printf("post\n");
            should_draw = false;
        }

        sleep_ms(WAIT_TIME_MS);

    }

    return 0;
}