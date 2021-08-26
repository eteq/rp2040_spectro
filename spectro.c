#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"


#define LED_GPIO 13

// Channel 0 is GPIO26
#define ADC_CHANNEL 0
#define N_SAMPLES 1024

uint8_t samples[N_SAMPLES];
uint dma_chan;


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

int main() {
    bi_decl(bi_program_description("This is an in-progress spetrometer binary."));
    bi_decl(bi_1pin_with_name(LED_GPIO, "On-board LED"));
    
    stdio_init_all();

    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);

    setup_adc();

    printf("ADC raw result: %d\n", adc_read());
    sleep_ms(100);

    printf("Getting DMA Ready\n");
    setup_dma();

    for (int i=0; i < 3; i++) {
        gpio_put(LED_GPIO, 1);
        sleep_ms(250);
        gpio_put(LED_GPIO, 0);
        sleep_ms(250);
    }

    capture_dma();

    return 0;
}