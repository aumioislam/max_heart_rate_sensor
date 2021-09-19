#include <iostream>
#include "../src/max30101/max30101.hpp"
#include "../src/ampd/ampd.hpp"
#include "unistd.h"

int main() {
    MAX30101 sensor = MAX30101();

    if(!sensor.verify())
        return -1;

    config conf = {MAX_MODE_HR,
                   MAX_ADC_RNG_4096,
                   MAX_SMP_RATE_800,
                   MAX_LED_PWM_411,
                   0,
                   MAX_SMP_AVG_4,
                   MAX_FIFO_ROLL_ENABLE,
                   0};

    sensor.init(conf);

    uint16_t num = 1500;
    unsigned int reading, out, av; 
    uint32_t **buf = (uint32_t **) malloc(sensor.num_leds*sizeof(uint32_t *));
    for (int i = 0; i < sensor.num_leds; i++) {
        buf[i] = (uint32_t*) malloc(sizeof(uint32_t)*num);
    }

    float time, bpm, bpm_sum, bpm_avg;
    uint16_t peaks, bpm_read_count;

    while (1) {
        time = bpm = bpm_sum = 0;
        bpm_read_count = 0;
        while (bpm_read_count < 5) {
            sensor.set_pulse_amp(0x06, MAX_RED);
            time = sensor.read_fifo(buf, num);
            sensor.set_pulse_amp(0x0, MAX_RED);

            peaks = count_peaks_ampd(buf[0], num); 
            bpm_read_count++;

            bpm = ((float) peaks)/time;
            bpm *= 60;
            bpm_sum += bpm;
            bpm_avg = bpm_sum/bpm_read_count;
            std::cout << "BPM: " << bpm << std::endl;
            std::cout << "BPM Average: " << bpm_avg << std::endl;
        }
    }

    for (int i = 0; i < sensor.num_leds; i++) {
        free(buf[i]);
    }
    free(buf);
}
