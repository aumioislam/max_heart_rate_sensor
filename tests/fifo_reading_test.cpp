#include <iostream>
#include "../src/max30101/max30101.hpp"
#include "unistd.h"

int main() {
    MAX30101 sensor = MAX30101();

    if(!sensor.verify())
        return -1;

    config conf = {MAX_MODE_HR,
                   MAX_ADC_RNG_4096,
                   MAX_SMP_RATE_50,
                   MAX_LED_PWM_411,
                   0,
                   MAX_SMP_AVG_4,
                   MAX_FIFO_ROLL_ENABLE,
                   0};

    sensor.init(conf);

    uint16_t num = 128;
    unsigned int reading, out, av; 
    uint32_t **buf = (uint32_t **) malloc(sensor.num_leds*sizeof(uint32_t *));
    for (int i = 0; i < sensor.num_leds; i++) {
        buf[i] = (uint32_t*) malloc(sizeof(uint32_t)*num);
    }

    sensor.set_pulse_amp(0x06, MAX_RED);
    sensor.read_fifo(buf, num);
    sensor.set_pulse_amp(0x0, MAX_RED);

    for (int i = 0; i < num; i++) {
        out = (unsigned int) buf[0][i];
        std::cout << " " <<  std::dec << out << std::endl;
    }

    for (int i = 0; i < sensor.num_leds; i++) {
        free(buf[i]);
    }

    free(buf);
}
