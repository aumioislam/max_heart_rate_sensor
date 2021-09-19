/*
 * Some code for interfacing with the sensor was adapted from Sparkfun's 
 * opensource Arduino library for the breakout board used
 * https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library 
 */

#include <iostream>
#include <unistd.h>
#include <chrono>
#include "max30101.hpp"
#include "../i2c/i2c.h"

/*
 * constructor for max30101 sensor
 */
MAX30101::MAX30101() {
    device = 0;
    result = 0;
    num_leds = 0;
    device = i2c_init("/dev/i2c-1", MAX_I2C_ADDR);
}

/*
 * Reset function
 * Timeout after 250 ms 
 */
void MAX30101::reset() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::high_resolution_clock;

    result = mask_register(_MAX_MODE_CONF, MAX_MODE_RESET_MASK, MAX_MODE_RESET_ENABLE);

    auto start = high_resolution_clock::now();
    auto curr = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(start - curr);

    while (duration.count() < 250) {
        if ((check_interrupt(_MAX_MODE_CONF, MAX_MODE_RESET_ENABLE)) == 0) break;
        usleep(1000);
        curr = high_resolution_clock::now();
        duration = duration_cast<milliseconds>(start - curr);
    }
}

void MAX30101::wake() {
    result = mask_register(_MAX_MODE_CONF, MAX_MODE_SHTDN_MASK, MAX_MODE_SHTDN_DISABLE);
}

void MAX30101::sleep() {
    result = mask_register(_MAX_MODE_CONF, MAX_MODE_SHTDN_MASK, MAX_MODE_SHTDN_ENABLE);
}

/*
 * Check that part ID matches value from datasheet
 */
bool MAX30101::verify() {
    //check that I2C communications have been initialized
    if (device < 0) {
        std::cout << "Failed to initialize I2C communications" << std::endl;
        return false;
    }

    //check that the part ID is as expected to ensure that the correct device
    //is being used
    uint8_t partID;

    result = read_register(_MAX_PARTID, &partID);

    if (partID != MAX_EXPECTED_PARTID) {
        std::cout << "Wrong value read from part ID register" << std::endl;
        std::cout << "Part ID: 0x" << std::hex << (int) partID << std::endl;
        return false;
    }

    return true;
}

void MAX30101::set_LED_mode(uint8_t mode) {
    switch(mode) {
        case(2):
            num_leds = 1;
            break;
        case(3):
            num_leds = 2;
            break;
        case(7):
            num_leds = 3;
            break;
        default:
            break;
    }
    result = mask_register(_MAX_MODE_CONF, MAX_MODE_CNTL_MASK, mode);
}

void MAX30101::set_ADC_range(uint8_t range) {
    result = mask_register(_MAX_SPO2_CONF, MAX_SPO2_ADC_RNG_MASK, range);
}

void MAX30101::set_sample_rate(uint8_t rate) {
    result = mask_register(_MAX_SPO2_CONF, MAX_SMP_RATE_MASK, rate);
}

void MAX30101::set_pwm(uint8_t pwm) {
    result = mask_register(_MAX_SPO2_CONF, MAX_LED_PWM_MASK, pwm);
}

void MAX30101::set_pulse_amp(uint8_t amp, uint8_t led) {
    switch(led) {
        case(1):
            result = write_register(_MAX_LED_PA1, amp);
            break;
        case(2):
            result = write_register(_MAX_LED_PA2, amp);
            break;
        case(3):
            result = write_register(_MAX_LED_PA3, amp);
            break;
        case(4):
            result = write_register(_MAX_LED_PA4, amp);
            break;
        default:
            break;
    }
}

void MAX30101::enable_slot(uint8_t slot, uint8_t led) {
    switch(slot) {
        case(1):
            result = mask_register(_MAX_MUL_LED1, MAX_MULTI_SLOT1_MASK, led);
            break;
        case(2):
            result = mask_register(_MAX_MUL_LED1, MAX_MULTI_SLOT2_MASK, led << 4);
            break;
        case(3):
            result = mask_register(_MAX_MUL_LED2, MAX_MULTI_SLOT3_MASK, led);
            break;
        case(4):
            result = mask_register(_MAX_MUL_LED2, MAX_MULTI_SLOT4_MASK, led << 4);
            break;
        default:
            break;
    }
}

void MAX30101::disable_slots() {
    result = write_register(_MAX_MUL_LED1, 0x0);
    result = write_register(_MAX_MUL_LED2, 0x0);
}

/*
 * Initialize LED configuration
 * config struct is defined in max30101.hpp
 * Only set upt to initialize in Heart Rate Mode
 * TODO: Configure for SPO2 and Multi-LED mode
 */
bool MAX30101::init(config conf) {
    wake();
    reset();

    set_LED_mode(conf.mode);
    set_ADC_range(conf.range);
    set_sample_rate(conf.rate);
    set_pwm(conf.pwm);

    enable_slot(1, MAX_RED);
    reset_fifo();
    set_fifo_avg(conf.avg);
    set_fifo_roll(conf.fifo_roll);
    return true;
}


void MAX30101::set_fifo_avg(uint8_t avg) {
    result = mask_register(_MAX_FIFO_CONF, MAX_SMP_AVG_MASK, avg);
}

void MAX30101::set_fifo_roll(uint8_t roll) {
    result = mask_register(_MAX_FIFO_CONF, MAX_FIFO_ROLL_MASK, roll);
}

void MAX30101::set_fifo_a_full(uint8_t val) {
    if (val <= 0xf)
        result = mask_register(_MAX_FIFO_CONF, MAX_FIFO_A_FULL_MASK, val);
}

uint8_t MAX30101::get_fifo_write_ptr() {
    uint8_t ptr; 
    result = read_register(_MAX_FIFO_WR_PTR, &ptr);
    return ptr;
}

uint8_t MAX30101::get_fifo_read_ptr() {
    uint8_t ptr; 
    result = read_register(_MAX_FIFO_RD_PTR, &ptr);
    return ptr;
}
/*
 * Reset all FIFO values - should be done before starting new reading
 */
void MAX30101::reset_fifo() {
    result = write_register(_MAX_FIFO_RD_PTR, 0x0);
    result = write_register(_MAX_FIFO_WR_PTR, 0x0);
    result = write_register(_MAX_FIFO_OVF_CNT, 0x0);
    result = write_register(_MAX_FIFO_DATA, 0x0);
}

/*
 * Read block for data - will be 3, 6, or 9 bytes of data depending on how
 * many LEDs are on 
 *
 * @param: array of pointers to the addresses where the data should be copied to
 */
int MAX30101::read_sample(uint32_t *data_ptr[]) {
    uint8_t bytes = 3*num_leds;
    uint8_t sample_data[bytes];
    uint32_t uc_temp;

    result = read_block(_MAX_FIFO_DATA, sample_data, bytes);

    for (int i = 0; i < num_leds; i++) {
        //bitwise operations and mask according to datasheet
        *data_ptr[i] = (sample_data[i*3] << 16) + (sample_data[i*3+1] << 8) + (sample_data[i*3+2]); 
        *data_ptr[i] &= 0x3FFFF;
    }

    return 1;
}

/*
 * @param num_samples: number of samples requested 
 * 
 * @param **buf: array of pointers to arrays of size num_samples
 *               each "channel" correspondings to an LED
 *
 * @return: time elasped to read the samples
 */
float MAX30101::read_fifo(uint32_t **buf, uint16_t num_samples) {
    //initialize clocks 
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::seconds;
    using std::chrono::high_resolution_clock;

    //nothing to do, exit
    if (num_leds == 0 || buf == nullptr) {
        return -1;
    }

    uint16_t sample_count = 0;
    uint32_t *cur_data[num_leds];
    uint8_t read_ptr, write_ptr;
    int available;
    //start clock
    auto start = high_resolution_clock::now();

    //while there are still samples to be read
    while (sample_count < num_samples) {

        read_ptr = get_fifo_read_ptr();
        write_ptr = get_fifo_write_ptr();

        //if read and write pointers are different
        //there is data to be read
        if (read_ptr != write_ptr) { 
            available = write_ptr - read_ptr;

            //due to wrap condition, add 32 if difference is negative
            if (available < 0) available += 32;

            //if only few samples are left to be read, adjusted requested number
            //of bytes to prevent overflow
            if ((available + sample_count) > num_samples) available = num_samples - sample_count;

            for (int i = 0; i < available; i++) {
                for (int j = 0; j < num_leds; j++) {
                    //for each channel, add pointer to next sample location
                    cur_data[j] = &buf[j][sample_count];
                }

                //read samples
                result = read_sample(cur_data);
                sample_count++;
            }
        }
    }

    //stop timer
    auto finish = high_resolution_clock::now();
    duration<float> s_float = finish - start;
    //return function duration
    return s_float.count();
}

/*
 * Function to request and read temperature data from sensor
 * Value return is in degrees Celsius
 */
float MAX30101::read_temperature() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::high_resolution_clock;

    //enable temperature interrupt
    set_interrupt(_MAX_INTENABLE2, MAX_TEMP_RDY_MASK, MAX_TEMP_RDY_ENABLE);

    //request temperature reading
    result = write_register(_MAX_TEMP_ENABLE, MAX_TEMP_CONFIG_MASK);

    int status = check_interrupt(_MAX_INTSTAT2, MAX_TEMP_RDY_ENABLE);
    auto start = high_resolution_clock::now();
    auto curr = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(start - curr);

    //wait for interrupt indicating temp reading is ready
    //timeout after 250 ms
    while ((duration.count() < 250) && !status) {
        status = check_interrupt(_MAX_INTSTAT2, MAX_TEMP_RDY_ENABLE);
        usleep(1000);
        curr = high_resolution_clock::now();
        duration = duration_cast<milliseconds>(start - curr);
    }

    if (!status) {
        std::cout << "Temperature reading timed out..." << std::endl;
        return 0.0;
    }

    uint8_t temp[2];

    //read data from sensor
    result = read_register(_MAX_TEMP_INT, &temp[0]);
    result = read_register(_MAX_TEMP_FRAC, &temp[1]);

    //return converted value
    return ((float) temp[0] + (float) (temp[1]*TEMP_CONVERSION));
}

//check if flag is set
int MAX30101::check_interrupt(uint8_t reg, uint8_t flag) {
    uint8_t interrupt_val;
    result = read_register(reg, &interrupt_val);

    return interrupt_val & flag;
}

//set interrupt flag
void MAX30101::set_interrupt(uint8_t reg, uint8_t mask, uint8_t flag) {
    result = mask_register(reg, mask, flag);
}

//wrappers for reading and writing to the sensor via I2C
int MAX30101::read_register(uint8_t reg, uint8_t *dest) {
   return i2c_smbus_read_register8(device, reg, dest);
}

int MAX30101::write_register(uint8_t reg, uint8_t data) {
    return i2c_smbus_write_register8(device, reg, data);
}

int MAX30101::read_block(uint8_t reg, uint8_t *buf, uint8_t num_bytes) {
    return i2c_read(device, MAX_I2C_ADDR, reg, buf, num_bytes*sizeof(uint8_t));
}

//set bits in registers
int MAX30101::mask_register(uint8_t reg, uint8_t mask, uint8_t flags) {
    //read current register contents
    uint8_t content;
    result = read_register(reg, &content);

    //clear bits of interest
    content &= mask;

    //set bits of interest
    content |= flags;

    //write back to the register
    result = write_register(reg, content);
    return result;
}
