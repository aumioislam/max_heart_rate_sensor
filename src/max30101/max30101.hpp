#ifndef MAX30101_H
#define MAX30101_H

//Variables starting with '_' are register addresses

//initialization variables 
const uint8_t MAX_I2C_ADDR           	= 0x57; //I2C 7-bit Address
const uint8_t _MAX_PARTID            	= 0xFF; //Part ID register
const uint8_t MAX_EXPECTED_PARTID    	= 0x15; //Expected Part ID 

//interrupt and status registers
const uint8_t _MAX_INTSTAT1          	= 0x00;
const uint8_t _MAX_INTSTAT2          	= 0x01;
const uint8_t _MAX_INTENABLE1        	= 0x02;
const uint8_t _MAX_INTENABLE2        	= 0x03;

//FIFO registers
const uint8_t _MAX_FIFO_WR_PTR       	= 0x04;
const uint8_t _MAX_FIFO_OVF_CNT      	= 0x05;
const uint8_t _MAX_FIFO_RD_PTR       	= 0x06;
const uint8_t _MAX_FIFO_DATA         	= 0x07;

//Configuration registers
const uint8_t _MAX_FIFO_CONF         	= 0x08;   
const uint8_t _MAX_MODE_CONF         	= 0x09;
const uint8_t _MAX_SPO2_CONF         	= 0x0A;

//LED Pulse AMP registers
const uint8_t _MAX_LED_PA1           	= 0x0C;
const uint8_t _MAX_LED_PA2           	= 0x0D;
const uint8_t _MAX_LED_PA3           	= 0x0E;
const uint8_t _MAX_LED_PA4           	= 0x0F;

//LED Multi LED Mode control registers
const uint8_t _MAX_MUL_LED1          	= 0x11;
const uint8_t _MAX_MUL_LED2          	= 0x12;

//Temperature Registers
const uint8_t _MAX_TEMP_INT          	= 0x1F;
const uint8_t _MAX_TEMP_FRAC         	= 0x20;
const uint8_t _MAX_TEMP_ENABLE       	= 0x21;

//Temperature Masks and variables
const uint8_t MAX_TEMP_CONFIG_MASK      = 0x1;
const uint8_t MAX_TEMP_FRAC_MASK        = 0xF;
const float TEMP_CONVERSION             = 0.0625;

//Interrupt variables
//fifo almost full interrupt
const uint8_t MAX_A_FULL_MASK        	= ~(0b1 << 7);
const uint8_t MAX_A_FULL_ENABLE      	= (0b1 << 7);
const uint8_t MAX_A_FULL_DISABLE     	= 0x0;

//ppg data ready interrupt
const uint8_t MAX_PPG_RDY_MASK       	= ~(0b1 << 6);
const uint8_t MAX_PPG_RDY_ENABLE     	= (0b1 << 6);
const uint8_t MAX_PPG_RDY_DISABLE    	= 0x0;

//ambient light overflow interrupt
const uint8_t MAX_ALC_OVF_MASK       	= ~(0b1 << 5);
const uint8_t MAX_ALC_OVF_ENABLE     	= (0b1 << 5);
const uint8_t MAX_ALC_OVF_DISABLE    	= 0x0;

//power ready interrupt
const uint8_t MAX_PWR_RDY_MASK       	= ~(0b1);
const uint8_t MAX_PWR_RDY_ENABLE     	= 0b1;
const uint8_t MAX_PWR_RDY_DISABLE    	= 0x0;

//temperature reading ready interrupt
const uint8_t MAX_TEMP_RDY_MASK      	= ~(0b10);
const uint8_t MAX_TEMP_RDY_ENABLE    	= 0b10;
const uint8_t MAX_TEMP_RDY_DISABLE   	= 0x0;

//Configuration variables
//fifo configuration variables
const uint8_t MAX_SMP_AVG_MASK       	= ~(0b111 << 5);
const uint8_t MAX_SMP_AVG_1          	= 0x00;
const uint8_t MAX_SMP_AVG_2          	= 0x20;
const uint8_t MAX_SMP_AVG_4          	= 0x40;
const uint8_t MAX_SMP_AVG_8          	= 0x60;
const uint8_t MAX_SMP_AVG_16         	= 0x80;
const uint8_t MAX_SMP_AVG_32         	= 0xA0;

const uint8_t MAX_FIFO_ROLL_MASK     	= ~(0b1 << 4);
const uint8_t MAX_FIFO_ROLL_ENABLE   	= (0b1 << 4);
const uint8_t MAX_FIFO_ROLL_DISABLE  	= 0x0;

const uint8_t MAX_FIFO_A_FULL_MASK   	= 0xF0;

//mode configuration variables
const uint8_t MAX_MODE_SHTDN_MASK    	= ~(0b1 << 7);
const uint8_t MAX_MODE_SHTDN_ENABLE  	= (0b1 << 7);
const uint8_t MAX_MODE_SHTDN_DISABLE 	= 0x0;

const uint8_t MAX_MODE_RESET_MASK    	= ~(0b1 << 6);
const uint8_t MAX_MODE_RESET_ENABLE  	= (0b1 << 6);
const uint8_t MAX_MODE_RESET_DISABLE 	= 0x0;

const uint8_t MAX_MODE_CNTL_MASK     	= ~(0b111);
const uint8_t MAX_MODE_HR            	= 0b010;
const uint8_t MAX_MODE_SPO2          	= 0b011; 
const uint8_t MAX_MODE_MULTILED      	= 0b111;

//SPO2 configuration variables
const uint8_t MAX_SPO2_ADC_RNG_MASK  	= ~(0b11 << 5);
const uint8_t MAX_ADC_RNG_2048       	= 0x0;
const uint8_t MAX_ADC_RNG_4096       	= (0b01 << 5);
const uint8_t MAX_ADC_RNG_8192       	= (0b10 << 5);
const uint8_t MAX_ADC_RNG_16384      	= (0b11 << 5);

const uint8_t MAX_SMP_RATE_MASK      	= ~(0b111 << 2);
const uint8_t MAX_SMP_RATE_50        	= 0x0;
const uint8_t MAX_SMP_RATE_100       	= (1 << 2);
const uint8_t MAX_SMP_RATE_200       	= (2 << 2);
const uint8_t MAX_SMP_RATE_400       	= (3 << 2);
const uint8_t MAX_SMP_RATE_800       	= (4 << 2);
const uint8_t MAX_SMP_RATE_1000      	= (5 << 2);
const uint8_t MAX_SMP_RATE_1600      	= (6 << 2);
const uint8_t MAX_SMP_RATE_3200      	= (7 << 2);

const uint8_t MAX_LED_PWM_MASK       	= ~(0b11);
const uint8_t MAX_LED_PWM_69         	= 0b00;
const uint8_t MAX_LED_PWM_118        	= 0b01;
const uint8_t MAX_LED_PWM_215        	= 0b10;
const uint8_t MAX_LED_PWM_411        	= 0b11;

//Multi LED Mode configuration
const uint8_t MAX_MULTI_SLOT1_MASK      = ~(0b111);
const uint8_t MAX_MULTI_SLOT2_MASK      = ~(0b111 << 4);
const uint8_t MAX_MULTI_SLOT3_MASK      = ~(0b111);
const uint8_t MAX_MULTI_SLOT4_MASK      = ~(0b111 << 4);

//consts for LEDs
const uint8_t MAX_NONE             = 0x0; 
const uint8_t MAX_RED              = 0x1; 
const uint8_t MAX_IR               = 0x2; 
const uint8_t MAX_GREEN            = 0x3; 
const uint8_t MAX_GREEN2           = 0x4; 

//configuration struct
typedef struct {
    uint8_t mode;
    uint8_t range;
    uint8_t rate;
    uint8_t pwm;
    uint8_t amp;
    uint8_t avg;
    uint8_t fifo_roll;
    uint8_t fifo_a_full;
} config; 

class MAX30101 {
    public:
        //constructor
        MAX30101(void);

        void reset(void);
        void wake(void);
        void sleep(void);

        bool verify(void); 

        //getter
        int get_num_leds(void);
        
        //configuration functions
        void set_LED_mode(uint8_t mode);
        void set_ADC_range(uint8_t range);
        void set_sample_rate(uint8_t rate);
        void set_pwm(uint8_t pwm);

        void set_pulse_amp(uint8_t amp, uint8_t led);

        void enable_slot(uint8_t slot, uint8_t led);
        void disable_slots(void);

        bool init(config conf);

        //fifo configuration
        void set_fifo_avg(uint8_t avg);
        void set_fifo_roll(uint8_t roll);
        void set_fifo_a_full(uint8_t val);
        
        //fifo read
        uint8_t get_fifo_write_ptr(void);
        uint8_t get_fifo_read_ptr(void);
        void reset_fifo(void);
        int read_sample(uint32_t *data_ptr[]);
        float read_fifo(uint32_t **buf, uint16_t num_samples);

        //read temperature
        float read_temperature(void);

        //interrupt functions
        int check_interrupt(uint8_t reg, uint8_t flag);
        void set_interrupt(uint8_t reg, uint8_t mask, uint8_t flag);

    //private:
        int device, result, num_leds;

        int read_register(uint8_t addr, uint8_t *dest);
        int write_register(uint8_t reg, uint8_t data);
        int read_block(uint8_t start_reg, uint8_t *buf, uint8_t num_bytes);
        int mask_register(uint8_t reg, uint8_t mask, uint8_t flags);
};
#endif
