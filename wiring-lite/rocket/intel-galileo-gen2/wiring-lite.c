/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at 
 * 
 * http://www.apache.org/licenses/LICENSE-2.0 
 * 
 * Unless required by applicable law or agreed to in writing, software distributed 
 * under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES 
 * OR CONDITIONS OF ANY KIND, either express or implied. See the License for
 * the specific language governing permissions and limitations under the License. 
 */
#ifdef CONFIG_BOARD_GALILEO

#include <zephyr.h>
#include <gpio.h>
#include <i2c.h>
#include <pwm.h>
#include <pinmux.h>
#include <adc.h>
#include <microkernel/ticks.h>
#include <string.h>
#include "Arduino-lite.h"


#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#define DBG_PRINT(...)

#define P0(pin) (pin)
#define P1(pin) ((pin)+8)

struct device *gpio07;
struct device *gpioLegacy;
struct device *gpio_sus;
struct device *gpioEXP0;
struct device *gpioEXP1;
struct device *gpioEXP2;
struct device *gpioPWM;
struct device *i2c;
struct device *adc;
struct device *pinmux;



/* Buffer holds ADC conversion result */
uint16_t seq_buffer[] = { 0, 0, 0, 0, 0, 0 };


/*
 * Setup ADC conversion on channel A0.
 * We are only reading one channel so buffer contains one value.
 * minimal delay before start of the conversion (avoiding zero)
 */

struct adc_seq_entry sample[] = {
    { .sampling_delay = 10, .channel_id = 0, .buffer = (uint8_t*)(&seq_buffer[0]), .buffer_length  = 2, .stride = {0, 0, 0}}, 
    { .sampling_delay = 10, .channel_id = 1, .buffer = (uint8_t*)(&seq_buffer[1]), .buffer_length  = 2, .stride = {0, 0, 0}}, 
    { .sampling_delay = 10, .channel_id = 2, .buffer = (uint8_t*)(&seq_buffer[2]), .buffer_length  = 2, .stride = {0, 0, 0}}, 
    { .sampling_delay = 10, .channel_id = 3, .buffer = (uint8_t*)(&seq_buffer[3]), .buffer_length  = 2, .stride = {0, 0, 0}}, 
    { .sampling_delay = 10, .channel_id = 4, .buffer = (uint8_t*)(&seq_buffer[4]), .buffer_length  = 2, .stride = {0, 0, 0}}, 
    { .sampling_delay = 10, .channel_id = 5, .buffer = (uint8_t*)(&seq_buffer[5]), .buffer_length  = 2, .stride = {0, 0, 0}}
};

struct adc_seq_table table =
{
    .entries     = sample,
    .num_entries = 6,
    .stride = {0, 0, 0}
};



/*
 * @brief PWM Pin Mapping configuration
 *
 */
static struct _pwm_map {
	uint8_t pin;
	uint8_t pwm;
} pwm_map[] =
{
	{ 3 , 1  },
	{ 5 , 3  },
	{ 6 , 5  },
	{ 9 , 7  },
	{ 10, 11 },
	{ 11, 9  },
};

/***************************************************************************************************
 *
 * Missing APIs for PWM control
 *
 **************************************************************************************************/

const uint8_t pwm_map_size = sizeof(pwm_map) / sizeof(struct _pwm_map);
// We need a PWM duty cycle API that takes a 8 bit value
#define HARDCODED_CONVERSION(val) (map(val, 0, 255, 0, 4095)) // Oops I peeked in the driver file and saw it was 12 bits

#define pwm_pin_set_duty_cycle_by_8_bit_value(dev,pwm,value)  \
     ((struct pwm_driver_api *)dev->driver_api)->set_values(dev, PWM_ACCESS_BY_PIN, pwm, 0, HARDCODED_CONVERSION(value))


struct pwm_pca9685_config {
        /** The master I2C device's name */
        const char * const i2c_master_dev_name;

        /** The slave address of the chip */
        uint16_t i2c_slave_addr;
};

/** Runtime driver data */
struct pwm_pca9685_drv_data {
        /** Master I2C device */
        struct device *i2c_master;
};
#define MODE1 0x00
#define SLEEP 0x10
#define PRESCALE 0xFE
int pwm_set_frequency(struct device *dev, uint32_t freq)
{
        const struct pwm_pca9685_config * const config = dev->config->config_info;
        struct pwm_pca9685_drv_data * const drv_data = (struct pwm_pca9685_drv_data * const)dev->driver_data;
        struct device * const i2c_master = drv_data->i2c_master;
        uint16_t i2c_addr = config->i2c_slave_addr;
        uint8_t mode1;
        uint8_t buf[] = { MODE1, 0 };

        i2c_write(i2c_master, buf, 1, i2c_addr);
        i2c_read(i2c_master, &buf[1], 1, i2c_addr);
        mode1 = buf[1];

        buf[1] |= SLEEP;

        i2c_write(i2c_master, buf, sizeof(buf), i2c_addr);

        buf[0] = PRESCALE;
        buf[1] =  ((25000000 / (4096 * freq) ));
        i2c_write(i2c_master, buf, sizeof(buf), i2c_addr);

        buf[0] = MODE1;
        buf[1] = mode1;
       return i2c_write(i2c_master, buf, sizeof(buf), i2c_addr);
}




/***************************************************************************************************
 *
 * Interface implementation
 *
 **************************************************************************************************/
/*
 * gpioOutputSet - set the output value of a pin
 *
 * outputPin: Arduino connector pin number
 * value: 0=set output to low, otherwise set high
 *
 * Note: comments below "gpioXX" refer to the Linux GPIO naming convention
 */

int gpioOutputSet (int outputPin, int value)
{
    int rc1 = DEV_OK;
    switch (outputPin)
        {
        case 0:
            /* gpio11=signal */
            gpio_pin_write(gpio07, 3, value);
            break;
        case 1:
            /* gpio12=signal */
            gpio_pin_write(gpio07, 4, value);
            break;
        case 2:
            /* gpio13=signal */
            gpio_pin_write(gpio07, 5, value);
            break;
        case 3:
            /* gpio14=signal */
            gpio_pin_write(gpio07, 6, value);
            break;
        case 4:
            /* gpio6=signal */
            gpio_pin_write(gpio_sus, 4, value);
            break;
        case 5:
            /* gpio0=signal */
            gpio_pin_write(gpioLegacy, 0, value);
            break;
        case 6:
            /* gpio1=signal */
            gpio_pin_write(gpioLegacy, 1, value);
            break;
        case 7:
            /* gpio38=signal */
            gpio_pin_write(gpioEXP1, P0(6), value);
            break;
        case 8:
            /* gpio40=signal*/
            gpio_pin_write(gpioEXP1, P1(0), value);
            break;
        case 9:
            /* gpio4=signal */
            gpio_pin_write(gpio_sus, 2, value);
            break;
        case 10:
            /* gpio10=signal */
            gpio_pin_write(gpio07, 2, value);
            break;
        case 11:
            /* gpio5=signal */
            gpio_pin_write(gpio_sus, 3, value);
            break;
        case 12:
            /* gpio15=signal*/
            gpio_pin_write(gpio07, 7, value);
            break;
        case 13:
            /* gpio7=signal */
            gpio_pin_write(gpio_sus, 5, value);
            break;

        default:
            rc1 = DEV_FAIL;
            break;
        }

    if (rc1 != DEV_OK)
        {
        PRINT("GPIO set error %d!!\n", rc1);
        }

    return rc1;
}

/*
 * gpioInputGet - get value of a pin
 *
 * inputPin: Arduino connector pin number
 * Returns: 0 if value is low,
 *          1 if value is high
 *
 * Note: comments below "gpioXX" refer to the Linux GPIO naming convention
 */

int gpioInputGet (int inputPin)
{
    uint32_t value;
    int rc1 = DEV_OK;
    switch (inputPin)
        {
        case 0:
            /* gpio11=signal */
            gpio_pin_read(gpio07, 3, &value);
            break;
        case 1:
            /* gpio12=signal */
            gpio_pin_read(gpio07, 4, &value);
            break;
        case 2:
            /* gpio13=signal*/
            gpio_pin_read(gpio07, 5, &value);
            break;
        case 3:
            /* gpio14=signal */
            gpio_pin_read(gpio07, 6, &value);
            break;
        case 4:
            /* gpio6=signal */
            gpio_pin_read(gpio_sus, 4, &value);
            break;
        case 5:
            /* gpio0=signal*/
            gpio_pin_read(gpioLegacy, 0, &value);
            break;
        case 6:
            /* gpio1=signal */
            gpio_pin_read(gpioLegacy, 1, &value);
            break;
        case 7:
            /* gpio38=signal */
            gpio_pin_read(gpioEXP1, P0(6), &value);
            break;
        case 8:
            /* gpio40=signal */
            gpio_pin_read(gpioEXP1, P1(0), &value);
            break;
        case 9:
            /* gpio4=signal */
            gpio_pin_read(gpio_sus, 2, &value);
            break;
        case 10:
            /* gpio10=signal */
            gpio_pin_read(gpio07, 2, &value);
            break;
        case 11:
            /* gpio5=signal */
            gpio_pin_read(gpio_sus, 3, &value);
            break;
        case 12:
            /* gpio15=signal */
            gpio_pin_read(gpio07, 7, &value);
            break;
        case 13:
            /* gpio7=signal */
            gpio_pin_read(gpio_sus, 5, &value);
            break;
        default:
            rc1 = DEV_FAIL;
            break;
        }

    if (rc1 != DEV_OK)
        {
        PRINT("GPIO read error %d!!\n", rc1);
        }

    if (0 == value)
        return 0;
    else
        return 1;

}


void pinMode(uint8_t pin, uint8_t mode) {

    DBG_PRINT("Initialized pin%d\n", pin);

    /* set direction  */
    switch (mode) {
	case INPUT_PULLUP:
    	  DBG_PRINT("Setting mode to INPUT_PULLUP\n");
	case INPUT:
    	DBG_PRINT("Setting mode INPUT\n");
           if (pinmux_pin_set(pinmux, pin, PINMUX_FUNC_B)) {
    	      DBG_PRINT("failed\n");
           } else {
    	      DBG_PRINT("success\n");
           }
	break;
        case OUTPUT:
    	   DBG_PRINT("Setting mode to OUTPUT...");
           if (pinmux_pin_set(pinmux, pin, PINMUX_FUNC_A)) {
    	      DBG_PRINT("failed\n");
           } else {
    	      DBG_PRINT("success\n");
           }
    }
}

void digitalWrite(register uint8_t pin, register uint8_t value)
{


    DBG_PRINT("Initialized pin%d\n", pin);

    if ( value == HIGH )
    	DBG_PRINT("Setting pin to HIGH...");
    else
    	DBG_PRINT("Setting direction to LOW...");
    if (gpioOutputSet(pin, value)) { 
    	DBG_PRINT("Failed\n");
    } else {
    	DBG_PRINT("Success\n");
    }
}

int digitalRead(uint_t pin)
{
	uint8_t val;

	val = gpioInputGet (pin);
        DBG_PRINT("Gpio%d is %d\n", pin, val);

	return val;
}
	
void analogWrite(uint32_t pin, uint32_t value)
{
	
    uint8_t pwm = 0xFF;
    uint8_t i = 0;

    DBG_PRINT("Initialized pin%d\n", pin);
    if ( ((pin) == 3 || (pin) == 5 || (pin) == 6 || (pin) == 9 || (pin) == 10 ||
		(pin) == 11 )) {
	    DBG_PRINT("GPIO pin %d\n", pin);
	    /* find the PWM mapping */
	    for (i = 0; i < pwm_map_size; i++) {
		    if (pin == pwm_map[i].pin)
			    pwm = pwm_map[i].pwm;
	    }
	    if (pwm == 0xFF) {
		    DBG_PRINT("%s bad pwm pin %d\n", __FUNCTION__, pin);
		    return ;
	    }
		
	    DBG_PRINT("Writing [%d] to PWM pin [%d]\n",value,pwm);
	    if (pinmux_pin_set(pinmux, pin, PINMUX_FUNC_C)) {
		    DBG_PRINT("failed\n");
	    } else {
		    DBG_PRINT("success\n");
	    }
	    pwm_pin_set_duty_cycle_by_8_bit_value(gpioPWM,pwm,value);
    }
    else if ( ((pin) == 14 || (pin) == 15 || (pin) == 16 || (pin) == 17 || (pin) == 18 ||
			    (pin) == 19 )) {
	    DBG_PRINT("ADC pin %d\n", pin);
	    DBG_PRINT("   -- NOT SUPPORTED  -- ADC pin %d\n", pin);
    }
    else {
	    DBG_PRINT("INVALID Operation for pin %d\n", pin);
    }
}

uint16_t analogRead(uint8_t pin)
{
    uint16_t val=0;

    DBG_PRINT("Initialized pin%d\n", pin);
    if ( ((pin) == 3 || (pin) == 5 || (pin) == 6 || (pin) == 9 || (pin) == 10 ||
		(pin) == 11 )) {
	    DBG_PRINT("PWM pin %d\n", pin);
	    DBG_PRINT("   -- NOT SUPPORTED  -- PWM pin %d\n", pin);
    }
    else if ( ((pin) == 14 || (pin) == 15 || (pin) == 16 || (pin) == 17 || (pin) == 18 ||
			    (pin) == 19 )) {
	    DBG_PRINT("ADC pin %d\n", pin);
	    DBG_PRINT("reading ADC...\n");
	    sample[0].channel_id = 0;
	    

    	if (adc_read(adc, &table) != DEV_OK) {
			PRINT("ADC read data error\n");
		} else {
	        val = seq_buffer[pin-14];
	       val = map(val, 0, 4095, 0, 1023); // 12 -> 10 bit
	       
		}
    }
    else {
	    DBG_PRINT("INVALID Operation for pin %d\n", pin);
    }
    return val;
}

void analogReference(uint8_t type)
{
}


void hardware_init() 
{
    int status;

    /* get the device bindings */

    /* On-chip Quark GPIO */
    gpio07 = device_get_binding(CONFIG_GPIO_DW_0_NAME);            // GPIO[7:0]
    gpioLegacy = device_get_binding(CONFIG_GPIO_SCH_0_DEV_NAME);  // GPIO[9:8]
    gpio_sus = device_get_binding(CONFIG_GPIO_SCH_1_DEV_NAME);  //GPIO_SUS[5:0]

    /* external components */
    gpioEXP0 = device_get_binding(CONFIG_GPIO_PCAL9535A_0_DEV_NAME);
    gpioEXP1 = device_get_binding(CONFIG_GPIO_PCAL9535A_1_DEV_NAME);
    gpioEXP2 = device_get_binding(CONFIG_GPIO_PCAL9535A_2_DEV_NAME);

    /* i2c master of gpioEXP0/1/2 */
    i2c = device_get_binding(CONFIG_GPIO_PCAL9535A_1_I2C_MASTER_DEV_NAME);

    /* PWM */
    gpioPWM  = device_get_binding(CONFIG_PWM_PCA9685_0_DEV_NAME);
    adc  = device_get_binding(CONFIG_ADC_TI_ADC108S102_DRV_NAME);
    pwm_set_frequency(gpioPWM, PWM_FREQUENCY);

    pinmux = device_get_binding(PINMUX_NAME);

    if (!gpio07)
        {
        PRINT("GPIO DW not found!!\n");
        }

    if (!gpioLegacy)
        {
        PRINT("GPIO MMIO 0 not found!!\n");
        }

    if (!gpio_sus)
        {
        PRINT("GPIO MMIO 1 not found!!\n");
        }

    if (!gpioEXP0)
        {
        PRINT("EXP0 not found!!\n");
        }

    if (!gpioEXP1)
        {
        PRINT("EXP1 not found!!\n");
        }

    if (!gpioEXP2)
        {
        PRINT("EXP2 not found!!\n");
        }

    if (!i2c)
        {
        PRINT("I2C not found!!\n");
        }

    if (!gpioPWM)
        {
        PRINT("PWM not found!!\n");
	}

    if (!adc)
        {
        PRINT("ADC not found!!\n");
	}

    if (!pinmux)
        {
        PRINT("Pinmux not found!!\n");
        }

    if (!(gpio07 && gpioLegacy && gpio_sus && gpioEXP0 &&
          gpioEXP1 && gpioEXP2 && i2c && pinmux))
        {
        PRINT("Stopped.\n");
        return;
        }

    status = i2c_configure(i2c, (I2C_SPEED_FAST << 1) | I2C_MODE_MASTER);

    if (status != DEV_OK)
        {
        PRINT("I2C configuration error: %d Stopped.\n", status);
        return;
        }

    // adc_set_callback(adc, adcCallback);


    return;
}

#endif /* CONFIG_BOARD_GALILEO */
