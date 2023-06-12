/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "FreeRTOS.h"
#include "task.h"

#define LPS22HB_BIT(x) ((uint8_t)x)

#define LPS22HB_WHO_AM_I	0X0F //Who am I
#define LPS22HB_RES_CONF	0X1A //Resolution
#define LPS22HB_CTRL_REG1	0X10
#define LPS22HB_CTRL_REG2	0X11
#define LPS22HB_STATUS_REG	0X27
#define LPS22HB_PRES_OUT_XL	0X28 //LSB
#define LPS22HB_PRES_OUT_L	0X29
#define LPS22HB_PRES_OUT_H	0X2A //MSB
#define LPS22HB_TEMP_OUT_L	0X2B //LSB
#define LPS22HB_TEMP_OUT_H	0X2C //MSB
#define LPS22HB_WHO_AM_I_VALUE	0xB1 // Expected return value of WHO_AM_I register

#define LPS22HB_ODR_MASK                0x70
#define LPS22HB_LPFP_MASK               0x08
#define LPS22HB_LPFP_CUTOFF_MASK        0x04
#define LPS22HB_BDU_MASK                0x02
#define LPS22HB_SIM_MASK                0x01
#define LPS22HB_LCEN_MASK               0x01
#define LPS22HB_ADD_INC_MASK            0x10
#define LPS22HB_ADD_INC_BIT             LPS22HB_BIT(4)
#define LPS22HB_LPFP_BIT                LPS22HB_BIT(3)

#define SOLENOID_LOCK_DELAY 	43859642// 1000ms

/******** proximity registers ****************************/

#define COLOR14_REG_MAIN_CTRL       0x00
#define COLOR14_REG_PS_VCSEL        0x01
#define COLOR14_REG_PS_PULSES       0x02
#define COLOR14_REG_PS_MEASRATE     0x03
#define COLOR14_REG_LS_MEAS_RATE    0x04
#define COLOR14_REG_LS_GAIN         0x05
#define COLOR14_REG_PART_ID         0x06
#define COLOR14_REG_MAIN_STATUS     0x07
#define COLOR14_REG_PS_DATA_0       0x08
#define COLOR14_REG_PS_DATA_1       0x09
#define COLOR14_REG_LS_DATA_IR_0    0x0A
#define COLOR14_REG_LS_DATA_IR_1    0x0B
#define COLOR14_REG_LS_DATA_IR_2    0x0C
#define COLOR14_REG_LS_DATA_GREEN_0 0x0D
#define COLOR14_REG_LS_DATA_GREEN_1 0x0E
#define COLOR14_REG_LS_DATA_GREEN_2 0x0F
#define COLOR14_REG_LS_DATA_BLUE_0  0x10
#define COLOR14_REG_LS_DATA_BLUE_1  0x11
#define COLOR14_REG_LS_DATA_BLUE_2  0x12
#define COLOR14_REG_LS_DATA_RED_0   0x13
#define COLOR14_REG_LS_DATA_RED_1   0x14
#define COLOR14_REG_LS_DATA_RED_2   0x15
#define COLOR14_REG_INT_CFG         0x19
#define COLOR14_REG_INT_PST         0x1A
#define COLOR14_REG_PS_THRES_UP_0   0x1B
#define COLOR14_REG_PS_THRES_UP_1   0x1C
#define COLOR14_REG_PS_THRES_LOW_0  0x1D
#define COLOR14_REG_PS_THRES_LOW_1  0x1E
#define COLOR14_REG_PS_CAN_0        0x1F
#define COLOR14_REG_PS_CAN_1_ANA    0x20
#define COLOR14_REG_LS_THRES_UP_0   0x21
#define COLOR14_REG_LS_THRES_UP_1   0x22
#define COLOR14_REG_LS_THRES_UP_2   0x23
#define COLOR14_REG_LS_THRES_LOW_0  0x24
#define COLOR14_REG_LS_THRES_LOW_1  0x25
#define COLOR14_REG_LS_THRES_LOW_2  0x26
#define COLOR14_REG_LS_THRES_VAR    0x27

#define COLOR14_EXAMPLE_PS_LS   3
#define COLOR14_EXAMPLE_RGB     6

#define LED_DRIVER_1 0x01
#define LED_DRIVER_2 0x02
#define LED_DRIVER_3 0x04
#define LED_DRIVER_4 0x08
#define LED_DRIVER_5 0x10
#define LED_DRIVER_6 0x20
#define LED_DRIVER_ALL 0x3F

#define LED_1_RED   0X0F
#define LED_1_GREEN 0X10
#define LED_1_BLUE  0X11
#define LED_2_RED   0X12
#define LED_2_GREEN 0X13
#define LED_2_BLUE  0X14
#define LED_3_RED   0X15
#define LED_3_GREEN 0X16
#define LED_3_BLUE  0X17
#define LED_4_RED   0X18
#define LED_4_GREEN 0X19
#define LED_4_BLUE  0X1A
#define LED_5_RED   0X1B
#define LED_5_GREEN 0X1C
#define LED_5_BLUE  0X1D
#define LED_6_RED   0X1E
#define LED_6_GREEN 0X1F
#define LED_6_BLUE  0X20

#define LED1 1
#define LED2 2
#define LED3 3
#define LED4 4
#define LED5 5
#define LED6 6
  /*RES_CONF see LC_EN bit*/
 /**
* @brief  LPS22HB Power/Noise Mode configuration.
*/
typedef enum {
  LPS22HB_LOW_NOISE   =  (uint8_t)0x00,       /*!< Low Noise mode */
  LPS22HB_LOW_POWER   =  (uint8_t)0x01        /*!< Low Current mode */
} pressure_powermode_t;

/**
* @brief  Output data rate configuration.
*/
typedef enum {

  LPS22HB_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  LPS22HB_ODR_1HZ       = (uint8_t)0x10,         /*!< Output Data Rate: 1Hz */
  LPS22HB_ODR_10HZ       = (uint8_t)0x20,         /*!< Output Data Rate: 10Hz */
  LPS22HB_ODR_25HZ    = (uint8_t)0x30,         /*!< Output Data Rate: 25Hz */
  LPS22HB_ODR_50HZ      = (uint8_t)0x40,          /*!< Output Data Rate: 50Hz */
  LPS22HB_ODR_75HZ      = (uint8_t)0x50          /*!< Output Data Rate: 75Hz */
} pressure_odr_t;

typedef enum
{
	LPS22HB_DISABLE = (uint8_t)0,
	LPS22HB_ENABLE = !LPS22HB_DISABLE
} pressure_state_t;

typedef enum {
  LPS22HB_BDU_CONTINUOUS_UPDATE     =  0x00,  /*!< Data updated continuously */
  LPS22HB_BDU_NO_UPDATE             =  (uint8_t)0x02   /*!< Data updated after a read operation */

}pressure_bdu_t;

/**
* @brief  Low Pass Filter Cutoff Configuration.
*/
typedef enum {

  LPS22HB_ODR_9  = (uint8_t)0x00,         /*!< Filter Cutoff ODR/9 */
  LPS22HB_ODR_20 = (uint8_t)0x04          /*!< Filter Cutoff ODR/20 */
} pressure_lpf_cutoff_t;

/**
* @brief  LPS22HB Spi Mode configuration.
*/
typedef enum {
  LPS22HB_SPI_4_WIRE   =  (uint8_t)0x00,
  LPS22HB_SPI_3_WIRE   =  (uint8_t)0x01
} pressure_spi_mode_t;



void sensors_init(void);
ret_t pressure_sensor_enable(void);
void pressure_sensor_id_read(void);
ret_t pressure_sensor_get_pressure(float* pfdata);
ret_t get_raw_pressure(int32_t *raw_press);
ret_t get_pressure(int32_t* pout);
ret_t pressure_set_power_mode(pressure_powermode_t mode);
ret_t pressure_set_odr(pressure_odr_t odr);
ret_t pressure_get_odr(pressure_odr_t* odr);
ret_t get_odr(float* odr);
ret_t pressure_set_spi_interface(pressure_spi_mode_t spimode);
ret_t pressure_set_bdu(pressure_bdu_t bdu);
ret_t pressure_set_odr_when_enabled(float odr);
ret_t pressure_set_lowpass_filter(pressure_state_t state);
ret_t pressure_set_lowPass_filter_cutoff(pressure_lpf_cutoff_t cutoff);
ret_t pressure_set_automatic_increment_reg_address(pressure_state_t status);


ret_t get_proximity (uint16_t *ps_data);

int8_t get_cart_1_hall_sensor_value(void);
int8_t get_cart_2_hall_sensor_value(void);
int8_t get_cart_3_hall_sensor_value(void);
int8_t get_cart_4_hall_sensor_value(void);
int8_t get_cart_5_hall_sensor_value(void);
int8_t get_cart_1_limit_sw_value(void);
int8_t get_cart_2_limit_sw_value(void);
int8_t get_cart_3_limit_sw_value(void);
int8_t get_cart_4_limit_sw_value(void);
int8_t get_cart_5_limit_sw_value(void);
int8_t get_cart_1_ir_sensor_value(void);
int8_t get_cart_2_ir_sensor_value(void);
int8_t get_cart_3_ir_sensor_value(void);
int8_t get_cart_4_ir_sensor_value(void);
int8_t get_cart_5_ir_sensor_value(void);
int8_t get_master_ir_sensor_two_value(void);
int8_t get_master_ir_sensor_one_value(void);

void cart_1_solenoid(void);
void cart_2_solenoid(void);
void cart_3_solenoid(void);
void cart_4_solenoid(void);
void cart_5_solenoid(void);
void top_lid_solenoid(void);
void drawer_solenoid(void);

int8_t get_smoke_sensor_value(void);
uint32_t hx711_weight_value(void);
uint8_t get_battery_voltage(void);
int8_t get_battery_states(void);

void LED_DRIVER_COLOR_CONFIG(uint8_t index,uint8_t red,uint8_t green,uint8_t blue);

extern volatile bool sensors_init_done_flag;
extern TaskHandle_t sensors_init_task_handle;
extern volatile bool start_read_loadcell_value_flag;
extern bool i2c_tx_runing_flag;
extern bool i2c_rx_runing_flag;


#endif /* PRESSURE_SENSOR_H */
/* End of file ---------------------------------------------------------------*/
