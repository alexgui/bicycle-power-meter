/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 *

** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_saadc.h"
#include "bsp.h"
#include "app_uart.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_twi.h"
#include "bmp280test.h"
#include "mma7660.h"

#include "SEGGER_RTT.h"



#define MAX_PENDING_TRANSACTIONS    50

#define APP_TIMER_PRESCALER         0
#define APP_TIMER_OP_QUEUE_SIZE     2

#define SAMPLES_IN_BUFFER 5
volatile uint8_t state = 1;

// Pin number for indicating communication with sensors.
#ifdef BSP_LED_3
    #define READ_ALL_INDICATOR  BSP_LED_3
#else
    #error "Please choose an output pin"
#endif

static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);


static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(0);

// Buffer for data read from sensors.
#define BUFFER_SIZE  50
static uint8_t m_buffer[BUFFER_SIZE];
static uint8_t p_buffer[BUFFER_SIZE];
static uint8_t cal_buffer[24];

// Data structures needed for averaging of data read from sensors.
// [max 32, otherwise "int16_t" won't be sufficient to hold the sum
//  of temperature samples]
#define NUMBER_OF_SAMPLES  16
typedef struct
{
	  int32_t temp;
	  int32_t pressure;
    int16_t x;
    int16_t y;
    int16_t z;
} sum_t;
static sum_t m_sum = { 0, 0, 0, 0, 0 };

typedef struct
{
    /*// [use bit fields to fit whole structure into one 32-bit word]
    int16_t temp : 11;
    int8_t  x    : 6;
    int8_t  y    : 6;
    int8_t  z    : 6;
	*/
		int32_t temp;
	  int32_t pressure;
    int16_t x;
    int16_t y;
    int16_t z;
} sample_t;
static sample_t m_samples[NUMBER_OF_SAMPLES] = { { 0, 0, 0, 0, 0 } };
static uint8_t m_sample_idx = 0;

// Value previously read from MMA7660's Tilt register - used to detect change
// in orientation, shake signaling etc.
static uint8_t m_prev_tilt = 0;

#ifdef  __GNUC__
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    asm (".global _printf_float");
#endif


////////////////////////////////////////////////////////////////////////////////
// Reading of data from sensors - current temperature and pressure from BMP280 and from
// MMA7660: X, Y, Z and tilt status.
#if (BUFFER_SIZE < 6)
    #error Buffer too small.
#endif
/*
#define GET_ACC_VALUE(axis, reg_data) \
    do { \
        if (MMA7660_DATA_IS_VALID(reg_data)) \
        { \
            axis = MMA7660_GET_ACC(reg_data); \
        } \
    } while (0)
		*/
void read_all_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        printf("read_all_cb - error: %d\r\n", (int)result);
        return;
    }

    sample_t * p_sample = &m_samples[m_sample_idx];
    m_sum.temp -= p_sample->temp;
		m_sum.pressure -= p_sample->pressure;
    m_sum.x    -= p_sample->x;
    m_sum.y    -= p_sample->y;
    m_sum.z    -= p_sample->z;

    uint8_t temp_MSB = m_buffer[0];
		uint8_t temp_LSB = m_buffer[1];
    uint8_t temp_XLSB = m_buffer[2];

		uint8_t press_MSB = p_buffer[0];
		uint8_t press_LSB = p_buffer[1];
    uint8_t press_XLSB = p_buffer[2];
		
		SEGGER_RTT_printf(0,"m_buffer[0]:%#02X\r\n",m_buffer[0]);
		SEGGER_RTT_printf(0,"m_buffer[1]:%#02X\r\n",m_buffer[1]);
		SEGGER_RTT_printf(0,"m_buffer[2]:%#02X\r\n",m_buffer[2]);
		
		SEGGER_RTT_printf(0,"p_buffer[0]:%#02X\r\n",p_buffer[0]);
		SEGGER_RTT_printf(0,"p_buffer[1]:%#02X\r\n",p_buffer[1]);
		SEGGER_RTT_printf(0,"p_buffer[2]:%#02X\r\n",p_buffer[2]);
		SEGGER_RTT_printf(0,"p_buffer[3]:%#02X\r\n",p_buffer[3]);
		SEGGER_RTT_printf(0,"p_buffer[4]:%#02X\r\n",p_buffer[4]);
		SEGGER_RTT_printf(0,"p_buffer[5]:%#02X\r\n",p_buffer[5]);

    uint8_t x_out   = m_buffer[6];
    uint8_t y_out   = m_buffer[7];
    uint8_t z_out   = m_buffer[8];
    uint8_t tilt    = m_buffer[9];
		
		bmp280_calib_data cal_data;
		cal_data.dig_T1 = BMP280_GET_CAL_U((uint8_t)cal_buffer[0],(uint8_t)cal_buffer[1]);
		cal_data.dig_T2 = BMP280_GET_CAL_U((uint8_t)cal_buffer[2],(uint8_t)cal_buffer[3]);
		cal_data.dig_T3 = BMP280_GET_CAL_U((uint8_t)cal_buffer[4],(uint8_t)cal_buffer[5]);
		
		cal_data.dig_P1 = BMP280_GET_CAL_U((uint8_t)cal_buffer[6],(uint8_t)cal_buffer[7]);
		cal_data.dig_P2 = BMP280_GET_CAL_U((uint8_t)cal_buffer[8],(uint8_t)cal_buffer[9]);
		cal_data.dig_P3 = BMP280_GET_CAL_U((uint8_t)cal_buffer[10],(uint8_t)cal_buffer[11]);
	  cal_data.dig_P4 = BMP280_GET_CAL_U((uint8_t)cal_buffer[12],(uint8_t)cal_buffer[13]);
		cal_data.dig_P5 = BMP280_GET_CAL_U((uint8_t)cal_buffer[14],(uint8_t)cal_buffer[15]);
		cal_data.dig_P6 = BMP280_GET_CAL_U((uint8_t)cal_buffer[16],(uint8_t)cal_buffer[17]);
		cal_data.dig_P7 = BMP280_GET_CAL_U((uint8_t)cal_buffer[18],(uint8_t)cal_buffer[19]);
		cal_data.dig_P8 = BMP280_GET_CAL_U((uint8_t)cal_buffer[20],(uint8_t)cal_buffer[21]);
		cal_data.dig_P9 = BMP280_GET_CAL_U((uint8_t)cal_buffer[22],(uint8_t)cal_buffer[23]);

		// Scan the uncompensated temp and pressure
		long signed uncompensatedTemp = BMP280_GET_TEMPERATURE_VALUE(temp_MSB, temp_LSB, temp_XLSB);
		long signed uncompensatedPress = BMP280_GET_PRESSURE_VALUE(press_MSB, press_LSB, press_XLSB);
    // define temporary variables for compensation
		long var1, var2, t_fine, p;
		// temperature compensation equation from the BMP280 datasheet
		var1  = ((((uncompensatedTemp>>3)-((cal_data.dig_T1<<1))) * (cal_data.dig_T2)) >> 11);
		var2  = (((((uncompensatedTemp>>4) -(cal_data.dig_T1)) * ((uncompensatedTemp>>4) -((cal_data.dig_T1))) >> 12) * (cal_data.dig_T3)) >> 14);
		t_fine = var1 + var2;
		SEGGER_RTT_printf(0,"t_fine:%d\r\n",t_fine);
		long compensatedTemp = (t_fine*5+128)>>8;
		SEGGER_RTT_printf(0,"BMP280 uncompensated temp:%d\r\n",uncompensatedTemp);
		SEGGER_RTT_printf(0,"BMP280 compensated temp:%d\r\n",compensatedTemp);
		
		SEGGER_RTT_printf(0,"CAL data T1:%#02X\r\n",cal_data.dig_T1);
		SEGGER_RTT_printf(0,"CAL data T2:%#02X\r\n",cal_data.dig_T2);
		SEGGER_RTT_printf(0,"CAL data T3:%#02X\r\n",cal_data.dig_T3);
		SEGGER_RTT_printf(0,"CAL buffer:%#12X\r\n",cal_buffer);
		
		// pressure compensation equation from the BMP280 datasheet
		var1 = (t_fine>>1)-64000;
		var2 = (((var1>>2) * (var1>>2)) >> 11 ) * (cal_data.dig_P6);
		var2 = var2 + ((var1*(cal_data.dig_P5))<<1);
		var2 = (var2>>2)+((cal_data.dig_P4)<<16);
		var1 = (((cal_data.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + (((cal_data.dig_P2) * var1)>>1))>>18;
		var1 =((((32768+var1))*(cal_data.dig_P1))>>15);
		long compensatedPress;
		if (var1 == 0){
			p=0;
		}
		p = ((((1048576)-uncompensatedPress)-(var2>>12)))*3125;
		if (p < 0x80000000) {
			p = (p << 1)/(var1);
		}
		else {
			p = (p / var1) * 2;
		}
		var1 = ((cal_data.dig_P9) * ((((p>>3) * (p>>3))>>13)))>>12;
		var2 = (((p>>2)) * (cal_data.dig_P8))>>13;
		p = (p + ((var1 + var2 + cal_data.dig_P7) >> 4));
		compensatedPress=p;
		SEGGER_RTT_printf(0,"BMP280 uncompensated press:%d\r\n",uncompensatedPress);
		SEGGER_RTT_printf(0,"BMP280 compensated press:%d\r\n",compensatedPress);
		
		p_sample->temp = compensatedTemp;
		p_sample->pressure = compensatedPress;		
		
		//printf("Uncompensated temp%d",(int)uncompensatedPress);
		//printf("Compensated temp%d",(int)p_sample->temp);		
		//printf("Uncompensated press%d",(int)uncompensatedPress);
		//printf("Compensated press%d",(int)p_sample->pressure);
		
		
		/*
    GET_ACC_VALUE(p_sample->x, x_out);
    GET_ACC_VALUE(p_sample->y, y_out);
    GET_ACC_VALUE(p_sample->z, z_out);
    if (!MMA7660_DATA_IS_VALID(tilt))
    {
        tilt = m_prev_tilt;
    }

    m_sum.temp += p_sample->temp;
    m_sum.x    += p_sample->x;
    m_sum.y    += p_sample->y;
    m_sum.z    += p_sample->z;

    ++m_sample_idx;
    if (m_sample_idx >= NUMBER_OF_SAMPLES)
    {
        m_sample_idx = 0;
    }

    // Show current average values every time sample index rolls over (for RTC
    // ticking at 32 Hz and 16 samples it will be every 500 ms) or when tilt
    // status changes.
    if (m_sample_idx == 0 || (m_prev_tilt && m_prev_tilt != tilt))
    {
        char const * orientation;
        switch (MMA7660_GET_ORIENTATION(tilt))
        {
            case MMA7660_ORIENTATION_LEFT:  orientation = "LEFT";  break;
            case MMA7660_ORIENTATION_RIGHT: orientation = "RIGHT"; break;
            case MMA7660_ORIENTATION_DOWN:  orientation = "DOWN";  break;
            case MMA7660_ORIENTATION_UP:    orientation = "UP";    break;
            default:                        orientation = "?";     break;
        }
        printf("Temp: %5.1f | X: %3d, Y: %3d, Z: %3d | %s%s%s\r\n",
            (m_sum.temp * 0.125) / NUMBER_OF_SAMPLES,
            m_sum.x / NUMBER_OF_SAMPLES,
            m_sum.y / NUMBER_OF_SAMPLES,
            m_sum.z / NUMBER_OF_SAMPLES,
            orientation,
            MMA7660_TAP_DETECTED(tilt)   ? " TAP"   : "",
            MMA7660_SHAKE_DETECTED(tilt) ? " SHAKE" : "");
        m_prev_tilt = tilt;
			
    }*/
}
static void read_all(void)
{
    // Signal on LED that something is going on.
    nrf_gpio_pin_toggle(READ_ALL_INDICATOR);

    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static app_twi_transfer_t const transfers[] =
    {
        BMP280_READ_TEMP(&m_buffer)
        ,
			  BMP280_READ_PRESSURE(&p_buffer)
        ,
			  BMP280_READ_CAL(&cal_buffer)
        ,
        //MMA7660_READ_XYZ_AND_TILT(&m_buffer[8])
			  //,
    };
    static app_twi_transaction_t const transaction =
    {
        .callback            = read_all_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}


static void print_data(char const * name,
                       uint8_t const * p_buffer, uint8_t length)
{
    printf("\r\n%s", name);

    do {
        printf(" %02X", *p_buffer++);
    } while (--length);

    printf("\r\n\r\n");
}


#if (BUFFER_SIZE < 7)
    #error Buffer too small.
#endif

//reads the uncompensated data and prints it out
static void read_BMP280_registers_cb(ret_code_t result, void * p_user_data)
{
		SEGGER_RTT_WriteString(0, "CHECK 2\n");
    if (result != NRF_SUCCESS)
    {
				SEGGER_RTT_WriteString(0, "ERROR\n");
        printf("read_BMP280_registers_cb - error: %d\r\n", (int)result);
        return;
    }
		uint8_t temp_MSB = m_buffer[0];
		uint8_t temp_LSB = m_buffer[1];
    uint8_t temp_XLSB = m_buffer[2];

		uint8_t press_MSB = p_buffer[0];
		uint8_t press_LSB = p_buffer[1];
    uint8_t press_XLSB = p_buffer[2];
		
		uint32_t test = BMP280_GET_TEMPERATURE_VALUE(temp_MSB, temp_LSB, temp_XLSB);
		
		printf("BMP280 temp:%d\r\n",test);
    print_data("BMP280:", m_buffer, 7);
		
		SEGGER_RTT_printf(0,"BMP280 uncompensated temp:%d\r\n",test);
	SEGGER_RTT_WriteString(0, "Read BMP280 Registers done\n");
}
static void read_BMP280_registers(void)
{
	
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static app_twi_transfer_t const transfers[] =
    {
        //BMP280_READ(&BMP280_conf_reg_addr,  &m_buffer[0], 1),
        //BMP280_READ(&BMP280_temp_reg_addr,  &m_buffer[1], 2),
        //BMP280_READ(&BMP280_tos_reg_addr,   &m_buffer[3], 2),
        //BMP280_READ(&BMP280_thyst_reg_addr, &m_buffer[5], 2)
			
			  //BMP280_READ(&BMP280_cont_reg_addr,  &m_buffer[0], 1),
        //BMP280_READ(&BMP280_conf_reg_addr,  &m_buffer[1], 2),
        //BMP280_READ(&BMP280_temp_reg_addr,   &m_buffer[3], 3),
        //BMP280_READ(&BMP280_pressure_reg_addr, &m_buffer[6], 3),
			  //BMP280_READ(&BMP280_cal_reg_addr, &cal_buffer[0], 24)
				
        BMP280_READ(&BMP280_temp_reg_addr,   &m_buffer[0], 3),
        BMP280_READ(&BMP280_pressure_reg_addr, &p_buffer[0], 3),
			  BMP280_READ(&BMP280_cal_reg_addr, &cal_buffer[0], 24)
    };
		SEGGER_RTT_WriteString(0, "CHECK 1\n");

    static app_twi_transaction_t const transaction =
    {
        .callback            = read_BMP280_registers_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers)/sizeof(transfers[0])
    };
		nrf_delay_ms(500);
		SEGGER_RTT_WriteString(0, "CHECK 2\n");
    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
		
}


#if (BUFFER_SIZE < MMA7660_NUMBER_OF_REGISTERS)
    #error Buffer too small.
#endif
static void read_mma7660_registers_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        printf("read_mma7660_registers_cb - error: %d\r\n", (int)result);
        return;
    }

    print_data("MMA7660:", m_buffer, MMA7660_NUMBER_OF_REGISTERS);
}
static void read_mma7660_registers(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static app_twi_transfer_t const transfers[] =
    {
        MMA7660_READ(&mma7660_xout_reg_addr,
            m_buffer, MMA7660_NUMBER_OF_REGISTERS)
    };
    static app_twi_transaction_t const transaction =
    {
        .callback            = read_mma7660_registers_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers)/sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}

////////////////////////////////////////////////////////////////////////////////
// TWI (with transaction manager) initialization.
//
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };
		
    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
		if(err_code != NRF_SUCCESS){
			SEGGER_RTT_WriteString(0, "Init Config Failed\n");
		}
		else{
			SEGGER_RTT_WriteString(0, "Init Config Succeeded\n");
		}
}

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

////////////////////////////////////////////////////////////////////////////////
// Buttons handling (by means of BSP).
//
static void bsp_event_handler(bsp_event_t event)
{
  LEDS_CONFIGURE(LEDS_MASK);  
	// Each time the button 1 or 4 is pushed we start a transaction reading
    // values of all registers from LM75B or MMA7660 respectively.
    switch (event)
    {
    case BSP_EVENT_KEY_0: // Button 1 pushed.
				LEDS_INVERT(1 << leds_list[1]);
				nrf_delay_ms(500);
				SEGGER_RTT_WriteString(0, "read started\n");
				read_BMP280_registers();
				SEGGER_RTT_WriteString(0, "read complete\n");
				nrf_delay_ms(500);
        break;

		case BSP_EVENT_KEY_1: // Button 4 pushed.
        SEGGER_RTT_WriteString(0, "Button 1 Pressed\n");
				read_all();
				nrf_delay_ms(500);
        break;
		
    case BSP_EVENT_KEY_3: // Button 4 pushed.
        SEGGER_RTT_WriteString(0, "Button 4 Pressed\n");
				twi_config();
				nrf_delay_ms(500);
        break;

		case BSP_EVENT_KEY_2: // Button 3 pushed.
        LEDS_INVERT(1 << leds_list[2]);
				SEGGER_RTT_WriteString(0, "Button 3 Pressed\n");
				nrf_delay_ms(500);
        break;
				
    default:
        break;
    }
}
static void bsp_config(void)
{
    uint32_t err_code;

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    err_code = bsp_init(BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}

////////////////////////////////////////////////////////////////////////////////
// RTC tick events generation.
//
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        // On each RTC tick (their frequency is set in "nrf_drv_config.h")
        // we read data from our sensors.
        read_all();
    }
}
static void rtc_config(void)
{
    uint32_t err_code;

    // Initialize RTC instance with default configuration.
    err_code = nrf_drv_rtc_init(&m_rtc, NULL, rtc_handler);
    APP_ERROR_CHECK(err_code);

    // Enable tick event and interrupt.
    nrf_drv_rtc_tick_enable(&m_rtc, true);

    // Power on RTC instance.
    nrf_drv_rtc_enable(&m_rtc);
}


static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}


int main(void)
{
    LEDS_CONFIGURE(1U << READ_ALL_INDICATOR);
    LEDS_OFF(1U << READ_ALL_INDICATOR);

    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER and for "read_all" ticks generation
    // (by RTC).
    lfclk_config();

    bsp_config();
//    uart_config();

    printf("\n\rTWI master example\r\n");
		

    twi_config();

    // Initialize sensors.
    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, BMP280_init_transfers,
        BMP280_INIT_TRANSFER_COUNT, NULL));
    //APP_ERROR_CHECK(app_twi_perform(&m_app_twi, mma7660_init_transfers,
    //    MMA7660_INIT_TRANSFER_COUNT, NULL));
		
    rtc_config();
		
		read_all();
    while (true)
    {
			  
        __WFI();
    }
}


/** @} */
