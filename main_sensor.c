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
 */

/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_uart.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_twi.h"
#include "lm75b.h"
#include "mma7660.h"
#include "lis3dh.h"
#include "l3gd20h.h"
#include "app_trace.h"
#include "bmp280.h"

// LIS3DH STUFF
#define SENSORS_GRAVITY_STANDARD	9.81 // m/s^2
//#define LIS3DH_SENSITIVITY_2G  (0.001F)
//#define LIS3DH_SENSITIVITY_4G  (0.002F)
//#define LIS3DH_SENSITIVITY_8G  (0.004F)
//#define LIS3DH_SENSITIVITY_16G (0.012F)
#define LIS3DH_RANGE_16G  (0b11)
#define LIS3DH_RANGE_8G  (0b10)
#define LIS3DH_RANGE_4G  (0b01)
#define LIS3DH_RANGE_2G (0b00)
#define LIS3DH_SENSITIVITYDIVIDER_2G  16380
#define LIS3DH_SENSITIVITYDIVIDER_4G  8190
#define LIS3DH_SENSITIVITYDIVIDER_8G  4096
#define LIS3DH_SENSITIVITYDIVIDER_16G 1365

#define MAX_PENDING_TRANSACTIONS    20 //5

#define UART_TX_BUF_SIZE            256
#define UART_RX_BUF_SIZE            1

#define APP_TIMER_PRESCALER         0
#define APP_TIMER_OP_QUEUE_SIZE     2

// Pin number for indicating communication with sensors.
#ifdef BSP_LED_3
    #define READ_ALL_INDICATOR  BSP_LED_3
#else
    #error "Please choose an output pin"
#endif


static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(0);


// Buffer for data read from sensors.
#define BUFFER_SIZE  11
static uint8_t m_buffer[BUFFER_SIZE];
static uint8_t acc_buffer[6];
static uint8_t acc_whoami_buffer[1];
static uint8_t acc_status_buffer[1];
static uint8_t gyro_buffer[6];
static uint8_t gyro_whoami_buffer[1];
static uint8_t gyro_status_buffer[1];

// Data structures needed for averaging of data read from sensors.
// [max 32, otherwise "int16_t" won't be sufficient to hold the sum
//  of temperature samples]
#define NUMBER_OF_SAMPLES  16
typedef struct
{
    //int16_t temp;
    int16_t x;
    int16_t y;
    int16_t z;
} sum_t;
//static sum_t m_sum = { 0, 0, 0, 0 };
static sum_t m_sum = { 0, 0, 0 };

typedef struct
{
    // [use bit fields to fit whole structure into one 32-bit word]
//    int16_t temp : 11;
//    int8_t  x    : 6;
//  int8_t  y    : 6;
//    int8_t  z    : 6;
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} sample_t;
//static sample_t m_samples[NUMBER_OF_SAMPLES] = { { 0, 0, 0, 0 } };
static sample_t m_samples[NUMBER_OF_SAMPLES] = { { 0, 0, 0, 0, 0, 0 } };
static uint8_t m_sample_idx = 0;

#ifdef  __GNUC__
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    asm (".global _printf_float");
#endif


////////////////////////////////////////////////////////////////////////////////
// Reading of data from sensors - current temperature from LM75B and from
// MMA7660: X, Y, Z and tilt status.
//#if (BUFFER_SIZE < 6)
//    #error Buffer too small.
//#endif
//#define GET_ACC_VALUE(axis, reg_data) \
//    do { \
//        if (MMA7660_DATA_IS_VALID(reg_data)) \
//        { \
//            axis = MMA7660_GET_ACC(reg_data); \
//        } \
//    } while (0)
void read_all_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        app_trace_log("read_all_cb - error: %d\r\n", (int)result);
        return;
    }

    sample_t * p_sample = &m_samples[m_sample_idx];
    //m_sum.x    -= p_sample->x;
    //m_sum.y    -= p_sample->y;
    //m_sum.z    -= p_sample->z;
		
		int16_t x_out = (int16_t)(acc_buffer[1] << 8 | acc_buffer[0]); // Store buffer readings
    int16_t y_out = (int16_t)(acc_buffer[3] << 8 | acc_buffer[2]);
    int16_t z_out = (int16_t)(acc_buffer[5] << 8 | acc_buffer[4]);
		
		int16_t gyro_x_out = (int16_t)(gyro_buffer[1] << 8 | gyro_buffer[0]); // Store buffer readings
    int16_t gyro_y_out = (int16_t)(gyro_buffer[3] << 8 | gyro_buffer[2]);
    int16_t gyro_z_out = (int16_t)(gyro_buffer[5] << 8 | gyro_buffer[4]);

		// The LIS3DH returns a raw g value and needs to be adjusted according to the data sheet.
		// To convert toa normal g value, divide it by the appropriate sensitivity
		// To convert it to m/s^2, multiply it by standard gravity
		p_sample->x = ((float)x_out / LIS3DH_SENSITIVITYDIVIDER_2G)*(SENSORS_GRAVITY_STANDARD); // 2G by default
		p_sample->y = ((float)y_out / LIS3DH_SENSITIVITYDIVIDER_2G)*(SENSORS_GRAVITY_STANDARD);
		p_sample->z = ((float)z_out / LIS3DH_SENSITIVITYDIVIDER_2G)*(SENSORS_GRAVITY_STANDARD);
		
		p_sample->gyro_x = ((float)gyro_x_out * L3GD20H_SENSITIVITY_250DPS); // 250 DPS is the default. Change this if you change to a different resolution!
		p_sample->gyro_y = ((float)gyro_y_out * L3GD20H_SENSITIVITY_250DPS);
		p_sample->gyro_z = ((float)gyro_z_out * L3GD20H_SENSITIVITY_250DPS);

    //m_sum.x    += p_sample->x;
    //m_sum.y    += p_sample->y;
    //m_sum.z    += p_sample->z;

    ++m_sample_idx;
    if (m_sample_idx >= NUMBER_OF_SAMPLES)
    {
        m_sample_idx = 0;
    }

    // Show current average values every time sample index rolls over (for RTC
    // ticking at 32 Hz and 16 samples it will be every 500 ms) or when tilt
    // status changes.
//    if (m_sample_idx == 0 || (m_prev_tilt && m_prev_tilt != tilt))
		if (m_sample_idx != 0)
    {
			app_trace_log("X: %3d, Y: %3d, Z: %3d\r\n\n",
				x_out,y_out,z_out
//				p_sample->x,p_sample->y,p_sample->z
//				gyro_x_out,gyro_y_out,gyro_z_out
//				p_sample->gyro_x,p_sample->gyro_y,p_sample->gyro_z
            //(m_sum.temp * 0.125) / NUMBER_OF_SAMPLES,
            //m_sum.x / NUMBER_OF_SAMPLES,m_sum.y / NUMBER_OF_SAMPLES,m_sum.z / NUMBER_OF_SAMPLES
            //orientation,
            //MMA7660_TAP_DETECTED(tilt)   ? " TAP"   : "",
            //MMA7660_SHAKE_DETECTED(tilt) ? " SHAKE" : ""
			);
    }
}

/**************************************************************************/
/*!
    @brief  A function for reading data from all of the sensors
*/
/**************************************************************************/
static void read_all(void)
{
    // Signal on LED that something is going on.
    nrf_gpio_pin_toggle(READ_ALL_INDICATOR);

    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static app_twi_transfer_t const transfers[] =
    {
			LIS3DH_WHOAMI(&acc_whoami_buffer[0]),
			LIS3DH_STATUS(&acc_status_buffer[0]),
			LIS3DH_READ_XYZ(&acc_buffer[0]), //acc_buffer is the buffer for storing data
			
			L3GD20H_WHOAMI(&gyro_whoami_buffer[0]),
			L3GD20H_STATUS(&gyro_status_buffer[0]),
			L3GD20H_READ_XYZ(&gyro_buffer[0])
    };
    static app_twi_transaction_t const transaction =
    {
        .callback            = read_all_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

		//app_trace_log("LIS3DH_READ_XYZ complete\r\n");
		//uint8_t x_temp;
		//x_temp = (uint8_t)acc_buffer[0];
		//app_trace_log("X: %3d\r\n",x_temp);

		// CHECK ACC WHOAMI
		uint8_t deviceid;
		deviceid = (uint8_t)acc_whoami_buffer[0];
		app_trace_log("ACC WHOAMI: %#X\r\n",deviceid);
//		if (deviceid!=0x33)
//		{
//			app_trace_log("ACC WHO AM I: FALSE\r\n");
//		}
//		else
//		{
//			app_trace_log("ACC WHO AM I: TRUE\r\n");
//		}
		
		// CHECK ACC STATUS
		uint8_t status;
		status = (uint8_t)acc_status_buffer[0];
		app_trace_log("ACC STATUS: %#X\r\n",status);
//		app_trace_log("ACC STATUS: ");
//		for (int i = 0; i<8; i++){
//			app_trace_log("%d",(status & 0x8000) >> 7);
//			status <<= 1;
//		}
//		app_trace_log("\r\n");
		
		// CHECK GYRO WHOAMI
		//uint8_t deviceid;
		deviceid = (uint8_t)gyro_whoami_buffer[0];
		app_trace_log("GYRO WHOAMI: %#X\r\n",deviceid);
		
		// CHECK GYRO STATUS
		//uint8_t status;
		status = (uint8_t)gyro_status_buffer[0];
		app_trace_log("GYRO STATUS: ");
		for (int i = 0; i<8; i++){
			app_trace_log("%d",(status & 0x8000) >> 7);
			status <<= 1;
		}
		app_trace_log("\r\n");
		app_trace_log("GYRO STATUS: %#X\r\n",status);
		
		
		
		
		
		
		
		
		
		
    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}







/**************************************************************************/
/*!
    @brief  A function for printing data
*/
/**************************************************************************/
static void print_data(char const * name,
                       uint8_t const * p_buffer, uint8_t length)
{
    printf("\r\n%s", name);

    do {
        printf(" %02X", *p_buffer++);
    } while (--length);

    printf("\r\n\r\n");
}


/**************************************************************************/
/*!
    @brief  LM75B Functions
*/
/**************************************************************************/
#if (BUFFER_SIZE < 7)
    #error Buffer too small.
#endif
static void read_lm75b_registers_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        printf("read_lm75b_registers_cb - error: %d\r\n", (int)result);
        return;
    }

    print_data("LM75B:", m_buffer, 7);
}
static void read_lm75b_registers(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static app_twi_transfer_t const transfers[] =
    {
        LM75B_READ(&lm75b_conf_reg_addr,  &m_buffer[0], 1),
        LM75B_READ(&lm75b_temp_reg_addr,  &m_buffer[1], 2),
        LM75B_READ(&lm75b_tos_reg_addr,   &m_buffer[3], 2),
        LM75B_READ(&lm75b_thyst_reg_addr, &m_buffer[5], 2)
    };
    static app_twi_transaction_t const transaction =
    {
        .callback            = read_lm75b_registers_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers)/sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}


/**************************************************************************/
/*!
    @brief  MMA7660 Functions
*/
/**************************************************************************/
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


/**************************************************************************/
/*!
    @brief  LIS3DH Functions
*/
/**************************************************************************/
#if (BUFFER_SIZE < LIS3DH_NUMBER_OF_REGISTERS) //????????????
    #error Buffer too small.
#endif
static void read_lis3dh_registers_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
//        printf("read_lis3dh_registers_cb - error: %d\r\n", (int)result);
			  app_trace_log("read_lis3dh_registers_cb - error: %d\r\n", (int)result);

        return;
    }

    print_data("LIS3DH:", acc_buffer, LIS3DH_NUMBER_OF_REGISTERS);
}
static void read_lis3dh_registers(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static app_twi_transfer_t const transfers[] =
    {
        LIS3DH_READ(&lis3dh_register_out_x_addr,
            acc_buffer, LIS3DH_NUMBER_OF_REGISTERS)
    };
    static app_twi_transaction_t const transaction =
    {
        .callback            = read_lis3dh_registers_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers)/sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}

























////////////////////////////////////////////////////////////////////////////////
// Buttons handling (by means of BSP).
//
static void bsp_event_handler(bsp_event_t event)
{
    // Each time the button 1 or 4 is pushed we start a transaction reading
    // values of all registers from LM75B or MMA7660 respectively.
    switch (event)
    {
    case BSP_EVENT_KEY_0: // Button 1 pushed.
        read_lm75b_registers();
        break;

    case BSP_EVENT_KEY_3: // Button 4 pushed.
        read_mma7660_registers();
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
// UART handling.
//
//static void uart_event_handler(app_uart_evt_t * p_event)
//{
//    switch (p_event->evt_type)
//    {
//        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;

//        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;

//        default:
//            break;
//    }
//}
//static void uart_config(void)
//{
//    uint32_t err_code;

//    app_uart_comm_params_t const comm_params =
//    {
//        RX_PIN_NUMBER,
//        TX_PIN_NUMBER,
//        RTS_PIN_NUMBER,
//        CTS_PIN_NUMBER,
//        APP_UART_FLOW_CONTROL_ENABLED,
//        false,
//        UART_BAUDRATE_BAUDRATE_Baud115200
//    };

//    APP_UART_FIFO_INIT(&comm_params,
//                       UART_RX_BUF_SIZE,
//                       UART_TX_BUF_SIZE,
//                       uart_event_handler,
//                       APP_IRQ_PRIORITY_LOW,
//                       err_code);
//    APP_ERROR_CHECK(err_code);
//}


////////////////////////////////////////////////////////////////////////////////
// TWI (with transaction manager) initialization.
//
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
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

		app_trace_init();
    app_trace_log("App trace initialized\n\r");
	
    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER and for "read_all" ticks generation
    // (by RTC).
    lfclk_config();
		app_trace_log("LFCLK initialized\n\r");

		bsp_config();
		app_trace_log("BSP initialized\n\r");

    //uart_config();
    //printf("\n\rTWI master example\r\n");
	
		twi_config();
		app_trace_log("TWI initialized\n\r");

    // Initialize sensors.
		APP_ERROR_CHECK(app_twi_perform(&m_app_twi, lis3dh_init_transfers,
				LIS3DH_INIT_TRANSFER_COUNT,NULL));
		app_trace_log("LIS3DH initialized\n\r");

		APP_ERROR_CHECK(app_twi_perform(&m_app_twi, l3gd20h_init_transfers,
				L3GD20H_INIT_TRANSFER_COUNT,NULL));
		app_trace_log("LG3D20H initialized\n\r");

    rtc_config();

    while (true)
    {
        __WFI();
    }
}


/** @} */
