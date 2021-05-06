/*
This software is subject to the license described in the license.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2012
All rights reserved.
*/

/**@file
 * @defgroup ant_bpwr_sensor_main ANT Bicycle Power Display and Bicycle Speed Cadence Sensor with SAADC Force Input
 * @{
 * @ingroup nrf_ant_bicycle_power
 *
 * @brief ANT+ bicycle power profile display, combined with an ANT+ bicycle speed/cadence sensor. This project is combined with help from the NRF52 SDK by Nordic.
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

// From BPWR
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "app_uart.h"
#include "nrf_soc.h"
#include "bsp.h"
#include "app_error.h"
#include "nordic_common.h"
#include "ant_stack_config.h"
#include "softdevice_handler.h"
#include "ant_bpwr.h"
#include "app_trace.h"
#include "ant_state_indicator.h"
#include "ant_key_manager.h"
#include "app_timer.h"
#include "ant_bpwr_simulator.h"
// Additional #includes from BSC
#include <stdbool.h>
#include <stdint.h>
#include "nrf_sdm.h"
#include "ant_bsc.h"
// Additional #includes from SAADC
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

#ifndef MODIFICATION_TYPE // can be provided as preprocesor global symbol
/**
 * @brief Depending of this define value Heart Rate value will be: @n
 *          - periodicaly rise and fall, use value  MODIFICATION_TYPE_AUTO
 *          - changing by button, use value         MODIFICATION_TYPE_BUTTON
 */
    #define MODIFICATION_TYPE (MODIFICATION_TYPE_AUTO)
#endif

#define MODIFICATION_TYPE_BUTTON 0 /* predefined value, MUST REMAIN UNCHANGED */
#define MODIFICATION_TYPE_AUTO   1 /* predefined value, MUST REMAIN UNCHANGED */

#if (MODIFICATION_TYPE != MODIFICATION_TYPE_BUTTON) \
    && (MODIFICATION_TYPE != MODIFICATION_TYPE_AUTO)

    #error Unsupported value of MODIFICATION_TYPE.
#endif

// BPWR DEFINITIONS
#ifndef SENSOR_TYPE // can be provided as preprocesor global symbol
    #define SENSOR_TYPE (TORQUE_NONE)
#endif

#define APP_TIMER_PRESCALER         0x00 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE     0x04 /**< Size of timer operation queues. */

#define BPWR_CHANNEL_NUMBER         0x00 /**< Channel number assigned to Bicycle Power profile. */

#define BPWR_DEVICE_NUMBER          1u /**< Denotes the used ANT device number. */
#define BPWR_TRANSMISSION_TYPE      5u /**< Denotes the used ANT transmission type. */

#define HW_REVISION                 0x7Fu   /**< Hardware revision for manufacturer's identification common page. */
#define MANUFACTURER_ID             0xAAAAu /**< Manufacturer ID for manufacturer's identification common page. */
#define MODEL_NUMBER                0x5555u /**< Model number for manufacturer's identification common page. */

#define SW_REVISION_MAJOR           0xAAu       /**< Software revision major number for product information common page. */
#define SW_REVISION_MINOR           0xFFu       /**< Software revision minor number for product information common page, unused value. */
#define SERIAL_NUMBER               0xAA55AA55u /**< Serial number for product information common page. */

#define ANTPLUS_NETWORK_NUMBER      0x00 		/**< Network number. */
#define CALIBRATION_DATA            0x55AAu /**< General calibration data value. */

// BSC DEFINITIONS
//#define APP_TIMER_PRESCALER         0x00    /**< Value of the RTC1 PRESCALER register. */
//#define APP_TIMER_OP_QUEUE_SIZE     0x04    /**< Size of timer operation queues. */

#define BSC_CHANNEL_NUMBER          0x01 //0x00		/**< Channel number assigned to BSC profile. */

#define WILDCARD_TRANSMISSION_TYPE  0x00    /**< Wildcard transmission type. */
#define WILDCARD_DEVICE_NUMBER      0x00    /**< Wildcard device number. */

#define ANTPLUS_NETWORK_NUMBER      0x00    /**< Network number. */

#define WHEEL_CIRCUMFERENCE         2130 //2070    /**< Bike wheel circumference [mm] */
#define BSC_EVT_TIME_FACTOR         1024    /**< Time unit factor for BSC events */
#define BSC_RPM_TIME_FACTOR         60      /**< Time unit factor for RPM unit */
#define BSC_MS_TO_KPH_NUM           36      /**< Numerator of [m/s] to [kph] ratio */
#define BSC_MS_TO_KPH_DEN           10   		/**< Denominator of [m/s] to [kph] ratio */
#define BSC_MM_TO_M_FACTOR          1000		/**< Unit factor [m/s] to [mm/s] */
#define BSC_SPEED_UNIT_FACTOR       (BSC_MS_TO_KPH_DEN * BSC_MM_TO_M_FACTOR)                        /**< Speed unit factor */
#define SPEED_COEFFICIENT           (WHEEL_CIRCUMFERENCE * BSC_EVT_TIME_FACTOR * BSC_MS_TO_KPH_NUM) /**< Coefficient for speed value calculation */
#define CADENCE_COEFFICIENT         (BSC_EVT_TIME_FACTOR * BSC_RPM_TIME_FACTOR)                     /**< Coefficient for cadence value calculation */
#define DISPLAY_TYPE 								(BSC_COMBINED_DEVICE_TYPE) 		/** Combined bike speed/cadence sensor */

#define BSC_RESET_CTR								30 /**< Number of iterations before BSC values reset to 0 */

// Here is some stuff for SAADC!
//#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE 1   /**< UART RX buffer size. */

#define SAMPLES_IN_SAADC_BUFFER 5
volatile uint8_t state = 1;

static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(1); // The softdevice will use timer(0), so use timer(1) here for saadc
static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_SAADC_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;

int16_t SAADC_VALUE = 0;

uint32_t speed = 0;
uint32_t cadence = 0;
uint32_t power_in_watts = 0;
uint32_t *power_pointer = &power_in_watts;

#define MIN_SAADC 38 // SSAADC value when SAADC is tied to ground
#define MAX_SAADC 630 // SAADC value when Arduino is outputting MAX_FORCE
#define MAX_FORCE 300 // Max force input should correlate to the limits set in the Arduino for max force, Newtons
#define MIN_FORCE 0 // Min force input from Arduino, Newtons

#ifdef ENABLE_DEBUG_LOG_SUPPORT
static int32_t accumulated_s_rev_cnt, previous_s_evt_cnt, prev_s_accumulated_rev_cnt,
               accumulated_s_evt_time, previous_s_evt_time, prev_s_accumulated_evt_time,
							 s_evt_ctr, psarc = 0;

static int32_t accumulated_c_rev_cnt, previous_c_evt_cnt, prev_c_accumulated_rev_cnt,
               accumulated_c_evt_time, previous_c_evt_time, prev_c_accumulated_evt_time,
							 c_evt_ctr, pcarc = 0;

// static int32_t COMPUTEDSPEED, COMPUTEDCADENCE = 0;
#endif // ENABLE_DEBUG_LOG_SUPPORT

//APP_TIMER_DEF(bsc_spd_timer_id);
//APP_TIMER_DEF(bsc_cad_timer_id);

/** @snippet [ANT BPWR TX Instance] */
void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event);
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1);

BPWR_SENS_CHANNEL_CONFIG_DEF(m_ant_bpwr,
                             BPWR_CHANNEL_NUMBER,
                             BPWR_TRANSMISSION_TYPE,
                             BPWR_DEVICE_NUMBER,
                             ANTPLUS_NETWORK_NUMBER);
BPWR_SENS_PROFILE_CONFIG_DEF(m_ant_bpwr,
                            (ant_bpwr_torque_t)(SENSOR_TYPE),
                            ant_bpwr_calib_handler,
                            ant_bpwr_evt_handler);

ant_bpwr_profile_t m_ant_bpwr;
/** @snippet [ANT BPWR TX Instance] */

/** @snippet [ANT BSC RX Instance] */
void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event);

BSC_DISP_CHANNEL_CONFIG_DEF(m_ant_bsc,
                            BSC_CHANNEL_NUMBER,
                            WILDCARD_TRANSMISSION_TYPE,
                            DISPLAY_TYPE,
                            WILDCARD_DEVICE_NUMBER,
                            ANTPLUS_NETWORK_NUMBER,
                            BSC_MSG_PERIOD_4Hz);
BSC_DISP_PROFILE_CONFIG_DEF(m_ant_bsc,
                            ant_bsc_evt_handler);
ant_bsc_profile_t m_ant_bsc;
/** @snippet [ANT BSC RX Instance] */

ant_bpwr_simulator_t m_ant_bpwr_simulator;    /**< Simulator used to simulate profile data. */

void ant_bpwr_bsc_evt_dispatch(ant_evt_t * p_ant_evt)
{
	switch (p_ant_evt->channel)
	{
		case BPWR_CHANNEL_NUMBER: // BPWR
			ant_bpwr_sens_evt_handler(&m_ant_bpwr, p_ant_evt);
			ant_state_indicator_evt_handler(p_ant_evt);
			break;
		case BSC_CHANNEL_NUMBER: // BSC
			ant_bsc_disp_evt_handler(&m_ant_bsc, p_ant_evt);
			ant_state_indicator_evt_handler(p_ant_evt);
			break;
		default:
				break;
	}
}


#ifdef ENABLE_DEBUG_LOG_SUPPORT
__STATIC_INLINE uint32_t calculate_speed(int32_t rev_cnt, int32_t evt_time)
{
    static uint32_t computed_speed   = 0;

		if (s_evt_ctr==BSC_RESET_CTR)
		{
			psarc = accumulated_s_rev_cnt; // Update once every BSC_RESET_CTR cycles
			s_evt_ctr = 0; // Restart at 0
		}
		else
		{
			s_evt_ctr++;
		}
	
    if (rev_cnt != previous_s_evt_cnt)
    {
        accumulated_s_rev_cnt  += rev_cnt - previous_s_evt_cnt;
        accumulated_s_evt_time += evt_time - previous_s_evt_time;

        /* Process rollover */
        if (previous_s_evt_cnt > rev_cnt)
        {
            accumulated_s_rev_cnt += UINT16_MAX + 1;
        }
        if (previous_s_evt_time > evt_time)
        {
            accumulated_s_evt_time += UINT16_MAX + 1;
        }

        previous_s_evt_cnt  = rev_cnt;
        previous_s_evt_time = evt_time;

        computed_speed   = SPEED_COEFFICIENT *
                           (accumulated_s_rev_cnt  - prev_s_accumulated_rev_cnt) /
                           (accumulated_s_evt_time - prev_s_accumulated_evt_time)/
                           BSC_SPEED_UNIT_FACTOR;

        prev_s_accumulated_rev_cnt  = accumulated_s_rev_cnt;
        prev_s_accumulated_evt_time = accumulated_s_evt_time;
    }
		
		if ((psarc == accumulated_s_rev_cnt) && (accumulated_s_rev_cnt != 0))
		// This will be true if the accumulated_c_rev_cnt has not changed for at least 1000 cycles
		{
			computed_speed = 0;
		}
		
		if (computed_speed > 120)
		{
			// No one is going over 120 kph on a bike. If the value is above that, it's probably erroneous.
			computed_speed = 0;
		}

    return (uint32_t) computed_speed;
}

static uint32_t calculate_cadence(int32_t rev_cnt, int32_t evt_time)
{
    static uint32_t computed_cadence = 0;

		if (c_evt_ctr==BSC_RESET_CTR)
		{
			pcarc = accumulated_c_rev_cnt; // Update once every BSC_RESET_CTR cycles
			c_evt_ctr = 0; // Restart at 0
		}
		else
		{
			c_evt_ctr++;
		}

    if (rev_cnt != previous_c_evt_cnt)
    {
        accumulated_c_rev_cnt  += rev_cnt - previous_c_evt_cnt;
        accumulated_c_evt_time += evt_time - previous_c_evt_time;

        /* Process rollover */
        if (previous_c_evt_cnt > rev_cnt)
        {
            accumulated_c_rev_cnt += UINT16_MAX + 1;
        }
        if (previous_c_evt_time > evt_time)
        {
            accumulated_c_evt_time += UINT16_MAX + 1;
        }

        previous_c_evt_cnt  = rev_cnt;
        previous_c_evt_time = evt_time;
				
				

        computed_cadence = CADENCE_COEFFICIENT * // cadence coefficient converts the calculated cadence to units of rpm!
                           (accumulated_c_rev_cnt  - prev_c_accumulated_rev_cnt) /
                           (accumulated_c_evt_time - prev_c_accumulated_evt_time);

				
        prev_c_accumulated_rev_cnt  = accumulated_c_rev_cnt;
        prev_c_accumulated_evt_time = accumulated_c_evt_time;
    }
		
		if ((pcarc == accumulated_c_rev_cnt) && (accumulated_c_rev_cnt != 0))
		// This will be true if the accumulated_c_rev_cnt has not changed for at least 1000 cycles
		{
			computed_cadence = 0;
		}
		
		if (computed_cadence > 120)
		{
			// Not many people are going to be pedaling over 120 rpm, 
			// so if the computed_cadence is above this value it is probably erroneous.
			computed_cadence = 0;
		}

    return (uint32_t) computed_cadence;
}
#endif // ENABLE_DEBUG_LOG_SUPPORT

void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event)
{
    switch (event)
    {
        case ANT_BSC_PAGE_0_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_2_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_3_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_4_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_5_UPDATED:
						/* fall through */
        case ANT_BSC_COMB_PAGE_0_UPDATED:					
						speed = calculate_speed(p_profile->BSC_PROFILE_speed_rev_count,
																							p_profile->BSC_PROFILE_speed_event_time);
						cadence = calculate_cadence(p_profile->BSC_PROFILE_cadence_rev_count,
																							p_profile->BSC_PROFILE_cadence_event_time);

            app_trace_log("%-30s %u kph\n\r",
                          "Computed speed value:", (unsigned int) speed);
            app_trace_log("%-30s %u rpm\n\r",
                          "Computed cadence value:", (unsigned int) cadence);
            app_trace_log("\r\n\r\n");
            
						// Update the BPWR simulator cadence value
						ant_bpwr_simulator_update_cadence(&m_ant_bpwr_simulator, cadence);
						ant_bpwr_simulator_update_speed(&m_ant_bpwr_simulator, speed);
						break;
        default:
            break;
    }
}


/**@brief Function for BSC profile initialization.
 *
 * @details Initializes the BSC profile and open ANT channel.
 */
static void bsc_profile_setup(void)
{
/** @snippet [ANT BSC RX Profile Setup] */
    uint32_t err_code;

    err_code = ant_bsc_disp_init(&m_ant_bsc,
                                 BSC_DISP_CHANNEL_CONFIG(m_ant_bsc),
                                 BSC_DISP_PROFILE_CONFIG(m_ant_bsc));
    APP_ERROR_CHECK(err_code);

    err_code = ant_bsc_disp_open(&m_ant_bsc);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);
/** @snippet [ANT BSC RX Profile Setup] */
}

/**@brief Function for handling bsp events.
 */
/** @snippet [ANT BPWR simulator button] */
void bsp_evt_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            ant_bpwr_simulator_increment(&m_ant_bpwr_simulator);
            break;

        case BSP_EVENT_KEY_1:
            ant_bpwr_simulator_decrement(&m_ant_bpwr_simulator);
            break;

        case BSP_EVENT_KEY_2:
            ant_bpwr_calib_response(&m_ant_bpwr);
            break;

        default:
            break;
    }
}
/** @snippet [ANT BPWR simulator button] */


/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR simulator call] */
void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event)
{
    switch (event)
    {
        case ANT_BPWR_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_16_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_17_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_18_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_80_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_81_UPDATED:
            ant_bpwr_simulator_one_iteration(&m_ant_bpwr_simulator, event);
            break;

        default:
            break;
    }
}
/** @snippet [ANT BPWR simulator call] */


/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR calibration] */
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1)
{
    switch (p_page1->calibration_id)
    {
        case ANT_BPWR_CALIB_ID_MANUAL:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_AUTO:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_auto_zero_status   = p_page1->auto_zero_status;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_REQ:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_REQ_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_UPDATE:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_UPDATE_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        default:
            break;
    }
}
/** @snippet [ANT BPWR calibration] */


/**@brief Function for the BPWR simulator initialization.
 */
void bpwr_simulator_setup(void)
{
    /** @snippet [ANT BPWR simulator init] */
    const ant_bpwr_simulator_cfg_t simulator_cfg =
    {
        .p_profile   = &m_ant_bpwr,
        .sensor_type = (ant_bpwr_torque_t)(SENSOR_TYPE),
    };

    /** @snippet [ANT BPWR simulator init] */

#if MODIFICATION_TYPE == MODIFICATION_TYPE_AUTO
    /** @snippet [ANT BPWR simulator auto init] */
    ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &simulator_cfg, true);
    /** @snippet [ANT BPWR simulator auto init] */
#else
    /** @snippet [ANT BPWR simulator button init] */
    ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &simulator_cfg, false);
    /** @snippet [ANT BPWR simulator button init] */
#endif
}


/**
 * @brief Function for Bicycle Power profile initialization.
 *
 * @details Initializes the Bicycle Power profile and open ANT channel.
 */
static void bpwr_profile_setup(void)
{
/** @snippet [ANT BPWR TX Profile Setup] */
    uint32_t err_code;

    err_code = ant_bpwr_sens_init(&m_ant_bpwr,
                                  BPWR_SENS_CHANNEL_CONFIG(m_ant_bpwr),
                                  BPWR_SENS_PROFILE_CONFIG(m_ant_bpwr));
    APP_ERROR_CHECK(err_code);

    // fill manufacturer's common data page.
    m_ant_bpwr.page_80 = ANT_COMMON_page80(HW_REVISION,
                                           MANUFACTURER_ID,
                                           MODEL_NUMBER);
    // fill product's common data page.
    m_ant_bpwr.page_81 = ANT_COMMON_page81(SW_REVISION_MAJOR,
                                           SW_REVISION_MINOR,
                                           SERIAL_NUMBER);

    m_ant_bpwr.BPWR_PROFILE_auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;

    err_code = ant_bpwr_sens_open(&m_ant_bpwr);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);
/** @snippet [ANT BPWR TX Profile Setup] */
}


/**
 * @brief Function for setup all things not directly associated with ANT stack/protocol.
 * @desc Initialization of: @n
 *         - app_trace for debug.
 *         - app_timer, pre-setup for bsp.
 *         - bsp for signaling LEDs and user buttons.
 */
static void bpwr_bsc_utils_setup(void)
{
    uint32_t err_code;

    app_trace_init();

    // Initialize and start a single continuous mode timer, which is used to update the event time
    // on the main data page.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_evt_handler);
    APP_ERROR_CHECK(err_code);
	
		// These lines are from BSC, but unneeded in this implementation. Reference only.
		// app_trace_init();
    // APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	  // err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    // APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void bpwr_bsc_softdevice_setup(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC; // This is the low frequency clock source used by the softdevice

		// BPWR + BSC softdevice ant evt handler
		err_code = softdevice_ant_evt_handler_set(ant_bpwr_bsc_evt_dispatch);
    APP_ERROR_CHECK(err_code);
	
		// Soft device handler
    err_code = softdevice_handler_init(&clock_lf_cfg, NULL, 0, NULL);
    APP_ERROR_CHECK(err_code);

		// ANT stack config
    err_code = ant_stack_static_config(); // Make sure to set the number of channels to 2
    APP_ERROR_CHECK(err_code);

		// Set network number
    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
    APP_ERROR_CHECK(err_code);
}


void timer_handler(nrf_timer_event_t event_type, void* p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
		static uint32_t last_power = 0;
		static float force_in_newtons = 0;
		static float speed_in_mps = 0;
		ant_bpwr_simulator_t * p_sim = &m_ant_bpwr_simulator;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
     
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_SAADC_BUFFER);
        APP_ERROR_CHECK(err_code);

        m_adc_evt_counter++;
				
			// Do a quick loop to average SAMPLES_IN_SAADC_BUFFER number of samples
			int i;
			SAADC_VALUE = 0; // Reset
			for (i=0; i<SAMPLES_IN_SAADC_BUFFER; i++)
			{
				SAADC_VALUE += p_event->data.done.p_buffer[i];
			}
			SAADC_VALUE /= SAMPLES_IN_SAADC_BUFFER;
			//SAADC_VALUE = p_event->data.done.p_buffer[0];
			
			// Get stored values from the bpwr sim structure
			cadence = (uint32_t)p_sim->p_profile->BPWR_PROFILE_instantaneous_cadence;
			speed = (uint32_t)p_sim->p_profile->BPWR_PROFILE_pedal_power.distribution;
		
			speed_in_mps = speed*BSC_MS_TO_KPH_DEN/BSC_MS_TO_KPH_NUM; // speed in meters per second
			
			//force_in_newtons = SAADC_VALUE;
			//force_in_newtons = (uint32_t)(MIN_FORCE+((SAADC_VALUE - MIN_SAADC)*(MAX_FORCE - MIN_FORCE)/(MAX_SAADC - MIN_SAADC))); // linear interpolation for SAADC_VALUE->Force (in Newtons)
			force_in_newtons = (SAADC_VALUE-37)*200/(630-37);//(uint32_t)((SAADC_VALUE - 37)*200/(630-37));
			
			//int force_sensitivity = 4;
			
			if(cadence<1)
			{
				power_in_watts = 0;
			}
			else
			{
				//power_in_watts = (uint32_t)force_in_newtons*speed_in_mps;
				power_in_watts = (uint32_t)force_in_newtons*speed_in_mps*0.25;
			}
			
			// 2000 watts? that's a strong human!
			if (power_in_watts>2000)
			{
				power_in_watts = 2000;
			}			
			
			int power_incr = 1;
			if (power_in_watts==last_power)
			{
				//Fall through
			}
			else if (power_in_watts<last_power)
			{
				int diff = last_power - power_in_watts;
				diff /= power_incr; //POWER_INCR;
				int j;
				
				for (j=0; j<diff; j++)
				{
					ant_bpwr_simulator_decrement(p_sim);
				}
			}
			else
			{
				int diff = power_in_watts - last_power;
				diff /= power_incr;
				int j;
				
				for (j=0; j<diff; j++)
				{
					ant_bpwr_simulator_increment(p_sim);
				}
			}
			last_power = power_in_watts;
			
			// At the end of the callback, update the bpwr page!
			ant_bpwr_simulator_update_cadence(&m_ant_bpwr_simulator, cadence);
			ant_bpwr_simulator_update_speed(&m_ant_bpwr_simulator, speed);
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0); // Initialize SAADC using GPIO pin 2
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_SAADC_BUFFER);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_SAADC_BUFFER);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry, does not return.
 */
int main(void)
{
		// The main function for this bicycle power meter consists of the following three steps:
		// Step1: Initialize I2C sensors
		// Step2: Initialize ANT+ communication
		// Step3: Loop forever and ever and ever
	
    uint32_t err_code;
	
		// ANT+ BPWR + BSC Initialization
		// This will open a BPWR channel and a BSC channel
		// BPWR will pair to a Garmin automatically if the Garmin is looking for the appropriate device number
		// Spin the wheel/crank to wake up the Garmin GSC 10; pairing will be automatic
		bpwr_bsc_utils_setup();
		bpwr_bsc_softdevice_setup();
		ant_state_indicator_init(m_ant_bpwr.channel_number, BPWR_SENS_CHANNEL_TYPE);
		ant_state_indicator_init(m_ant_bsc.channel_number, BSC_DISP_CHANNEL_TYPE);
		bpwr_simulator_setup();
		bpwr_profile_setup();
		app_trace_log("BPWR Initialized\n\r");
		bsc_profile_setup();
		app_trace_log("BSC Initialized\n\r");

		// Setup SAADC
    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();
		app_trace_log("SAADC Initialized\n\r");

    while(1)
    {
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);			
    }
}


/**
 *@}
 **/
