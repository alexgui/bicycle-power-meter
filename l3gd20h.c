/* l3gd20h.c
 *
 * Cool information here.
 *
 * Really good reference:
 * https://github.com/microbuilder/LPC11U_LPC13U_CodeBase/blob/master/src/drivers/sensors/gyroscopes/l3gd20.c
 *
 * To use:
 * Copy this and the .h file into the same main folder as main.c
 * Add to "Application" group
 * Insert all the right #include
 * Compile!
 */

#include "l3gd20h.h"

uint8_t const l3gd20h_register_out_x_addr = L3GD20H_REGISTER_OUT_X_L;
uint8_t const l3gd20h_register_whoami = L3GD20H_REGISTER_WHO_AM_I;
uint8_t const l3gd20h_register_status = L3GD20H_REGISTER_STATUS_REG;

// Set default mode
static uint8_t const default_config1[] = {L3GD20H_REGISTER_CTRL_REG1,0x0F}; // 95 Hz, 12.5 Cut-Off, Normal, ZYX Enabled
//static uint8_t const default_config4[] = {L3GD20H_REGISTER_CTRL_REG4, 0x1 << 7};

//static uint8_t const default_config1[] = {L3GD20H_REGISTER_CTRL_REG1,0x00}; // OFF
//static uint8_t const default_config1[] = {L3GD20H_REGISTER_CTRL_REG1,0xBF}; // 380 Hz, 100 Cut-Off, Normal, ZYX Enabled

//static uint8_t const default_config4[] = {L3GD20H_REGISTER_CTRL_REG4, 0x80}; // BDU Enabled, Others Default
//static uint8_t const default_config4[] = {L3GD20H_REGISTER_CTRL_REG4, 0x00}; // All default
static uint8_t const default_config4[] = {L3GD20H_REGISTER_CTRL_REG4, 0x1 << 7}; // BDU
app_twi_transfer_t const l3gd20h_init_transfers[L3GD20H_INIT_TRANSFER_COUNT] =
{
		APP_TWI_WRITE(L3GD20H_ADDRESS, default_config1, sizeof(default_config1), 0),
		APP_TWI_WRITE(L3GD20H_ADDRESS, default_config4, sizeof(default_config4), 0)
};
