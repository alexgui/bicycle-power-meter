/* lis3dh.c
 *
 * Cool information here.
 *
 * Really good reference:
 * https://github.com/microbuilder/LPC11U_LPC13U_CodeBase/blob/master/src/drivers/sensors/accelerometers/lis3dh.c
 *
 * To use:
 * Copy this and the .h file into the same main folder as main.c
 * Add to "Application" group
 * Insert all the right #include
 * Compile!
 */

#include "lis3dh.h"

//#define LIS3DH_SENSITIVITY_2G  (0.001F)
//#define LIS3DH_SENSITIVITY_4G  (0.002F)
//#define LIS3DH_SENSITIVITY_8G  (0.004F)
//#define LIS3DH_SENSITIVITY_16G (0.012F)

uint8_t const lis3dh_register_out_x_addr = (LIS3DH_REGISTER_OUT_X_L | 0x80); // 0x80 for auto increment
uint8_t const lis3dh_register_whoami = LIS3DH_REGISTER_WHO_AM_I;
uint8_t const lis3dh_register_status = LIS3DH_REGISTER_STATUS_REG_AUX;//LIS3DH_STATUS_REG_ZYXDA;

// Set default mode
//static uint8_t const default_config1[] = {LIS3DH_REGISTER_CTRL_REG1,LIS3DH_CTRL_REG1_XYZEN | LIS3DH_CTRL_REG1_DATARATE_100HZ};
static uint8_t const default_config1[] = {LIS3DH_REGISTER_CTRL_REG1,0x07}; // Enable all axes, normal mode

//static uint8_t const default_config4[] = {LIS3DH_CTRL_REG4_BLOCKDATAUPDATE, LIS3DH_CTRL_REG4_BLOCKDATAUPDATE | LIS3DH_CTRL_REG4_SCALE_2G};
static uint8_t const default_config4[] = {LIS3DH_REGISTER_CTRL_REG4, 0x88}; // High res and BDU enabled

//static uint8_t const default_config3[] = {LIS3DH_REGISTER_CTRL_REG3, 0x10}; // DRDY on INT1
//static uint8_t const default_config2[] = {LIS3DH_REGISTER_CTRL_REG2, 0}; // Unused
//static uint8_t const default_configTempCfg[] = {LIS3DH_REGISTER_TEMP_CFG_REG, 0x80}; // Enable ADCs

//static uint8_t const default_config[] = { LIS3DH_REGISTER_TEMP_CFG_REG, 0 };
app_twi_transfer_t const lis3dh_init_transfers[LIS3DH_INIT_TRANSFER_COUNT] =
{
   // APP_TWI_WRITE(LIS3DH_ADDRESS, default_config, sizeof(default_config), 0),
		APP_TWI_WRITE(LIS3DH_ADDRESS, default_config1, sizeof(default_config1), 0), // Enable XYZ axes and set datarate to 50 hz
		APP_TWI_WRITE(LIS3DH_ADDRESS, default_config4, sizeof(default_config4), 0), // BDU and 2g scale
//		APP_TWI_WRITE(LIS3DH_ADDRESS, default_config3, sizeof(default_config3), 0),
//		APP_TWI_WRITE(LIS3DH_ADDRESS, default_configTempCfg, sizeof(default_configTempCfg), 0)
};
