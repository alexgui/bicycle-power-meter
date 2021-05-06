/* l3gd20h.h
 *
 * L3GD20H 3 AXIS GYROSCOPE
 *
 * Modified from the following, for use with the NRF52 DK
 * https://github.com/microbuilder/LPC11U_LPC13U_CodeBase/blob/master/src/drivers/sensors/gyroscopes/l3gd20.h *
 */

#ifndef L3GD20H_H__
#define L3GD20H_H__

#include "app_twi.h"

#define L3GD20H_ADDRESS                0x6B //(0x6B<<1)     // 1101001
#define L3GD20H_READBIT                (0x01)
#define L3GD20H_POLL_TIMEOUT           (100)         // Maximum number of read attempts in l3gd20Poll()
#define L3GD20H_ID                     0xD7 //BIN8(11010100)

#define L3GD20H_NUMBER_OF_REGISTERS 6 // size for 3 signed 16 bit values for x/y/z data

#define L3GD20H_SENSITIVITY_250DPS  (0.00875F)   // Roughly 22/256 for fixed point match
#define L3GD20H_SENSITIVITY_500DPS  (0.0175F)    // Roughly 45/256
#define L3GD20H_SENSITIVITY_2000DPS (0.070F)     // Roughly 18/256
#define L3GD20H_DPS_TO_RADS					(0.017453293F) // Degrees/s to rad/s multiplier

/* Struct to hold the gyroscope sensor data */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} l3gd20hData_t;

// Core registers
enum
{                                               // DEFAULT    TYPE
  L3GD20H_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
  L3GD20H_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
  L3GD20H_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
  L3GD20H_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
  L3GD20H_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
  L3GD20H_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
  L3GD20H_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
  L3GD20H_REGISTER_OUT_TEMP            = 0x26,   //            r
  L3GD20H_REGISTER_STATUS_REG          = 0x27,   //            r
  L3GD20H_REGISTER_OUT_X_L             = 0x28,   //            r
  L3GD20H_REGISTER_OUT_X_H             = 0x29,   //            r
  L3GD20H_REGISTER_OUT_Y_L             = 0x2A,   //            r
  L3GD20H_REGISTER_OUT_Y_H             = 0x2B,   //            r
  L3GD20H_REGISTER_OUT_Z_L             = 0x2C,   //            r
  L3GD20H_REGISTER_OUT_Z_H             = 0x2D,   //            r
  L3GD20H_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
  L3GD20H_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
  L3GD20H_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
  L3GD20H_REGISTER_INT1_SRC            = 0x31,   //            r
  L3GD20H_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
  L3GD20H_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
  L3GD20H_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
  L3GD20H_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
  L3GD20H_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
  L3GD20H_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
  L3GD20H_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
};

#define L3GD20H_DATA_IS_VALID(reg_data)  1 //(!((reg_data) & (1U << 6)))

extern uint8_t const l3gd20h_register_out_x_addr;
extern uint8_t const l3gd20h_register_whoami;
extern uint8_t const l3gd20h_register_status;

#define L3GD20H_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(L3GD20H_ADDRESS, p_reg_addr, 1, 			  APP_TWI_NO_STOP), \
    APP_TWI_READ (L3GD20H_ADDRESS, p_buffer,   byte_cnt, 0)

#define L3GD20H_READ_XYZ(p_buffer) \
    L3GD20H_READ(&l3gd20h_register_out_x_addr, p_buffer, 6)

#define L3GD20H_WHOAMI(p_buffer) \
    L3GD20H_READ(&l3gd20h_register_whoami, p_buffer, 1)

#define L3GD20H_STATUS(p_buffer) \
    L3GD20H_READ(&l3gd20h_register_status, p_buffer, 1)

#define L3GD20H_INIT_TRANSFER_COUNT 2
extern app_twi_transfer_t const l3gd20h_init_transfers[L3GD20H_INIT_TRANSFER_COUNT];

#endif // L3GD20H_H__
