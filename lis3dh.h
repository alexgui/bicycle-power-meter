/* lis3dh.h
 *
 * LIS3DH 3 AXIS ACCELEROMETER
 *
 * Modified from the following, for use with the NRF52 DK
 * https://github.com/microbuilder/LPC11U_LPC13U_CodeBase/blob/master/src/drivers/sensors/accelerometers/lis3dh.h
 *
 */

#ifndef LIS3DH_H__
#define LIS3DH_H__

#include "app_twi.h"

#define LIS3DH_ADDRESS                (0x18) //(0x18<<1)   // 0011000 (Assumes SDO/SA0 = GND)
#define LIS3DH_READBIT                (0x01)
#define LIS3DH_POLL_TIMEOUT           (100)       // Maximum number of read attempts in lis3dhPoll()

#define LIS3DH_NUMBER_OF_REGISTERS 6 // size for 3 signed 16 bit values for x/y/z data

/* Struct to hold the accelerometer sensor data */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} lis3dhData_t;

// Core registers
enum
{                                               // BINARY   DEFAULT   TYPE
  LIS3DH_REGISTER_STATUS_REG_AUX      = 0x07,   // 00000111           r
  LIS3DH_REGISTER_OUT_ADC1_L          = 0x08,   // 00001000 output    r
  LIS3DH_REGISTER_OUT_ADC1_H          = 0x09,   // 00001001 output    r
  LIS3DH_REGISTER_OUT_ADC2_L          = 0x0A,   // 00001010 output    r
  LIS3DH_REGISTER_OUT_ADC2_H          = 0x0B,   // 00001011 output    r
  LIS3DH_REGISTER_OUT_ADC3_L          = 0x0C,   // 00001100 output    r
  LIS3DH_REGISTER_OUT_ADC3_H          = 0x0D,   // 00001101 output    r
  LIS3DH_REGISTER_INT_COUNTER_REG     = 0x0E,   // 00001110           r
  LIS3DH_REGISTER_WHO_AM_I            = 0x0F,   // 00001111 00110011  r
  LIS3DH_REGISTER_TEMP_CFG_REG        = 0x1F,   // 00011111           rw
  LIS3DH_REGISTER_CTRL_REG1           = 0x20,   // 00100000 00000111  rw
  LIS3DH_REGISTER_CTRL_REG2           = 0x21,   // 00100001 00000000  rw
  LIS3DH_REGISTER_CTRL_REG3           = 0x22,   // 00100010 00000000  rw
  LIS3DH_REGISTER_CTRL_REG4           = 0x23,   // 00100011 00000000  rw
  LIS3DH_REGISTER_CTRL_REG5           = 0x24,   // 00100100 00000000  rw
  LIS3DH_REGISTER_CTRL_REG6           = 0x25,   // 00100101 00000000  rw
  LIS3DH_REGISTER_REFERENCE           = 0x26,   // 00100110 00000000  rw
  LIS3DH_REGISTER_STATUS_REG2         = 0x27,   // 00100111 00000000  r
  LIS3DH_REGISTER_OUT_X_L             = 0x28,   // 00101000 output    r
  LIS3DH_REGISTER_OUT_X_H             = 0x29,   // 00101001 output    r
  LIS3DH_REGISTER_OUT_Y_L             = 0x2A,   // 00101010 output    r
  LIS3DH_REGISTER_OUT_Y_H             = 0x2B,   // 00101011 output    r
  LIS3DH_REGISTER_OUT_Z_L             = 0x2C,   // 00101100 output    r
  LIS3DH_REGISTER_OUT_Z_H             = 0x2D,   // 00101101 output    r
  LIS3DH_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00101110 00000000  rw
  LIS3DH_REGISTER_FIFO_SRC_REG        = 0x2F,   // 00101111           r
  LIS3DH_REGISTER_INT1_CFG            = 0x30,   // 00110000 00000000  rw
  LIS3DH_REGISTER_INT1_SOURCE         = 0x31,   // 00110001 00000000  r
  LIS3DH_REGISTER_INT1_THS            = 0x32,   // 00110010 00000000  rw
  LIS3DH_REGISTER_INT1_DURATION       = 0x33,   // 00110011 00000000  rw
  LIS3DH_REGISTER_CLICK_CFG           = 0x38,   // 00111000 00000000  rw
  LIS3DH_REGISTER_CLICK_SRC           = 0x39,   // 00111001 00000000  r
  LIS3DH_REGISTER_CLICK_THS           = 0x3A,   // 00111010 00000000  rw
  LIS3DH_REGISTER_TIME_LIMIT          = 0x3B,   // 00111011 00000000  rw
  LIS3DH_REGISTER_TIME_LATENCY        = 0x3C,   // 00111100 00000000  rw
  LIS3DH_REGISTER_TIME_WINDOW         = 0x3D    // 00111101 00000000  rw
};

// Bit twiddling keys for use with different registers
enum
{
  LIS3DH_STATUS_REG_ZYXDA             = 0x08,   // STATUS_REG: XYZ - sample ready
  LIS3DH_STATUS_REG_ZYXOR             = 0x80,   // STATUS_REG: XYZ - new set of data has overwritten the previous ones
  LIS3DH_CTRL_REG1_XEN                = 0x01,   // CTRL_REG1: X enable
  LIS3DH_CTRL_REG1_YEN                = 0x02,   // CTRL_REG1: Y enable
  LIS3DH_CTRL_REG1_ZEN                = 0x04,   // CTRL_REG1: Z enable
  LIS3DH_CTRL_REG1_XYZEN              = 0x07,   // CTRL_REG1: X+Y+Z enable
  LIS3DH_CTRL_REG1_LPEN               = 0x08,   // CTRL_REG1: Low power enable
  LIS3DH_CTRL_REG1_DATARATE_POWERDOWN = 0x00,   // CTRL_REG1: 0000 xxxx
  LIS3DH_CTRL_REG1_DATARATE_1HZ       = 0x10,   // CTRL_REG1: 0001 xxxx
  LIS3DH_CTRL_REG1_DATARATE_10HZ      = 0x20,   // CTRL_REG1: 0010 xxxx
  LIS3DH_CTRL_REG1_DATARATE_25HZ      = 0x30,   // CTRL_REG1: 0011 xxxx
  LIS3DH_CTRL_REG1_DATARATE_50HZ      = 0x40,   // CTRL_REG1: 0100 xxxx
  LIS3DH_CTRL_REG1_DATARATE_100HZ     = 0x50,   // CTRL_REG1: 0101 xxxx
  LIS3DH_CTRL_REG1_DATARATE_200HZ     = 0x60,   // CTRL_REG1: 0110 xxxx
  LIS3DH_CTRL_REG1_DATARATE_400HZ     = 0x70,   // CTRL_REG1: 0111 xxxx
  LIS3DH_CTRL_REG1_DATARATE_1500HZ    = 0x80,   // CTRL_REG1: 1000 xxxx
  LIS3DH_CTRL_REG1_DATARATE_5000HZ    = 0x90,   // CTRL_REG1: 1001 xxxx
  LIS3DH_CTRL_REG4_BLOCKDATAUPDATE    = 0x80,   // CTRL_REG4: 1xxx xxxx
  LIS3DH_CTRL_REG4_SCALE_2G           = 0x00,   // CTRL_REG4: xx00 xxxx
  LIS3DH_CTRL_REG4_SCALE_4G           = 0x10,   // CTRL_REG4: xx01 xxxx
  LIS3DH_CTRL_REG4_SCALE_8G           = 0x20,   // CTRL_REG4: xx10 xxxx
  LIS3DH_CTRL_REG4_SCALE_16G          = 0x30,   // CTRL_REG4: xx11 xxxx
};

#define LIS3DH_DATA_IS_VALID(reg_data)  1 //(!((reg_data) & (1U << 6)))


//// [use "/ 32" instead of ">> 5", as the result of right-shifting of a signed
////  type value is implementation-defined]
//#define LM75B_GET_TEMPERATURE_VALUE(temp_hi, temp_lo) \
//    ((((int16_t)temp_hi << 8) | temp_lo) / 32)

extern uint8_t const lis3dh_register_out_x_addr;
extern uint8_t const lis3dh_register_whoami;
extern uint8_t const lis3dh_register_status;


#define LIS3DH_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(LIS3DH_ADDRESS, p_reg_addr, 1, 			  APP_TWI_NO_STOP), \
    APP_TWI_READ (LIS3DH_ADDRESS, p_buffer,   byte_cnt, 0)

#define LIS3DH_READ_XYZ(p_buffer) \
    LIS3DH_READ(&lis3dh_register_out_x_addr, p_buffer, 6)

#define LIS3DH_WHOAMI(p_buffer) \
    LIS3DH_READ(&lis3dh_register_whoami, p_buffer, 1)
		
#define LIS3DH_STATUS(p_buffer) \
    LIS3DH_READ(&lis3dh_register_status, p_buffer, 1)

#define LIS3DH_INIT_TRANSFER_COUNT 2
extern app_twi_transfer_t const lis3dh_init_transfers[LIS3DH_INIT_TRANSFER_COUNT];

#endif // LIS3DH_H__
