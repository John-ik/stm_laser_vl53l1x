/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

#include "main.h"
#include "i2c.h"


#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

#ifdef VL53L1_LOG_ENABLE
#define trace_print(level, ...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_PLATFORM, level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif

#ifndef HAL_I2C_MODULE_ENABLED
#warning "HAL I2C module must be enable "
#endif

#define VL53L0X_pI2cHandle    hi2c1

/* when not customized by application define dummy one */
#ifndef VL53L1_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L1_GetI2cBus(...) (void)0
#endif

#ifndef VL53L1_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L1_PutI2cBus(...) (void)0
#endif

uint8_t _I2CBuffer[256];

int _I2CWrite(uint16_t Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Transmit(&VL53L0X_pI2cHandle, Dev, pdata, count, i2c_time_out);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

int _I2CRead(uint16_t Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Receive(&VL53L0X_pI2cHandle, Dev|1, pdata, count, i2c_time_out);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    return 0; // to be implemented
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
    return 0; // to be implemented
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    return 0; // to be implemented
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    return 0; // to be implemented
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    return 0; // to be implemented
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    return 0; // to be implemented
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    return 0; // to be implemented
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    return 0; // to be implemented
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
    (void)dev;
    HAL_Delay(wait_ms);
    return 0;
}
