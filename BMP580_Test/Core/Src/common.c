/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "main.h"
#include "bmp5_defs.h"

/******************************************************************************/
/*!                         Macro definitions                                 */
#define i2c hi2c1
/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

//    return coines_read_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)length);
    return HAL_I2C_Mem_Read(&i2c, (device_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, (uint16_t)length, 100);

}

/*!
 * I2C write function map
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

//    return coines_write_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
    return HAL_I2C_Mem_Write(&i2c, (device_addr << 1) , reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, (uint16_t)length, 100);
}

/*!
 * SPI read function map (not implemented and tested...)
 */
//BMP5_INTF_RET_TYPE bmp5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//		reg_addr -> CS pin?
//		reg_data -> pointer to buffer to store data
//		length -> how many bytes to send
//		intf_pointer???
//    uint8_t device_addr = *(uint8_t*)intf_ptr;
//
//    (void)intf_ptr;
//
//		HAL_SPI_Receive(SPI_HandleTypeDef * hspi, uint8_t * pData, uint16_t Size,uint32_t Timeout);
//		HAL_GPIO_WritePin(device_addr);
//		HAL_SPI_Receive(&hspi1, reg_data, (uint16_t)length, 100);

////    return coines_read_spi(COINES_SPI_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)length);
//}

/*!
 * SPI write function map
 */
//BMP5_INTF_RET_TYPE bmp5_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    uint8_t device_addr = *(uint8_t*)intf_ptr;
//
//    (void)intf_ptr;
//
//		HAL_SPI_Transmit (SPI_HandleTypeDef * hspi, uint8_t * pData, uint16_t Size, uint32_t Timeout)
//		HAL_GPIO_WritePin(device_addr);
//		HAL_SPI_Transmit (&hspi1, reg_data, (uint16_t)length, 100);
//
////    return coines_write_spi(COINES_SPI_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)length);

//}

/*!
 * Delay function map
 */
void bmp5_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
//    coines_delay_usec(period);
    HAL_Delay(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
//void bmp5_error_codes_print_result(const char api_name[], int8_t rslt)
//{
//    if (rslt != BMP5_OK)
//    {
//        printf("%s\t", api_name);
//        if (rslt == BMP5_E_NULL_PTR)
//        {
//            printf("Error [%d] : Null pointer\r\n", rslt);
//        }
//        else if (rslt == BMP5_E_COM_FAIL)
//        {
//            printf("Error [%d] : Communication failure\r\n", rslt);
//        }
//        else if (rslt == BMP5_E_DEV_NOT_FOUND)
//        {
//            printf("Error [%d] : Device not found\r\n", rslt);
//        }
//        else if (rslt == BMP5_E_INVALID_CHIP_ID)
//        {
//            printf("Error [%d] : Invalid chip id\r\n", rslt);
//        }
//        else if (rslt == BMP5_E_POWER_UP)
//        {
//            printf("Error [%d] : Power up error\r\n", rslt);
//        }
//        else if (rslt == BMP5_E_POR_SOFTRESET)
//        {
//            printf("Error [%d] : Power-on reset/softreset failure\r\n", rslt);
//        }
//        else if (rslt == BMP5_E_INVALID_POWERMODE)
//        {
//            printf("Error [%d] : Invalid powermode\r\n", rslt);
//        }
//        else
//        {
//            /* For more error codes refer "*_defs.h" */
//            printf("Error [%d] : Unknown error code\r\n", rslt);
//        }
//    }
//}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmp5_interface_init(struct bmp5_dev *bmp5_dev, uint8_t intf)
{
    int8_t rslt = BMP5_OK;

        /* Bus configuration : I2C */
        if (intf == BMP5_I2C_INTF)
        {
            printf("I2C Interface\n");

            dev_addr = BMP5_I2C_ADDR_SEC;		//PRIM (SDO set to 0), SEC (SDO set to 1);
            bmp5_dev->read = bmp5_i2c_read;
            bmp5_dev->write = bmp5_i2c_write;
            bmp5_dev->intf = BMP5_I2C_INTF;

            /* SDO pin is made low */
        }
        /* Bus configuration : SPI */
//        else if (intf == BMP5_SPI_INTF)
//        {
//            printf("SPI Interface\n");
//
//            dev_addr = COINES_SHUTTLE_PIN_7;
//            bmp5_dev->read = bmp5_spi_read;
//            bmp5_dev->write = bmp5_spi_write;
//            bmp5_dev->intf = BMP5_SPI_INTF;
//
//        }
        else
    	{
            rslt = BMP5_E_NULL_PTR;
        }
        /* Holds the I2C device addr or SPI chip selection */
        bmp5_dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bmp5_dev->delay_us = bmp5_delay_us;

    return rslt;
}

