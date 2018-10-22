/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       EK_TM4C1294XL.h
 *
 *  @brief      EK_TM4C1294XL Board Specific APIs
 *
 *  The EK_TM4C1294XL header file should be included in an application as
 *  follows:
 *  @code
 *  #include <EK_TM4C1294XL.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __EK_TM4C1294XL_H
#define __EK_TM4C1294XL_H

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on EK_TM4C1294XL are active high. */
#define EK_TM4C1294XL_GPIO_LED_OFF (0)
#define EK_TM4C1294XL_GPIO_LED_ON  (1)

/*!
 *  @def    EK_TM4C1294XL_GPIOName
 *  @brief  Enum of LED names on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_GPIOName {
    EK_TM4C1294XL_GPIO_USR_SW1 = 0,
    EK_TM4C1294XL_GPIO_USR_SW2,
    EK_TM4C1294XL_SPI_MASTER_READY,
    EK_TM4C1294XL_SPI_SLAVE_READY,
    EK_TM4C1294XL_GPIO_D1,
    EK_TM4C1294XL_GPIO_D2,

    EK_TM4C1294XL_SDSPI_CS,

    /* Sharp 96x96 LCD Pins */
    EK_TM4C1294XL_LCD_CS,
    EK_TM4C1294XL_LCD_POWER,
    EK_TM4C1294XL_LCD_ENABLE,

    EK_TM4C1294XL_GPIOCOUNT
} EK_TM4C1294XL_GPIOName;

/*!
 *  @def    EK_TM4C1294XL_I2CName
 *  @brief  Enum of I2C names on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_I2CName {
    EK_TM4C1294XL_I2C0 = 0,
    EK_TM4C1294XL_I2C7,

    EK_TM4C1294XL_I2CCOUNT
} EK_TM4C1294XL_I2CName;

/*!
 *  @def    EK_TM4C1294XL_NVSName
 *  @brief  Enum of NVS names on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_NVSName {
    EK_TM4C1294XL_NVSMSP432E40 = 0,

    EK_TM4C1294XL_NVSCOUNT
} EK_TM4C1294XL_NVSName;

/*!
 *  @def    EK_TM4C1294XL_PWMName
 *  @brief  Enum of PWM names on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_PWMName {
    EK_TM4C1294XL_PWM0 = 0,
    EK_TM4C1294XL_PWM1,
    EK_TM4C1294XL_PWM2,
    EK_TM4C1294XL_PWM3,
    EK_TM4C1294XL_PWM4,
    EK_TM4C1294XL_PWM5,
    EK_TM4C1294XL_PWM6,
    EK_TM4C1294XL_PWM7,

    EK_TM4C1294XL_PWMCOUNT
} EK_TM4C1294XL_PWMName;

/*!
 *  @def    EK_TM4C1294XL_SDFatFSName
 *  @brief  Enum of SDFatFS names on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_SDFatFSName {
    EK_TM4C1294XL_SDFatFS0 = 0,

    EK_TM4C1294XL_SDFatFSCOUNT
} EK_TM4C1294XL_SDFatFSName;

/*!
 *  @def    EK_TM4C1294XL_SDName
 *  @brief  Enum of SD names on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_SDName {
    EK_TM4C1294XL_SDSPI0 = 0,

    EK_TM4C1294XL_SDCOUNT
} EK_TM4C1294XL_SDName;

/*!
 *  @def    EK_TM4C1294XL_SPIName
 *  @brief  Enum of SPI names on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_SPIName {
    EK_TM4C1294XL_SPI2 = 0,
    EK_TM4C1294XL_SPI3,

    EK_TM4C1294XL_SPICOUNT
} EK_TM4C1294XL_SPIName;

/*!
 *  @def    EK_TM4C1294XL_UARTName
 *  @brief  Enum of UARTs on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_UARTName {
    EK_TM4C1294XL_UART0 = 0,

    EK_TM4C1294XL_UARTCOUNT
} EK_TM4C1294XL_UARTName;

/*
 *  @def    EK_TM4C1294XL_WatchdogName
 *  @brief  Enum of Watchdogs on the EK_TM4C1294XL dev board
 */
typedef enum EK_TM4C1294XL_WatchdogName {
    EK_TM4C1294XL_WATCHDOG0 = 0,

    EK_TM4C1294XL_WATCHDOGCOUNT
} EK_TM4C1294XL_WatchdogName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 *  This includes:
 *     - Enable clock sources for peripherals
 */
extern void EK_TM4C1294XL_initGeneral(void);

#ifdef __cplusplus
}
#endif

#endif /* __EK_TM4C1294XL_H */
