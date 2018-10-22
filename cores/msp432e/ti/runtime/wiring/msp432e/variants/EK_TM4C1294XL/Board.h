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

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "EK_TM4C1294XL.h"

#define Board_initGeneral           EK_TM4C1294XL_initGeneral

#define Board_GPIO_LED_ON           EK_TM4C1294XL_GPIO_LED_ON
#define Board_GPIO_LED_OFF          EK_TM4C1294XL_GPIO_LED_OFF
#define Board_GPIO_LED0             EK_TM4C1294XL_GPIO_D1
#define Board_GPIO_LED1             EK_TM4C1294XL_GPIO_D2
#define Board_GPIO_LED2             EK_TM4C1294XL_GPIO_D2
#define Board_GPIO_BUTTON0          EK_TM4C1294XL_GPIO_USR_SW1
#define Board_GPIO_BUTTON1          EK_TM4C1294XL_GPIO_USR_SW2

#define Board_I2C0                  EK_TM4C1294XL_I2C0
#define Board_I2C_TMP               EK_TM4C1294XL_I2C0
#define Board_I2C_TPL0401           EK_TM4C1294XL_I2C0

#define Board_NVS0                  EK_TM4C1294XL_NVSMSP432E40

#define Board_PWM0                  EK_TM4C1294XL_PWM0
#define Board_PWM1                  EK_TM4C1294XL_PWM0

#define Board_SPI0                  EK_TM4C1294XL_SPI2
#define Board_SPI1                  EK_TM4C1294XL_SPI3

#define Board_UART0                 EK_TM4C1294XL_UART0

#define Board_WATCHDOG0             EK_TM4C1294XL_WATCHDOG0

/* Board specific I2C addresses */
#define Board_TMP_ADDR              (0x40)
#define Board_SENSORS_BP_TMP_ADDR   Board_TMP_ADDR
#define Board_TPL0401_ADDR          (0x40)

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
