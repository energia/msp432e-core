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
 *  @file       EMACMSP432E4.h
 *
 *  @brief      EMACMSP432E4 Driver
 *
 *  The EMACMSP432E4 header file should be included in an application as
 *  follows:
 *
 *  @code
 *  #include <ti/drivers/emac/EMACMSP432E4.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef ti_drivers_emac_EMACMSP432E4__include
#define ti_drivers_emac_EMACMSP432E4__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/pin_map.h>

#include <ti/drivers/gpio/GPIOMSP432E4.h>

#include <ti/ndk/inc/stkmain.h>

/*!
 * @brief PF0 is used for EN0LED0
 */
#define EMACMSP432E4_PF0_EN0LED0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 0, GPIO_PF0_EN0LED0)

/*!
 * @brief PK4 is used for EN0LED0
 */
#define EMACMSP432E4_PK4_EN0LED0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 4, GPIO_PK4_EN0LED0)

/*!
 * @brief PF4 is used for EN0LED1
 */
#define EMACMSP432E4_PF4_EN0LED1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 4, GPIO_PF4_EN0LED1)

/*!
 * @brief PK6 is used for EN0LED1
 */
#define EMACMSP432E4_PK6_EN0LED1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 6, GPIO_PK6_EN0LED1)

/*!
 * @brief PF1 is used for EN0LED2
 */
#define EMACMSP432E4_PF1_EN0LED2 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 1, GPIO_PF1_EN0LED2)

/*!
 * @brief PK5 is used for EN0LED2
 */
#define EMACMSP432E4_PK5_EN0LED2 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 5, GPIO_PK5_EN0LED2)

/*!
 *  @brief  EMACMSP432E4 Hardware attributes
 *
 *  intPriority is the EMAC peripheral's interrupt priority, as defined by the
 *  underlying OS.  It is passed unmodified to the underlying OS's interrupt
 *  handler creation code, so you need to refer to the OS documentation
 *  for usage.  For example, for SYS/BIOS applications, refer to the
 *  ti.sysbios.family.arm.m3.Hwi documentation for SYS/BIOS usage of
 *  interrupt priorities.  If the driver uses the ti.dpl interface
 *  instead of making OS calls directly, then the HwiP port handles the
 *  interrupt priority in an OS specific way.  In the case of the SYS/BIOS
 *  port, intPriority is passed unmodified to Hwi_create().
 */
typedef struct EMACMSP432E4_HWAttrs {
    uint32_t baseAddr;       /*!< EMAC port */
    uint32_t intNum;         /*!< Interrupt Vector Id */
    uint32_t intPriority;    /*!< Interrupt priority */
    uint32_t led0Pin;        /*!< LED0 Pin */
    uint32_t led1Pin;        /*!< LED1 Pin */
    uint8_t *macAddress;     /*!< Pointer to MAC address */
} EMACMSP432E4_HWAttrs;

/*!
 *  @brief  This function returns the link state of the EMACMSP432E4 driver
 *
 *  @brief  This function returns true if the link is up
 *
 *  @return true is the link is up. false if it is down.
 */
extern bool EMACMSP432E4_isLinkUp();

/*!
 *  @brief  This function is included in the NIMUDeviceTable that is used
 *          by the NDK to initialize and register the driver.  This function
 *          should not be called by the user. The NIMUDeviceTable is typically
 *          provided in the "board.c" file.
 *
 *  @param  hEvent NDK event semaphore
 */
extern int EMACMSP432E4_NIMUInit(STKEVENT_Handle hEvent);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_emac_EMACMSP432E4__include */
