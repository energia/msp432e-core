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

/*
 *  ======== Board_init.c ========
 *  This file is responsible for setting up the board specific items for the
 *  MSP_EXP432E401Y board.
 */
#include <stdint.h>
#include <stdlib.h>

#ifndef __MSP432E401Y__
#define __MSP432E401Y__
#endif

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/adc.h>
#include <ti/devices/msp432e4/driverlib/interrupt.h>
#include <ti/devices/msp432e4/driverlib/pwm.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/devices/msp432e4/driverlib/udma.h>

#include <ti/devices/msp432e4/driverlib/rom.h>
#include <ti/devices/msp432e4/driverlib/rom_map.h>
#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>

#include <ti/drivers/Power.h>

#include "MSP_EXP432E401Y.h"

/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCMSP432E4.h>

extern ADC_Handle ADCMSP432E4_open(ADC_Handle handle, ADC_Params *params);
extern void ADCMSP432E4_close(ADC_Handle handle);
extern void ADCMSP432E4_init(ADC_Handle handle);
extern int_fast16_t ADCMSP432E4_convert(ADC_Handle handle, uint16_t *value);

/* ADC function table for ADCMSP432 implementation */
const ADC_FxnTable myADCMSP432E4_fxnTable = {
    ADCMSP432E4_close,
    NULL, /* ADCMSP432_control, */
    ADCMSP432E4_convert,
    NULL, /* ADCMSP432_convertRawToMicroVolts, */
    ADCMSP432E4_init,
    ADCMSP432E4_open
};

/* ADC objects */
ADCMSP432E4_Object adcMSP432E4Objects[20];

/* ADC configuration structure */
ADCMSP432E4_HWAttrsV1 adcMSP432E4HWAttrs[20] = {
    {
        .adcPin = ADCMSP432E4_PE_3_A0,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PE_2_A1,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PE_1_A2,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PE_0_A3,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PD_7_A4,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PD_6_A5,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PD_5_A6,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PD_4_A7,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PE_5_A8,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PE_4_A9,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PB_4_A10,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PB_5_A11,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PD_3_A12,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PD_2_A13,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PD_1_A14,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PD_0_A15,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PK_0_A16,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PK_1_A17,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PK_2_A18,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    },
    {
        .adcPin = ADCMSP432E4_PK_3_A19,
        .refVoltage = ADCMSP432E4_VREF_INTERNAL,
        .adcModule = ADCMSP432E4_MOD0,
        .adcSeq = ADCMSP432E4_SEQ0
    }
};

const ADC_Config ADC_config[20] = {
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[0],
        .hwAttrs = &adcMSP432E4HWAttrs[0]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[1],
        .hwAttrs = &adcMSP432E4HWAttrs[1]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[2],
        .hwAttrs = &adcMSP432E4HWAttrs[2]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[3],
        .hwAttrs = &adcMSP432E4HWAttrs[3]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[4],
        .hwAttrs = &adcMSP432E4HWAttrs[4]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[5],
        .hwAttrs = &adcMSP432E4HWAttrs[5]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[6],
        .hwAttrs = &adcMSP432E4HWAttrs[6]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[7],
        .hwAttrs = &adcMSP432E4HWAttrs[7]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[8],
        .hwAttrs = &adcMSP432E4HWAttrs[8]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[9],
        .hwAttrs = &adcMSP432E4HWAttrs[9]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[10],
        .hwAttrs = &adcMSP432E4HWAttrs[10]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[11],
        .hwAttrs = &adcMSP432E4HWAttrs[11]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[12],
        .hwAttrs = &adcMSP432E4HWAttrs[12]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[13],
        .hwAttrs = &adcMSP432E4HWAttrs[13]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[14],
        .hwAttrs = &adcMSP432E4HWAttrs[14]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[15],
        .hwAttrs = &adcMSP432E4HWAttrs[15]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[16],
        .hwAttrs = &adcMSP432E4HWAttrs[16]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[17],
        .hwAttrs = &adcMSP432E4HWAttrs[17]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[18],
        .hwAttrs = &adcMSP432E4HWAttrs[18]
    },
    {
        .fxnTablePtr = &myADCMSP432E4_fxnTable,
        .object = &adcMSP432E4Objects[19],
        .hwAttrs = &adcMSP432E4HWAttrs[19]
    }
};

const uint_least8_t ADC_count = 20;

/*
 *  ============================= Display =============================
 */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/display/DisplaySharp.h>
#define MAXPRINTLEN 1024

#ifndef BOARD_DISPLAY_SHARP_SIZE
#define BOARD_DISPLAY_SHARP_SIZE    96
#endif

DisplayUart_Object displayUartObject;
DisplaySharp_Object    displaySharpObject;

static char displayBuf[MAXPRINTLEN];
static uint_least8_t sharpDisplayBuf[BOARD_DISPLAY_SHARP_SIZE * BOARD_DISPLAY_SHARP_SIZE / 8];


const DisplayUart_HWAttrs displayUartHWAttrs = {
    .uartIdx = MSP_EXP432E401Y_UART0,
    .baudRate = 115200,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf = displayBuf,
    .strBufLen = MAXPRINTLEN
};

const DisplaySharp_HWAttrsV1 displaySharpHWattrs = {
    .spiIndex    = MSP_EXP432E401Y_SPI2,
    .csPin       = MSP_EXP432E401Y_LCD_CS,
    .powerPin    = MSP_EXP432E401Y_LCD_POWER,
    .enablePin   = MSP_EXP432E401Y_LCD_ENABLE,
    .pixelWidth  = BOARD_DISPLAY_SHARP_SIZE,
    .pixelHeight = BOARD_DISPLAY_SHARP_SIZE,
    .displayBuf  = sharpDisplayBuf,
};

#ifndef BOARD_DISPLAY_USE_UART
#define BOARD_DISPLAY_USE_UART 1
#endif
#ifndef BOARD_DISPLAY_USE_UART_ANSI
#define BOARD_DISPLAY_USE_UART_ANSI 0
#endif
#ifndef BOARD_DISPLAY_USE_LCD
#define BOARD_DISPLAY_USE_LCD 0
#endif

/*
 * This #if/#else is needed to workaround a problem with the
 * IAR compiler. The IAR compiler doesn't like the empty array
 * initialization. (IAR Error[Pe1345])
 */
 #if (BOARD_DISPLAY_USE_UART || BOARD_DISPLAY_USE_LCD)
const Display_Config Display_config[] = {
    {
#  if (BOARD_DISPLAY_USE_UART_ANSI)
        .fxnTablePtr = &DisplayUartAnsi_fxnTable,
#  else /* Default to minimal UART with no cursor placement */
        .fxnTablePtr = &DisplayUartMin_fxnTable,
#  endif
        .object = &displayUartObject,
        .hwAttrs = &displayUartHWAttrs
    },
#endif
#if (BOARD_DISPLAY_USE_LCD)
    {
        .fxnTablePtr = &DisplaySharp_fxnTable,
        .object      = &displaySharpObject,
        .hwAttrs     = &displaySharpHWattrs
    },
#endif
};

const uint_least8_t Display_count = sizeof(Display_config) / sizeof(Display_Config);

/*
 *  =============================== DMA ===============================
 */
#include <ti/drivers/dma/UDMAMSP432E4.h>

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[64];

/*
 *  ======== dmaErrorFxn ========
 *  This is the handler for the uDMA error interrupt.
 */
static void dmaErrorFxn(uintptr_t arg)
{
    int status = uDMAErrorStatusGet();
    uDMAErrorStatusClear();

    /* Suppress unused variable warning */
    (void)status;

    while (1);
}

UDMAMSP432E4_Object udmaMSP432E4Object;

const UDMAMSP432E4_HWAttrs udmaMSP432E4HWAttrs = {
    .controlBaseAddr = (void *)dmaControlTable,
    .dmaErrorFxn = (UDMAMSP432E4_ErrorFxn)dmaErrorFxn,
    .intNum = INT_UDMAERR,
    .intPriority = (~0)
};

const UDMAMSP432E4_Config UDMAMSP432E4_config = {
    .object = &udmaMSP432E4Object,
    .hwAttrs = &udmaMSP432E4HWAttrs
};

/*
 *  =============================== EMAC ===============================
 */
/* minimize scope of ndk/inc/stk_main.h -- this eliminates TIPOSIX path dependencies */
#define NDK_NOUSERAPIS 1
#include <ti/drivers/emac/EMACMSP432E4.h>

/*
 *  Required by the Networking Stack (NDK). This array must be NULL terminated.
 *  This can be removed if NDK is not used.
 *  Double curly braces are needed to avoid GCC bug #944572
 *  https://bugs.launchpad.net/gcc-linaro/+bug/944572
 */
NIMU_DEVICE_TABLE_ENTRY NIMUDeviceTable[2] = {
    {
        /* Default: use Ethernet driver */
        .init = EMACMSP432E4_NIMUInit
    },
    {NULL}
};

/*
 *  Ethernet MAC address
 *  NOTE: By default (i.e. when each octet is 0xff), the driver reads the MAC
 *        address that's stored in flash. To override this behavior, manually
 *        set the octets of the MAC address you wish to use into the array here:
 */
unsigned char macAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* EMAC configuration structure */
const EMACMSP432E4_HWAttrs EMACMSP432E4_hwAttrs = {
    .baseAddr = EMAC0_BASE,
    .intNum = INT_EMAC0,
    .intPriority = (~0),
    .led0Pin = EMACMSP432E4_PF0_EN0LED0,
    .led1Pin = EMACMSP432E4_PF4_EN0LED1,
    .macAddress = macAddress
};

/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in MSP_EXP432E401Y.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* port_pin */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  0  - dummy */

    /* pins 1-10 */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  1  - 3.3V */
    GPIOMSP432E4_PE4 | GPIO_DO_NOT_CONFIG,          /*  2  - PE4 */
    GPIOMSP432E4_PC4 | GPIO_DO_NOT_CONFIG,          /*  3  - PC4 */
    GPIOMSP432E4_PC5 | GPIO_DO_NOT_CONFIG,          /*  4  - PC5 */
    GPIOMSP432E4_PC6 | GPIO_DO_NOT_CONFIG,          /*  5  - PC6 */
    GPIOMSP432E4_PE5 | GPIO_DO_NOT_CONFIG,          /*  6  - PE5 */
    GPIOMSP432E4_PD3 | GPIO_DO_NOT_CONFIG,          /*  7  - PD3 */
    GPIOMSP432E4_PC7 | GPIO_DO_NOT_CONFIG,          /*  8  - PC7 */
    GPIOMSP432E4_PB2 | GPIO_DO_NOT_CONFIG,          /*  9  - PB2 */
    GPIOMSP432E4_PB3 | GPIO_DO_NOT_CONFIG,          /*  10 - PB3 */

    /* pins 11-20 */
    GPIOMSP432E4_PP2 | GPIO_DO_NOT_CONFIG,          /*  11 - PP2 */
    GPIOMSP432E4_PN3 | GPIO_DO_NOT_CONFIG,          /*  12 - PN3 */
    GPIOMSP432E4_PN2 | GPIO_DO_NOT_CONFIG,          /*  13 - PN2 */
    GPIOMSP432E4_PD0 | GPIO_DO_NOT_CONFIG,          /*  14 - PD0 */
    GPIOMSP432E4_PD1 | GPIO_DO_NOT_CONFIG,          /*  15 - PD1 */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  16 - RESET */
    GPIOMSP432E4_PH3 | GPIO_DO_NOT_CONFIG,          /*  17 - PH3 */
    GPIOMSP432E4_PH2 | GPIO_DO_NOT_CONFIG,          /*  18 - PH2 */
    GPIOMSP432E4_PM3 | GPIO_DO_NOT_CONFIG,          /*  19 - PM3 */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  20 - GND */

    /* pins 21-30 */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  21 - 5V */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  22 - GND */
    GPIOMSP432E4_PE0 | GPIO_DO_NOT_CONFIG,          /*  23 - PE0 */
    GPIOMSP432E4_PE1 | GPIO_DO_NOT_CONFIG,          /*  24 - PE1 */
    GPIOMSP432E4_PE2 | GPIO_DO_NOT_CONFIG,          /*  25 - PE2 */
    GPIOMSP432E4_PE3 | GPIO_DO_NOT_CONFIG,          /*  26 - PE3 */
    GPIOMSP432E4_PD7 | GPIO_DO_NOT_CONFIG,          /*  27 - PD7  NMI */
    GPIOMSP432E4_PD6 | GPIO_DO_NOT_CONFIG,          /*  28 - PD6 */
    GPIOMSP432E4_PM4 | GPIO_DO_NOT_CONFIG,          /*  29 - PM4 */
    GPIOMSP432E4_PM5 | GPIO_DO_NOT_CONFIG,          /*  30 - PM5 */

    /* pins 31-40 */
    GPIOMSP432E4_PL3 | GPIO_DO_NOT_CONFIG,          /*  31 - PL3 */
    GPIOMSP432E4_PL2 | GPIO_DO_NOT_CONFIG,          /*  32 - PL2 */
    GPIOMSP432E4_PL1 | GPIO_DO_NOT_CONFIG,          /*  33 - PL1 */
    GPIOMSP432E4_PL0 | GPIO_DO_NOT_CONFIG,          /*  34 - PL0 */
    GPIOMSP432E4_PL5 | GPIO_DO_NOT_CONFIG,          /*  35 - PL5 */
    GPIOMSP432E4_PL4 | GPIO_DO_NOT_CONFIG,          /*  36 - PL4 */
    GPIOMSP432E4_PG0 | GPIO_DO_NOT_CONFIG,          /*  37 - PG0 */
    GPIOMSP432E4_PF3 | GPIO_DO_NOT_CONFIG,          /*  38 - PF3 */
    GPIOMSP432E4_PF2 | GPIO_DO_NOT_CONFIG,          /*  39 - PF2 */
    GPIOMSP432E4_PF1 | GPIO_DO_NOT_CONFIG,          /*  40 - PF1 */

    /* pins 41-50 */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  41  - 3.3V */
    GPIOMSP432E4_PD2 | GPIO_DO_NOT_CONFIG,          /*  42 - PD2 */
    GPIOMSP432E4_PP0 | GPIO_DO_NOT_CONFIG,          /*  43 - PP0 */
    GPIOMSP432E4_PP1 | GPIO_DO_NOT_CONFIG,          /*  44 - PP1 */
    GPIOMSP432E4_PD4 | GPIO_DO_NOT_CONFIG,          /*  45 - PD4 */
    GPIOMSP432E4_PD5 | GPIO_DO_NOT_CONFIG,          /*  46 - PD5 */
    GPIOMSP432E4_PQ0 | GPIO_DO_NOT_CONFIG,          /*  47 - PQ0 */
    GPIOMSP432E4_PP4 | GPIO_DO_NOT_CONFIG,          /*  48 - PP4 */
    GPIOMSP432E4_PN5 | GPIO_DO_NOT_CONFIG,          /*  49 - PN5 */
    GPIOMSP432E4_PN4 | GPIO_DO_NOT_CONFIG,          /*  50 - PN4 */

    /* pins 51-60 */
    GPIOMSP432E4_PM6 | GPIO_DO_NOT_CONFIG,          /*  51 - PM6 */
    GPIOMSP432E4_PQ1 | GPIO_DO_NOT_CONFIG,          /*  52 - PQ1 */
    GPIOMSP432E4_PP3 | GPIO_DO_NOT_CONFIG,          /*  53 - PP3 */
    GPIOMSP432E4_PQ3 | GPIO_DO_NOT_CONFIG,          /*  54 - PQ3 */
    GPIOMSP432E4_PQ2 | GPIO_DO_NOT_CONFIG,          /*  55 - PQ2 */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  56 - RESET */
    GPIOMSP432E4_PA7 | GPIO_DO_NOT_CONFIG,          /*  57 - PA7 */
    GPIOMSP432E4_PP5 | GPIO_DO_NOT_CONFIG,          /*  58 - PP5 */
    GPIOMSP432E4_PM7 | GPIO_DO_NOT_CONFIG,          /*  59 - PM7 */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  60 - GND */

    /* pins 61-70 */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  61 - 5V */
    GPIOMSP432E4_EMPTY_PIN | GPIO_DO_NOT_CONFIG,    /*  62 - GND */
    GPIOMSP432E4_PB4 | GPIO_DO_NOT_CONFIG,          /*  63 - PB4 */
    GPIOMSP432E4_PB5 | GPIO_DO_NOT_CONFIG,          /*  64 - PB5 */
    GPIOMSP432E4_PK0 | GPIO_DO_NOT_CONFIG,          /*  65 - PK0 */
    GPIOMSP432E4_PK1 | GPIO_DO_NOT_CONFIG,          /*  66 - PK1 */
    GPIOMSP432E4_PK2 | GPIO_DO_NOT_CONFIG,          /*  67 - PK2 */
    GPIOMSP432E4_PK3 | GPIO_DO_NOT_CONFIG,          /*  68 - PK3 */
    GPIOMSP432E4_PA4 | GPIO_DO_NOT_CONFIG,          /*  69 - PA4 */
    GPIOMSP432E4_PA5 | GPIO_DO_NOT_CONFIG,          /*  70 - PA5 */

    /* pins 71-80 */
    GPIOMSP432E4_PK7 | GPIO_DO_NOT_CONFIG,          /*  71 - PK7 */
    GPIOMSP432E4_PK6 | GPIO_DO_NOT_CONFIG,          /*  72 - PK6 */
    GPIOMSP432E4_PH1 | GPIO_DO_NOT_CONFIG,          /*  73 - PH1 */
    GPIOMSP432E4_PH0 | GPIO_DO_NOT_CONFIG,          /*  74 - PH0 */
    GPIOMSP432E4_PM2 | GPIO_DO_NOT_CONFIG,          /*  75 - PM2 */
    GPIOMSP432E4_PM1 | GPIO_DO_NOT_CONFIG,          /*  76 - PM1 */
    GPIOMSP432E4_PM0 | GPIO_DO_NOT_CONFIG,          /*  77 - PM0 */
    GPIOMSP432E4_PK5 | GPIO_DO_NOT_CONFIG,          /*  78 - PK5 */
    GPIOMSP432E4_PK4 | GPIO_DO_NOT_CONFIG,          /*  79 - PK4 */
    GPIOMSP432E4_PG1 | GPIO_DO_NOT_CONFIG,          /*  80 - PG1 */

    /* virtual pins 81-95 */
    GPIOMSP432E4_PN1 | GPIO_DO_NOT_CONFIG,          /*  81 - PN1 D1 */
    GPIOMSP432E4_PN0 | GPIO_DO_NOT_CONFIG,          /*  82 - PN0 D2 */
    GPIOMSP432E4_PF4 | GPIO_DO_NOT_CONFIG,          /*  83 - PF4 D3 */
    GPIOMSP432E4_PF0 | GPIO_DO_NOT_CONFIG,          /*  84 - PF0 D4 */
    GPIOMSP432E4_PJ0 | GPIO_DO_NOT_CONFIG,          /*  85 - PJ0 SW1 */
    GPIOMSP432E4_PJ1 | GPIO_DO_NOT_CONFIG,          /*  86 - PJ1 SW2 */
    GPIOMSP432E4_PD6 | GPIO_DO_NOT_CONFIG,          /*  87 - PD6 A5 */
    GPIOMSP432E4_PA0 | GPIO_DO_NOT_CONFIG,          /*  88 - PA0 JP4 */
    GPIOMSP432E4_PA1 | GPIO_DO_NOT_CONFIG,          /*  89 - PA1 JP5 */
    GPIOMSP432E4_PA2 | GPIO_DO_NOT_CONFIG,          /*  90 - PA2 */
    GPIOMSP432E4_PA3 | GPIO_DO_NOT_CONFIG,          /*  91 - PA3 */
    GPIOMSP432E4_PL6 | GPIO_DO_NOT_CONFIG,          /*  92 - PL6 */
    GPIOMSP432E4_PL7 | GPIO_DO_NOT_CONFIG,          /*  93 - PL7 */
    GPIOMSP432E4_PB0 | GPIO_DO_NOT_CONFIG,          /*  94 - PB0 */
    GPIOMSP432E4_PB1 | GPIO_DO_NOT_CONFIG,          /*  95 - PB1 */
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in MSP_EXP432E401Y.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    /* port_pin */
    NULL,  /*  0  - dummy */

    /* pins 1-10 */
    NULL,  /*  1  - 3.3V */
    NULL,  /*  2  - PE4 */
    NULL,  /*  3  - PC4 */
    NULL,  /*  4  - PC5 */
    NULL,  /*  5  - PC6 */
    NULL,  /*  6  - PE5 */
    NULL,  /*  7  - PD3 */
    NULL,  /*  8  - PC7 */
    NULL,  /*  9  - PB2 */
    NULL,  /*  10 - PB3 */

    /* pins 11-20 */
    NULL,  /*  11 - PP2 */
    NULL,  /*  12 - PN3 */
    NULL,  /*  13 - PN2 */
    NULL,  /*  14 - PD0 */
    NULL,  /*  15 - PD1 */
    NULL,  /*  16 - RESET */
    NULL,  /*  17 - PH3 */
    NULL,  /*  18 - PH2 */
    NULL,  /*  19 - PM3 */
    NULL,  /*  20 - GND */

    /* pins 21-30 */
    NULL,  /*  21 - 5V */
    NULL,  /*  22 - GND */
    NULL,  /*  23 - PE0 */
    NULL,  /*  24 - PE1 */
    NULL,  /*  25 - PE2 */
    NULL,  /*  26 - PE3 */
    NULL,  /*  27 - PD7 NMI */
    NULL,  /*  28 - PD6 */
    NULL,  /*  29 - PM4 */
    NULL,  /*  30 - PM5 */

    /* pins 31-40 */
    NULL,  /*  31 - PL3 */
    NULL,  /*  32 - PL2 */
    NULL,  /*  33 - PL1 */
    NULL,  /*  34 - PL0 */
    NULL,  /*  35 - PL5 */
    NULL,  /*  36 - PL4 */
    NULL,  /*  37 - PG0 */
    NULL,  /*  38 - PF3 */
    NULL,  /*  39 - PF2 */
    NULL,  /*  40 - PF1 */

    /* pins 41-50 */
    NULL,  /*  41  - 3.3V */
    NULL,  /*  42 - PD2 */
    NULL,  /*  43 - PP0 */
    NULL,  /*  44 - PP1 */
    NULL,  /*  45 - PD4 */
    NULL,  /*  46 - PD5 */
    NULL,  /*  47 - PQ0 */
    NULL,  /*  48 - PP4 */
    NULL,  /*  49 - PN5 */
    NULL,  /*  50 - PN4 */

    /* pins 51-60 */
    NULL,  /*  51 - PM6 */
    NULL,  /*  52 - PQ1 */
    NULL,  /*  53 - PP3 */
    NULL,  /*  54 - PQ3 */
    NULL,  /*  55 - PQ2 */
    NULL,  /*  56 - RESET */
    NULL,  /*  57 - PA7 */
    NULL,  /*  58 - PP5 */
    NULL,  /*  59 - PM7 */
    NULL,  /*  60 - GND */

    /* pins 61-70 */
    NULL,  /*  61 - 5V */
    NULL,  /*  62 - GND */
    NULL,  /*  63 - PB4 */
    NULL,  /*  64 - PB5 */
    NULL,  /*  65 - PK0 */
    NULL,  /*  66 - PK1 */
    NULL,  /*  67 - PK2 */
    NULL,  /*  68 - PK3 */
    NULL,  /*  69 - PA4 */
    NULL,  /*  70 - PA5 */

    /* pins 71-80 */
    NULL,  /*  71 - PK7 */
    NULL,  /*  72 - PK6 */
    NULL,  /*  73 - PH1 */
    NULL,  /*  74 - PH0 */
    NULL,  /*  75 - PM2 */
    NULL,  /*  76 - PM1 */
    NULL,  /*  77 - PM0 */
    NULL,  /*  78 - PK5 */
    NULL,  /*  79 - PK4 */
    NULL,  /*  80 - PG1 */
    
    /* virtual pins 81-95 */
    NULL,  /*  81 - PN1 D1 */
    NULL,  /*  82 - PN0 D2 */
    NULL,  /*  83 - PF4 D3 */
    NULL,  /*  84 - PF0 D4 */
    NULL,  /*  85 - PJ0 SW1 */
    NULL,  /*  86 - PJ1 SW2 */
    NULL,  /*  87 - PD6 A5 */
    NULL,  /*  88 - PA0 JP4 */
    NULL,  /*  89 - PA1 JP5 */
    NULL,  /*  90 - PA2 */
    NULL,  /*  91 - PA3 */
    NULL,  /*  92 - PL6 */
    NULL,  /*  93 - PL7 */
    NULL,  /*  94 - PB0 */
    NULL,  /*  95 - PB1 */
};

/* The device-specific GPIO_config structure */
const GPIOMSP432E4_Config GPIOMSP432E4_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};

/*
 *  =============================== I2C ===============================
 */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSP432E4.h>

#define Board_I2CCOUNT 5

I2CMSP432E4_Object i2cMSP432E4Objects[Board_I2CCOUNT];

const I2CMSP432E4_HWAttrs i2cMSP432E4HWAttrs[Board_I2CCOUNT] = {
    /* Wire */
    {
        .baseAddr = I2C0_BASE,
        .intNum = INT_I2C0,
        .intPriority = (~0),
        .sclPin  = I2CMSP432E4_PB2_I2C0SCL,
        .sdaPin = I2CMSP432E4_PB3_I2C0SDA
    },
    /* Wire1 */
    {
        .baseAddr = I2C2_BASE,
        .intNum = INT_I2C2,
        .intPriority = (~0),
        .sclPin  = I2CMSP432E4_PN5_I2C2SCL,
        .sdaPin = I2CMSP432E4_PN4_I2C2SDA
    },
    /* Wire2 */
    {
        .baseAddr = I2C7_BASE,
        .intNum = INT_I2C7,
        .intPriority = (~0),
        .sclPin  = I2CMSP432E4_PD0_I2C7SCL,
        .sdaPin = I2CMSP432E4_PD1_I2C7SDA
    },
    /* Wire3 */
    {
        .baseAddr = I2C8_BASE,
        .intNum = INT_I2C8,
        .intPriority = (~0),
        .sclPin  = I2CMSP432E4_PA2_I2C8SCL,
        .sdaPin = I2CMSP432E4_PA3_I2C8SDA
    },
    /* Wire4 */
    {
        .baseAddr = I2C2_BASE,
        .intNum = INT_I2C2,
        .intPriority = (~0),
        .sclPin  = I2CMSP432E4_PL1_I2C2SCL,
        .sdaPin = I2CMSP432E4_PL0_I2C2SDA
    }
};

/*
 *  ======== I2C_config ========
 */
const I2C_Config I2C_config[Board_I2CCOUNT] = {
    /* Wire */
    {
        .fxnTablePtr = &I2CMSP432E4_fxnTable,
        .object = &i2cMSP432E4Objects[0],
        .hwAttrs = &i2cMSP432E4HWAttrs[0]
    },
    /* Wire1 */
    {
        .fxnTablePtr = &I2CMSP432E4_fxnTable,
        .object = &i2cMSP432E4Objects[1],
        .hwAttrs = &i2cMSP432E4HWAttrs[1]
    },
    /* Wire2 */
    {
        .fxnTablePtr = &I2CMSP432E4_fxnTable,
        .object = &i2cMSP432E4Objects[2],
        .hwAttrs = &i2cMSP432E4HWAttrs[2]
    },
    /* Wire3 */
    {
        .fxnTablePtr = &I2CMSP432E4_fxnTable,
        .object = &i2cMSP432E4Objects[3],
        .hwAttrs = &i2cMSP432E4HWAttrs[3]
    },
    /* Wire4 */
    {
        .fxnTablePtr = &I2CMSP432E4_fxnTable,
        .object = &i2cMSP432E4Objects[4],
        .hwAttrs = &i2cMSP432E4HWAttrs[4]
    }
};

const uint_least8_t I2C_count = Board_I2CCOUNT;

/*
 *  =============================== NVS ===============================
 *  Non-Volatile Storage configuration.
 */
#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSMSP432E4.h>


NVSMSP432E4_Object nvsMSP432E4Objects[1];

extern uint8_t __NVS_BASE__;
extern uint8_t __NVS_SIZE__;

const NVSMSP432E4_HWAttrs nvsMSP432E4HWAttrs[] = {
    {
        .regionBase = (void *)&__NVS_BASE__,   /* base of unused flash aligned on 4k boundary */
        .regionSize = (size_t)(&__NVS_SIZE__) 
    },
};

const NVS_Config NVS_config[] = {
    {
        .fxnTablePtr = &NVSMSP432E4_fxnTable,
        .object = &nvsMSP432E4Objects[0],
        .hwAttrs = &nvsMSP432E4HWAttrs[0],
    },
};

const uint_least8_t NVS_count = MSP_EXP432E401Y_NVSCOUNT;

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/power/PowerMSP432E4.h>
const PowerMSP432E4_Config PowerMSP432E4_config = {
    .policyFxn = &PowerMSP432E4_sleepPolicy,
    .enablePolicy = true
};

/*
 *  =============================== PWM ===============================
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMMSP432E4.h>

PWMMSP432E4_Object pwmMSP432E4Objects[MSP_EXP432E401Y_PWMCOUNT];

const PWMMSP432E4_HWAttrs pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWMCOUNT] = {
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_0,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PF0_M0PWM0
    },
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_1,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PF1_M0PWM1
    },
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_2,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PF2_M0PWM2
    },
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_3,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PF3_M0PWM3
    },
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_4,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PG0_M0PWM4
    },
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_5,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PG1_M0PWM5
    },
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_6,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PK4_M0PWM6
    },
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_7,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PK5_M0PWM7
    }
};

const PWM_Config PWM_config[MSP_EXP432E401Y_PWMCOUNT] = {
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM0],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM0]
    },
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM1],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM1]
    },
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM2],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM2]
    },
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM3],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM3]
    },
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM4],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM4]
    },
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM5],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM5]
    },
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM6],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM6]
    },
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM7],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM7]
    }
};

const uint_least8_t PWM_count = MSP_EXP432E401Y_PWMCOUNT;

/*
 *  =============================== SDFatFS ===============================
 */
#include <ti/drivers/SD.h>
#include <ti/drivers/SDFatFS.h>

/*
 * Note: The SDFatFS driver provides interface functions to enable FatFs
 * but relies on the SD driver to communicate with SD cards.  Opening a
 * SDFatFs driver instance will internally try to open a SD driver instance
 * reusing the same index number (opening SDFatFs driver at index 0 will try to
 * open SD driver at index 0).  This requires that all SDFatFs driver instances
 * have an accompanying SD driver instance defined with the same index.  It is
 * acceptable to have more SD driver instances than SDFatFs driver instances
 * but the opposite is not supported & the SDFatFs will fail to open.
 */
SDFatFS_Object sdfatfsObjects[MSP_EXP432E401Y_SDFatFSCOUNT];

const SDFatFS_Config SDFatFS_config[MSP_EXP432E401Y_SDFatFSCOUNT] = {
    {
        .object = &sdfatfsObjects[MSP_EXP432E401Y_SDFatFS0]
    }
};

const uint_least8_t SDFatFS_count = MSP_EXP432E401Y_SDFatFSCOUNT;

/*
 *  =============================== SD ===============================
 */
#include <ti/drivers/SD.h>
#include <ti/drivers/sd/SDSPI.h>

SDSPI_Object sdspiObjects[MSP_EXP432E401Y_SDCOUNT];

const SDSPI_HWAttrs sdspiHWAttrs[MSP_EXP432E401Y_SDCOUNT] = {
    {
        .spiIndex = MSP_EXP432E401Y_SPI2,
        .spiCsGpioIndex = MSP_EXP432E401Y_SDSPI_CS
    }
};

const SD_Config SD_config[MSP_EXP432E401Y_SDCOUNT] = {
    {
        .fxnTablePtr = &SDSPI_fxnTable,
        .object = &sdspiObjects[MSP_EXP432E401Y_SDSPI0],
        .hwAttrs = &sdspiHWAttrs[MSP_EXP432E401Y_SDSPI0]
    },
};

const uint_least8_t SD_count = MSP_EXP432E401Y_SDCOUNT;

/*
 *  =============================== SPI ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIMSP432E4DMA.h>

SPIMSP432E4DMA_Object spiMSP432E4DMAObjects[MSP_EXP432E401Y_SPICOUNT];

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiMSP432E4DMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
uint16_t spiMSP432E4DMAscratchBuf[MSP_EXP432E401Y_SPICOUNT];

/*
 * NOTE: The SPI instances below can be used by the SD driver to communicate
 * with a SD card via SPI.  The 'defaultTxBufValue' fields below are set to 0xFF
 * to satisfy the SDSPI driver requirement.
 */
const SPIMSP432E4DMA_HWAttrs spiMSP432E4DMAHWAttrs[MSP_EXP432E401Y_SPICOUNT] = {
    /* SPI */
    {
        .baseAddr = SSI2_BASE,
        .intNum = INT_SSI2,
        .intPriority = (~0),
        .scratchBufPtr = &spiMSP432E4DMAscratchBuf[MSP_EXP432E401Y_SPI2],
        .defaultTxBufValue = 0xFF,
        .rxDmaChannel = UDMA_CH12_SSI2RX,
        .txDmaChannel = UDMA_CH13_SSI2TX,
        .minDmaTransferSize = 10,
        .clkPinMask = SPIMSP432E4_PD3_SSI2CLK,
        .fssPinMask = SPIMSP432E4_PD2_SSI2FSS,
        .xdat0PinMask = SPIMSP432E4_PD1_SSI2XDAT0,
        .xdat1PinMask = SPIMSP432E4_PD0_SSI2XDAT1
    },
    /* SPI1 */
    {
        .baseAddr = SSI3_BASE,
        .intNum = INT_SSI3,
        .intPriority = (~0),
        .scratchBufPtr = &spiMSP432E4DMAscratchBuf[MSP_EXP432E401Y_SPI3],
        .defaultTxBufValue = 0xFF,
        .minDmaTransferSize = 10,
        .rxDmaChannel = UDMA_CH14_SSI3RX,
        .txDmaChannel = UDMA_CH15_SSI3TX,
        .clkPinMask = SPIMSP432E4_PQ0_SSI3CLK,
        .fssPinMask = SPIMSP432E4_PQ1_SSI3FSS,
        .xdat0PinMask = SPIMSP432E4_PQ2_SSI3XDAT0,
        .xdat1PinMask = SPIMSP432E4_PQ3_SSI3XDAT1
    }
};

const SPI_Config SPI_config[MSP_EXP432E401Y_SPICOUNT] = {
    {
        .fxnTablePtr = &SPIMSP432E4DMA_fxnTable,
        .object = &spiMSP432E4DMAObjects[MSP_EXP432E401Y_SPI2],
        .hwAttrs = &spiMSP432E4DMAHWAttrs[MSP_EXP432E401Y_SPI2]
    },
    {
        .fxnTablePtr = &SPIMSP432E4DMA_fxnTable,
        .object = &spiMSP432E4DMAObjects[MSP_EXP432E401Y_SPI3],
        .hwAttrs = &spiMSP432E4DMAHWAttrs[MSP_EXP432E401Y_SPI3]
    },
};

const uint_least8_t SPI_count = MSP_EXP432E401Y_SPICOUNT;

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTMSP432E4.h>

#define Board_UARTCOUNT 5

static unsigned char uartMSP432E4RingBuffer0[32];
UARTMSP432E4_Object uartMSP432E4Objects0;

/* Serial */
static const UARTMSP432E4_HWAttrs uartMSP432E4HWAttrs0 = {
    .baseAddr           = UART0_BASE,
    .intNum             = INT_UART0,
    .intPriority        = (~0),
    .flowControl        = UARTMSP432E4_FLOWCTRL_NONE,
    .ringBufPtr         = uartMSP432E4RingBuffer0,
    .ringBufSize        = sizeof(uartMSP432E4RingBuffer0),
    .rxPin              = UARTMSP432E4_PA0_U0RX,
    .txPin              = UARTMSP432E4_PA1_U0TX,
    .ctsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .rtsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .errorFxn           = NULL
};

static unsigned char uartMSP432E4RingBuffer1[32];
UARTMSP432E4_Object uartMSP432E4Objects1;

/* Serial1 */
static const UARTMSP432E4_HWAttrs uartMSP432E4HWAttrs1 = {
    .baseAddr           = UART7_BASE,
    .intNum             = INT_UART7,
    .intPriority        = (~0),
    .flowControl        = UARTMSP432E4_FLOWCTRL_NONE,
    .ringBufPtr         = uartMSP432E4RingBuffer1,
    .ringBufSize        = sizeof(uartMSP432E4RingBuffer1),
    .rxPin              = UARTMSP432E4_PC4_U7RX,
    .txPin              = UARTMSP432E4_PC5_U7TX,
    .ctsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .rtsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .errorFxn           = NULL
};

static unsigned char uartMSP432E4RingBuffer2[32];
UARTMSP432E4_Object uartMSP432E4Objects2;

/* Serial2 */
static const UARTMSP432E4_HWAttrs uartMSP432E4HWAttrs2 = {
    .baseAddr           = UART6_BASE,
    .intNum             = INT_UART6,
    .intPriority        = (~0),
    .flowControl        = UARTMSP432E4_FLOWCTRL_NONE,
    .ringBufPtr         = uartMSP432E4RingBuffer2,
    .ringBufSize        = sizeof(uartMSP432E4RingBuffer2),
    .rxPin              = UARTMSP432E4_PP0_U6RX,
    .txPin              = UARTMSP432E4_PP1_U6TX,
    .ctsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .rtsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .errorFxn           = NULL
};

static unsigned char uartMSP432E4RingBuffer3[32];
UARTMSP432E4_Object uartMSP432E4Objects3;

/* Serial3 */
static const UARTMSP432E4_HWAttrs uartMSP432E4HWAttrs3 = {
    .baseAddr           = UART3_BASE,
    .intNum             = INT_UART3,
    .intPriority        = (~0),
    .flowControl        = UARTMSP432E4_FLOWCTRL_NONE,
    .ringBufPtr         = uartMSP432E4RingBuffer3,
    .ringBufSize        = sizeof(uartMSP432E4RingBuffer3),
    .rxPin              = UARTMSP432E4_PA4_U3RX,
    .txPin              = UARTMSP432E4_PA5_U3TX,
    .ctsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .rtsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .errorFxn           = NULL
};

static unsigned char uartMSP432E4RingBuffer4[32];
UARTMSP432E4_Object uartMSP432E4Objects4;

/* Serial4 */
static const UARTMSP432E4_HWAttrs uartMSP432E4HWAttrs4 = {
    .baseAddr           = UART4_BASE,
    .intNum             = INT_UART4,
    .intPriority        = (~0),
    .flowControl        = UARTMSP432E4_FLOWCTRL_NONE,
    .ringBufPtr         = uartMSP432E4RingBuffer4,
    .ringBufSize        = sizeof(uartMSP432E4RingBuffer4),
    .rxPin              = UARTMSP432E4_PK0_U4RX,
    .txPin              = UARTMSP432E4_PK1_U4TX,
    .ctsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .rtsPin             = UARTMSP432E4_PIN_UNASSIGNED, 
    .errorFxn           = NULL
};

const UART_Config UART_config[Board_UARTCOUNT] = {
    {   // Serial
        .fxnTablePtr = &UARTMSP432E4_fxnTable,
        .object      = &uartMSP432E4Objects0,
        .hwAttrs     = &uartMSP432E4HWAttrs0
    },
    {   // Serial1
        .fxnTablePtr = &UARTMSP432E4_fxnTable,
        .object      = &uartMSP432E4Objects1,
        .hwAttrs     = &uartMSP432E4HWAttrs1
    },
    {   // Serial2
        .fxnTablePtr = &UARTMSP432E4_fxnTable,
        .object      = &uartMSP432E4Objects2,
        .hwAttrs     = &uartMSP432E4HWAttrs2
    },
    {   // Serial3
        .fxnTablePtr = &UARTMSP432E4_fxnTable,
        .object      = &uartMSP432E4Objects3,
        .hwAttrs     = &uartMSP432E4HWAttrs3
    },
    {   // Serial4
        .fxnTablePtr = &UARTMSP432E4_fxnTable,
        .object      = &uartMSP432E4Objects4,
        .hwAttrs     = &uartMSP432E4HWAttrs4
    },
};

const uint_least8_t UART_count = Board_UARTCOUNT;

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogMSP432E4.h>

WatchdogMSP432E4_Object watchdogMSP432E4Objects[MSP_EXP432E401Y_WATCHDOGCOUNT];

const WatchdogMSP432E4_HWAttrs watchdogMSP432E4HWAttrs[MSP_EXP432E401Y_WATCHDOGCOUNT] = {
    {
        .baseAddr = WATCHDOG0_BASE,
        .intNum = INT_WATCHDOG,
        .intPriority = (~0),
        .reloadValue = 80000000 /* 1 second period at default CPU clock freq */
    },
};

const Watchdog_Config Watchdog_config[MSP_EXP432E401Y_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogMSP432E4_fxnTable,
        .object = &watchdogMSP432E4Objects[MSP_EXP432E401Y_WATCHDOG0],
        .hwAttrs = &watchdogMSP432E4HWAttrs[MSP_EXP432E401Y_WATCHDOG0]
    },
};

const uint_least8_t Watchdog_count = MSP_EXP432E401Y_WATCHDOGCOUNT;

/*
 *  =============================== General ===============================
 */
/*
 *  ======== Board_init ========
 */
void Board_init(void)
{
    Power_init();
    GPIO_init();

    /* Supply clocks to GPIOD and GPIOF */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /* Unlock and commit NMI pins PD7 and PF0 */
    GPIOD->LOCK = GPIO_LOCK_KEY;
    GPIOD->CR |= 0x80;
    GPIOF->LOCK = GPIO_LOCK_KEY;
    GPIOF->CR |= 0x01;

    /* Grant the DMA access to all FLASH memory */
    FLASH_CTRL->PP |= FLASH_PP_DFA;

    /* Region start address - match FLASH start address */
    FLASH_CTRL->DMAST = 0x00000000;

    /*
     * Access to FLASH is granted to the DMA in 2KB regions.  The value
     * assigned to DMASZ is the amount of 2KB regions to which the DMA will
     * have access.  The value can be determined via the following:
     *     2 * (num_regions + 1) KB
     *
     * To grant full access to entire 1MB of FLASH:
     *     2 * (511 + 1) KB = 1024 KB (1 MB)
     */
    FLASH_CTRL->DMASZ = 511;
}
