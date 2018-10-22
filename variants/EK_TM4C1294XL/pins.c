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

#include <ti/runtime/wiring/wiring_private.h>
#include <ti/runtime/wiring/msp432e/wiring_analog.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>

uint8_t digital_pin_to_pin_function[] = {
    /* port_pin */
    PIN_FUNC_INVALID, /*  0  - dummy */

    /* pins 1-10 */
    PIN_FUNC_INVALID, /*  1  - 3.3V */
    PIN_FUNC_UNUSED,  /*  2  - PE4 */
    PIN_FUNC_UNUSED,  /*  3  - PC4 */
    PIN_FUNC_UNUSED,  /*  4  - PC5 */
    PIN_FUNC_UNUSED,  /*  5  - PC6 */
    PIN_FUNC_UNUSED,  /*  6  - PE5 */
    PIN_FUNC_UNUSED,  /*  7  - PD3 */
    PIN_FUNC_UNUSED,  /*  8  - PC7 */
    PIN_FUNC_UNUSED,  /*  9  - PB2 */
    PIN_FUNC_UNUSED,  /*  10 - PB3 */

    /* pins 11-20 */
    PIN_FUNC_UNUSED,  /*  11 - PP2 */
    PIN_FUNC_UNUSED,  /*  12 - PN3 */
    PIN_FUNC_UNUSED,  /*  13 - PN2 */
    PIN_FUNC_UNUSED,  /*  14 - PD0 */
    PIN_FUNC_UNUSED,  /*  15 - PD1 */
    PIN_FUNC_INVALID, /*  16 - RESET */
    PIN_FUNC_UNUSED,  /*  17 - PH3 */
    PIN_FUNC_UNUSED,  /*  18 - PH2 */
    PIN_FUNC_UNUSED,  /*  19 - PM3 */
    PIN_FUNC_UNUSED,  /*  20 - GND */

    /* pins 21-30 */
    PIN_FUNC_INVALID, /*  21 - 5V */
    PIN_FUNC_INVALID, /*  22 - GND */
    PIN_FUNC_UNUSED,  /*  23 - PE0 */
    PIN_FUNC_UNUSED,  /*  24 - PE1 */
    PIN_FUNC_UNUSED,  /*  25 - PE2 */
    PIN_FUNC_UNUSED,  /*  26 - PE3 */
    PIN_FUNC_UNUSED,  /*  27 - PD7 */
    PIN_FUNC_UNUSED,  /*  28 - PA6 */
    PIN_FUNC_UNUSED,  /*  29 - PM4 */
    PIN_FUNC_UNUSED,  /*  30 - PM5 */

    /* pins 31-40 */
    PIN_FUNC_UNUSED,  /*  31 - PL3 */
    PIN_FUNC_UNUSED,  /*  32 - PL2 */
    PIN_FUNC_UNUSED,  /*  33 - PL1 */
    PIN_FUNC_UNUSED,  /*  34 - PL0 */
    PIN_FUNC_UNUSED,  /*  35 - PL5 */
    PIN_FUNC_UNUSED,  /*  36 - PL4 */
    PIN_FUNC_UNUSED,  /*  37 - PG0 */
    PIN_FUNC_UNUSED,  /*  38 - PF3 */
    PIN_FUNC_UNUSED,  /*  39 - PF2 */
    PIN_FUNC_UNUSED,  /*  40 - PF1 */

    /* pins 41-50 */
    PIN_FUNC_INVALID, /*  41  - 3.3V */
    PIN_FUNC_UNUSED,  /*  42 - PD2 */
    PIN_FUNC_UNUSED,  /*  43 - PP0 */
    PIN_FUNC_UNUSED,  /*  44 - PP1 */
    PIN_FUNC_UNUSED,  /*  45 - PD4 */
    PIN_FUNC_UNUSED,  /*  46 - PD5 */
    PIN_FUNC_UNUSED,  /*  47 - PQ0 */
    PIN_FUNC_UNUSED,  /*  48 - PP4 */
    PIN_FUNC_UNUSED,  /*  49 - PN5 */
    PIN_FUNC_UNUSED,  /*  50 - PN4 */

    /* pins 51-60 */
    PIN_FUNC_UNUSED,  /*  51 - PM6 */
    PIN_FUNC_UNUSED,  /*  52 - PQ1 */
    PIN_FUNC_UNUSED,  /*  53 - PP3 */
    PIN_FUNC_UNUSED,  /*  54 - PQ3 */
    PIN_FUNC_UNUSED,  /*  55 - PQ2 */
    PIN_FUNC_INVALID, /*  56 - RESET */
    PIN_FUNC_UNUSED,  /*  57 - PA7 */
    PIN_FUNC_UNUSED,  /*  58 - PP5 */
    PIN_FUNC_UNUSED,  /*  59 - PM7 */
    PIN_FUNC_UNUSED,  /*  60 - GND */

    /* pins 61-70 */
    PIN_FUNC_INVALID, /*  61 - 5V */
    PIN_FUNC_INVALID, /*  62 - GND */
    PIN_FUNC_UNUSED,  /*  63 - PB4 */
    PIN_FUNC_UNUSED,  /*  64 - PB5 */
    PIN_FUNC_UNUSED,  /*  65 - PK0 */
    PIN_FUNC_UNUSED,  /*  66 - PK1 */
    PIN_FUNC_UNUSED,  /*  67 - PK2 */
    PIN_FUNC_UNUSED,  /*  68 - PK3 */
    PIN_FUNC_UNUSED,  /*  69 - PA4 */
    PIN_FUNC_UNUSED,  /*  70 - PA5 */

    /* pins 71-80 */
    PIN_FUNC_UNUSED,  /*  71 - PK7 */
    PIN_FUNC_UNUSED,  /*  72 - PK6 */
    PIN_FUNC_UNUSED,  /*  73 - PH1 */
    PIN_FUNC_UNUSED,  /*  74 - PH0 */
    PIN_FUNC_UNUSED,  /*  75 - PM2 */
    PIN_FUNC_UNUSED,  /*  76 - PM1 */
    PIN_FUNC_UNUSED,  /*  77 - PM0 */
    PIN_FUNC_UNUSED,  /*  78 - PK5 */
    PIN_FUNC_UNUSED,  /*  79 - PK4 */
    PIN_FUNC_UNUSED,  /*  80 - PG1 */
    
    /* virtual pins 81-95 */
    PIN_FUNC_UNUSED,  /*  81 - PN1 D1 */
    PIN_FUNC_UNUSED,  /*  82 - PN0 D2 */
    PIN_FUNC_UNUSED,  /*  83 - PF4 D3 */
    PIN_FUNC_UNUSED,  /*  84 - PF0 D4 */
    PIN_FUNC_UNUSED,  /*  85 - PJ0 SW1 */
    PIN_FUNC_UNUSED,  /*  86 - PJ1 SW2 */
    PIN_FUNC_UNUSED,  /*  87 - PD6 A5 */
    PIN_FUNC_UNUSED,  /*  88 - PA0 JP4 */
    PIN_FUNC_UNUSED,  /*  89 - PA1 JP5 */
    PIN_FUNC_UNUSED,  /*  90 - PA2 */
    PIN_FUNC_UNUSED,  /*  91 - PA3 */
    PIN_FUNC_UNUSED,  /*  92 - PL6 */
    PIN_FUNC_UNUSED,  /*  93 - PL7 */
    PIN_FUNC_UNUSED,  /*  94 - PB0 */
    PIN_FUNC_UNUSED,  /*  95 - PB1 */
};

/*
 * When a mappable pin is being used for analogWrite(),
 * its corresponding entry in this table is replaced with the
 * PWM channel index it is using.
 *
 * If/when a pin is then changed back to a digitial pin, the
 * pin's entry in this table is restored to PWM_MAPPABLE.
 *
 * Fixed map entries are not modified.
 */
uint8_t digital_pin_to_pwm_index[] = {
    /* port_pin */
    PWM_NOT_MAPPABLE,  /*  0  - dummy */

    /* pins 1-10 */
    PWM_NOT_MAPPABLE,  /*  1  - 3.3V */
    PWM_NOT_MAPPABLE,  /*  2  - PE4 */
    PWM_NOT_MAPPABLE,  /*  3  - PC4 */
    PWM_NOT_MAPPABLE,  /*  4  - PC5 */
    PWM_NOT_MAPPABLE,  /*  5  - PC6 */
    PWM_NOT_MAPPABLE,  /*  6  - PE5 */
    PWM_NOT_MAPPABLE,  /*  7  - PD3 */
    PWM_NOT_MAPPABLE,  /*  8  - PC7 */
    PWM_NOT_MAPPABLE,  /*  9  - PB2 */
    PWM_NOT_MAPPABLE,  /*  10 - PB3 */

    /* pins 11-20 */
    PWM_NOT_MAPPABLE,   /*  11 - PP2 */
    PWM_NOT_MAPPABLE,   /*  12 - PN3 */
    PWM_NOT_MAPPABLE,   /*  13 - PN2 */
    PWM_NOT_MAPPABLE,   /*  14 - PD0 */
    PWM_NOT_MAPPABLE,   /*  15 - PD1 */
    PWM_NOT_MAPPABLE,   /*  16 - RESET */
    PWM_NOT_MAPPABLE,   /*  17 - PH3 */
    PWM_NOT_MAPPABLE,   /*  18 - PH2 */
    PWM_NOT_MAPPABLE,   /*  19 - PM3 */
    PWM_NOT_MAPPABLE,   /*  20 - GND */

    /* pins 21-30 */
    PWM_NOT_MAPPABLE,   /*  21 - 5V */
    PWM_NOT_MAPPABLE,   /*  22 - GND */
    PWM_NOT_MAPPABLE,   /*  23 - PE0 */
    PWM_NOT_MAPPABLE,   /*  24 - PE1 */
    PWM_NOT_MAPPABLE,   /*  25 - PE2 */
    PWM_NOT_MAPPABLE,   /*  26 - PE3 */
    PWM_NOT_MAPPABLE,   /*  27 - PD7 */
    PWM_NOT_MAPPABLE,   /*  28 - PA6 */
    PWM_NOT_MAPPABLE,   /*  29 - PM4 */
    PWM_NOT_MAPPABLE,   /*  30 - PM5 */

    /* pins 31-40 */
    PWM_NOT_MAPPABLE,   /*  31 - PL3 */
    PWM_NOT_MAPPABLE,   /*  32 - PL2 */
    PWM_NOT_MAPPABLE,   /*  33 - PL1 */
    PWM_NOT_MAPPABLE,   /*  34 - PL0 */
    PWM_NOT_MAPPABLE,   /*  35 - PL5 */
    PWM_NOT_MAPPABLE,   /*  36 - PL4 */
    PWM_FIXED_INDEX_4,  /*  37 - PG0 PWM4 */
    PWM_FIXED_INDEX_3,  /*  38 - PF3 PWM3 */
    PWM_FIXED_INDEX_2,  /*  39 - PF2 PWM2 */
    PWM_FIXED_INDEX_1,  /*  40 - PF1 PWM1 */

    /* pins 41-50 */
    PWM_NOT_MAPPABLE,   /*  41  - 3.3V */
    PWM_NOT_MAPPABLE,   /*  42 - PD2 */
    PWM_NOT_MAPPABLE,   /*  43 - PP0 */
    PWM_NOT_MAPPABLE,   /*  44 - PP1 */
    PWM_NOT_MAPPABLE,   /*  45 - PD4 */
    PWM_NOT_MAPPABLE,   /*  46 - PD5 */
    PWM_NOT_MAPPABLE,   /*  47 - PQ0 */
    PWM_NOT_MAPPABLE,   /*  48 - PP4 */
    PWM_NOT_MAPPABLE,   /*  49 - PN5 */
    PWM_NOT_MAPPABLE,   /*  50 - PN4 */

    /* pins 51-60 */
    PWM_NOT_MAPPABLE,   /*  51 - PM6 */
    PWM_NOT_MAPPABLE,   /*  52 - PQ1 */
    PWM_NOT_MAPPABLE,   /*  53 - PP3 */
    PWM_NOT_MAPPABLE,   /*  54 - PQ3 */
    PWM_NOT_MAPPABLE,   /*  55 - PQ2 */
    PWM_NOT_MAPPABLE,   /*  56 - RESET */
    PWM_NOT_MAPPABLE,   /*  57 - PA7 */
    PWM_NOT_MAPPABLE,   /*  58 - PP5 */
    PWM_NOT_MAPPABLE,   /*  59 - PM7 */
    PWM_NOT_MAPPABLE,   /*  60 - GND */

    /* pins 61-70 */
    PWM_NOT_MAPPABLE,   /*  61 - 5V */
    PWM_NOT_MAPPABLE,   /*  62 - GND */
    PWM_NOT_MAPPABLE,   /*  63 - PB4 */
    PWM_NOT_MAPPABLE,   /*  64 - PB5 */
    PWM_NOT_MAPPABLE,   /*  65 - PK0 */
    PWM_NOT_MAPPABLE,   /*  66 - PK1 */
    PWM_NOT_MAPPABLE,   /*  67 - PK2 */
    PWM_NOT_MAPPABLE,   /*  68 - PK3 */
    PWM_NOT_MAPPABLE,   /*  69 - PA4 */
    PWM_NOT_MAPPABLE,   /*  70 - PA5 */

    /* pins 71-80 */
    PWM_NOT_MAPPABLE,   /*  71 - PK7 */
    PWM_NOT_MAPPABLE,   /*  72 - PK6 */
    PWM_NOT_MAPPABLE,   /*  73 - PH1 */
    PWM_NOT_MAPPABLE,   /*  74 - PH0 */
    PWM_NOT_MAPPABLE,   /*  75 - PM2 */
    PWM_NOT_MAPPABLE,   /*  76 - PM1 */
    PWM_NOT_MAPPABLE,   /*  77 - PM0 */
    PWM_FIXED_INDEX_7,  /*  78 - PK5 PWM7 */
    PWM_FIXED_INDEX_6,  /*  79 - PK4 PWM6 */
    PWM_FIXED_INDEX_5,  /*  80 - PG1 PWM5 */
    
    /* virtual pins 81-95 */
    PWM_NOT_MAPPABLE,   /*  81 - PN1 D1 */
    PWM_NOT_MAPPABLE,   /*  82 - PN0 D2 */
    PWM_NOT_MAPPABLE,   /*  83 - PF4 D3 */
    PWM_FIXED_INDEX_0,  /*  84 - PF0 D4 PWM0 */
    PWM_NOT_MAPPABLE,   /*  85 - PJ0 SW1 */
    PWM_NOT_MAPPABLE,   /*  86 - PJ1 SW2 */
    PWM_NOT_MAPPABLE,   /*  87 - PD6 A5 */
    PWM_NOT_MAPPABLE,   /*  88 - PA0 JP4 */
    PWM_NOT_MAPPABLE,   /*  89 - PA1 JP5 */
    PWM_NOT_MAPPABLE,   /*  90 - PA2 */
    PWM_NOT_MAPPABLE,   /*  91 - PA3 */
    PWM_NOT_MAPPABLE,   /*  92 - PL6 */
    PWM_NOT_MAPPABLE,   /*  93 - PL7 */
    PWM_NOT_MAPPABLE,   /*  94 - PB0 */
    PWM_NOT_MAPPABLE,   /*  95 - PB1 */
};

/*
 * mapping of pins to an ADC channel
 */
const uint8_t digital_pin_to_adc_index[] = {
    /* port_pin */
    NOT_ON_ADC,  /*  0  - dummy */

    /* pins 1-10 */
    NOT_ON_ADC,  /*  1  - 3.3V */
    9,           /*  2  - PE4 A9 */
    NOT_ON_ADC,  /*  3  - PC4 */
    NOT_ON_ADC,  /*  4  - PC5 */
    NOT_ON_ADC,  /*  5  - PC6 */
    8,           /*  6  - PE5 A8 */
    12,          /*  7  - PD3 A12 */
    NOT_ON_ADC,  /*  8  - PC7 */
    NOT_ON_ADC,  /*  9  - PB2 */
    NOT_ON_ADC,  /*  10 - PB3 */

    /* pins 11-20 */
    NOT_ON_ADC,  /*  11 - PP2 */
    NOT_ON_ADC,  /*  12 - PN3 */
    NOT_ON_ADC,  /*  13 - PN2 */
    15,          /*  14 - PD0 A15 */
    14,          /*  15 - PD1 A14 */
    NOT_ON_ADC,  /*  16 - RESET */
    NOT_ON_ADC,  /*  17 - PH3 */
    NOT_ON_ADC,  /*  18 - PH2 */
    NOT_ON_ADC,  /*  19 - PM3 */
    NOT_ON_ADC,  /*  20 - GND */

    /* pins 21-30 */
    NOT_ON_ADC,  /*  21 - 5V */
    NOT_ON_ADC,  /*  22 - GND */
    3,           /*  23 - PE0 A3 */
    2,           /*  24 - PE1 A2 */
    1,           /*  25 - PE2 A1 */
    0,           /*  26 - PE3 A0 */
    4,           /*  27 - PD7 A4 */
    NOT_ON_ADC,  /*  28 - PA6 */
    NOT_ON_ADC,  /*  29 - PM4 */
    NOT_ON_ADC,  /*  30 - PM5 */

    /* pins 31-40 */
    NOT_ON_ADC,  /*  31 - PL3 */
    NOT_ON_ADC,  /*  32 - PL2 */
    NOT_ON_ADC,  /*  33 - PL1 */
    NOT_ON_ADC,  /*  34 - PL0 */
    NOT_ON_ADC,  /*  35 - PL5 */
    NOT_ON_ADC,  /*  36 - PL4 */
    NOT_ON_ADC,  /*  37 - PG0 */
    NOT_ON_ADC,  /*  38 - PF3 */
    NOT_ON_ADC,  /*  39 - PF2 */
    NOT_ON_ADC,  /*  40 - PF1 */

    /* pins 41-50 */
    NOT_ON_ADC,  /*  41  - 3.3V */
    13,          /*  42 - PD2 A13 */
    NOT_ON_ADC,  /*  43 - PP0 */
    NOT_ON_ADC,  /*  44 - PP1 */
    7,           /*  45 - PD4 A7 */
    6,           /*  46 - PD5 A6 */
    NOT_ON_ADC,  /*  47 - PQ0 */
    NOT_ON_ADC,  /*  48 - PP4 */
    NOT_ON_ADC,  /*  49 - PN5 */
    NOT_ON_ADC,  /*  50 - PN4 */

    /* pins 51-60 */
    NOT_ON_ADC,  /*  51 - PM6 */
    NOT_ON_ADC,  /*  52 - PQ1 */
    NOT_ON_ADC,  /*  53 - PP3 */
    NOT_ON_ADC,  /*  54 - PQ3 */
    NOT_ON_ADC,  /*  55 - PQ2 */
    NOT_ON_ADC,  /*  56 - RESET */
    NOT_ON_ADC,  /*  57 - PA7 */
    NOT_ON_ADC,  /*  58 - PP5 */
    NOT_ON_ADC,  /*  59 - PM7 */
    NOT_ON_ADC,  /*  60 - GND */

    /* pins 61-70 */
    NOT_ON_ADC,  /*  61 - 5V */
    NOT_ON_ADC,  /*  62 - GND */
    10,          /*  63 - PB4 A10 */
    11,          /*  64 - PB5 A11 */
    16,          /*  65 - PK0 A16 */
    17,          /*  66 - PK1 A17 */
    18,          /*  67 - PK2 A18 */
    19,          /*  68 - PK3 A19 */
    NOT_ON_ADC,  /*  69 - PA4 */
    NOT_ON_ADC,  /*  70 - PA5 */

    /* pins 71-80 */
    NOT_ON_ADC,  /*  71 - PK7 */
    NOT_ON_ADC,  /*  72 - PK6 */
    NOT_ON_ADC,  /*  73 - PH1 */
    NOT_ON_ADC,  /*  74 - PH0 */
    NOT_ON_ADC,  /*  75 - PM2 */
    NOT_ON_ADC,  /*  76 - PM1 */
    NOT_ON_ADC,  /*  77 - PM0 */
    NOT_ON_ADC,  /*  78 - PK5 */
    NOT_ON_ADC,  /*  79 - PK4 */
    NOT_ON_ADC,  /*  80 - PG1 */
    
    /* virtual pins 81-95 */
    NOT_ON_ADC,  /*  81 - PN1 D1 */
    NOT_ON_ADC,  /*  82 - PN0 D2 */
    NOT_ON_ADC,  /*  83 - PF4 D3 */
    NOT_ON_ADC,  /*  84 - PF0 D4 */
    NOT_ON_ADC,  /*  85 - PJ0 SW1 */
    NOT_ON_ADC,  /*  86 - PJ1 SW2 */
    5,           /*  87 - PD_6 A5 */
    NOT_ON_ADC,  /*  88 - PA_0 JP4 */
    NOT_ON_ADC,  /*  89 - PA_1 JP5 */
    NOT_ON_ADC,  /*  90 - PA_2 */
    NOT_ON_ADC,  /*  91 - PA_3 */
    NOT_ON_ADC,  /*  92 - PL_6 */
    NOT_ON_ADC,  /*  93 - PL_7 */
    NOT_ON_ADC,  /*  94 - PB_0 */
    NOT_ON_ADC,  /*  95 - PB_1 */
};
