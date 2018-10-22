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

#define ARDUINO_MAIN

#include <ti/runtime/wiring/wiring_private.h>
#include "wiring_analog.h"

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMMSP432E4.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCMSP432E4.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include "driverlib/adc.h"
#include <driverlib/gpio.h>
#include "driverlib/sysctl.h"

extern GPIO_PinConfig gpioPinConfigs[];
extern ADCMSP432E4_HWAttrsV1 adcMSP432E4HWAttrs[];
extern ADC_Config ADC_config[];

void stopAnalogWriteFxn(uint8_t);
void stopAnalogReadFxn(uint8_t);

StopFunc stopAnalogWriteFxnPtr = NULL;
StopFunc stopAnalogReadFxnPtr = NULL;

/* Carefully selected hal Timer IDs for tone and servo */
uint32_t toneTimerId = (~0);  /* use Timer_ANY for tone timer */
uint32_t servoTimerId = (~0); /* use Timer_ANY for servo timer */

/*
 * analogWrite() support
 */

PWM_Handle pwmHandles[8];

/*
 * Do whatever is necessary to prepare the digital input pin to be
 * configured in any other mode
 */
void stopDigitalRead(uint8_t pin)
{
}

/*
 * Do whatever is necessary to prepare the digital output pin to be
 * configured in any other mode
 */
 void stopDigitalWrite(uint8_t pin)
{
    digitalRead(pin);
}

/*
 * For the MSP432E4, the timers used for PWM are clocked at 12MHz.
 * The period is set to 2.04ms in the PWM_open() call.
 * The PWM objects are configured for PWM_DUTY_COUNTS mode to minimize
 * the PWM_setDuty() processing overhead.
 * The 2.04ms period yields a period count of 24480.
 * The Arduino analogWrite() API takes a value of 0-255 for the duty cycle.
 * The PWM scale factor is then 24480 / 255 = 96
 */

#define PWM_SCALE_FACTOR 2040/255

void analogWrite(uint8_t pin, int val)
{
    uint8_t pwmIndex;
    uint32_t hwiKey;

    hwiKey = Hwi_disable();

    pwmIndex = digital_pin_to_pwm_index[pin];

    if (digital_pin_to_pin_function[pin] != PIN_FUNC_ANALOG_OUTPUT) {
        /* re-configure pin if possible */
        PWM_Params pwmParams;
        PWM_Handle pwmHandle;

        if (pwmIndex == PWM_NOT_MAPPABLE) {
            Hwi_restore(hwiKey);
            return; /* can't get there from here */
        }

        /* undo pin's current plumbing */
        switch (digital_pin_to_pin_function[pin]) {
            case PIN_FUNC_ANALOG_INPUT:
                stopAnalogRead(pin);
                break;
            case PIN_FUNC_DIGITAL_INPUT:
                stopDigitalRead(pin);
                break;
            case PIN_FUNC_DIGITAL_OUTPUT:
                stopDigitalWrite(pin);
                break;
        }

        /* idempotent */
        PWM_init();

        /* Open the PWM port */
        PWM_Params_init(&pwmParams);

        pwmParams.periodUnits = PWM_PERIOD_US;
        pwmParams.periodValue = 2040; /* arduino period is 2.04ms (490Hz) */
        pwmParams.dutyUnits = PWM_DUTY_US;

        pwmHandle = PWM_open(pwmIndex, &pwmParams);

        if (pwmHandle == NULL) {
            Hwi_restore(hwiKey);
            return; /* no available PWM ports */
        }

        pwmHandles[pwmIndex] = pwmHandle;

        /*
         * To reduce footprint when analogWrite isn't used,
         * reference stopAnalogWriteFxn only if analogWrite
         * has been called.
         */
        stopAnalogWriteFxnPtr = stopAnalogWriteFxn;

        digital_pin_to_pin_function[pin] = PIN_FUNC_ANALOG_OUTPUT;

        /* start PWM */
        PWM_start(pwmHandle);
    }

    Hwi_restore(hwiKey);

    PWM_setDuty(pwmHandles[pwmIndex], (val * PWM_SCALE_FACTOR));
}

/*
 * This internal API is used to de-configure a pin that has been
 * put in analogWrite() mode.
 *
 * It will free up the pin's PWM resource after
 * it is no longer being used to support analogWrite() on a different
 * pin. It is called by pinMap() when a pin's function is being modified.
 */
void stopAnalogWrite(uint8_t pin)
{
    stopAnalogWriteFxnPtr(pin);
}

void stopAnalogWriteFxn(uint8_t pin)
{
    uint16_t pwmIndex = digital_pin_to_pwm_index[pin];

    /* Close PWM port */
    PWM_close(pwmHandles[pwmIndex]);

    pwmHandles[pwmIndex] = NULL;
}

/*
 * analogRead() support
 */

int8_t analogReadShift = 2; /* 12 - 2 = 10 bits by default */

/*
 * \brief           configure the A/D reference voltage
 * \param mode      DEFAULT, INTERNAL, EXTERNAL, ...
 * \return          void
 */
void analogReference(uint16_t mode)
{
    uint_fast16_t refVoltage;
    uint8_t i;

    switch (mode) {
        case INTERNAL1V45:
        case INTERNAL1V2:
        case INTERNAL:
        case INTERNAL2V5:
            refVoltage = ADCMSP432E4_VREF_INTERNAL;
            break;

        case DEFAULT:  /* Use VCC as reference (3.3V) */
        case EXTERNAL:
            refVoltage = ADCMSP432E4_VREF_EXTERNAL_3V;
            break;
    }

    /* update all adc HWAttrs accordingly */
    for (i = 0; i < 24; i++) {
        adcMSP432E4HWAttrs[i].refVoltage = refVoltage;
    }
}

/*
 * This internal API is used to de-configure a pin that has been
 * put in analogRead() mode.
 *
 * It is called by pinMap() when a pin's function is
 * being modified.
 */
void stopAnalogRead(uint8_t pin)
{
    stopAnalogReadFxnPtr(pin);
}

void stopAnalogReadFxn(uint8_t pin)
{
    uint8_t adcIndex = digital_pin_to_adc_index[pin];

    /* Close ADC port */
    ADC_close((ADC_Handle)&(ADC_config[adcIndex]));

    digital_pin_to_pin_function[pin] = PIN_FUNC_UNUSED;
}

/*
 * \brief sets the number of bits to shift the value read by ADCFIFORead()
 */
void analogReadResolution(uint16_t bits)
{
    analogReadShift = 12 - bits;
}
