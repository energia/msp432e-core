/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
 *  @file       ADCBufMSP432E4.h
 *
 *  @brief      ADCBuf driver implementation for a MSP432E4 analog-to-digital
 *              converter
 *
 * # Driver include #
 *  The ADCBuf header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/ADCBuf.h>
 *  #include <ti/drivers/adcbuf/ADCBufMSP432E4.h>
 *  @endcode
 *
 *  Refer to @ref ADCBuf.h for a complete description of APIs & example use.
 *
 *  @note This driver is hardcoded to use the TIMER1_BASE hardware peripheral.
 *        This will be fixed in a future release.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_adcbuf_adcbufmsp432e4__include
#define ti_drivers_adcbuf_adcbufmsp432e4__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/devices/msp432e4/driverlib/adc.h>

/*
 *  ADCBuf port/pin defines for pin configuration.  Ports B, D, E, and K are
 *  configurable through the port mapping controller.  None of the port
 *  mappings support ADC.
 *  Channel specifies the ADC channel and ranges from 0 to 19.
 *  pin range: 0 - 7, port range: 0 - 15
 *
 *
 *    15 - 10    19 - 16  15 - 8  7 - 0
 *  ---------------------------------
 *  |  CHANNEL | X | X | PORT | PIN |
 *  ---------------------------------
 *
 *  channel = (((config) >> 10) & 0x1F)
 *  port = (((config) << 4) & 0x40000000)
 *  pin = ((config) & 0xFF)
 *
 *
 */
/* Port B */
#define ADCBufMSP432E4_PB_4_A10 ((10 << 16) | 0x5910) /* ch 10, port B, pin 4 */
#define ADCBufMSP432E4_PB_5_A11 ((11 << 16) | 0x5920) /* ch 11, port B, pin 5 */

/* Port D */
#define ADCBufMSP432E4_PD_0_A15 ((15 << 16) | 0x5B01) /* ch 15, port D, pin 0 */
#define ADCBufMSP432E4_PD_1_A14 ((14 << 16) | 0x5B02) /* ch 14, port D, pin 1 */
#define ADCBufMSP432E4_PD_2_A13 ((13 << 16) | 0x5B04) /* ch 13, port D, pin 2 */
#define ADCBufMSP432E4_PD_3_A12 ((12 << 16) | 0x5B08) /* ch 12, port D, pin 3 */
#define ADCBufMSP432E4_PD_4_A7  ((7 << 16) | 0x5B10)  /* ch 7, port D, pin 4 */
#define ADCBufMSP432E4_PD_5_A6  ((6 << 16) | 0x5B20)  /* ch 6, port D, pin 5 */
#define ADCBufMSP432E4_PD_6_A5  ((5 << 16) | 0x5B40)  /* ch 5, port D, pin 6 */
#define ADCBufMSP432E4_PD_7_A4  ((4 << 16) | 0x5B80)  /* ch 4, port D, pin 7 */

/* Port E */
#define ADCBufMSP432E4_PE_0_A3  ((3 << 16) | 0x5C01) /* ch 3, port E, pin 0 */
#define ADCBufMSP432E4_PE_1_A2  ((2 << 16) | 0x5C02) /* ch 2, port E, pin 1 */
#define ADCBufMSP432E4_PE_2_A1  ((1 << 16) | 0x5C04) /* ch 1, port E, pin 2 */
#define ADCBufMSP432E4_PE_3_A0  ((0 << 16) | 0x5C08) /* ch 0, port E, pin 3 */
#define ADCBufMSP432E4_PE_4_A9  ((9 << 16) | 0x5C10) /* ch 9, port E, pin 4 */
#define ADCBufMSP432E4_PE_5_A8  ((8 << 16) | 0x5C20) /* ch 8, port E, pin 5 */
#define ADCBufMSP432E4_PE_6_A20  ((20 << 16) | 0x5C40) /* ch 20, port E, pin 6 */
#define ADCBufMSP432E4_PE_7_A21  ((21 << 16) | 0x5C80) /* ch 21, port E, pin 7 */

/* Port K */
#define ADCBufMSP432E4_PK_0_A16 ((16 << 16) | 0x6101) /* ch 16, port K, pin 0 */
#define ADCBufMSP432E4_PK_1_A17 ((17 << 16) | 0x6102) /* ch 17, port K, pin 1 */
#define ADCBufMSP432E4_PK_2_A18 ((18 << 16) | 0x6104) /* ch 18, port K, pin 2 */
#define ADCBufMSP432E4_PK_3_A19 ((19 << 16) | 0x6108) /* ch 19, port K, pin 3 */

/* Port P */
#define ADCBufMSP432E4_PP_6_A22  ((22 << 16) | 0x6540) /* ch 22, port P, pin 6 */
#define ADCBufMSP432E4_PP_7_A23  ((23 << 16) | 0x6580) /* ch 23, port P, pin 7 */

 /*
 * =============================================================================
 * Constants
 * =============================================================================
 */
#define ADCBufMSP432E4_PIN_NONE 0
#define SSFIFO_BASE 0x48
#define SSFIFO_OFFSET 0x20

/* Number of available ADC channels on device */
#define MSP432E4_NUM_ADC_CHANNELS (24)

/* ADC function table pointer */
extern const ADCBuf_FxnTable ADCBufMSP432E4_fxnTable;

typedef enum ADCBufMSP432E4_Sequencer {
    ADCBufMSP432E4_Seq_0 = 0,
    ADCBufMSP432E4_Seq_1 = 1,
    ADCBufMSP432E4_Seq_2 = 2,
    ADCBufMSP432E4_Seq_3 = 3
} ADCBufMSP432E4_Sequencer;

/*!
 *  @brief  ADCBufMSP432E4 Internal Source Mode
 *  These fields are used by ADCBufMSP432E4_HWAttrs to specify if a internal
 *  source mode is selected, i.e. temperature sensor
 */
typedef enum ADCBufMSP432E4_InternalSourceMode {
    ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF = 0,
    ADCBufMSP432E4_TEMPERATURE_MODE = ADC_CTL_TS,
} ADCBufMSP432E4_InternalSourceMode;

/*!
 *  @brief  ADCBufMSP432E4 Differential Mode
 *  These fields are used by ADCBufMSP432E4_HWAttrs to specify if ADC
 *  differential sampling mode is selected
 */
typedef enum ADCBufMSP432E4_DifferentialMode {
    ADCBufMSP432E4_SINGLE_ENDED = 0,
    ADCBufMSP432E4_DIFFERENTIAL = ADC_CTL_D
} ADCBufMSP432E4_DifferentialMode;

/*!
 *  @brief  ADCBufMSP432E4 trigger source
 *  These fields are used by ADCBufMSP432E4_HWAttrs to specify the trigger source for the ADC
 *
 */
typedef enum ADCBufMSP432E4_TriggerSource {
    ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER = ADC_TRIGGER_ALWAYS,
    ADCBufMSP432E4_TIMER_TRIGGER = ADC_TRIGGER_TIMER,
}ADCBufMSP432E4_TriggerSource;

/*!
 *  @brief  ADCBufMSP432E4 phase delay
 *  These fields are used by ADCBufMSP432E4_HWAttrs to specify the phase delay for the ADC module
 *
 */
typedef enum ADCBufMSP432E4_Phase {
    ADCBufMSP432E4_Phase_Delay_0 = ADC_PHASE_0,
    ADCBufMSP432E4_Phase_Delay_22_5 = ADC_PHASE_22_5,
    ADCBufMSP432E4_Phase_Delay_45 = ADC_PHASE_45,
    ADCBufMSP432E4_Phase_Delay_67_5 = ADC_PHASE_67_5,
    ADCBufMSP432E4_Phase_Delay_90 = ADC_PHASE_90,
    ADCBufMSP432E4_Phase_Delay_112_5 = ADC_PHASE_112_5,
    ADCBufMSP432E4_Phase_Delay_135 = ADC_PHASE_135,
    ADCBufMSP432E4_Phase_Delay_157_5 = ADC_PHASE_157_5,
    ADCBufMSP432E4_Phase_Delay_180 = ADC_PHASE_180,
    ADCBufMSP432E4_Phase_Delay_202_5 = ADC_PHASE_202_5,
    ADCBufMSP432E4_Phase_Delay_225 = ADC_PHASE_225,
    ADCBufMSP432E4_Phase_Delay_247_5 = ADC_PHASE_247_5,
    ADCBufMSP432E4_Phase_Delay_270 = ADC_PHASE_270,
    ADCBufMSP432E4_Phase_Delay_292_5 = ADC_PHASE_292_5,
    ADCBufMSP432E4_Phase_Delay_315 = ADC_PHASE_315,
    ADCBufMSP432E4_Phase_Delay_337_5 = ADC_PHASE_337_5,
} ADCBufMSP432E4_Phase;

/*!
 *  @brief  ADCMSP432E4 sampling duration
 *  These fields define the MSP432E4 ADC sampling duration (sample and hold time)
 *  in the pulse width unit. User can specify the differnt sampling duration
 *  in the ADCBufMSP432E4_ParamsExtension when opening the ADC.
 *
 */
typedef enum ADCBufMSP432E4_SamplingDuration {
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_4 = ADC_CTL_SHOLD_4,
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_8 = ADC_CTL_SHOLD_8,
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_16 = ADC_CTL_SHOLD_16,
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_32 = ADC_CTL_SHOLD_32,
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_64 = ADC_CTL_SHOLD_64,
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_128 = ADC_CTL_SHOLD_128,
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_256 = ADC_CTL_SHOLD_256
} ADCBufMSP432E4_SamplingDuration;

/*!
 *  @brief  ADCBufMSP432E4 reference source
 *  These fields are used by ADCBufMSP432E4_HWAttrs to specify the reference voltage
 *  for each channel.
 *
 */
typedef enum ADCBufMSP432E4_ReferenceSource {
    ADCBufMSP432E4_VREF_INTERNAL = ADC_REF_INT,
    ADCBufMSP432E4_VREF_EXTERNAL_3V = ADC_REF_EXT_3V
}ADCBufMSP432E4_ReferenceSource;

/*!
 *  @brief      MSP432E4 specfic extension to ADCBuf_Params
 *
 *  To use non-default MSP432E4 specific parameters when calling ADCBuf_open(), a pointer
 *  to an instance of this struct must be specified in ADCBuf_Params::custom. Alternatively,
 *  these values can be set using the control function after calling ADCBuf_open().
 */
typedef struct ADCBufMSP432E4_ParamsExtension{
    ADCBufMSP432E4_SamplingDuration samplingDuration;   /*! ADC sampling duration (sample&hold time), unit is pulse width */
} ADCBufMSP432E4_ParamsExtension;

/*!
 *  @brief  ADCBufMSP432E4 Channel setting
 *  These fields define channel-specific settings: GPIO, reference voltage, temperature mode, etc. These settings
 *  happen when ADCBuf_convert() is called.
 *
 */
typedef struct ADCBufMSP432E4_Channels {
    uint_fast16_t adcPin;                                  /*!< ADC pin, port channel */
    ADCBufMSP432E4_Sequencer adcSequence;                  /*!< ADC sequencer */
    ADCBufMSP432E4_DifferentialMode adcInputMode;          /*!< ADC differential mode option */
    uint_fast16_t adcDifferentialPin;                      /*!< ADC differential pin, if adcInputMode = 1, specify pin */
    ADCBufMSP432E4_InternalSourceMode adcInternalSource;   /*!< ADC internal source mode select */
    uint32_t refVoltage;                                   /*!< ADC reference voltage in microVolts */
} ADCBufMSP432E4_Channels;

/*!
 *  @brief  ADCBufMSP432E4 Hardware attributes
 *  These fields are populated by PinMux tool but user is allowed to change for
 *  different channels setting.
 *
 *  A sample structure is shown below:
 *  @code
 *  ADCBufMSP432E4_Channels adcBufMSP432E4Channels[] = {
 *      {
 *       .adcPin = ADCBufMSP432E4_PE_3_A0,
 *       .adcSequence = ADCBufMSP432E4_Seq_0,
 *       .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
 *       .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
 *       .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
 *       .refVoltage = 3300000
 *      }
 *  };
 *  const ADCBufMSP432E4_HWAttrs adcbufMSP432E4HWAttrs[] = {
 *  {
 *       .intPriority = ~0,
 *       .adcBase = ADC0_BASE,
 *       .channelSetting = adcBuf0MSP432E4Channels,
 *       .sequencePriority = priorities,
 *       .adcTriggerSource = triggerSource,
 *       .modulePhase =  ADCBufMSP432E4_Phase_Delay_0,
 *       .refSource = ADCBufMSP432E4_VREF_INTERNAL,
 *       .useDMA = 1,
 *  }
};
 *  @endcode
 */

typedef struct ADCBufMSP432E4_HWAttrs {
    uint32_t intPriority; /*!< ADC interrupt priority*/
    uint32_t adcBase; /*!< ADC Module Base */
    ADCBufMSP432E4_Channels *channelSetting; /*!< ADC channel specific setting */
    uint8_t *sequencePriority; /*!< Array of ADC sequence Priorities for sequencer 0-3, selecting a priority > 3 disables sequencer */
    ADCBufMSP432E4_TriggerSource *adcTriggerSource; /*!< ADC Trigger Source Array for Sequencers 0-3*/
    ADCBufMSP432E4_Phase modulePhase; /*!< ADC phase delay*/
    ADCBufMSP432E4_ReferenceSource refSource; /*!< ADC reference source */
    uint8_t useDMA; /*!< Enable DMA for ADC */
} ADCBufMSP432E4_HWAttrs;

/*!
 *  @brief  ADCBufMSP432E4 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct ADCBufMSP432E4_Object {
    SemaphoreP_Handle mutex;                /* Grants exclusive access to ADC */
    SemaphoreP_Handle convertComplete;      /* Notify finished ADC convert */
    HwiP_Handle       sequencerHwiHandles[4]; /* Hardware interrupt handles, one per sequencer */

    uint_fast8_t        pingpongFlag[4];          /* PingPong flag indicates which sample buffer is active in the conversion */
    uint_fast8_t        channelCount[4];          /* Count of sampling channels */
    ADCBuf_Conversion   *conversions[4];

    ADCBuf_Callback     callBackFxn;           /* Callback function pointer */

    uint16_t            *conversionSampleBuf[4];
    uint_fast16_t        conversionSampleIdx[4];        /* Internal dec. conversion buffer counter */
    uint_fast16_t        conversionSampleCount[4];      /* Total sampling count per channel */

    ADCBufMSP432E4_SamplingDuration  samplingDuration;           /*!< ADC sampling duration */
    uint32_t                        semaphoreTimeout;           /*!< Timeout for read semaphore in ::ADCBuf_RETURN_MODE_BLOCKING */
    uint32_t                        samplingFrequency;          /*!< Frequency in Hz at which the ADC is triggered */
    ADCBuf_Recurrence_Mode          recurrenceMode;             /*!< Should we convert continuously or one-shot */
    ADCBuf_Return_Mode              returnMode;                 /*!< Mode for all conversions */

    bool                 isOpen;               /* To determine if the ADC is open */
    UDMAMSP432E4_Handle    dmaHandle;
} ADCBufMSP432E4_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_adcbuf_ADCBufMSP432E4__include */
