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

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufMSP432E4.h>
#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>

/* driverlib header files */
#include <ti/devices/msp432e4/driverlib/driverlib.h>

#define pinConfigChannel(config) (((config) >> 16) & 0x1F)
#define pinConfigPort(config) (((config << 4) & 0x000FF000) | 0x40000000)
#define pinConfigPin(config) ((config) & 0xFF)

void ADCBufMSP432E4_close(ADCBuf_Handle handle);
int_fast16_t ADCBufMSP432E4_control(ADCBuf_Handle handle, uint_fast16_t cmd, void * arg);
void ADCBufMSP432E4_init(ADCBuf_Handle handle);
ADCBuf_Handle ADCBufMSP432E4_open(ADCBuf_Handle handle, const ADCBuf_Params *params);
int_fast16_t ADCBufMSP432E4_convert(ADCBuf_Handle handle, ADCBuf_Conversion *conversions, uint_fast8_t channelCount);
int_fast16_t ADCBufMSP432E4_convertCancel(ADCBuf_Handle handle);
uint_fast8_t ADCBufMSP432E4_getResolution(ADCBuf_Handle handle);
int_fast16_t ADCBufMSP432E4_adjustRawValues(ADCBuf_Handle handle, void *sampleBuffer,
        uint_fast16_t sampleCount, uint32_t adcChannel);
int_fast16_t ADCBufMSP432E4_convertAdjustedToMicroVolts(ADCBuf_Handle handle, uint32_t adcChannel,
        void *adjustedSampleBuffer, uint32_t outputMicroVoltBuffer[], uint_fast16_t sampleCount);
static bool initHw(ADCBufMSP432E4_Object *object, ADCBufMSP432E4_HWAttrs const *hwAttrs);
static int_fast16_t configDMA(ADCBufMSP432E4_Object *object,
        ADCBufMSP432E4_HWAttrs const *hwAttrs, ADCBuf_Conversion *conversions);
static void completeConversion(ADCBuf_Handle handle, uint8_t sequencer);
static int_fast16_t primeConvert(ADCBufMSP432E4_Object *object,
        ADCBufMSP432E4_HWAttrs const *hwAttrs, ADCBuf_Conversion *conversions,
        uint_fast8_t channelCount);
static void blockingConvertCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
        void *activeADCBuffer, uint32_t completedChannel);
void ADCBufMSP432E4_hwiIntFxn(uintptr_t arg);
void ADCBufMSP423E4_noDMAhwiIntFxn(uintptr_t arg);
static uint_fast16_t ADCBufMSP432E4_getPowerResourceId(uint32_t port);

/* Global mutex ensuring exclusive access to ADC during conversions */
static SemaphoreP_Handle globalMutex[2] = {NULL, NULL};

/* Table of the maximum channels allowed per sequencer */
static const uint8_t adcMaxSamples[4] = {8, 4, 4, 1};

/* Table of ADC sequencer interrupt vectors */
static const uint8_t adcInterrupts[2][4] = {
    {INT_ADC0SS0, INT_ADC0SS1, INT_ADC0SS2, INT_ADC0SS3},
    {INT_ADC1SS0, INT_ADC1SS1, INT_ADC1SS2, INT_ADC1SS3}
};

/* Table of ADC sequencer UDMA channels */
static const uint32_t udmaChannels[2][4] = {
    {UDMA_CH14_ADC0_0, UDMA_CH15_ADC0_1, UDMA_CH16_ADC0_2, UDMA_CH17_ADC0_3},
    {UDMA_CH24_ADC1_0, UDMA_CH25_ADC1_1, UDMA_CH26_ADC1_2, UDMA_CH27_ADC1_3}
 };

/* ADC function table for ADCBufMSP432E4 implementation */
const ADCBuf_FxnTable ADCBufMSP432E4_fxnTable = {
    ADCBufMSP432E4_close,
    ADCBufMSP432E4_control,
    ADCBufMSP432E4_init,
    ADCBufMSP432E4_open,
    ADCBufMSP432E4_convert,
    ADCBufMSP432E4_convertCancel,
    ADCBufMSP432E4_getResolution,
    ADCBufMSP432E4_adjustRawValues,
    ADCBufMSP432E4_convertAdjustedToMicroVolts
};

extern const ADCBuf_Params ADCBuf_defaultParams;

/*
 *  ======== blockingConvertCallback ========
 */
static void blockingConvertCallback(ADCBuf_Handle handle,
        ADCBuf_Conversion *conversion, void *activeADCBuffer,
   uint32_t completedChannel)
{
    ADCBufMSP432E4_Object *object = handle->object;

    DebugP_log0("ADCBuf: posting transferComplete semaphore");

    /* Indicate transfer complete */
    SemaphoreP_post(object->convertComplete);
}

/*
 *  ======== completeConversion ========
 */
static void completeConversion(ADCBuf_Handle handle, uint8_t sequencer)
{
    ADCBufMSP432E4_Object        *object = handle->object;

    /* Perform callback in a HWI context. The callback ideally is invoked in
     * SWI instead of HWI. This should get called once per sequencer group */
    object->callBackFxn(handle, &object->conversions[sequencer][0],
        (!object->pingpongFlag[sequencer]) ? object->conversions[sequencer][0].sampleBuffer : object->conversions[sequencer][0].sampleBufferTwo,
        object->conversions[sequencer][0].adcChannel);

    if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS) {
        /* Toggle the pingpong flag */
        object->pingpongFlag[sequencer] ^= 1;

        /* Reset sample index */
        object->conversionSampleIdx[sequencer] = 0;
        /* Toggle the pingpong flag */
        if (!object->pingpongFlag[sequencer]) {
            object->conversionSampleBuf[sequencer] = object->conversions[sequencer]->sampleBuffer;
        }
        else {
            object->conversionSampleBuf[sequencer] = object->conversions[sequencer]->sampleBufferTwo;
        }
    }
    else {
        /* Clear the object conversions if in the one shot mode */
        object->conversions[sequencer] = NULL;
    }
}

/*
 *  ======== initHW ========
 *
 *  Configures ADC peripheral
 */
static bool initHw(ADCBufMSP432E4_Object *object,
        ADCBufMSP432E4_HWAttrs const *hwAttrs)
{
    uint8_t i;
    /* Configure system clock */
    Power_setDependency(ADCBufMSP432E4_getPowerResourceId(hwAttrs->adcBase));

    /* Initialize trigger source */
    for (i=0; i<4; i++) {
        if (hwAttrs->sequencePriority[i] < 4) {
            MAP_ADCSequenceConfigure(hwAttrs->adcBase, i, hwAttrs->adcTriggerSource[i], hwAttrs->sequencePriority[i]);

            /* do trigger source specific init*/
            if (hwAttrs->adcTriggerSource[i] == ADCBufMSP432E4_TIMER_TRIGGER) {
                Power_setDependency(PowerMSP432E4_PERIPH_TIMER1);
                MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);
                MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, 120000000/object->samplingFrequency);
                MAP_TimerADCEventSet(TIMER1_BASE, TIMER_ADC_TIMEOUT_A);
                MAP_TimerControlTrigger(TIMER1_BASE, TIMER_A, true);
            }
        }
    }
    /* Set the ADC reference voltage */
    ADCReferenceSet(hwAttrs->adcBase, hwAttrs->refSource);

    /* Set ADC phase delay */
    ADCPhaseDelaySet(hwAttrs->adcBase, hwAttrs->modulePhase);

    return (ADCBuf_STATUS_SUCCESS);
}

/*
 *  ======== ADCBufMSP432E4_init ========
 */
void ADCBufMSP432E4_init(ADCBuf_Handle handle)
{
    ADCBufMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t         key;
    SemaphoreP_Handle sem0;
    SemaphoreP_Handle sem1;

    /* speculatively create a binary semaphore for thread safety per ADC module */
    sem0 = SemaphoreP_createBinary(1);
    sem1 = SemaphoreP_createBinary(1);
    /* sem == NULL will be detected in 'open' */

    key = HwiP_disable();

    /* Create Semaphore for ADC0 */
    if (globalMutex[0] == NULL) {
        /* use the binary sem created above */
        globalMutex[0] = sem0;

        HwiP_restore(key);
    }
    else {
        /* init already called */
        HwiP_restore(key);

        if (sem0) {
            /* delete unused Semaphore */
            SemaphoreP_delete(sem0);
        }
    }

    key = HwiP_disable();

    /* Create Semaphore for ADC1 */
    if (globalMutex[1] == NULL) {
        /* use the binary sem created above */
        globalMutex[1] = sem1;

        HwiP_restore(key);
    }
    else {
        /* init already called */
        HwiP_restore(key);

        if (sem1) {
            /* delete unused Semaphore */
            SemaphoreP_delete(sem1);
        }
    }

    /* Initialize UDMA peripheral */
    if (hwAttrs->useDMA) {
        UDMAMSP432E4_init();
    }
}

/*
 *  ======== configDMA ========
 *  This functions configures the DMA to automatically transfer ADC
 *  output data into a provided array
 *
 *  @pre    ADCBufMSP432E4_open() has to be called first.
 *
 *  @pre    There must not currently be a conversion in progress
 *
 *  @pre    Function assumes that the handle and transaction is not NULL
 *
 *  @param  object An ADCBufMSP432 handle->object returned from ADCBufMSP432E4_open()
 *
 *  @param  hwAttrs An ADCBufMSP432 handle->hwAttrs from board file
 *
 *  @param  conversion A pointer to an ADCBuf_Conversion
 *
 */
static int_fast16_t configDMA(ADCBufMSP432E4_Object *object,
    ADCBufMSP432E4_HWAttrs const *hwAttrs, ADCBuf_Conversion *conversion)
{
    uint_fast8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    uint_fast8_t sequencer = hwAttrs->channelSetting[conversion[0].adcChannel].adcSequence;

    /* Enable the DMA request from selected ADC base and sequencer  */
    MAP_ADCSequenceDMAEnable(hwAttrs->adcBase, sequencer);

    /* Map the ADC sequencer to corresponding DMA channel*/
    uDMAChannelAssign(udmaChannels[base][sequencer]);

    /* Put the attributes in a known state for the ADC sequencer uDMA
     * channel. These should already be disabled by default. */
    MAP_uDMAChannelAttributeDisable(udmaChannels[base][sequencer],
                        UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                        UDMA_ATTR_HIGH_PRIORITY |
                        UDMA_ATTR_REQMASK);

    /* Configure the control parameters for the primary control structure for
     * the ADC sequencer channel. The primary control structure is used for
     * copying the data from the ADC Sequencer FIFO to conversion->sampleBuffer. The transfer
     * data size is 16 bits and the source address is not incremented while
     * the destination address is incremented at 16-bit boundary.
     */
    MAP_uDMAChannelControlSet(udmaChannels[base][sequencer] | UDMA_PRI_SELECT,
                        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                        UDMA_ARB_1);

    /* Set up the transfer parameters for the ADC Sequencer primary control
     * structure. The mode is pingpong mode so it will run to continuously. */
    MAP_uDMAChannelTransferSet(udmaChannels[base][sequencer] | UDMA_PRI_SELECT,
                        UDMA_MODE_PINGPONG,
                        (void *) (hwAttrs->adcBase + SSFIFO_BASE + sequencer*SSFIFO_OFFSET),
                        (void *) conversion->sampleBuffer,
                        conversion->samplesRequestedCount);

    /* Set up transfer parameters for ADC Sequencer alternate control structure */
    MAP_uDMAChannelControlSet(udmaChannels[base][sequencer] | UDMA_ALT_SELECT,
                        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                        UDMA_ARB_1);
    MAP_uDMAChannelTransferSet(udmaChannels[base][sequencer] | UDMA_ALT_SELECT,
                        UDMA_MODE_PINGPONG,
                        (void *) (hwAttrs->adcBase + SSFIFO_BASE + sequencer*SSFIFO_OFFSET),
                        (void *) conversion->sampleBufferTwo,
                        conversion->samplesRequestedCount);

    /* The uDMA ADC Sequencer channel is primed to start a transfer. As
     * soon as the channel is enabled and the Timer will issue an ADC trigger,
     * the ADC will perform the conversion and send a DMA Request. The data
     * transfers will begin. */
    MAP_uDMAChannelEnable(udmaChannels[base][sequencer]);

    return (ADCBuf_STATUS_SUCCESS);
}

/*
 *  ======== ADCBufCCMSP432E4_adjustRawValues ========
 */
int_fast16_t ADCBufMSP432E4_adjustRawValues(ADCBuf_Handle handle,
    void *sampleBuffer, uint_fast16_t sampleCount, uint32_t adcChannel)
{
    /* This hardware peripheral does not support Calibration */
    return (ADCBuf_STATUS_UNSUPPORTED);
}

/*
 *  ======== ADCBufMSP432E4_close ========
 */
void ADCBufMSP432E4_close(ADCBuf_Handle handle)
{
    uintptr_t         key;
    ADCBufMSP432E4_Object        *object = handle->object;
    ADCBufMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint8_t i;
    uint8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;

    key = HwiP_disable();

    /* Disable interrupts and the ADC */
    MAP_ADCIntDisableEx(hwAttrs->adcBase, 0xFFF);

    for (i = 0; i < 4; i++) {
        if (hwAttrs->useDMA) {
            MAP_ADCSequenceDMADisable(hwAttrs->adcBase, i);
            uDMAChannelDisable(udmaChannels[base][i]);
        }
        else {
            MAP_ADCSequenceDisable(hwAttrs->adcBase, i);
        }

        /* Destruct driver resources */
        if (object->sequencerHwiHandles[i]) {
            HwiP_delete(object->sequencerHwiHandles[i]);
            object->sequencerHwiHandles[i] = NULL;
        }
    }

    HwiP_restore(key);

    if (object->convertComplete) {
        SemaphoreP_delete(object->convertComplete);
    }

    if (hwAttrs->useDMA && object->dmaHandle) {
        UDMAMSP432E4_close(object->dmaHandle);
        object->dmaHandle = NULL;
    }

    object->isOpen = false;

    DebugP_log0("ADCBuf: Object closed.");

}

/*
 *  ======== ADCBufMSP432E4_control ========
 */
int_fast16_t ADCBufMSP432E4_control(ADCBuf_Handle handle, uint_fast16_t cmd, void * arg)
{
    /* No implementation yet */
    return (ADCBuf_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== primeConvert ========
 */
static int_fast16_t primeConvert(ADCBufMSP432E4_Object *object,
        ADCBufMSP432E4_HWAttrs const *hwAttrs, ADCBuf_Conversion *conversions,
        uint_fast8_t channelCount)
{
    uint_fast8_t i=0;
    uint32_t channel;
    uint32_t port;
    uint8_t pin;
    uint_fast16_t powerID;
    uint_fast8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    uint8_t sequencer = hwAttrs->channelSetting[conversions[0].adcChannel].adcSequence;
    uint32_t refVoltage = hwAttrs->channelSetting[conversions[0].adcChannel].refVoltage;

    if ((channelCount > adcMaxSamples[sequencer]) || (hwAttrs->sequencePriority[sequencer] > 4)) {
        return (ADCBuf_STATUS_ERROR);
    }

    /* Store the conversions struct array into object */
    object->conversions[sequencer] = conversions;
    /* Store the channel count into object */
    object->channelCount[sequencer] = channelCount;

    /* Store the samples count into object - one channel*/
    object->conversionSampleBuf[sequencer] = conversions->sampleBuffer;
    object->conversionSampleCount[sequencer] = conversions->samplesRequestedCount;
    object->conversionSampleIdx[sequencer] = 0;

    /* Initialize GPIOs and Temperature mode*/
    for (i=0; i < channelCount; i++) {
        if (hwAttrs->channelSetting[conversions[i].adcChannel].refVoltage != refVoltage) {
            return (ADCBuf_STATUS_ERROR);
        }

        channel = pinConfigChannel(hwAttrs->channelSetting[conversions[i].adcChannel].adcPin);
        port = pinConfigPort(hwAttrs->channelSetting[conversions[i].adcChannel].adcPin);
        pin = pinConfigPin(hwAttrs->channelSetting[conversions[i].adcChannel].adcPin);

        /* If using temperature mode, skip GPIO initialization and use ADC_CTL_TS instead of channel */
        if (hwAttrs->channelSetting[conversions[i].adcChannel].adcInternalSource == ADCBufMSP432E4_TEMPERATURE_MODE) {
            channel = ADCBufMSP432E4_TEMPERATURE_MODE;
        }
        else {
            powerID = ADCBufMSP432E4_getPowerResourceId(port);
            if (powerID < PowerMSP432E4_NUMRESOURCES) {
                Power_setDependency(powerID);
            }
            else {
                return (ADCBuf_STATUS_ERROR);
            }
            GPIOPinTypeADC(port, pin);
        }

        /* Initialize differential pin channel for differential mode*/
        if (hwAttrs->channelSetting[conversions[i].adcChannel].adcInputMode == ADCBufMSP432E4_DIFFERENTIAL) {
            MAP_GPIOPinTypeADC(pinConfigPort(hwAttrs->channelSetting[conversions[i].adcChannel].adcDifferentialPin),
                pinConfigPin(hwAttrs->channelSetting[conversions[i].adcChannel].adcDifferentialPin));
            channel = channel / 2;
            channel = channel | ADCBufMSP432E4_DIFFERENTIAL;
        }

        if (i == channelCount - 1) {
            channel |= ADC_CTL_IE | ADC_CTL_END;
        }
        MAP_ADCSequenceStepConfigure(hwAttrs->adcBase, sequencer, i, channel|object->samplingDuration);
    }

    /* If DMA enabled, setup peripheral*/
    if (hwAttrs->useDMA) {
        MAP_ADCIntClearEx(hwAttrs->adcBase, ADC_INT_DMA_SS0 << sequencer);
        MAP_ADCIntEnableEx(hwAttrs->adcBase, ADC_INT_DMA_SS0 <<sequencer);

        configDMA(object, hwAttrs, conversions);
    }
    else {
        MAP_ADCIntClear(hwAttrs->adcBase, sequencer);
        MAP_ADCIntEnable(hwAttrs->adcBase, sequencer);
    }

    /* Since sample sequence is now configured, it must be enabled. */
    MAP_ADCSequenceEnable(hwAttrs->adcBase, sequencer);

    /* Enable the Interrupt generation from the specified ADC Sequencer */
    MAP_IntEnable(adcInterrupts[base][sequencer]);

    /* Enable timer if using timer trigger */
    if (hwAttrs->adcTriggerSource[sequencer] == ADCBufMSP432E4_TIMER_TRIGGER) {
        MAP_TimerEnable(TIMER1_BASE, TIMER_A);
    }

    return (ADCBuf_STATUS_SUCCESS);
}

/*
 *  ======== ADCBufMSP432E4_convert ========
 */
int_fast16_t ADCBufMSP432E4_convert(ADCBuf_Handle handle,
    ADCBuf_Conversion *conversions,
    uint_fast8_t channelCount)
{
    ADCBufMSP432E4_Object        *object = handle->object;
    ADCBufMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    int_fast16_t ret;

    /* Acquire the lock for this particular ADC handle */
    SemaphoreP_pend(object->mutex, SemaphoreP_WAIT_FOREVER);

    /* Execute core conversion */
    ret = primeConvert(object, hwAttrs, conversions, channelCount);
    if (ret == ADCBuf_STATUS_ERROR) {
        SemaphoreP_post(object->mutex);
        return (ret);
    }

    if (object->returnMode == ADCBuf_RETURN_MODE_BLOCKING) {
        DebugP_log0("ADCBuf: Pending on transferComplete semaphore");
        /*
         * Wait for the transfer to complete here.
         * It's OK to block from here because the ADC's Hwi will unblock
         * upon errors
         */
        if (SemaphoreP_OK != SemaphoreP_pend(object->convertComplete,
                    object->semaphoreTimeout)) {
            DebugP_log0("ADCBuf: Convert timeout");

            ret = ADCBuf_STATUS_ERROR;
        }
        else {
            DebugP_log0("ADCBuf: Convert completed");

            ret = ADCBuf_STATUS_SUCCESS;
        }
    }
    else {
        /* Always return true if in Asynchronous mode */
        ret = ADCBuf_STATUS_SUCCESS;
    }

    /* Release the lock for this particular ADC handle */
    SemaphoreP_post(object->mutex);

    /* Return the number of bytes transfered by the ADC */
    return (ret);
}

/*
 *  ======== ADCBufMSP432E4_convertAdjustedToMicroVolts ========
 */
int_fast16_t ADCBufMSP432E4_convertAdjustedToMicroVolts(ADCBuf_Handle handle,
    uint32_t adcChannel, void *adjustedSampleBuffer,
    uint32_t outputMicroVoltBuffer[], uint_fast16_t sampleCount)
{
    uint32_t i;
    ADCBufMSP432E4_HWAttrs const  *hwAttrs = handle->hwAttrs;
    uint32_t refVoltage = hwAttrs->channelSetting[adcChannel].refVoltage;
    uint16_t *adjustedRawSampleBuf = (uint16_t *) adjustedSampleBuffer;

    for (i = 0; i < sampleCount; i++) {
        if (adjustedRawSampleBuf[i] == 0xFFF) {
            outputMicroVoltBuffer[i] = refVoltage;
        }
        else {
            outputMicroVoltBuffer[i] = ((uint32_t)adjustedRawSampleBuf[i] * (refVoltage / 0x1000));
        }
    }

    return (ADCBuf_STATUS_SUCCESS);
}

/*
 *  ======== ADCBufMSP432E4_convertCancel ========
 */
int_fast16_t ADCBufMSP432E4_convertCancel(ADCBuf_Handle handle)
{
    return (ADCBuf_STATUS_SUCCESS);
}

/*
 *  ======== ADCBufMSP432E4_getResolution ========
 */
uint_fast8_t ADCBufMSP432E4_getResolution(ADCBuf_Handle handle)
{
    return (12);
}

void ADCBufMSP423E4_noDMAhwiIntFxn(uintptr_t arg)
{
    ADCBufMSP432E4_Object    *object = ((ADCBuf_Handle) arg)->object;
    ADCBufMSP432E4_HWAttrs const  *hwAttrs = ((ADCBuf_Handle) arg)->hwAttrs;

    uint8_t i;
    uint32_t intStatus;
    uint32_t intMask;
    uint32_t adcBuffer[8];
    uint16_t *sampleBuffer;
    uint8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    uint8_t sequencer = 0;

    intMask = ADC_INT_SS0;
    for (sequencer = 0; sequencer < 4; sequencer++) {
        /* Get the interrupt status of the ADC controller*/
        intStatus = MAP_ADCIntStatus(hwAttrs->adcBase, sequencer, true);
        if ((intStatus & intMask) == intMask) {
            MAP_ADCIntClear(hwAttrs->adcBase, sequencer);

            /* get ADC values and store in adcBuffer*/
            MAP_ADCSequenceDataGet(hwAttrs->adcBase, sequencer, adcBuffer);

            sampleBuffer = (!object->pingpongFlag[sequencer]) ?
                (uint16_t*) (object->conversions[sequencer]->sampleBuffer) : (uint16_t*) (object->conversions[sequencer]->sampleBufferTwo);

            /* store sample values in sampleBuffer */
            for (i = 0; i < object->channelCount[sequencer]; i++) {
                sampleBuffer[object->conversionSampleIdx[sequencer]] = (uint16_t) adcBuffer[i];
                object->conversionSampleIdx[sequencer]++;
            }

            /* ADC conversion complete */
            if (object->conversionSampleIdx[sequencer] == object->conversionSampleCount[sequencer]) {
                if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_ONE_SHOT) {
                    MAP_ADCIntDisable(hwAttrs->adcBase, sequencer);
                    MAP_ADCSequenceDisable(hwAttrs->adcBase, sequencer);
                    MAP_IntDisable(adcInterrupts[base][sequencer]);
                    if (hwAttrs->adcTriggerSource[sequencer] == ADCBufMSP432E4_TIMER_TRIGGER) {
                        MAP_TimerDisable(TIMER1_BASE, TIMER_A);
                    }
                }
                completeConversion((ADCBuf_Handle)arg, sequencer);
            }
            else {
                MAP_ADCSequenceEnable(hwAttrs->adcBase, sequencer);
            }
        }
        intMask = intMask << 1;
    }
}

/*
 *  ======== ADCBufMSP432E4_hwiIntFxn ========
 */
void ADCBufMSP432E4_hwiIntFxn(uintptr_t arg)
{
    uint32_t intStatus;
    uint32_t intMask;
    ADCBufMSP432E4_Object    *object = ((ADCBuf_Handle) arg)->object;
    ADCBufMSP432E4_HWAttrs const  *hwAttrs = ((ADCBuf_Handle) arg)->hwAttrs;

    uint8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    uint8_t sequencer = 0;

    /* check interrupt status */
    intStatus = MAP_ADCIntStatusEx(hwAttrs->adcBase, true);
    intMask = ADC_INT_DMA_SS0;

    /* clear interrupt flag */
    for (sequencer = 0; sequencer < 4; sequencer ++) {
        uint8_t dmaChannel = udmaChannels[base][sequencer];
        if ((intStatus & intMask) == intMask) {
            MAP_ADCIntClearEx(hwAttrs->adcBase, intMask);
            completeConversion((ADCBuf_Handle) arg, sequencer);

            if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS) {
                /* Switch between primary and alternate buffers for DMA's PingPong mode */
                if (object->pingpongFlag[sequencer] != 0) {
                    MAP_uDMAChannelControlSet(dmaChannel| UDMA_PRI_SELECT,
                        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                        UDMA_ARB_1);

                    MAP_uDMAChannelTransferSet(dmaChannel | UDMA_PRI_SELECT,
                        UDMA_MODE_PINGPONG,
                        (void *)(hwAttrs->adcBase + SSFIFO_BASE + sequencer*SSFIFO_OFFSET),
                        (void *)object->conversions[sequencer]->sampleBuffer,
                        object->conversions[sequencer]->samplesRequestedCount);
                }
                else {
                    MAP_uDMAChannelControlSet(dmaChannel | UDMA_ALT_SELECT,
                        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                        UDMA_ARB_1);
                    MAP_uDMAChannelTransferSet(dmaChannel | UDMA_ALT_SELECT,
                        UDMA_MODE_PINGPONG,
                        (void *)(hwAttrs->adcBase + SSFIFO_BASE + sequencer*SSFIFO_OFFSET),
                        (void *)object->conversions[sequencer]->sampleBufferTwo,
                        object->conversions[sequencer]->samplesRequestedCount);
                }

                MAP_uDMAChannelEnable(dmaChannel);
                MAP_ADCSequenceDMAEnable(hwAttrs->adcBase, sequencer);
                /* Re-enable sample sequence */
                MAP_ADCSequenceEnable(hwAttrs->adcBase, sequencer);
            }
            else {
                /* If using one shot mode, disable interrupts, DMA, and sequencer*/
                MAP_uDMAChannelDisable(dmaChannel);
                MAP_ADCIntDisableEx(hwAttrs->adcBase, intMask);
                MAP_ADCSequenceDisable(hwAttrs->adcBase, sequencer);
                MAP_IntDisable(adcInterrupts[base][sequencer]);

                if (hwAttrs->adcTriggerSource[sequencer] == ADCBufMSP432E4_TIMER_TRIGGER) {
                    MAP_TimerDisable(TIMER1_BASE, TIMER_A);
                }
            }
        }
        intMask = intMask << 1;
    }
}

/*
 *  ======== ADCBufMSP432E4_open ========
 */
ADCBuf_Handle ADCBufMSP432E4_open(ADCBuf_Handle handle,
                                const ADCBuf_Params *params)
{
    uintptr_t                key;
    ADCBufMSP432E4_Object    *object = handle->object;
    ADCBufMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint8_t i=0;
    uint8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    HwiP_Params          hwiParams;

    /* Confirm that 'init' has successfully completed */
    if (globalMutex[base] == NULL){
        ADCBufMSP432E4_init(handle);
        if (globalMutex[base] == NULL) {
            DebugP_log0("ADCBuf: mutex Semaphore_create() failed:.");
            ADCBufMSP432E4_close(handle);
            return (NULL);
        }
    }
    object->mutex = globalMutex[base];

    /* Use defaults if params are NULL. */
    if (params == NULL) {
        params = (ADCBuf_Params *) &ADCBuf_defaultParams;
    }

    /* Check that a callback is provided if using callback mode */
    DebugP_assert(params->returnMode == ADCBuf_RETURN_MODE_CALLBACK &&
        params->callbackFxn != NULL);

    if (ADCBuf_RETURN_MODE_CALLBACK == params->returnMode) {
        if (params->callbackFxn == NULL) {
            return (NULL);
        }
    }
    /* Check that if it is the callback mode when using continuous mode */
    DebugP_assert((ADCBuf_RECURRENCE_MODE_CONTINUOUS == params->recurrenceMode) &&
            (ADCBuf_RETURN_MODE_CALLBACK == params->returnMode));

    if (params->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS) {
        if (params->returnMode != ADCBuf_RETURN_MODE_CALLBACK) {
            return (NULL);
        }
    }
    key = HwiP_disable();

   if (object->isOpen) {
        HwiP_restore(key);
        DebugP_log0("ADCBuf:Error! Already in use.");
        return (NULL);
    }
    object->isOpen = true;

    HwiP_restore(key);

    /* Creat Hwi object for ADC */
    if (hwAttrs->useDMA) {
        object->dmaHandle = UDMAMSP432E4_open();
        if (object->dmaHandle == NULL) {
            ADCBufMSP432E4_close(handle);
            return (NULL);
        }
    }

    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t) handle;
    hwiParams.priority = hwAttrs->intPriority;

    /* Initialize interrupts for each sequencer if user sets priorty less than 4*/
    for (i = 0; i < 4; i++) {
        if (hwAttrs->sequencePriority[i] < 4) {
            if (hwAttrs->useDMA) {
                object->sequencerHwiHandles[i] = HwiP_create(adcInterrupts[base][i], ADCBufMSP432E4_hwiIntFxn,
                    &hwiParams);
            }
            else {
                object->sequencerHwiHandles[i] = HwiP_create(adcInterrupts[base][i], ADCBufMSP423E4_noDMAhwiIntFxn,
                    &hwiParams);
            }
            if (object->sequencerHwiHandles[i] == NULL) {
                ADCBufMSP432E4_close(handle);
                return (NULL);
            }
        }
        object->pingpongFlag[i] = 0;
    }

    /* Configure driver to Callback or Blocking operating mode */
    if (params->returnMode == ADCBuf_RETURN_MODE_CALLBACK) {
        object->callBackFxn = params->callbackFxn;
        DebugP_log0("ADCBuf: in ADC_MODE_CALLBACK mode");
    }
    else {
        /* Semaphore to block task for the duration of the ADC convert */
        object->convertComplete = SemaphoreP_createBinary(0);
        if (!object->convertComplete) {
            ADCBufMSP432E4_close(handle);
            return (NULL);
        }
        object->callBackFxn = blockingConvertCallback;
        DebugP_log0("ADCBuf: in ADC_MODE_BLOCKING mode");
    }

    /*
     * Store ADC parameters & initialize peripheral.  These are used to
     * re/initialize the peripheral when opened or changing performance level.
     */

    object->returnMode = params->returnMode;
    object->recurrenceMode = params->recurrenceMode;
    object->semaphoreTimeout = params->blockingTimeout;
    object->samplingFrequency = params->samplingFrequency;

    /* Check the ExtensionParam is set */
    if (params->custom) {
        /* If MSP432E4 specific params were specified, use them */
        object->samplingDuration =
            ((ADCBufMSP432E4_ParamsExtension *)(params->custom))->
            samplingDuration;
    }
    else {
        /* Initialize MSP432E4 specific settings to defaults */
        object->samplingDuration = ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_4;
    }

    /* Initialize ADC related hardware */
    if (ADCBuf_STATUS_SUCCESS != initHw(object, hwAttrs)) {
        ADCBufMSP432E4_close(handle);
        return (NULL);
    }

    DebugP_log0("ADCBuf: Object opened.");

    return (handle);
}

static uint_fast16_t ADCBufMSP432E4_getPowerResourceId(uint32_t port) {
    uint_fast16_t resourceID;
    switch (port) {
    case GPIO_PORTB_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOB;
        break;
    case GPIO_PORTD_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOD;
        break;
    case GPIO_PORTE_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOE;
        break;
    case GPIO_PORTK_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOK;
        break;
    case GPIO_PORTP_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOP;
        break;
    case ADC0_BASE:
        resourceID = PowerMSP432E4_PERIPH_ADC0;
        break;
    case ADC1_BASE:
        resourceID = PowerMSP432E4_PERIPH_ADC1;
        break;
    default:
        resourceID = (uint_fast16_t)-1;
        break;
    }
    return (resourceID);
}
