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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/ssi.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/devices/msp432e4/driverlib/udma.h>

#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIMSP432E4DMA.h>

#define MAX_DMA_TRANSFER_AMOUNT (1024)

void SPIMSP432E4DMA_close(SPI_Handle handle);
int_fast16_t SPIMSP432E4DMA_control(SPI_Handle handle, uint_fast16_t cmd,
    void *arg);
void SPIMSP432E4DMA_init(SPI_Handle handle);
static void SPIMSP432E4DMA_hwiFxn(uintptr_t arg);
SPI_Handle SPIMSP432E4DMA_open(SPI_Handle handle, SPI_Params *params);
bool SPIMSP432E4DMA_transfer(SPI_Handle handle, SPI_Transaction *transaction);
void SPIMSP432E4DMA_transferCancel(SPI_Handle handle);

static void blockingTransferCallback(SPI_Handle handle,
    SPI_Transaction *transaction);
static void configDMA(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs, SPI_Transaction *transaction);
static inline uint32_t getDmaRemainingXfers(SPIMSP432E4DMA_HWAttrs const *hwAttrs);
static uint8_t getPowerMgrId(uint32_t baseAddr);
static void initHw(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs);
static void spiPollingTransfer(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs, SPI_Transaction *transaction);

/* SPI function table for SPIMSP432E4DMA implementation */
const SPI_FxnTable SPIMSP432E4DMA_fxnTable = {
    SPIMSP432E4DMA_close,
    SPIMSP432E4DMA_control,
    SPIMSP432E4DMA_init,
    SPIMSP432E4DMA_open,
    SPIMSP432E4DMA_transfer,
    SPIMSP432E4DMA_transferCancel
};

static const uint8_t frameFormat[] = {
    SSI_FRF_MOTO_MODE_0,    /* SPI_POL0_PHA0 */
    SSI_FRF_MOTO_MODE_1,    /* SPI_POL0_PHA1 */
    SSI_FRF_MOTO_MODE_2,    /* SPI_POL1_PHA0 */
    SSI_FRF_MOTO_MODE_3,    /* SPI_POL1_PHA1 */
    SSI_FRF_TI              /* SPI_TI */
};

/*
 * These lookup tables are used to configure the DMA channels for the
 * appropriate (8bit or 16bit) transfer sizes.
 */
 /* Table for an SPI DMA TX channel */
static const uint32_t dmaTxConfig[] = {
    UDMA_SIZE_8  | UDMA_SRC_INC_8    | UDMA_DST_INC_NONE | UDMA_ARB_4, /* 8bit */
    UDMA_SIZE_16 | UDMA_SRC_INC_16   | UDMA_DST_INC_NONE | UDMA_ARB_4  /* 16bit */
};

 /* Table for an SPI DMA RX channel */
static const uint32_t dmaRxConfig[] = {
    UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_8    | UDMA_ARB_4, /* 8bit */
    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16   | UDMA_ARB_4  /* 16bit */
};

/* Table for SPI DMA RX & TX when either buffer is NULL */
static const uint32_t dmaNullConfig[] = {
    UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_4, /* 8bit */
    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_4  /* 16bit */
};

/*
 *  ======== SPIMSP432E4DMA_close ========
 */
void SPIMSP432E4DMA_close(SPI_Handle handle)
{
    uint8_t                       port;
    SPIMSP432E4DMA_Object        *object = handle->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    SSIDisable(hwAttrs->baseAddr);

    if (object->hwiHandle) {
        HwiP_delete(object->hwiHandle);
        object->hwiHandle = NULL;
    }

    if (object->dmaHandle) {
        UDMAMSP432E4_close(object->dmaHandle);
        object->dmaHandle = NULL;
    }

    if (object->transferComplete) {
        SemaphoreP_delete(object->transferComplete);
        object->transferComplete = NULL;
    }

    /* Undo the clk pin */
    GPIOMSP432E4_undoPinConfig(hwAttrs->clkPinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->clkPinMask);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Undo the xdat0 pin */
    GPIOMSP432E4_undoPinConfig(hwAttrs->xdat0PinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->xdat0PinMask);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Undo the xdat1 pin */
    GPIOMSP432E4_undoPinConfig(hwAttrs->xdat1PinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->xdat1PinMask);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Undo the fss pin */
    GPIOMSP432E4_undoPinConfig(hwAttrs->fssPinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->fssPinMask);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Release SPI power dependency */
    Power_releaseDependency(getPowerMgrId(hwAttrs->baseAddr));

    object->isOpen = false;
}

/*
 *  ======== SPIMSP432E4DMA_control ========
 */
int_fast16_t SPIMSP432E4DMA_control(SPI_Handle handle, uint_fast16_t cmd,
    void *arg)
{
    return (SPI_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== SPIMSP432E4DMA_hwiFxn ========
 */
static void SPIMSP432E4DMA_hwiFxn(uintptr_t arg)
{
    SPI_Transaction              *msg;
    SPIMSP432E4DMA_Object        *object = ((SPI_Handle) arg)->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = ((SPI_Handle) arg)->hwAttrs;

    /* RX DMA channel has completed */
    SSIDMADisable(hwAttrs->baseAddr, SSI_DMA_TX | SSI_DMA_RX);
    SSIIntDisable(hwAttrs->baseAddr, SSI_DMARX);
    SSIIntClear(hwAttrs->baseAddr, SSI_DMARX);

    if (object->transaction->count - object->amtDataXferred >
        MAX_DMA_TRANSFER_AMOUNT) {
        /* Data still remaining, configure another DMA transfer */
        object->amtDataXferred += object->currentXferAmt;

        configDMA(object, hwAttrs, object->transaction);
    }
    else {
        /*
         * All data sent; disable peripheral, set status, perform
         * callback & return
         */
        SSIDisable(hwAttrs->baseAddr);
        object->transaction->status = SPI_TRANSFER_COMPLETED;

        /*
         * Use a temporary transaction pointer in case the callback function
         * attempts to perform another SPI_transfer call
         */
        msg = object->transaction;

        /* Indicate we are done with this transfer */
        object->transaction = NULL;

        object->transferCallbackFxn((SPI_Handle) arg, msg);
    }
}

/*
 *  ======== SPIMSP432E4DMA_init ========
 */
void SPIMSP432E4DMA_init(SPI_Handle handle)
{
    UDMAMSP432E4_init();
}

/*
 *  ======== SPIMSP432E4DMA_open ========
 */
SPI_Handle SPIMSP432E4DMA_open(SPI_Handle handle, SPI_Params *params)
{
    uintptr_t                     key;
    uint8_t                       powerMgrId;
    uint8_t                       pin;
    uint8_t                       port;
    uint32_t                      pinMap;
    HwiP_Params                   hwiParams;
    SPIMSP432E4DMA_Object        *object = handle->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    key = HwiP_disable();

    if (object->isOpen) {
        HwiP_restore(key);

        return (NULL);
    }
    object->isOpen = true;

    HwiP_restore(key);

    /* SPI_MW is unsupported */
    if (params->frameFormat == SPI_MW) {
        object->isOpen = false;

        return (NULL);
    }

    /* Set SPI power dependency */
    powerMgrId = getPowerMgrId(hwAttrs->baseAddr);
    if (powerMgrId > PowerMSP432E4_NUMRESOURCES) {
        object->isOpen = false;

        return (NULL);
    }
    Power_setDependency(powerMgrId);

    /* Set GPIO power dependencies & configure SPI pins */
    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->clkPinMask);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->clkPinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->clkPinMask);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    GPIOPinConfigure(pinMap);
    GPIOPinTypeSSI(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->xdat0PinMask);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->xdat0PinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->xdat0PinMask);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    GPIOPinConfigure(pinMap);
    GPIOPinTypeSSI(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->xdat1PinMask);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->xdat1PinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->xdat1PinMask);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    GPIOPinConfigure(pinMap);
    GPIOPinTypeSSI(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->fssPinMask);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->fssPinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->fssPinMask);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    GPIOPinConfigure(pinMap);
    GPIOPinTypeSSI(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    object->dmaHandle = UDMAMSP432E4_open();
    if (object->dmaHandle == NULL) {
        SPIMSP432E4DMA_close(handle);

        return (NULL);
    }

    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t) handle;
    hwiParams.priority = hwAttrs->intPriority;
    object->hwiHandle = HwiP_create(hwAttrs->intNum, SPIMSP432E4DMA_hwiFxn,
        &(hwiParams));
    if (object->hwiHandle == NULL) {
        SPIMSP432E4DMA_close(handle);

        return (NULL);
    }

    if (params->transferMode == SPI_MODE_BLOCKING) {
        /*
         * Create a semaphore to block task execution for the duration of the
         * SPI transfer
         */
        object->transferComplete = SemaphoreP_createBinary(0);
        if (object->transferComplete == NULL) {
            SPIMSP432E4DMA_close(handle);

            return (NULL);
        }

        object->transferCallbackFxn = blockingTransferCallback;
    }
    else {
        if (params->transferCallbackFxn == NULL) {
            SPIMSP432E4DMA_close(handle);

            return (NULL);
        }

        object->transferCallbackFxn = params->transferCallbackFxn;
    }

    object->bitRate = params->bitRate;
    object->dataSize = params->dataSize;
    object->format = frameFormat[params->frameFormat];
    object->spiMode = params->mode;
    object->transaction = NULL;
    object->transferMode = params->transferMode;
    object->transferTimeout = params->transferTimeout;

    initHw(object, hwAttrs);

    return (handle);
}

/*
 *  ======== SPIMSP432E4DMA_transfer ========
 */
bool SPIMSP432E4DMA_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    uintptr_t                     key;
    uint8_t                       alignMask;
    bool                          buffersAligned;
    SPIMSP432E4DMA_Object        *object = handle->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    if ((transaction->count == 0) ||
        (transaction->rxBuf == NULL && transaction->txBuf == NULL)) {
        return (false);
    }

    key = HwiP_disable();

    /*
     * alignMask is used to determine if the RX/TX buffers addresses are
     * aligned to the transfer size.
     */
    alignMask = (object->dataSize < 9) ? 0x0 : 0x01;
    buffersAligned = ((((uint32_t) transaction->rxBuf & alignMask) == 0) &&
        (((uint32_t) transaction->txBuf & alignMask) == 0));

    if (object->transaction || !buffersAligned) {
        HwiP_restore(key);

        return (false);
    }
    else {
        object->transaction = transaction;
        object->transaction->status = SPI_TRANSFER_STARTED;
        object->amtDataXferred = 0;
        object->currentXferAmt = 0;
    }

    HwiP_restore(key);

    /* Polling transfer if BLOCKING mode & transaction->count < threshold */
    if (object->transferMode == SPI_MODE_BLOCKING &&
        transaction->count < hwAttrs->minDmaTransferSize) {
        spiPollingTransfer(object, hwAttrs, transaction);

        /* Transaction completed; set status & mark SPI ready */
        object->transaction->status = SPI_TRANSFER_COMPLETED;
        object->transaction = NULL;
    }
    else {
        /* Perform a DMA backed SPI transfer */
        configDMA(object, hwAttrs, transaction);

        if (object->transferMode == SPI_MODE_BLOCKING) {
            if (SemaphoreP_pend(object->transferComplete,
                object->transferTimeout) != SemaphoreP_OK) {
                /* Timeout occurred; cancel the transfer */
                object->transaction->status = SPI_TRANSFER_FAILED;
                SPIMSP432E4DMA_transferCancel(handle);

                /*
                 * TransferCancel() performs callback which posts
                 * transferComplete semaphore. This call consumes this extra post.
                 */
                SemaphoreP_pend(object->transferComplete, SemaphoreP_NO_WAIT);

                return (false);
            }
        }
    }
    return (true);
}

/*
 *  ======== SPIMSP432E4DMA_transferCancel ========
 */
void SPIMSP432E4DMA_transferCancel(SPI_Handle handle)
{
    uintptr_t                     key;
    SPI_Transaction              *msg;
    SPIMSP432E4DMA_Object        *object = handle->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    /*
     * There are 2 use cases in which to call transferCancel():
     *   1.  The driver is in CALLBACK mode.
     *   2.  The driver is in BLOCKING mode & there has been a transfer timeout.
     */
    if (object->transferMode == SPI_MODE_CALLBACK ||
        object->transaction->status == SPI_TRANSFER_FAILED) {

        key = HwiP_disable();

        if (object->transaction == NULL || object->cancelInProgress) {
            HwiP_restore(key);

            return;
        }
        object->cancelInProgress = true;

        /* Prevent interrupt from occurring while canceling the transfer */
        HwiP_disableInterrupt(hwAttrs->intNum);
        HwiP_clearInterrupt(hwAttrs->intNum);

        /* Clear DMA configuration */
        uDMAChannelDisable(hwAttrs->rxDmaChannel);
        uDMAChannelDisable(hwAttrs->txDmaChannel);

        SSIIntDisable(hwAttrs->baseAddr, SSI_DMARX);
        SSIIntClear(hwAttrs->baseAddr, SSI_DMARX);
        SSIDMADisable(hwAttrs->baseAddr, SSI_DMA_TX | SSI_DMA_RX);

        HwiP_restore(key);

        /*
         * Disables peripheral, clears all registers & reinitializes it to
         * parameters used in SPI_open()
         */
        initHw(object, hwAttrs);

        HwiP_enableInterrupt(hwAttrs->intNum);

        /*
         * Calculate amount of data which has already been sent & store
         * it in transaction->count
         */
        object->transaction->count = object->amtDataXferred +
            (object->currentXferAmt - getDmaRemainingXfers(hwAttrs));

        /* Set status CANCELED if we did not cancel due to timeout  */
        if (object->transaction->status == SPI_TRANSFER_STARTED) {
            object->transaction->status = SPI_TRANSFER_CANCELED;
        }

        /*
         * Use a temporary transaction pointer in case the callback function
         * attempts to perform another SPI_transfer call
         */
        msg = object->transaction;

        /* Indicate we are done with this transfer */
        object->transaction = NULL;
        object->cancelInProgress = false;
        object->transferCallbackFxn(handle, msg);
    }
}

/*
 *  ======== blockingTransferCallback ========
 */
static void blockingTransferCallback(SPI_Handle handle,
    SPI_Transaction *transaction)
{
    SPIMSP432E4DMA_Object *object = handle->object;

    SemaphoreP_post(object->transferComplete);
}

/*
 *  ======== configDMA ========
 *  This functions configures the transmit and receive DMA channels for a given
 *  SPI_Handle and SPI_Transaction
 */
static void configDMA(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs, SPI_Transaction *transaction)
{
    uintptr_t  key;
    void      *buf;
    uint8_t    optionsIndex;
    uint32_t   channelControlOptions;

    /*
     * The DMA options vary according to data frame size; options for 8-bit data
     * (or smaller) are in index 0.  Options for larger data frame sizes are in
     * index 1.
     */
    optionsIndex = (object->dataSize < 9) ? 0 : 1;

    /*
     * The DMA has a max transfer amount of 1024.  If the transaction is
     * greater; we must transfer it in chunks.  object->amtDataXferred has
     * how much data has already been sent.
     */
    if ((transaction->count - object->amtDataXferred) > MAX_DMA_TRANSFER_AMOUNT) {
        object->currentXferAmt = MAX_DMA_TRANSFER_AMOUNT;
    }
    else {
        object->currentXferAmt = (transaction->count - object->amtDataXferred);
    }

    if (transaction->txBuf) {
        channelControlOptions = dmaTxConfig[optionsIndex];
        /*
         * Add an offset for the amount of data transfered.  The offset is
         * calculated by: object->amtDataXferred * (optionsIndex + 1).  This
         * accounts for 8 or 16-bit sized transfers.
         */
        buf = (void *) ((uint32_t) transaction->txBuf +
            ((uint32_t) object->amtDataXferred * (optionsIndex + 1)));
    }
    else {
        channelControlOptions = dmaNullConfig[optionsIndex];
        *hwAttrs->scratchBufPtr = hwAttrs->defaultTxBufValue;
        buf = hwAttrs->scratchBufPtr;
    }

    /* Setup the TX transfer characteristics & buffers */
    uDMAChannelControlSet(hwAttrs->txDmaChannel | UDMA_PRI_SELECT,
        channelControlOptions);
    uDMAChannelTransferSet(hwAttrs->txDmaChannel | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC, buf, (void *) &(((SSI0_Type *) hwAttrs->baseAddr)->DR),
        object->currentXferAmt);

    if (transaction->rxBuf) {
        channelControlOptions = dmaRxConfig[optionsIndex];
        /*
         * Add an offset for the amount of data transfered.  The offset is
         * calculated by: object->amtDataXferred * (optionsIndex + 1).  This
         * accounts for 8 or 16-bit sized transfers.
         */
        buf = (void *) ((uint32_t) transaction->rxBuf +
            ((uint32_t) object->amtDataXferred * (optionsIndex + 1)));
    }
    else {
        channelControlOptions = dmaNullConfig[optionsIndex];
        buf = hwAttrs->scratchBufPtr;
    }

    /* Setup the RX transfer characteristics & buffers */
    uDMAChannelControlSet(hwAttrs->rxDmaChannel | UDMA_PRI_SELECT,
        channelControlOptions);
    uDMAChannelTransferSet(hwAttrs->rxDmaChannel | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC, (void *) &(((SSI0_Type *) hwAttrs->baseAddr)->DR), buf,
        object->currentXferAmt);

    /* A lock is needed because we are accessing shared uDMA memory */
    key = HwiP_disable();

    uDMAChannelAssign(hwAttrs->rxDmaChannel);
    uDMAChannelAssign(hwAttrs->txDmaChannel);

    /* Enable DMA to generate interrupt on SPI peripheral */
    SSIDMAEnable(hwAttrs->baseAddr, SSI_DMA_TX | SSI_DMA_RX);
    SSIIntClear(hwAttrs->baseAddr, SSI_DMARX);
    SSIIntEnable(hwAttrs->baseAddr, SSI_DMARX);

    /* Enable channels & start DMA transfers */
    uDMAChannelEnable(hwAttrs->txDmaChannel);
    uDMAChannelEnable(hwAttrs->rxDmaChannel);

    HwiP_restore(key);

    SSIEnable(hwAttrs->baseAddr);
}

/*
 *  ======== getDmaRemainingXfers ========
 */
static inline uint32_t getDmaRemainingXfers(SPIMSP432E4DMA_HWAttrs const *hwAttrs)
{
    uint32_t          controlWord;
    tDMAControlTable *controlTable;

    controlTable = uDMAControlBaseGet();
    controlWord = controlTable[(hwAttrs->rxDmaChannel & 0x3f)].ui32Control;

    return (((controlWord & UDMA_CHCTL_XFERSIZE_M) >> 4) + 1);
}

/*
 *  ======== getPowerMgrId ========
 */
static uint8_t getPowerMgrId(uint32_t baseAddr)
{
    switch (baseAddr) {
        case SSI0_BASE:
            return (PowerMSP432E4_PERIPH_SSI0);
        case SSI1_BASE:
            return (PowerMSP432E4_PERIPH_SSI1);
        case SSI2_BASE:
            return (PowerMSP432E4_PERIPH_SSI2);
        case SSI3_BASE:
            return (PowerMSP432E4_PERIPH_SSI3);
        default:
            return (~0);
    }
}

/*
 *  ======== getPeripheralResetId ========
 */
static uint32_t getPeripheralResetId(uint32_t baseAddr)
{
    switch (baseAddr) {
        case SSI0_BASE:
            return (SYSCTL_PERIPH_SSI0);
        case SSI1_BASE:
            return (SYSCTL_PERIPH_SSI1);
        case SSI2_BASE:
            return (SYSCTL_PERIPH_SSI2);
        case SSI3_BASE:
            return (SYSCTL_PERIPH_SSI3);
        default:
            /* Should never return default */
            return (~0);
    }
}

/*
 *  ======== initHw ========
 */
static void initHw(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs)
{
    ClockP_FreqHz freq;

    /*
     * SPI peripheral should remain disabled until a transfer is requested.
     * This is done to prevent the RX FIFO from gathering data from other
     * transfers.
     */
    SSIDisable(hwAttrs->baseAddr);

    /* Reset all peripheral registers */
    SysCtlPeripheralReset(getPeripheralResetId(hwAttrs->baseAddr));

    /* Set the SPI configuration */
    ClockP_getCpuFreq(&freq);
    SSIConfigSetExpClk(hwAttrs->baseAddr, freq.lo, object->format,
        object->spiMode, object->bitRate, object->dataSize);
}

/*
 *  ======== spiPollingTransfer ========
 */
static inline void spiPollingTransfer(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs, SPI_Transaction *transaction)
{
    uint8_t   increment;
    uint32_t  dummyBuffer;
    size_t    transferCount;
    void     *rxBuf;
    void     *txBuf;

    if (transaction->rxBuf) {
        rxBuf = transaction->rxBuf;
    }
    else {
        rxBuf = hwAttrs->scratchBufPtr;
    }

    if (transaction->txBuf) {
        txBuf = transaction->txBuf;
    }
    else {
        *hwAttrs->scratchBufPtr = hwAttrs->defaultTxBufValue;
        txBuf = hwAttrs->scratchBufPtr;
    }

    increment = (object->dataSize < 9) ? sizeof(uint8_t) : sizeof(uint16_t);
    transferCount = transaction->count;

    /* Start the polling transfer */
    SSIEnable(hwAttrs->baseAddr);

    while (transferCount--) {
        if (object->dataSize < 9) {
            SSIDataPut(hwAttrs->baseAddr, *((uint8_t *) txBuf));
            SSIDataGet(hwAttrs->baseAddr, &dummyBuffer);
            *((uint8_t *) rxBuf) = (uint8_t) dummyBuffer;
        }
        else {
            SSIDataPut(hwAttrs->baseAddr, *((uint16_t *) txBuf));
            SSIDataGet(hwAttrs->baseAddr, &dummyBuffer);
            *((uint16_t *) rxBuf) = (uint16_t) dummyBuffer;
        }

        /* Only increment source & destination if buffers were provided */
        if (transaction->rxBuf) {
            rxBuf = (void *) (((uint32_t) rxBuf) + increment);
        }
        if (transaction->txBuf) {
            txBuf = (void *) (((uint32_t) txBuf) + increment);
        }
    }

    SSIDisable(hwAttrs->baseAddr);
}
