/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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
#include <stdlib.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/i2c.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_i2c.h>
#include <ti/devices/msp432e4/driverlib/types.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/i2c/I2CMSP432E4.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>

/*
 * Specific I2C CMD MACROs that are not defined in driverlib/i2c.h are defined
 * here. Their equivalent values are taken from the existing MACROs in
 * driverlib/i2c.h
 */
#ifndef I2C_MASTER_CMD_BURST_RECEIVE_START_NACK
#define I2C_MASTER_CMD_BURST_RECEIVE_START_NACK  I2C_MASTER_CMD_BURST_SEND_START
#endif

#ifndef I2C_MASTER_CMD_BURST_RECEIVE_STOP
#define I2C_MASTER_CMD_BURST_RECEIVE_STOP        I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP
#endif

#ifndef I2C_MASTER_CMD_BURST_RECEIVE_CONT_NACK
#define I2C_MASTER_CMD_BURST_RECEIVE_CONT_NACK   I2C_MASTER_CMD_BURST_SEND_CONT
#endif

/* Prototypes */
void I2CMSP432E4_cancel(I2C_Handle handle);
void I2CMSP432E4_close(I2C_Handle handle);
int_fast16_t I2CMSP432E4_control(I2C_Handle handle, uint_fast16_t cmd,
                                 void *arg);
void I2CMSP432E4_init(I2C_Handle handle);
I2C_Handle I2CMSP432E4_open(I2C_Handle handle,
                            I2C_Params *params);
bool I2CMSP432E4_transfer(I2C_Handle handle,
                          I2C_Transaction *transaction);

static void primeTransfer(I2CMSP432E4_Object  *object,
                          I2CMSP432E4_HWAttrs const *hwAttrs,
                          I2C_Transaction *transaction);
static uint32_t getPowerMgrId(uint32_t baseAddress);

/* I2C function table for I2CMSP432E4 implementation */
const I2C_FxnTable I2CMSP432E4_fxnTable = {
    I2CMSP432E4_cancel,
    I2CMSP432E4_close,
    I2CMSP432E4_control,
    I2CMSP432E4_init,
    I2CMSP432E4_open,
    I2CMSP432E4_transfer
};

/*
 *  ======== blockingCallback ========
 */
static void blockingCallback(I2C_Handle handle, I2C_Transaction *msg,
                             bool transferStatus)
{
    I2CMSP432E4_Object  *object = handle->object;

    /* Indicate transfer complete */
    SemaphoreP_post(object->transferComplete);
}

/*
 *  ======== completeTransfer ========
 */
static void completeTransfer(I2C_Handle handle)
{
    /* Get the pointer to the object */
    I2CMSP432E4_Object         *object = handle->object;

    /*
     * Perform callback in a HWI context, thus any tasks or SWIs
     * made ready to run won't start until the interrupt has
     * finished
     */
    object->transferCallbackFxn(handle, object->currentTransaction,
                                !((bool)object->mode));

    /* See if we need to process any other transactions */
    if (object->headPtr == object->tailPtr) {
        /* No other transactions need to occur */
        object->currentTransaction = NULL;
        object->headPtr = NULL;
        object->tailPtr = NULL;
    }
    else {
        /* Another transfer needs to take place */
        object->headPtr = object->headPtr->nextPtr;

        primeTransfer(object, (I2CMSP432E4_HWAttrs const *)handle->hwAttrs,
                      object->headPtr);
    }
}

/*
 *  ======== getPowerMgrId ========
 */
static uint32_t getPowerMgrId(uint32_t baseAddress)
{
    switch (baseAddress) {
        case I2C0_BASE:
            return (PowerMSP432E4_PERIPH_I2C0);
        case I2C1_BASE:
            return (PowerMSP432E4_PERIPH_I2C1);
        case I2C2_BASE:
            return (PowerMSP432E4_PERIPH_I2C2);
        case I2C3_BASE:
            return (PowerMSP432E4_PERIPH_I2C3);
        case I2C4_BASE:
            return (PowerMSP432E4_PERIPH_I2C4);
        case I2C5_BASE:
            return (PowerMSP432E4_PERIPH_I2C5);
        case I2C6_BASE:
            return (PowerMSP432E4_PERIPH_I2C6);
        case I2C7_BASE:
            return (PowerMSP432E4_PERIPH_I2C7);
        case I2C8_BASE:
            return (PowerMSP432E4_PERIPH_I2C8);
        case I2C9_BASE:
            return (PowerMSP432E4_PERIPH_I2C9);
        default:
            return (~0);
    }
}

/*
 *  If Fast Mode Plus (1 Mbps) is desired, software should manually
 *  write the I2CMTPR after calling the I2CMasterInitExpClk() function.
 */
static inline void setFastModePlus(uint32_t baseAddress)
{
    /* Clear the timer period bits */
    HWREG(baseAddress + I2C_O_MTPR) &= ~(I2C_MTPR_TPR_M);

    /*
     * The timer period (TP) is calculated using the following:
     * TP = (System Clock / (2 * (SCL_LP + SCL_HP) * SCL)) - 1
     * The system clock is fixed to 120MHz.
     * The SCL low period is fixed to 6.
     * The SCL high period is fixed to 4.
     * The SCL represents the desired serial clock speed, 1,000,000.
     * This results in TP = 5. No need to compute this at runtime.
     * Write a timer period of 5 to the master timer period register.
     */
    HWREG(baseAddress + I2C_O_MTPR) |= 0x05;
}

/*
 *  ======== I2CMSP432E4_hwiFxn ========
 *  Hwi interrupt handler to service the I2C peripheral
 *
 *  The handler is a generic handler for a I2C object.
 */
static void I2CMSP432E4_hwiFxn(uintptr_t arg)
{
    /* Get the pointer to the object and hwAttrs */
    I2CMSP432E4_Object         *object = ((I2C_Handle)arg)->object;
    I2CMSP432E4_HWAttrs const  *hwAttrs = ((I2C_Handle)arg)->hwAttrs;
    uint32_t                errStatus;

    /* Get the interrupt status of the I2C controller */
    errStatus = I2CMasterErr(hwAttrs->baseAddr);

    /* Clear interrupt source to avoid additional interrupts */
    I2CMasterIntClear(hwAttrs->baseAddr);

    /* Check for I2C Errors */
    if ((errStatus == I2C_MASTER_ERR_NONE) ||
        (object->mode == I2CMSP432E4_ERROR)) {
        /* No errors, now check what we need to do next */
        switch (object->mode) {
            /*
             * ERROR case is OK because if an Error is detected, a STOP bit is
             * sent; which in turn will call another interrupt. This interrupt
             * call will then post the transferComplete semaphore to unblock the
             * I2C_transfer function
             */
            case I2CMSP432E4_ERROR:
            case I2CMSP432E4_IDLE_MODE:
                completeTransfer((I2C_Handle) arg);
                break;

            case I2CMSP432E4_WRITE_MODE:
                /* Decrement write Counter */
                object->writeCountIdx--;

                /* Check if more data needs to be sent */
                if (object->writeCountIdx) {
                    /* Write data contents into data register */
                    I2CMasterDataPut(hwAttrs->baseAddr,
                        *(object->writeBufIdx));
                    object->writeBufIdx++;

                    if ((object->writeCountIdx < 2) &&
                        !(object->readCountIdx)) {
                        /* Everything has been sent, nothing to receive */
                        /* Next state: Idle mode */
                        object->mode = I2CMSP432E4_IDLE_MODE;

                        /* Send last byte with STOP bit */
                        I2CMasterControl(hwAttrs->baseAddr,
                            I2C_MASTER_CMD_BURST_SEND_FINISH);
                    }
                    else {
                        /*
                         * Either there is more date to be transmitted or some
                         * data needs to be received next
                         */
                        I2CMasterControl(hwAttrs->baseAddr,
                            I2C_MASTER_CMD_BURST_SEND_CONT);
                    }
                }

                /* At this point, we know that we need to receive data */
                else {
                    /*
                     * We need to check after we are done transmitting data, if
                     * we need to receive any data.
                     * In a corner case when we have only one byte transmitted
                     * and no data to receive, the I2C will automatically send
                     * the STOP bit. In other words, here we only need to check
                     * if data needs to be received. If so, how much.
                     */
                    if (object->readCountIdx) {
                        /* Next state: Receive mode */
                        object->mode = I2CMSP432E4_READ_MODE;

                        /* Switch into Receive mode */
                        I2CMasterSlaveAddrSet(hwAttrs->baseAddr,
                            object->currentTransaction->slaveAddress,
                            true);

                        if (object->readCountIdx > 1) {
                            /* Send a repeated START */
                            I2CMasterControl(hwAttrs->baseAddr,
                                I2C_MASTER_CMD_BURST_RECEIVE_START);
                        }
                        else {
                            /*
                             * Send a repeated START with a NACK since it's the
                             * last byte to be received.
                             * I2C_MASTER_CMD_BURST_RECEIVE_START_NACK is
                             * is locally defined because there is no macro to
                             * receive data and send a NACK after sending a
                             * start bit (0x00000003)
                             */
                            I2CMasterControl(hwAttrs->baseAddr,
                                I2C_MASTER_CMD_BURST_RECEIVE_START_NACK);
                        }
                    }
                    else {
                        /* Done with all transmissions */
                        object->mode = I2CMSP432E4_IDLE_MODE;
                        /*
                         * No more data needs to be received, so follow up with
                         * a STOP bit
                         * Again, there is no equivalent macro (0x00000004) so
                         * I2C_MASTER_CMD_BURST_RECEIVE_STOP is used.
                         */
                        I2CMasterControl(hwAttrs->baseAddr,
                                         I2C_MASTER_CMD_BURST_RECEIVE_STOP);
                    }
                }
                break;

            case I2CMSP432E4_READ_MODE:
                /* Save the received data */
                *(object->readBufIdx) = I2CMasterDataGet(hwAttrs->baseAddr);
                object->readBufIdx++;

                /* Check if any data needs to be received */
                object->readCountIdx--;
                if (object->readCountIdx) {
                    if (object->readCountIdx > 1) {
                        /* More data to be received */
                        I2CMasterControl(hwAttrs->baseAddr,
                            I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                    }
                    else {
                        /*
                         * Send NACK because it's the last byte to be received
                         * There is no NACK macro equivalent (0x00000001) so
                         * I2C_MASTER_CMD_BURST_RECEIVE_CONT_NACK is used
                         */
                        I2CMasterControl(hwAttrs->baseAddr,
                            I2C_MASTER_CMD_BURST_RECEIVE_CONT_NACK);
                    }
                }
                else {
                    /* Next state: Idle mode */
                    object->mode = I2CMSP432E4_IDLE_MODE;

                    /*
                     * No more data needs to be received, so follow up with a
                     * STOP bit
                     * Again, there is no equivalent macro (0x00000004) so
                     * I2C_MASTER_CMD_BURST_RECEIVE_STOP is used
                     */
                    I2CMasterControl(hwAttrs->baseAddr,
                        I2C_MASTER_CMD_BURST_RECEIVE_STOP);
                }

                break;

            default:
                object->mode = I2CMSP432E4_ERROR;
                break;
        }

    }
    else {
        /* Some sort of error happened! */
        object->mode = I2CMSP432E4_ERROR;

        if (errStatus & (I2C_MASTER_ERR_ARB_LOST | I2C_MASTER_ERR_ADDR_ACK)) {
            completeTransfer((I2C_Handle) arg);
        }
        else {
            /* Try to send a STOP bit to end all I2C communications immediately */
            /*
             * I2C_MASTER_CMD_BURST_SEND_ERROR_STOP -and-
             * I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP
             * have the same values
             */
            I2CMasterControl(hwAttrs->baseAddr,
                I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        }
    }

    return;
}

/*
 *  ======== primeTransfer =======
 */
static void primeTransfer(I2CMSP432E4_Object  *object,
                          I2CMSP432E4_HWAttrs const *hwAttrs,
                          I2C_Transaction *transaction)
{
    /* Store the new internal counters and pointers */
    object->currentTransaction = transaction;

    object->writeBufIdx = transaction->writeBuf;
    object->writeCountIdx = transaction->writeCount;

    object->readBufIdx = transaction->readBuf;
    object->readCountIdx = transaction->readCount;

    /* Start transfer in Transmit mode */
    if (object->writeCountIdx) {
        /* Specify the I2C slave address */
        I2CMasterSlaveAddrSet(hwAttrs->baseAddr,
            object->currentTransaction->slaveAddress, false);

        /* Update the I2C mode */
        object->mode = I2CMSP432E4_WRITE_MODE;

        /* Write data contents into data register */
        I2CMasterDataPut(hwAttrs->baseAddr, *((object->writeBufIdx)++));

        /* Start the I2C transfer in master transmit mode */
        I2CMasterControl(hwAttrs->baseAddr, I2C_MASTER_CMD_BURST_SEND_START);
    }

    /* Start transfer in Receive mode */
    else {
        /* Specify the I2C slave address */
        I2CMasterSlaveAddrSet(hwAttrs->baseAddr,
            object->currentTransaction->slaveAddress,
            true);

        /* Update the I2C mode */
        object->mode = I2CMSP432E4_READ_MODE;

        if (object->readCountIdx < 2) {
            /* Start the I2C transfer in master receive mode */
            I2CMasterControl(hwAttrs->baseAddr,
                I2C_MASTER_CMD_BURST_RECEIVE_START_NACK);
        }
        else {
            /* Start the I2C transfer in master receive mode */
            I2CMasterControl(hwAttrs->baseAddr,
                I2C_MASTER_CMD_BURST_RECEIVE_START);
        }
    }
}

/*
 *  ======== I2CMSP432E4_cancel ========
 */
void I2CMSP432E4_cancel(I2C_Handle handle)
{
    I2CMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    I2CMSP432E4_Object *object = handle->object;
    ClockP_FreqHz freq;
    uintptr_t key;

    /* just return if no transfer is in progress */
    if (!object->headPtr) {
        return;
    }

    /* disable interrupts, send STOP to complete any transfer */
    key = HwiP_disable();
    I2CMasterIntDisable(hwAttrs->baseAddr);
    I2CMasterControl(hwAttrs->baseAddr, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);

    /* call the transfer callback for the current transfer, indicate failure */
    object->transferCallbackFxn(handle, object->currentTransaction, false);

    /* also dequeue and call the transfer callbacks for any queued transfers */
    while (object->headPtr != object->tailPtr) {
        object->headPtr = object->headPtr->nextPtr;
        object->transferCallbackFxn(handle, object->headPtr, false);
    }

    /* clean up object */
    object->currentTransaction = NULL;
    object->headPtr = NULL;
    object->tailPtr = NULL;
    object->mode = I2CMSP432E4_IDLE_MODE;

    /* disable and the re-initialize master mode */
    I2CMasterDisable(hwAttrs->baseAddr);
    ClockP_getCpuFreq(&freq);
    I2CMasterInitExpClk(hwAttrs->baseAddr, freq.lo, (bool) object->bitRate);

    /* If fast-mode plus is desired */
    if (object->bitRate == I2C_1000kHz) {
        setFastModePlus(hwAttrs->baseAddr);
    }

    /* clear any pending interrupts */
    I2CMasterIntClear(hwAttrs->baseAddr);

    /* enable the I2C Master for operation */
    I2CMasterEnable(hwAttrs->baseAddr);

    /* unmask I2C interrupts */
    I2CMasterIntEnable(hwAttrs->baseAddr);

    HwiP_restore(key);
}

/*
 *  ======== I2CMSP432E4_close ========
 */
void I2CMSP432E4_close(I2C_Handle handle)
{
    I2CMSP432E4_Object         *object = handle->object;
    I2CMSP432E4_HWAttrs const  *hwAttrs = handle->hwAttrs;
    uint8_t                     port;

    /* Mask I2C interrupts */
    I2CMasterIntDisable(hwAttrs->baseAddr);

    /* Disable the I2C Master */
    I2CMasterDisable(hwAttrs->baseAddr);

    /* Undo SCL pin and release dependency */
    GPIOMSP432E4_undoPinConfig(hwAttrs->sclPin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sclPin);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Undo SDA pin and release dependency */
    GPIOMSP432E4_undoPinConfig(hwAttrs->sdaPin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sdaPin);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Release peripheral dependency */
    Power_releaseDependency(getPowerMgrId(hwAttrs->baseAddr));

    if (object->hwiHandle) {
        HwiP_delete(object->hwiHandle);
        object->hwiHandle = NULL;
    }

    if (object->mutex) {
        SemaphoreP_delete(object->mutex);
        object->mutex = NULL;
    }

    if (object->transferComplete) {
        SemaphoreP_delete(object->transferComplete);
        object->transferComplete = NULL;
    }

    object->isOpen = false;

    return;
}

/*
 *  ======== I2CMSP432E4_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int_fast16_t I2CMSP432E4_control(I2C_Handle handle, uint_fast16_t cmd, void *arg)
{
    /* No implementation yet */
    return (I2C_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== I2CMSP432E4_init ========
 */
void I2CMSP432E4_init(I2C_Handle handle)
{
}

/*
 *  ======== I2CMSP432E4_open ========
 */
I2C_Handle I2CMSP432E4_open(I2C_Handle handle, I2C_Params *params)
{
    uintptr_t                   key;
    ClockP_FreqHz               freq;
    I2CMSP432E4_Object          *object = handle->object;
    I2CMSP432E4_HWAttrs const   *hwAttrs = handle->hwAttrs;
    HwiP_Params                 hwiParams;
    uint32_t                    pinMap;
    uint32_t                    powerMgrId;
    uint8_t                     port;
    uint8_t                     pin;

    /* Determine if the device index was already opened */
    key = HwiP_disable();
    if (object->isOpen) {
        HwiP_restore(key);

        return (NULL);
    }

    /* Mark the handle as being used */
    object->isOpen = true;
    HwiP_restore(key);

    powerMgrId = getPowerMgrId(hwAttrs->baseAddr);

    if (powerMgrId > PowerMSP432E4_NUMRESOURCES) {
        object->isOpen = false;

        return (NULL);
    }

    /* Set Power dependencies, SCL & SDA Pin may use different GPIO port */
    Power_setDependency(powerMgrId);

    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sclPin);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));

    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sdaPin);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Configure the appropriate pins to be I2C instead of GPIO. */
    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->sclPin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sclPin);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->sclPin);

    GPIOPinConfigure(pinMap);
    GPIOPinTypeI2CSCL(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->sdaPin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sdaPin);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->sdaPin);

    GPIOPinConfigure(pinMap);
    GPIOPinTypeI2C(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    /* Save parameters */
    object->transferMode = params->transferMode;
    object->transferCallbackFxn = params->transferCallbackFxn;
    object->bitRate = params->bitRate;

    /* Create Hwi object for this I2C peripheral */
    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t)handle;
    hwiParams.priority = hwAttrs->intPriority;
    object->hwiHandle = HwiP_create(hwAttrs->intNum, I2CMSP432E4_hwiFxn,
                            &hwiParams);

    if (object->hwiHandle == NULL) {
        I2CMSP432E4_close(handle);

        return (NULL);
    }

    /*
     * Create threadsafe handles for this I2C peripheral
     * Semaphore to provide exclusive access to the I2C peripheral
     */
    object->mutex = SemaphoreP_createBinary(1);

    if (object->mutex == NULL) {
        I2CMSP432E4_close(handle);

        return (NULL);
    }

    /*
     * Store a callback function that posts the transfer complete
     * semaphore for synchronous mode
     */
    if (object->transferMode == I2C_MODE_BLOCKING) {
        /*
         * Semaphore to cause the waiting task to block for the I2C
         * to finish
         */
        object->transferComplete = SemaphoreP_createBinary(0);

        if (object->transferComplete == NULL) {
            I2CMSP432E4_close(handle);

            return (NULL);
        }

        /* Store internal callback function */
        object->transferCallbackFxn = blockingCallback;
    }

    /* Specify the idle state for this I2C peripheral */
    object->mode = I2CMSP432E4_IDLE_MODE;

    /* Clear the head pointer */
    object->headPtr = NULL;
    object->tailPtr = NULL;

    /* Set the I2C configuration */
    ClockP_getCpuFreq(&freq);
    I2CMasterInitExpClk(hwAttrs->baseAddr, freq.lo, (bool) object->bitRate);

    /* If fast-mode plus is desired */
    if (object->bitRate == I2C_1000kHz) {
        setFastModePlus(hwAttrs->baseAddr);
    }

    /* Clear any pending interrupts */
    I2CMasterIntClear(hwAttrs->baseAddr);

    /* Enable the I2C Master for operation */
    I2CMasterEnable(hwAttrs->baseAddr);

    /* Unmask I2C interrupts */
    I2CMasterIntEnable(hwAttrs->baseAddr);

    /* Return the address of the handle */
    return (handle);
}

/*
 *  ======== I2CMSP432E4_transfer ========
 */
bool I2CMSP432E4_transfer(I2C_Handle handle, I2C_Transaction *transaction)
{
    uintptr_t                   key;
    I2CMSP432E4_Object         *object = handle->object;
    I2CMSP432E4_HWAttrs const  *hwAttrs = handle->hwAttrs;
    bool                        ret = false;

    /* Check if anything needs to be written or read */
    if ((transaction->writeCount == 0) &&
        (transaction->readCount == 0)) {
        /* Nothing to write or read */
        return (ret);
    }

    if (object->transferMode == I2C_MODE_CALLBACK) {
        /* Check if a transfer is in progress */
        key = HwiP_disable();
        if (object->headPtr) {
            /* Transfer in progress */

            /*
             * Update the message pointed by the tailPtr to point to the next
             * message in the queue
             */
            object->tailPtr->nextPtr = transaction;

            /* Update the tailPtr to point to the last message */
            object->tailPtr = transaction;

            /* I2C is still being used */
            HwiP_restore(key);
            return (true);
        }
        else {
            /* Store the headPtr indicating I2C is in use */
            object->headPtr = transaction;
            object->tailPtr = transaction;
        }
        HwiP_restore(key);
    }

    /* Acquire the lock for this particular I2C handle */
    SemaphoreP_pend(object->mutex, SemaphoreP_WAIT_FOREVER);

    /*
     * I2CMSP432E4_primeTransfer is a longer process and
     * protection is needed from the I2C interrupt
     */
    HwiP_disableInterrupt(hwAttrs->intNum);
    primeTransfer(object, hwAttrs, transaction);
    HwiP_enableInterrupt(hwAttrs->intNum);

    if (object->transferMode == I2C_MODE_BLOCKING) {
        /*
         * Wait for the transfer to complete here.
         * It's OK to block from here because the I2C's Hwi will unblock
         * upon errors
         */
        SemaphoreP_pend(object->transferComplete, SemaphoreP_WAIT_FOREVER);

        /* Hwi handle has posted a 'transferComplete' check for Errors */
        if (object->mode == I2CMSP432E4_IDLE_MODE) {
            ret = true;
        }
    }
    else {
        /* Always return true if in Asynchronous mode */
        ret = true;
    }

    /* Release the lock for this particular I2C handle */
    SemaphoreP_post(object->mutex);

    /* Return the number of bytes transfered by the I2C */
    return (ret);
}
