/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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

#include <ti/runtime/wiring/Energia.h>
#include <ti/runtime/wiring/wiring_private.h>

#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/lm4/Timer.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIMSP432E4DMA.h>

#include <driverlib/rom.h>
#include <driverlib/rom_map.h>

static Timer_Handle clockTimer = 0;
static uint32_t clockTimerFreq = 0;

/*
 *  ======== micros ========
 */
unsigned long micros(void)
{
    uint32_t key;
    Types_FreqHz freq;
    uint64_t micros, expired;

    if (clockTimer == 0) {
        clockTimer = Timer_getHandle(Clock_timerId);
        Timer_getFreq(clockTimer, &freq);
        clockTimerFreq = freq.lo;
    }

    key = Hwi_disable();

    micros = Clock_getTicks() * Clock_tickPeriod;
    /* capture timer ticks since last Clock tick */
    expired = Timer_getExpiredCounts(clockTimer);

    Hwi_restore(key);

    micros += (expired * 1000000) / clockTimerFreq;

    return (micros);
}

/*
 *  ======== millis ========
 */
unsigned long millis(void)
{
    return (Clock_getTicks());
}

/*
 *  ======== delayMicroseconds ========
 *  Delay for the given number of microseconds.
 */
void delayMicroseconds(unsigned int us)
{
    if (us <7) {
        //The overhead in calling and returning from this function takes about 6us
    }
    else if (us <=20) {
        int time;
        for (time = 5*(us-6); time > 0; time--) {
            asm("   nop");
        }
    }
    else if (us < 70) {
        int time;
        for (time = 5*us; time > 0; time--) {
            asm("   nop");
        }
    }
    else {
        uint32_t t0, deltaT;
        Types_FreqHz freq;

        Timestamp_getFreq(&freq);
        deltaT = us * (freq.lo/1000000);

        t0 = Timestamp_get32();

        while ((Timestamp_get32()-t0) < deltaT) {
            ;
        }
    }
}

/*
 *  ======== delay ========
 */
void delay(uint32_t milliseconds)
{
    if (milliseconds == 0) {
        Task_yield();
        return;
    }

    /* timeout is always in milliseconds so that Clock_workFunc() behaves properly */
    Task_sleep(milliseconds);
}

/*
 *  ======== getSpiInfo ========
 *
 *  A hack to work around spiPolling only being supported
 *  in SPI_MODE_BLOCKING. Remove if/when this is resolved
 *  in the SPI drivers.
 */
void getSpiInfo(void *spi, SpiInfo *spiInfo)
{
    SPIMSP432E4DMA_Object *obj;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs;
    SPI_Handle spiHandle = (SPI_Handle)spi;

    obj = (SPIMSP432E4DMA_Object *)(spiHandle->object);
    hwAttrs = (SPIMSP432E4DMA_HWAttrs *)(spiHandle->hwAttrs);

    spiInfo->transferModePtr = &obj->transferMode;
    spiInfo->minDmaTransferSize = hwAttrs->minDmaTransferSize;
}

/*
 *  ======== NDK_hookInit ========
 */
/*
 * Provide a weakly defined version of the NDK's Task register hook function
 * for when an application doesn't use NDK and doesn't link with the NDK
 * libraries.
 */
void NDK_hookInit(int) __attribute__((weak));
void NDK_hookInit(int x)
{
}
