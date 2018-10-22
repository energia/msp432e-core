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
/** ============================================================================
 *  @file       SPIMSP432E4DMA.h
 *
 *  @brief      SPI driver implementation for a MSP432E4 SPI controller using
 *              the micro DMA controller.
 *
 *  The SPI header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/SPI.h>
 *  #include <ti/drivers/spi/SPIMSP432E4DMA.h>
 *  @endcode
 *
 *  Refer to @ref SPI.h for a complete description of APIs & example of use.
 *
 *  This SPI driver implementation is designed to operate on a MSP432E4 SPI
 *  controller using a micro DMA controller.
 *
 *  ## Frame Formats #
 *  This SPI controller supports 4 phase & polarity formats as well as
 *  the TI synchronous serial frame format. Refer to the device specific data
 *  sheets & technical reference manuals for specifics on each format.
 *
 *  ## SPI Chip Select #
 *  This SPI controller supports a hardware chip select pin. Refer to the
 *  device's user manual on how this hardware chip select pin behaves in regards
 *  to the SPI frame format.
 *
 *  <table>
 *  <tr>
 *  <th>Chip select type</th>
 *  <th>SPI_MASTER mode</th>
 *  <th>SPI_SLAVE mode</th>
 *  </tr>
 *  <tr>
 *  <td>Hardware chip select</td>
 *  <td>No action is needed by the application to select the peripheral.</td>
 *  <td>See the device documentation on it's chip select requirements.</td>
 *  </tr>
 *  <tr>
 *  <td>Software chip select</td>
 *  <td>The application is responsible to ensure that correct SPI slave is
 *      selected before performing a SPI_transfer().</td>
 *  <td>See the device documentation on it's chip select requirements.</td>
 *  </tr>
 *  </table>
 *
 *  ## SPI data frames #
 *
 *  SPI data frames can be any size from 4-bits to 16-bits. If the dataSize in
 *  ::SPI_Params is greater than 8-bits, then the SPIMSP432E4DMA driver
 *  will assume that the ::SPI_Transaction txBuf and rxBuf point to 16-bit
 *  (uint16_t) arrays.
 *
 *  dataSize  | buffer element size |
 *  --------  | ------------------- |
 *  4-8 bits  | uint8_t             |
 *  9-16 bits | uint16_t            |
 *
 *  Data buffers in transactions (rxBuf & txBuf) must be address aligned
 *  according to the data frame size.  For example, if data frame is 9-bit or
 *  larger (driver assumes buffers are uint16_t) rxBuf & txBuf must be aligned
 *  on a 16-bit address boundary.
 *
 *  ## Interrupts #
 *  This driver is designed to make use of the micro DMA for transfers. The
 *  driver implementation automatically installs a hardware interrupt service
 *  routine for the peripheral on which micro DMA generates IRQ once all data
 *  has transfered from memory to the peripheral (TX) or from peripheral to
 *  memory (RX).
 *
 *  ## DMA accessible memory #
 *
 *  As this driver uses uDMA to transfer data to/from data buffers, it is the
 *  responsibility of the application to ensure that theses buffers reside in
 *  memory that is accessible by the DMA.
 *
 *  ## Scratch Buffers #
 *  A uint16_t scratch buffer is used to allow SPI_transfers where txBuf or
 *  rxBuf are NULL. Rather than requiring txBuf or rxBuf to have a dummy buffer
 *  of size of the transfer count, a single DMA accessible uint16_t scratch
 *  buffer is used. When rxBuf is NULL, the uDMA will transfer all the SPI data
 *  receives into the scratch buffer as a "bit-bucket". When txBuf is NULL, the
 *  scratch buffer is initialized to defaultTxBufValue so the uDMA will send
 *  some known value. Each SPI driver instance must have its own scratch buffer.
 *
 *  ## Polling SPI transfers #
 *  When used in blocking mode small SPI transfers are can be done by polling
 *  the peripheral & sending data frame-by-frame.  This will not block the task
 *  which requested the transfer, but instead immediately perform the transfer
 *  & return.  The minDmaTransferSize field in the hardware attributes is
 *  the threshold; if the transaction count is below the threshold a polling
 *  transfer is performed; otherwise a DMA transfer is done.  This is intended
 *  to reduce the overhead of setting up a DMA transfer to only send a few
 *  data frames.  Keep in mind that during polling transfers the current task
 *  is still being executed; there is no context switch to another task.
 *  ============================================================================
 */

#ifndef ti_drivers_spi_SPIMSP432E4DMA__include
#define ti_drivers_spi_SPIMSP432E4DMA__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/pin_map.h>

#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/SPI.h>

/*!
 * @brief PA2 is used for SSI0CLK
 */
#define SPIMSP432E4_PA2_SSI0CLK GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 2, GPIO_PA2_SSI0CLK)

/*!
 * @brief PA3 is used for SSI0FSS
 */
#define SPIMSP432E4_PA3_SSI0FSS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 3, GPIO_PA3_SSI0FSS)

/*!
 * @brief PA4 is used for SSI0XDAT0
 */
#define SPIMSP432E4_PA4_SSI0XDAT0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 4, GPIO_PA4_SSI0XDAT0)

/*!
 * @brief PA5 is used for SSI0XDAT1
 */
#define SPIMSP432E4_PA5_SSI0XDAT1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 5, GPIO_PA5_SSI0XDAT1)

/*!
 * @brief PB5 is used for SSI1CLK
 */
#define SPIMSP432E4_PB5_SSI1CLK GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 5, GPIO_PB5_SSI1CLK)

/*!
 * @brief PB4 is used for SSI1FSS
 */
#define SPIMSP432E4_PB4_SSI0FSS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 4, GPIO_PB4_SSI1FSS)

/*!
 * @brief PE4 is used for SSI1XDAT0
 */
#define SPIMSP432E4_PE4_SSI1XDAT0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTE, 4, GPIO_PE4_SSI1XDAT0)

/*!
 * @brief PE5 is used for SSI1XDAT1
 */
#define SPIMSP432E4_PE5_SSI1XDAT1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTE, 5, GPIO_PE5_SSI1XDAT1)

/*!
 * @brief PD3 is used for SSI2CLK
 */
#define SPIMSP432E4_PD3_SSI2CLK GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 3, GPIO_PD3_SSI2CLK)

/*!
 * @brief PG7 is used for SSI2CLK
 */
#define SPIMSP432E4_PG7_SSI2CLK GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 7, GPIO_PG7_SSI2CLK)

/*!
 * @brief PD2 is used for SSI2FSS
 */
#define SPIMSP432E4_PD2_SSI2FSS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 2, GPIO_PD2_SSI2FSS)

/*!
 * @brief PG6 is used for SSI2FSS
 */
#define SPIMSP432E4_PG6_SSI2FSS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 6, GPIO_PG6_SSI2FSS)

/*!
 * @brief PD1 is used for SSI2XDAT0
 */
#define SPIMSP432E4_PD1_SSI2XDAT0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 1, GPIO_PD1_SSI2XDAT0)

/*!
 * @brief PG5 is used for SSI2XDAT0
 */
#define SPIMSP432E4_PG5_SSI2XDAT0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 5, GPIO_PG5_SSI2XDAT0)

/*!
 * @brief PD0 is used for SSI2XDAT1
 */
#define SPIMSP432E4_PD0_SSI2XDAT1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 0, GPIO_PD0_SSI2XDAT1)

/*!
 * @brief PG4 is used for SSI2XDAT1
 */
#define SPIMSP432E4_PG4_SSI2XDAT1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 4, GPIO_PG4_SSI2XDAT1)

/*!
 * @brief PQ0 is used for SSI3CLK
 */
#define SPIMSP432E4_PQ0_SSI3CLK GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTQ, 0, GPIO_PQ0_SSI3CLK)

/*!
 * @brief PF3 is used for SSI3CLK
 */
#define SPIMSP432E4_PF3_SSI3CLK GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 3, GPIO_PF3_SSI3CLK)

/*!
 * @brief PQ1 is used for SSI3FSS
 */
#define SPIMSP432E4_PQ1_SSI3FSS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTQ, 1, GPIO_PQ1_SSI3FSS)

/*!
 * @brief PF2 is used for SSI3FSS
 */
#define SPIMSP432E4_PF2_SSI3FSS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 2, GPIO_PF2_SSI3FSS)

/*!
 * @brief PQ2 is used for SSI3XDAT0
 */
#define SPIMSP432E4_PQ2_SSI3XDAT0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTQ, 2, GPIO_PQ2_SSI3XDAT0)

/*!
 * @brief PF1 is used for SSI3XDAT0
 */
#define SPIMSP432E4_PF1_SSI3XDAT0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 1, GPIO_PF1_SSI3XDAT0)

/*!
 * @brief PQ3 is used for SSI3XDAT1
 */
#define SPIMSP432E4_PQ3_SSI3XDAT1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTQ, 3, GPIO_PQ3_SSI3XDAT1)

/*!
 * @brief PF0 is used for SSI3XDAT1
 */
#define SPIMSP432E4_PF0_SSI3XDAT1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 0, GPIO_PF0_SSI3XDAT1)


/**
 *  @addtogroup SPI_STATUS
 *  SPIMSP432E4DMA_STATUS_* macros are command codes only defined in the
 *  SPIMSP432E4DMA.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/spi/SPIMSP432E4DMA.h>
 *  @endcode
 *  @{
 */

/* Add SPIMSP432E4DMA_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup SPI_CMD
 *  SPIMSP432E4DMA_CMD_* macros are command codes only defined in the
 *  SPIMSP432E4DMA.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/spi/SPIMSP432E4DMA.h>
 *  @endcode
 *  @{
 */

/* Add SPIMSP432E4DMA_CMD_* macros here */

/** @}*/

/* SPI function table pointer */
extern const SPI_FxnTable SPIMSP432E4DMA_fxnTable;

/*!
 *  @brief  SPIMSP432E4DMA Hardware attributes
 *
 *  These fields, with the exception of intPriority, are used by
 *  driverlib APIs and therefore must be populated by driverlib macro
 *  definitions. For MSP432E4 driverlib these definitions are found in:
 *      - ti/devices/msp432e4/inc/msp432e4xx.h
 *      - ti/devices/msp432e4/driverlib/interrupt.h
 *      - ti/devices/msp432e4/driverlib/udma.h
 *
 *  intPriority is the SPI peripheral's interrupt priority, as defined by the
 *  underlying OS.  It is passed unmodified to the underlying OS's interrupt
 *  handler creation code, so you need to refer to the OS documentation
 *  for usage.  For example, for SYS/BIOS applications, refer to the
 *  ti.sysbios.family.arm.m3.Hwi documentation for SYS/BIOS usage of
 *  interrupt priorities.  If the driver uses the ti.dpl interface
 *  instead of making OS calls directly, then the HwiP port handles the
 *  interrupt priority in an OS specific way.  In the case of the SYS/BIOS
 *  port, intPriority is passed unmodified to Hwi_create().
 *
 *  A sample structure is shown below:
 *  @code
 *  uint32_t spiMSP432E4DMAscratchBuf;
 *  const SPIMSP432E4DMA_HWAttrs spiMSP432E4DMAobjects[] = {
 *      {
 *          .baseAddr = SSI2_BASE,
 *          .intNum = INT_SSI2,
 *          .intPriority = (~0),
 *          .scratchBufPtr = &spiMSP432E4DMAscratchBuf,
 *          .defaultTxBufValue = 0,
 *          .rxDmaChannel = UDMA_CH12_SSI2RX,
 *          .txDmaChannel = UDMA_CH13_SSI2TX,
 *          .minDmaTransferSize = 10,
 *          .clkPinMask = SPIMSP432E4_PD3_SSI2CLK,
 *          .fssPinMask = SPIMSP432E4_PD2_SSI2FSS,
 *          .xdat0PinMask = SPIMSP432E4_PD1_SSI2XDAT0,
 *          .xdat1PinMask = SPIMSP432E4_PD0_SSI2XDAT1
 *      }
 *  };
 *  @endcode
 */
typedef struct SPIMSP432E4DMA_HWAttrs {
    /*! SSI Peripheral's base address */
    uint32_t  baseAddr;
    /*! SSI MSP432E4DMA Peripheral's interrupt vector */
    uint32_t  intNum;
    /*! SPIMSP432E4DMA Peripheral's interrupt priority */
    uint32_t  intPriority;

    /*! Address of a scratch buffer of size uint32_t */
    uint16_t *scratchBufPtr;
    /*! Default TX value if txBuf == NULL */
    uint16_t  defaultTxBufValue;

    /*! uDMA channel for RX data */
    uint32_t  rxDmaChannel;
    /*! uDMA channel for TX data */
    uint32_t  txDmaChannel;

    /*! Minimum amount of data frames for DMA transfer */
    uint16_t minDmaTransferSize;

    /*! Pin mask for SSI CLK */
    uint32_t  clkPinMask;
    /*! Pin mask for SSI FSS */
    uint32_t  fssPinMask;
    /*! Pin mask for SSI XDAT0 */
    uint32_t  xdat0PinMask;
    /*! Pin mask for SSI XDAT1 */
    uint32_t  xdat1PinMask;
} SPIMSP432E4DMA_HWAttrs;

/*!
 *  @brief  SPIMSP432E4DMA Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct SPIMSP432E4DMA_Object {
    HwiP_Handle          hwiHandle;
    SemaphoreP_Handle    transferComplete;
    SPI_CallbackFxn      transferCallbackFxn;
    SPI_Transaction     *transaction;
    UDMAMSP432E4_Handle  dmaHandle;

    size_t               amtDataXferred;
    size_t               currentXferAmt;
    uint32_t             bitRate;
    uint32_t             dataSize;
    uint32_t             transferTimeout;

    SPI_Mode             spiMode;
    SPI_TransferMode     transferMode;

    bool                 cancelInProgress;
    bool                 isOpen;
    uint8_t              format;
} SPIMSP432E4DMA_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_spi_SPIMSP432E4DMA__include */
