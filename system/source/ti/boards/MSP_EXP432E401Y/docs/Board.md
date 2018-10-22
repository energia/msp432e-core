# MSP-EXP432E401Y Settings and Resources

The MSP-EXP432E401Y board contains a MSP432E401Y device.

![](./images/MSP_EXP432E401Y.jpg "MSP-EXP432E401Y")

## Jumper Settings

* Set JP1 to 5V-XDS position & set JP2 to provide power to the MSP432E401Y device.
* Set JP4 and JP5 to UART0 position as indicated on the board's silkscreen to provide UART communications via the onboard USB debugger.
* If you use a BoosterPack with this board, the BoosterPack fits over J1 and J2.

## Board resources used in driver examples

The following table lists the Board names of the resources used by
the driver examples, and their corresponding pins.  Note that there may be
other resources configured in the board files that are not used by any
driver example.  Only the example resources are listed in the table.

  |Board Resource|Pin|
  |--------------|:---|
  |`Board_ADC0`|PE3|
  |`Board_ADC1`|PE2|
  |`Board_ADCBUF0CHANNEL0`|PE3|
  |`Board_GPIO_LED0`|PN1|
  |`Board_GPIO_LED1`|PN0|
  |`Board_GPIO_BUTTON0`|PJ0|
  |`Board_GPIO_BUTTON1`|PJ1|
  |`Board_I2C0`|PD1 - `SDA`, PD0 - `SCL`  (Some I2C examples require Booster packs)|
  |`Board_I2C_TMP`|PB3 - `SDA`, PB2 - `SCL`|
  |`Board_PWM0`|PF0|
  |`Board_SD0`|PD3 - `CLK`, PD1 - `XDAT0`, PD0 - `XDAT1`, PD2 - `FSS`, PC7 - `CS` (XDAT0 serves as TX and XDAT1 serves as RX)|
  |`Board_SDFatFS0`|PD3 - `CLK`, PD1 - `XDAT0`, PD0 - `XDAT1`, PD2 - `FSS`, PC7 - `CS` (XDAT0 serves as TX and XDAT1 serves as RX)|
  |`Board_SPI0`|PD3 - `CLK`, PD1 - `XDAT0`, PD0 - `XDAT1`, PD2 - `FSS` (XDAT0 serves as TX and XDAT1 serves as RX)|
  |`Board_SPI1`|PQ0 - `CLK`, PQ2 - `XDAT0`, PQ3 - `XDAT1`, PQ1 - `FSS` (XDAT0 serves as TX and XDAT1 serves as RX)|
  |`Board_SPI_MASTER`|PD3 - `CLK`, PD1 - `XDAT0`, PD0 - `XDAT1`, PD2 - `FSS` (XDAT0 serves as TX and XDAT1 serves as RX)|
  |`Board_SPI_SLAVE`|PD3 - `CLK`, PD1 - `XDAT0`, PD0 - `XDAT1`, PD2 - `FSS` (XDAT1 serves as TX and XDAT0 serves as RX)|
  |`Board_SPI_MASTER_READY`|PM3|
  |`Board_SPI_SLAVE_READY`|PL0|
  |`Board_UART0`|PA0 - `RX`, PA1 - `TX`  (UART provided through emulation, JP4 and JP5 jumpers must be set to UART0 position)|

## Booster packs

The following examples require booster packs.

  |Example|Booster Pack|
  |-------|:------------|
  |fatsd|[microSD Card BoosterPack](http://boardzoo.com/index.php/boosterpacks/microsd-boosterpack.html#.WBjQnXr9xv4) or [SD Card BoosterPack](http://store.43oh.com/index.php?route=product/product&path=64&product_id=66)|
  |fatsdraw|[microSD Card BoosterPack](http://boardzoo.com/index.php/boosterpacks/microsd-boosterpack.html#.WBjR0nr9xv4) or [SD Card BoosterPack](http://store.43oh.com/index.php?route=product/product&path=64&product_id=66)|
  |i2ctmp007|[BOOSTXL-SENSORS Sensors BoosterPack](http://www.ti.com/tool/boostxl-sensors)|
  |i2ctpl0401evm|[TPL0401 EVM board](http://www.ti.com/tool/tpl0401evm)|
  |portable|[BOOSTXL-SENSORS Sensors BoosterPack](http://www.ti.com/tool/boostxl-sensors)|
  |sdraw|[microSD Card BoosterPack](http://boardzoo.com/index.php/boosterpacks/microsd-boosterpack.html#.WBjQnXr9xv4) or [SD Card BoosterPack](http://store.43oh.com/index.php?route=product/product&path=64&product_id=66)|
  |display|[430BOOST-SHARP96 LCD BoosterPack](http://www.ti.com/tool/430boost-sharp96)|

## Peripherals Used

The following list shows which MSP_EXP432E401Y peripherals are used by
driver and kernel applications. Driver examples control which peripherals (and which ports) are used.

* __TI-RTOS Kernel (SYS/BIOS).__ Uses the first general-purpose timer available and that timer's associated interrupts. Generally, this will be Timer0. The TI-RTOS Kernel manages the interrupt controller statically without an interrupt dispatcher.
* __Drivers.__
    * __I<sup>2</sup>C:__ The I<sup>2</sup>C driver is configured on I2C0 and I2C7 to support various BoosterPacks.
    * __NVS:__ The `Board_NVSINTERNAL` region uses on-chip flash memory. This NVS region is defined in the example application's board file.
    * __PWM:__ The PWM driver uses an onboard LED (PF0). These pins are configured for the PWM driver. While these pins can also be used by the GPIO driver, your application's board file must ensure that the pins are not simultaneously used by the GPIO and PWM drivers.
    * __SD:__ The SD driver is built on the GPIO & SPI drivers to communicate with a SD card via SPI.  `Board_SD0` uses `Board_SPI0` to send data to-and-from the SD card.
    * __SDFatFS:__ The SDFatFS driver relies on a SD driver instance to communicate with a SD card; `Board_SDFatFS0` uses the `Board_SD0` driver instance.
    * __SPI:__ The SPI driver is configured to use SSI2 and SSI3 for SPI communications.
    * __Timer:__ The Timer, PWM and Capture driver use the timer peripheral.
    * __UART:__ The UART driver uses UART0, which is attached to the onboard emulator to facilitate serial communications.
    * __Watchdog:__ The Watchdog driver example uses the Watchdog Timer peripheral.
