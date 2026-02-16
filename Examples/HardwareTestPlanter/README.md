# HardwareTestPlanter
# modified from patch_sm/HardwareTest from daisy examples library

Hardware verification test for Planter eurorack module

This is set up to use the Planter hardware to verify the Patch SM is functioning as expected.

To verify the various functionality:

* USB Serial port is used to print data
  * Read all 6 knob values 0-1
  * Read all 4 CV inputs
  * Gate input values
  * Toggle and Button State values
  * SDRAM Pass/Fail
  * QSPI Pass/Fail
  * SDMMC Pass/Fail
* If an SD Card is connected at bootup the it will write, and readback a text file to verify it functions.
* CV_OUT_1 and CV_OUT_2 will output 0.5Hz -5V to 5V ranp signal
* When midiflag is true, logging will pause and midi input via USB is enabled. 100Hz sine wave will be output, upon midi msg sine wave will update to midi defined frequency.
* LEDs will switch at rate of 0.5Hz
* The Gate Outputs will be outputting Squarewaves at 500mHz (one pulse per two seconds)
* Bootloader will be activated when button is pressed for >2s (serial monitoring must be active when midiflag is not enabled for this to work)

## Checking the values via USB

Use the Serial monitor VScode extension connected to the external USB-C