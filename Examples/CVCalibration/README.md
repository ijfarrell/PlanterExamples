# CVCalibration

Control Voltage IO calibration
Author: Isaiah Farrell

This is set up to calibrate the CV IO functions for the Planter module in plater.cpp

There are two primary calibration blocks, input and output

INPUT CALIBRATION
  * When panel LED Red, calibrating CV_OUT_1. When panel LED Blue, calibrating CV_OUT2. Also denoted in logging output.
  * Toggle positions calibrate high (5V) mid (0V) and low (-5V) outputs

    * Toggle 1 up, Toggle 2 and 3 down - High calibration (cal_dac_high), controlled by knobs 1 and 4 (coarse/fine)
      * configure knobs until voltage output = 5V
      * note log CAL CV_OUT_LOW, add to WriteCvOutBipolar function in planter.cpp for corresponding channel (CV_OUT_1, CV_OUT_2)

    * Toggle 2 up, Toggle 1 and 3 down - Mid calibration (cal_dac_mid), controlled by knobs 2 and 5 (coarse/fine)
      * configure knobs until voltage output = 0V
      * note log CAL CV_OUT_LOW, add to WriteCvOutBipolar function in planter.cpp for corresponding channel (CV_OUT_1, CV_OUT_2)

    * Toggle 3 up, Toggle 1 and 2 down - Low calibration (cal_dac_low), controlled by knobs 3 and 6 (coarse/fine)
      * configure knobs until voltage output = -5V
      * note log CAL CV_OUT_LOW, add to WriteCvOutBipolar function in planter.cpp for corresponding channel (CV_OUT_1, CV_OUT_2)

  * Use precision multimeter with at least 1mv reading @ +/- 5V for 1V/oct tuning
  * When calibrating CV_OUT_1 or CV_OUT_2 other output reflects calibrated output from palnter::WriteCvOutBipolar function

OUTPUT CALIBRATION
  * Inputs are calibrated using calibrated DC voltage, this can be sourced from CV_OUT calibrated with a precision multimeter
  * When all toggles are down, knobs are used for input calibration mode with further variable voltage
    * CV_OUT_1 is tuned with knobs 1, 2, 3 (coarse, medium, fine), from ~-8V to ~8V
    * CV_OUT_2 is tuned with knobs 2, 3, 4 (coarse, medium, fine), from ~-8V to ~8V
  * Each CV input must be measured at the following voltages and documented
    * -4V, low calibration: cal_adc_low, reading should be approx -0.8
    * 0V, mid calibration: cal_adc_mid, reading should be approx 0.0
    * 4V, high calibration: cal_adc_high, reading should be approx 0.8
  * CALIBRATION VALUES SHOULD BE TAKEN FROM "RAW CV_", "CAL CV_" reflects applied calibration data with GetCvIn function

After calibration data is entered and saved into planter.cpp, calibration should be validated by measuring voltage outputs on CV_OUTs and DAC logging values on CV ins

* Bootloader will be activated when button is pressed for >3s (serial monitoring must be active in this script for this to work)
  
