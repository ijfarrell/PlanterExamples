#include "planter.h"

using anachrome::planter;
using daisy::System;

namespace anachrome
{

  void planter::StartAudio(AudioHandle::InterleavingAudioCallback cb) {
    patch.StartAudio(cb);
  }

  void planter::StartAudio(AudioHandle::AudioCallback cb) {
    patch.StartAudio(cb);
  }

  // void planter::StartAdc() { patch.adc.Start(); }

  // void planter::StopAdc() { patch.adc.Stop(); }

  
  void planter::WriteCvOutBipolar(const int channel, float bipolar_voltage)
  {
      // Clamp desired voltage to valid range
      if(bipolar_voltage > 5.0f)
          bipolar_voltage = 5.0f;
      else if(bipolar_voltage < -5.0f)
          bipolar_voltage = -5.0f;

      // Calibration points: (desired_output_voltage, dac_input_value_to_achieve_it)
      // These should be measured using a multimeter on the output with known DAC inputs
      // Collect by: set WriteCvOut(channel, dac_val), measure actual output, record as calibration
      float cal_voltage_low, cal_dac_low, cal_voltage_mid, cal_dac_mid, cal_voltage_high, cal_dac_high;

      if (channel == 1){
        // Calibration points for CV_OUT_1
        cal_voltage_low = -5.0f;    // Desired voltage
        cal_dac_low = 4.146f;       // DAC input value to get -5.0V out (change based on measurement)
        cal_voltage_mid = 0.0f;     // Desired voltage
        cal_dac_mid = 2.556f;         // DAC input value to get 0.0V out
        cal_voltage_high = 5.0f;    // Desired voltage
        cal_dac_high = 0.967f;        // DAC input value to get +5.0V out
      }
      else{
        // Calibration points for CV_OUT_2
        cal_voltage_low = -5.0f;    // Desired voltage
        cal_dac_low = 4.141f;       // DAC input value to get -5.0V out (change based on measurement)
        cal_voltage_mid = 0.0f;     // Desired voltage
        cal_dac_mid = 2.552f;         // DAC input value to get 0.0V out
        cal_voltage_high = 5.0f;    // Desired voltage
        cal_dac_high = 0.965f;        // DAC input value to get +5.0V out
      }
 
      // Piecewise linear interpolation
      float dac_output;
      if(bipolar_voltage <= cal_voltage_mid)
      {
          // Interpolate between low and mid points
          float t = (bipolar_voltage - cal_voltage_low) / (cal_voltage_mid - cal_voltage_low);
          dac_output = cal_dac_low + t * (cal_dac_mid - cal_dac_low);
      }
      else
      {
          // Interpolate between mid and high points
          float t = (bipolar_voltage - cal_voltage_mid) / (cal_voltage_high - cal_voltage_mid);
          dac_output = cal_dac_mid + t * (cal_dac_high - cal_dac_mid);
      }

      // Clamp DAC output to valid 0-5V range
      if(dac_output > 5.0f)
          dac_output = 5.0f;
      else if(dac_output < 0.0f)
          dac_output = 0.0f;

      patch.WriteCvOut(channel, dac_output);
  }

  float planter::GetCvIn(int channel)
  {
      // Read raw ADC value from GetAdcValue (returns -1.0 to 1.0 for -5V to +5V)
      float raw = patch.GetAdcValue(channel);

      // Per-channel calibration points: (measured_adc_reading, normalized_output -1 to 1)
      // NOTE: Calibrate at ±4V instead of ±5V to avoid ADC saturation/clamping
      // Piecewise linear interpolation will extrapolate to ±1.0 for ±5V range
      float cal_adc_low, cal_out_low, cal_adc_mid, cal_out_mid, cal_adc_high, cal_out_high;

      switch(channel)
      {
          case 0: // CV_1
              cal_adc_high = 0.7959f;   // ADC reading when +4.0V applied (extrapolates to +1.0 at 5V)
              cal_out_high = 0.8f;      // Normalized output (4V / 5V = 0.8)
              cal_adc_mid  = -0.0267f;   // ADC reading when 0.0V applied
              cal_out_mid  = 0.0f;      // Normalized output
              cal_adc_low  = -0.8479f;  // ADC reading when -4.0V applied (extrapolates to -1.0 at 5V)
              cal_out_low  = -0.8f;     // Normalized output (-4V / 5V = -0.8)
              break;
          case 1: // CV_2
              cal_adc_high = 0.7934f;   // ADC reading when +4.0V applied
              cal_out_high = 0.8f;
              cal_adc_mid  = -0.0312f;   // ADC reading when 0.0V applied
              cal_out_mid  = 0.0f;
              cal_adc_low  = -0.8551f;  // ADC reading when -4.0V applied
              cal_out_low  = -0.8f;
              break;
          case 2: // CV_3
              cal_adc_high = 0.7924f;   // ADC reading when +4.0V applied
              cal_out_high = 0.8f;
              cal_adc_mid  = -0.0311f;   // ADC reading when 0.0V applied
              cal_out_mid  = 0.0f;
              cal_adc_low  = -0.8543f;  // ADC reading when -4.0V applied
              cal_out_low  = -0.8f;
              break;
          case 3: // CV_4
              cal_adc_high = 0.7937f;   // ADC reading when +4.0V applied
              cal_out_high = 0.8f;
              cal_adc_mid  = -0.0291f;   // ADC reading when 0.0V applied
              cal_out_mid  = 0.0f;
              cal_adc_low  = -0.8517f;  // ADC reading when -4.0V applied
              cal_out_low  = -0.8f;
              break;
          default:
              // No calibration for out-of-range channels
              return raw;
      }

      // Piecewise linear interpolation
      float calibrated;
      if(raw <= cal_adc_mid)
      {
          // Interpolate between low and mid
          float t = (raw - cal_adc_low) / (cal_adc_mid - cal_adc_low);
          calibrated = cal_out_low + t * (cal_out_mid - cal_out_low);
      }
      else
      {
          // Interpolate between mid and high
          float t = (raw - cal_adc_mid) / (cal_adc_high - cal_adc_mid);
          calibrated = cal_out_mid + t * (cal_out_high - cal_out_mid);
      }

      // Clamp to valid -1 to 1 range
      if(calibrated > 1.0f)  calibrated = 1.0f;
      if(calibrated < -1.0f) calibrated = -1.0f;

      return calibrated;
  }

  /** Fixed-function Digital I/O */
  void planter::InitLeds()
  {
    led_1.Init(patch.D8, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
    led_2.Init(patch.D9, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
  }

  void planter::SetLed(const int idx, bool state) { 
    if (idx == 1)
      led_1.Write(state);
    else
      led_2.Write(state);
  }

  float planter::GetKnobValue(int index)
  {
      if(index >= 4 && index < 8){
      // Get raw 16-bit value
      uint16_t raw = patch.adc.Get(index);;
      // Normalize to 0-1 and invert due to the inverting amplifier in CV circuit
      float normalized = (float)raw / 65535.0f;
      float scaled = ((0.513f - normalized) * 3.03f);
      
      // Clamp to 0-1 to handle op-amp offset drift
      if(scaled < 0.0f) scaled = 0.0f;
      if(scaled > 1.0f) scaled = 1.0f;
      
      return scaled;
      }
      else{
        return patch.GetAdcValue(index);
      }
  }

  void planter::BootloaderResetCheck() {
    if (button.TimeHeldMs() >= 3000) {
        patch.StopAdc();
        patch.StopAudio();
        
        planter::InitLeds();

        // Flash front panel LEDS 4 times
        for (int i = 0; i < 4; i++) {
          SetLed(1, true);
          SetLed(2, false);
          System::Delay(100);

          SetLed(1, false);
          SetLed(2, true);
          System::Delay(100);
        }

        // Reset system to bootloader
        System::ResetToBootloader(System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
      }

  }

}

