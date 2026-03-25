#include "planter.h"
#include "planter_cal.h"

using anachrome::planter;
using daisy::System;
using namespace anachrome::calibration;

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

      // Fixed reference voltages (same for all units)
      constexpr float cal_voltage_low  = -5.0f;
      constexpr float cal_voltage_mid  =  0.0f;
      constexpr float cal_voltage_high =  5.0f;

      // Per-unit measured DAC values from planter_calibration.h
      float cal_dac_low, cal_dac_mid, cal_dac_high;

      if (channel == 1){
        cal_dac_low  = CAL.dac_ch1_low;
        cal_dac_mid  = CAL.dac_ch1_mid;
        cal_dac_high = CAL.dac_ch1_high;
      }
      else{
        cal_dac_low  = CAL.dac_ch2_low;
        cal_dac_mid  = CAL.dac_ch2_mid;
        cal_dac_high = CAL.dac_ch2_high;
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

      // Fixed normalized output targets (same for all units)
      // Calibrate at ±4V (not ±5V) to avoid ADC saturation; piecewise linear extrapolates to ±1.0
      constexpr float cal_out_high =  0.8f;   // 4V / 5V
      constexpr float cal_out_mid  =  0.0f;
      constexpr float cal_out_low  = -0.8f;   // -4V / 5V

      // Per-unit measured ADC readings from planter_calibration.h
      float cal_adc_low, cal_adc_mid, cal_adc_high;

      switch(channel)
      {
          case 0: // CV_1
              cal_adc_high = CAL.adc_ch0_high;
              cal_adc_mid  = CAL.adc_ch0_mid;
              cal_adc_low  = CAL.adc_ch0_low;
              break;
          case 1: // CV_2
              cal_adc_high = CAL.adc_ch1_high;
              cal_adc_mid  = CAL.adc_ch1_mid;
              cal_adc_low  = CAL.adc_ch1_low;
              break;
          case 2: // CV_3
              cal_adc_high = CAL.adc_ch2_high;
              cal_adc_mid  = CAL.adc_ch2_mid;
              cal_adc_low  = CAL.adc_ch2_low;
              break;
          case 3: // CV_4
              cal_adc_high = CAL.adc_ch3_high;
              cal_adc_mid  = CAL.adc_ch3_mid;
              cal_adc_low  = CAL.adc_ch3_low;
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
      // Read filtered value from AnalogControl
      // Knobs 1-4 (indices 4-7): bipolar-initialized channels, output range depends on 3V3 pot into ±5V circuit
      // Knobs 5-6 (indices 8-9): unipolar-initialized channels, output range ~0-0.5 for 3V3 pot
      float raw = patch.GetAdcValue(index);

      // Per-unit measured min/max from planter_calibration.h
      float cal_raw_low, cal_raw_high;

      switch(index)
      {
          case KNOB_1:
              cal_raw_low  = CAL.knob1_low;
              cal_raw_high = CAL.knob1_high;
              break;
          case KNOB_2:
              cal_raw_low  = CAL.knob2_low;
              cal_raw_high = CAL.knob2_high;
              break;
          case KNOB_3:
              cal_raw_low  = CAL.knob3_low;
              cal_raw_high = CAL.knob3_high;
              break;
          case KNOB_4:
              cal_raw_low  = CAL.knob4_low;
              cal_raw_high = CAL.knob4_high;
              break;
          case KNOB_5:
              cal_raw_low  = CAL.knob5_low;
              cal_raw_high = CAL.knob5_high;
              break;
          case KNOB_6:
              cal_raw_low  = CAL.knob6_low;
              cal_raw_high = CAL.knob6_high;
              break;
          default:
              return raw;
      }

      // Linear interpolation from [cal_raw_low, cal_raw_high] -> [0.0, 1.0]
      float scaled = (raw - cal_raw_low) / (cal_raw_high - cal_raw_low);

      // Clamp to 0-1
      if(scaled < 0.0f) scaled = 0.0f;
      if(scaled > 1.0f) scaled = 1.0f;

      return scaled;
  }

    void planter::BootloaderResetCheck() {
        // Keep this check self-contained so bootloader entry remains reliable
        // even if caller debounce cadence changes.
        button.Debounce();

        static uint32_t hold_start_ms = 0;
        static bool     armed         = false;

        // Use raw state as the primary hold detector; polarity is already handled
        // by Switch::RawState() for the configured button type.
        const bool pressed_raw = button.RawState();

        if(pressed_raw)
        {
            if(!armed)
            {
                armed         = true;
                hold_start_ms = System::GetNow();
            }
        }
        else
        {
            armed         = false;
            hold_start_ms = 0;
        }

        const bool held_3s_raw = armed && (System::GetNow() - hold_start_ms >= 3000);
        const bool held_3s_db  = (button.TimeHeldMs() >= 300.0f);

        if (held_3s_raw || held_3s_db) {
                armed         = false;
                hold_start_ms = 0;

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

