#pragma once

/**
 * @file planter_calibration.h
 * @brief Per-unit calibration data for all Planter hardware units.
 *
 * All calibration values for every unit are stored in a single lookup table.
 * The active unit is selected at compile time via PLANTER_SERIAL (defined in
 * the project Makefile, e.g. CPPFLAGS += -DPLANTER_SERIAL=1).
 *
 * To add a new unit:
 *   1. Assign it the next serial number.
 *   2. Add a row to the UNIT_TABLE at the bottom of this file.
 *   3. Build with -DPLANTER_SERIAL=<number>.
 *
 * DAC calibration (WriteCvOutBipolar):
 *   Set WriteCvOut(channel, dac_val), measure actual output, record here.
 *   Three-point piecewise linear: low (-5V), mid (0V), high (+5V).
 *
 * ADC calibration (GetCvIn):
 *   Apply known voltages to CV inputs, record raw ADC readings here.
 *   Calibrate at ±4V (not ±5V) to avoid ADC saturation/clamping.
 *   Three-point piecewise linear: low (-4V), mid (0V), high (+4V).
 *
 * Knob calibration (GetKnobValue):
 *   Record GetAdcValue() at fully CCW and fully CW positions.
 *   Knobs 1-4 are bipolar ADC channels (CV_5-CV_8).
 *   Knobs 5-6 are unipolar ADC channels (ADC_9-ADC_10).
 */

#ifndef PLANTER_SERIAL
#define PLANTER_SERIAL 0 // Default to 0 if not defined, which will trigger an error in the static_assert below.
#endif

namespace anachrome
{
namespace calibration
{

/** All per-unit measured calibration values in one struct. */
struct CalibrationData
{
    int serial;

    // DAC output calibration
    float dac_ch1_low, dac_ch1_mid, dac_ch1_high;
    float dac_ch2_low, dac_ch2_mid, dac_ch2_high;

    // ADC input calibration (CV_1 through CV_4)
    float adc_ch0_low, adc_ch0_mid, adc_ch0_high;
    float adc_ch1_low, adc_ch1_mid, adc_ch1_high;
    float adc_ch2_low, adc_ch2_mid, adc_ch2_high;
    float adc_ch3_low, adc_ch3_mid, adc_ch3_high;

    // Knob calibration (KNOB_1 through KNOB_6)
    float knob1_low, knob1_high;
    float knob2_low, knob2_high;
    float knob3_low, knob3_high;
    float knob4_low, knob4_high;
    float knob5_low, knob5_high;
    float knob6_low, knob6_high;
};

// ============================================================================
//  Unit Table — one row per hardware unit
//  Format:
//    { serial,
//      dac1_low, dac1_mid, dac1_high,  dac2_low, dac2_mid, dac2_high,
//      adc0_low, adc0_mid, adc0_high,  adc1_low, adc1_mid, adc1_high,
//      adc2_low, adc2_mid, adc2_high,  adc3_low, adc3_mid, adc3_high,
//      knob1_low, knob1_high,  knob2_low, knob2_high,
//      knob3_low, knob3_high,  knob4_low, knob4_high,
//      knob5_low, knob5_high,  knob6_low, knob6_high }
// ============================================================================
constexpr CalibrationData UNIT_TABLE[] = {
    // --- Serial 0 default values ---
    // {   0,
    //     // DAC: ch1 low/mid/high, ch2 low/mid/high
    //     4.2f, 2.5f, 0.8f,   4.2f, 2.5f, 0.8f,
    //     // ADC: ch0 low/mid/high, ch1, ch2, ch3
    //    -0.8f, 0.0f, 0.8f,  -0.8f, 0.0f, 0.8f,
    //    -0.8f, 0.0f, 0.8f,  -0.8f, 0.0f, 0.8f,
    //     // Knobs: 1-6 low/high
    //     0.0f, 0.65f,   0.0f, 0.65f,
    //     0.0f, 0.65f,   0.0f, 0.65f,
    //     0.0f, 1.0f,   0.0f, 1.0f,
    // },
    {   0,
        // DAC: ch1 low/mid/high, ch2 low/mid/high
        5.0f, 2.5f, 0.0f,   5.0f, 2.5f, 0.0f,
        // ADC: ch0 low/mid/high, ch1, ch2, ch3
       -0.8f, 0.0f, 0.8f,  -0.8f, 0.0f, 0.8f,
       -0.8f, 0.0f, 0.8f,  -0.8f, 0.0f, 0.8f,
        // Knobs: 1-6 low/high
        0.0f, 0.65f,   0.0f, 0.65f,
        0.0f, 0.65f,   0.0f, 0.65f,
        0.0f, 1.0f,   0.0f, 1.0f,
    },
    {   1,
        // DAC: ch1 low/mid/high, ch2 low/mid/high
        4.144f, 2.552f, 0.964f,   4.140f, 2.550f, 0.963f,
        // ADC: ch0 low/mid/high, ch1, ch2, ch3
       -0.7918f, 0.0056f, 0.7994f,  -0.7964f, 0.0016f, 0.7976f,
       -0.7974f, 0.0031f, 0.7971f,  -0.7952f, 0.0051f, 0.7984f,
        // Knobs: 1-6 low/high
        0.0043f, 0.6482f,   0.0032f, 0.6460f,
        0.0082f, 0.6472f,   0.0054f, 0.6478f,
        0.000f, 0.9685f,   0.000f, 0.9685f,
    },  
    {   2,
        // DAC: ch1 low/mid/high, ch2 low/mid/high
        4.258f, 2.5497f, 0.842f,   4.265f, 2.551f, 0.837f,
        // ADC: ch0 low/mid/high, ch1, ch2, ch3
       -0.7793f, 0.0159f, 0.8092f,  -0.7858f, 0.0082f, 0.8011f,
       -0.7820f, 0.0123f, 0.8036f,  -0.8011f, 0.0015f, 0.7994f,
        // Knobs: 1-6 low/high
        0.0066f, 0.6478f,   0.0084f, 0.6470f,
        0.0078f, 0.6466f,   0.0037f, 0.6451f,
        0.000f, 1.000f,   0.000f, 1.000f,
    }, 
    {   3,
        // DAC: ch1 low/mid/high, ch2 low/mid/high
        4.284f, 2.564f, 0.843f,   4.289f, 2.568f, 0.849f,
        // ADC: ch0 low/mid/high, ch1, ch2, ch3
       -0.7937f, 0.0061f, 0.8071f,  -0.8014f, -0.0029f, 0.7986f,
       -0.7988f, 0.0002f, 0.7982f,  -0.8033f, -0.0013f, 0.7981f,
        // Knobs: 1-6 low/high
        0.0054f, 0.6458f,   0.0053f, 0.6464f,
        -0.0009f, 0.6419f,   0.0011f, 0.6437f,
        0.000f, 0.9686f,   0.000f, 0.9686f,
    }, 
    {   4,
        // DAC: ch1 low/mid/high, ch2 low/mid/high
        4.2565f, 2.5476f, 0.8388f,   4.2567f, 2.5483f, 0.8402f,
        // ADC: ch0 low/mid/high, ch1, ch2, ch3
       -0.7827f, 0.0094f, 0.8007f,  -0.7751f, 0.0183f, 0.8172f,
       -0.7832f, 0.0075f, 0.8043f,  -0.7861f, 0.0085f, 0.8015f,
        // Knobs: 1-6 low/high
        0.006f, 0.646f,   0.008f, 0.651f,
        0.011f, 0.653f,   0.009f, 0.651f,
        0.000f, 0.969f,   0.000f, 0.969f,
    },  
    // --- Add new units here ---
    // {   2,
    //     dac1_lo, dac1_mid, dac1_hi,   dac2_lo, dac2_mid, dac2_hi,
    //     adc0_lo, adc0_mid, adc0_hi,   adc1_lo, adc1_mid, adc1_hi,
    //     adc2_lo, adc2_mid, adc2_hi,   adc3_lo, adc3_mid, adc3_hi,
    //     k1_lo, k1_hi,   k2_lo, k2_hi,
    //     k3_lo, k3_hi,   k4_lo, k4_hi,
    //     k5_lo, k5_hi,   k6_lo, k6_hi,
    // },
};

// Map PLANTER_SERIAL to a constant table index.
// Add an #elif for each new unit in the same order as UNIT_TABLE above.
#if   PLANTER_SERIAL == 1
constexpr int ACTIVE_INDEX = 1;
#elif PLANTER_SERIAL == 2
constexpr int ACTIVE_INDEX = 2;
#elif PLANTER_SERIAL == 3
constexpr int ACTIVE_INDEX = 3;
#elif PLANTER_SERIAL == 4
constexpr int ACTIVE_INDEX = 4;
#else
 constexpr int ACTIVE_INDEX = 0;
#endif

/** The active calibration data, selected at compile time via ACTIVE_INDEX. */
const CalibrationData CAL = UNIT_TABLE[ACTIVE_INDEX];

}; // namespace calibration
} // namespace anachrome
