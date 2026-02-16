#pragma once

#include "daisy_patch_sm.h"


using daisy::AdcChannelConfig;
using daisy::AnalogControl;
using daisy::AudioHandle;
//using daisy::patch_sm;
using daisy::GPIO;
using daisy::Pin;
using daisy::SaiHandle;
using daisy::Switch;

namespace anachrome
{
/**
 * Minimal Planter platform (wrapper around daisy::patch_sm::DaisyPatchSM).
 * This provides a small, extendable class with a `patch` member and a single
 * maintained helper: WriteCvOutBipolar. Add features later (knobs, switches)
 * as needed.
 */
class planter
{
    public:
    enum
    {
        /** Knobs */
        KNOB_1 = 4, 
        KNOB_2, 
        KNOB_3, 
        KNOB_4, 
        KNOB_5, 
        KNOB_6, 
        KNOB_LAST
    };

    enum
    {
        /** User LEDs */
        LED_1, 
        LED_2, 
    };


    public:
        void StartAudio(AudioHandle::InterleavingAudioCallback cb);

        void StartAudio(AudioHandle::AudioCallback cb);

        void StopAdc();

        planter() = default;
        ~planter() = default;


        void Init() {
        // Initialize the hardware.
        patch.Init();
        
        InitLeds();
        button.Init(daisy::patch_sm::DaisyPatchSM::B7, patch.AudioCallbackRate());
        toggle1.Init(daisy::patch_sm::DaisyPatchSM::B8, patch.AudioCallbackRate());
        toggle2.Init(daisy::patch_sm::DaisyPatchSM::D1, patch.AudioCallbackRate());
        toggle3.Init(daisy::patch_sm::DaisyPatchSM::D10, patch.AudioCallbackRate());    
        
        // InitSwitches();
        // InitAnalogControls();
        patch.SetAudioBlockSize(48);
        }

        /** Convert bipolar (-5..+5V) to 0..5V and write to the specified channel */
        void WriteCvOutBipolar(const int channel, float bipolar_voltage);

        /** Read CV input (0-3 for CV_1 through CV_4) and return calibrated voltage (-5..+5V) */
        float GetCvIn(int channel);

        /** Init LED gpio*/
        GPIO led_1;
        GPIO led_2;
        void InitLeds();
        /** Toggle LEDs, idx LED enum*/
        void SetLed(const int idx, bool state);

        float GetKnobValue(int index);

        void BootloaderResetCheck();
        
        /** Print formatted debug log message
         */
        template <typename... VA>
        static void Print(const char* format, VA... va)
        {
            Log::Print(format, va...);
        }

        /** Print formatted debug log message with automatic line termination
         */
        template <typename... VA>
        static void PrintLine(const char* format, VA... va)
        {
            Log::PrintLine(format, va...);
        }

        /** Start the logging session. Optionally wait for terminal connection before proceeding.
         */
        static void StartLog(bool wait_for_pc = false)
        {
            Log::StartLog(wait_for_pc);
        }


        /** Button and toggle switch members */
        daisy::Switch button;
        daisy::Switch toggle1;
        daisy::Switch toggle2;
        daisy::Switch toggle3;

        

        // Expose the Daisy Patch SM object so users can extend functionality.
        daisy::patch_sm::DaisyPatchSM patch;

    private:
        using Log = daisy::Logger<daisy::LoggerDestination::LOGGER_EXTERNAL>;
};

} // namespace anachrome

