#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "planter.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace anachrome;

DaisyPatchSM   hw;
// Switch         button, toggle1, toggle2, toggle3;
// #define DSY_TEXT __attribute__((section(".text")))
// DSY_TEXT FIL            file; /**< Can't be made on the stack (DTCMRAM) */
// DSY_TEXT FatFSInterface fsi;
// MidiUsbHandler midi;
Oscillator     osc;
// dsy_gpio led_1, led_2;
planter euro;

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    euro.patch.ProcessAllControls();
    euro.button.Debounce();
    euro.toggle1.Debounce();
    euro.toggle2.Debounce();
    euro.toggle3.Debounce();
    // for(size_t i = 0; i < size; i++)
    // {
    //     out[0][i] = in[0][i];
    //     out[1][i] = in[1][i];
    // }
}

int main(void)
{
    euro.Init();    
    euro.StartLog(true);
    System::Delay(1000);
    euro.PrintLine("Planter started. Test Beginning");
    euro.PrintLine("PLANTER_SERIAL=%d", PLANTER_SERIAL);
    euro.PrintLine("CAL serial=%d", anachrome::calibration::CAL.serial);

    System::Delay(1000);

    euro.patch.StartAudio(AudioCallback);

    uint32_t now, usbt;
    usbt = now = System::GetNow();

    bool led1_state = true;
    bool led2_state = false;
    bool cv_out_state = true;
    bool last_button_state = false;

    euro.SetLed(1, led1_state);
    euro.SetLed(2, led2_state);

    while(1)
    {
        now = System::GetNow();

        // Detect rising edge by comparing button state frame-to-frame, rising edge detect without audiocallback rate
        bool current_button_state = euro.button.Pressed();
        if(current_button_state && !last_button_state)
        {
            led1_state = !led1_state;
            led2_state = !led2_state;
            euro.SetLed(1, led1_state);
            euro.SetLed(2, led2_state);
            cv_out_state = !cv_out_state;
        }
        last_button_state = current_button_state;

        // WHEN LED RED, CV_OUT_1 IS BEING CONTROLLED BY KNOBS and CV_OUT_2 IS CAL TESTED
        // WHEN LED BLUE, CV_OUT_2 IS BEING CONTROLLED BY KNOBS AND CV_OUT_1 IS CAL TESTED

        if(euro.toggle1.Pressed() == true && euro.toggle2.Pressed() == false && euro.toggle3.Pressed() == false){
            if(cv_out_state)
            {
                euro.WriteCvOutBipolar(CV_OUT_1, 5);
                euro.patch.WriteCvOut(CV_OUT_2, euro.GetKnobValue(planter::KNOB_1) + (euro.GetKnobValue(planter::KNOB_4) * 0.02));
            }
            else
            {
                euro.WriteCvOutBipolar(CV_OUT_2, 5);
                euro.patch.WriteCvOut(CV_OUT_1, euro.GetKnobValue(planter::KNOB_1) + (euro.GetKnobValue(planter::KNOB_4) * 0.02));
            }
        }
        if(euro.toggle1.Pressed() == false && euro.toggle2.Pressed() == true && euro.toggle3.Pressed() == false){
            if(cv_out_state)
            {
                euro.WriteCvOutBipolar(CV_OUT_1, 0);
                euro.patch.WriteCvOut(CV_OUT_2, euro.GetKnobValue(planter::KNOB_2) + (euro.GetKnobValue(planter::KNOB_5) * 0.02)+ 2.0f);
            }
            else
            {
                euro.WriteCvOutBipolar(CV_OUT_2, 0);
                euro.patch.WriteCvOut(CV_OUT_1, euro.GetKnobValue(planter::KNOB_2) + (euro.GetKnobValue(planter::KNOB_5) * 0.02)+ 2.0f);
            }
        }

        if(euro.toggle1.Pressed() == false && euro.toggle2.Pressed() == false && euro.toggle3.Pressed() == true){
            if(cv_out_state)
            {
                euro.WriteCvOutBipolar(CV_OUT_1, -5); 
                euro.patch.WriteCvOut(CV_OUT_2, euro.GetKnobValue(planter::KNOB_3) + (euro.GetKnobValue(planter::KNOB_6) * 0.02) + 4.0f);
            }
            else
            {
                euro.WriteCvOutBipolar(CV_OUT_2, -5); 
                euro.patch.WriteCvOut(CV_OUT_1, euro.GetKnobValue(planter::KNOB_3) + (euro.GetKnobValue(planter::KNOB_6) * 0.02) + 4.0f);
            }
        }

        if(euro.toggle1.Pressed() == false && euro.toggle2.Pressed() == false && euro.toggle3.Pressed() == false){
                euro.patch.WriteCvOut(CV_OUT_2, (euro.GetKnobValue(planter::KNOB_4) *4) + euro.GetKnobValue(planter::KNOB_5) + (euro.GetKnobValue(planter::KNOB_6) * 0.02));
                euro.patch.WriteCvOut(CV_OUT_1, (euro.GetKnobValue(planter::KNOB_1) *4) + euro.GetKnobValue(planter::KNOB_2) + (euro.GetKnobValue(planter::KNOB_3) * 0.02));
        }

        if(now - usbt > 100)
        {

            euro.PrintLine("######################");
            euro.PrintLine("Analog Out:");
            euro.PrintLine("CALIBRATING: %s", cv_out_state ? "CV_OUT_2" : "CV_OUT_1");

            float cal1 = euro.GetKnobValue(planter::KNOB_1) + (euro.GetKnobValue(planter::KNOB_4) * 0.02);
            euro.PrintLine("VAL CV_OUT_HIGH: %1.3f", cal1);

            float cal2 = euro.GetKnobValue(planter::KNOB_2) + (euro.GetKnobValue(planter::KNOB_5) * 0.02)+ 2.0f;
            euro.PrintLine("VAL CV_OUT_MID: %1.3f", cal2);
            
            float cal3 = euro.GetKnobValue(planter::KNOB_3) + (euro.GetKnobValue(planter::KNOB_6) * 0.02) + 4.0f;
            euro.PrintLine("VAL CV_OUT_LOW: %1.3f", cal3);

            
            euro.PrintLine("Analog In:");

            euro.PrintLine("CAL CV1: %1.4f", euro.GetCvIn(CV_1));
            euro.PrintLine("RAW CV1: %1.4f", euro.patch.GetAdcValue(CV_1));

            euro.PrintLine("CAL CV2: %1.4f", euro.GetCvIn(CV_2));
            euro.PrintLine("RAW CV2: %1.4f", euro.patch.GetAdcValue(CV_2));

            euro.PrintLine("CAL CV3: %1.4f", euro.GetCvIn(CV_3));
            euro.PrintLine("RAW CV3: %1.4f", euro.patch.GetAdcValue(CV_3));

            euro.PrintLine("CAL CV4: %1.4f", euro.GetCvIn(CV_4));
            euro.PrintLine("RAW CV4: %1.4f", euro.patch.GetAdcValue(CV_4));

            euro.PrintLine("Knobs:");

            euro.PrintLine("CAL Knob1: %1.4f", euro.GetKnobValue(planter::KNOB_1));
            euro.PrintLine("RAW Knob1: %1.4f", euro.patch.GetAdcValue(planter::KNOB_1));

            euro.PrintLine("CAL Knob2: %1.4f", euro.GetKnobValue(planter::KNOB_2));
            euro.PrintLine("RAW Knob2: %1.4f", euro.patch.GetAdcValue(planter::KNOB_2));

            euro.PrintLine("CAL Knob3: %1.4f", euro.GetKnobValue(planter::KNOB_3));
            euro.PrintLine("RAW Knob3: %1.4f", euro.patch.GetAdcValue(planter::KNOB_3));

            euro.PrintLine("CAL Knob4: %1.4f", euro.GetKnobValue(planter::KNOB_4));
            euro.PrintLine("RAW Knob4: %1.4f", euro.patch.GetAdcValue(planter::KNOB_4));

            euro.PrintLine("CAL Knob5: %1.4f", euro.GetKnobValue(planter::KNOB_5));
            euro.PrintLine("RAW Knob5: %1.4f", euro.patch.GetAdcValue(planter::KNOB_5));

            euro.PrintLine("CAL Knob6: %1.4f", euro.GetKnobValue(planter::KNOB_6));
            euro.PrintLine("RAW Knob6: %1.4f", euro.patch.GetAdcValue(planter::KNOB_6));

            usbt = now;
        }

        euro.BootloaderResetCheck();
    }

}
