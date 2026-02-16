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
    for(size_t i = 0; i < size; i++)
    {
        out[0][i] = out[1][i] = osc.Process() * (euro.toggle1.Pressed() ? 1.f : 0.f);
    }
}

int main(void)
{
    euro.Init();    
    euro.StartLog(true);
    //System::Delay(10000);
    euro.PrintLine("Daisy Patch SM started. Test Beginning");

    /** Memory tests first to keep test short if everything else fails */
    bool sdram_pass = euro.patch.ValidateSDRAM();
    euro.patch.PrintLine("SDRAM Test\t-\t%s", sdram_pass ? "PASS" : "FAIL");
    bool qspi_pass = euro.patch.ValidateQSPI();
    euro.patch.PrintLine("QSPI Test\t-\t%s", qspi_pass ? "PASS" : "FAIL");

    /** Initialize our test tone */
    osc.Init(euro.patch.AudioSampleRate());
    euro.patch.StartAudio(AudioCallback);

    uint32_t now, dact, usbt, gatet, ledt;
    now = dact = usbt = ledt = System::GetNow();
    gatet             = now;

    bool led1_state = true;
    bool led2_state = false;
    bool cv_out_state = true;
    bool last_button_state = false;

    euro.SetLed(1, led1_state);
    euro.SetLed(2, led2_state);

    while(1)
    {
        now = System::GetNow();
        // 500Hz samplerate for DAC output test

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
            euro.PrintLine("Analog IO:");
            euro.PrintLine("CALIBRATING: %s", cv_out_state ? "CV_OUT_2" : "CV_OUT_1");

            float cal1 = euro.GetKnobValue(planter::KNOB_1) + (euro.GetKnobValue(planter::KNOB_4) * 0.02);
            euro.PrintLine("CAL CV_OUT_HIGH: %1.3f", cal1);

            float cal2 = euro.GetKnobValue(planter::KNOB_2) + (euro.GetKnobValue(planter::KNOB_5) * 0.02)+ 2.0f;
            euro.PrintLine("CAL CV_OUT_MID: %1.3f", cal2);
            
            float cal3 = euro.GetKnobValue(planter::KNOB_3) + (euro.GetKnobValue(planter::KNOB_6) * 0.02) + 4.0f;
            euro.PrintLine("CAL CV_OUT_LOW: %1.3f", cal3);

            
            euro.PrintLine("CAL CV1: %1.4f", euro.GetCvIn(CV_1));
            euro.PrintLine("RAW CV1: %1.4f", euro.patch.GetAdcValue(CV_1));

            euro.PrintLine("CAL CV2: %1.4f", euro.GetCvIn(CV_2));
            euro.PrintLine("RAW CV2: %1.4f", euro.patch.GetAdcValue(CV_2));

            euro.PrintLine("CAL CV3: %1.4f", euro.GetCvIn(CV_3));
            euro.PrintLine("RAW CV3: %1.4f", euro.patch.GetAdcValue(CV_3));

            euro.PrintLine("CAL CV4: %1.4f", euro.GetCvIn(CV_4));
            euro.PrintLine("RAW CV4: %1.4f", euro.patch.GetAdcValue(CV_4));

            // for(int i = 0; i < ADC_LAST; i++)
            // {
                
                
                
            //     float val;
            //     if(i < ADC_11 && i > CV_4)
            //     {
            //         // CV inputs: use proper scaling
            //         val = euro.GetKnobValue(i);
            //     }
            //     else
            //     {
            //         // ADC inputs: use standard GetAdcValue
            //         val = euro.patch.GetAdcValue(i);
            //     }
            //     euro.Print("%s_%d: " FLT_FMT3,
            //              i < ADC_9 ? "CV" : "ADC",
            //              i + 1,
            //              FLT_VAR3(val));
            //     if(i != 0 && (i + 1) % 4 == 0)
            //         euro.Print("\n");
            //     else
            //         euro.Print("\t");
            // }
            usbt = now;
        }

        euro.BootloaderResetCheck();
    }
}
