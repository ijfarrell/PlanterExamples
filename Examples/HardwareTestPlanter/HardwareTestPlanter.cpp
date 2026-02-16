#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "planter.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace anachrome;

DaisyPatchSM   hw;
// Switch         button, toggle1, toggle2, toggle3;
#define DSY_TEXT __attribute__((section(".text")))
DSY_TEXT FIL            file; /**< Can't be made on the stack (DTCMRAM) */
DSY_TEXT FatFSInterface fsi;
MidiUsbHandler midi;
Oscillator     osc;
dsy_gpio led_1, led_2;
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


bool midiflag = false; //flag for midi listening test, disables serial logging


int main(void)
{
    euro.Init();    
    euro.StartLog(true);
    //System::Delay(10000);

    euro.PrintLine("Planter started. Test Beginning");

    // Begin section reference from DaisyExamples/patch_sm/HardwareTest/HardwareTest.cpp from electro-smith
    /** Memory tests first to keep test short if everything else fails */
    bool sdram_pass = euro.patch.ValidateSDRAM();
    euro.PrintLine("SDRAM Test\t-\t%s", sdram_pass ? "PASS" : "FAIL");
    bool qspi_pass = euro.patch.ValidateQSPI();
    euro.PrintLine("QSPI Test\t-\t%s", qspi_pass ? "PASS" : "FAIL");

    /** SD card next */
    SdmmcHandler::Config sd_config;
    SdmmcHandler         sdcard;
    sd_config.Defaults();
    sdcard.Init(sd_config);

    fsi.Init(FatFSInterface::Config::MEDIA_SD);


    /** Write/Read text file */
    const char *test_string = "Testing Daisy Patch SM";
    const char *test_fname  = "DaisyPatchSM-Test.txt";
    FRESULT     fres = FR_DENIED; /**< Unlikely to actually experience this */
    if(f_mount(&fsi.GetSDFileSystem(), "/", 0) == FR_OK)
    {
        /** Write Test */
        if(f_open(&file, test_fname, (FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
        {
            UINT   bw  = 0;
            size_t len = strlen(test_string);
            fres       = f_write(&file, test_string, len, &bw);
        }
        f_close(&file);
        if(fres == FR_OK)
        {
            /** Read Test only if Write passed */
            if(f_open(&file, test_fname, (FA_OPEN_EXISTING | FA_READ)) == FR_OK)
            {
                UINT   br = 0;
                char   readbuff[32];
                size_t len = strlen(test_string);
                fres       = f_read(&file, readbuff, len, &br);
            }
            f_close(&file);
        }
    }
    bool sdmmc_pass = fres == FR_OK;
    euro.PrintLine("SDMMC Test\t-\t%s", sdmmc_pass ? "PASS" : "FAIL");

    /** 5 second delay before logging*/

    System::Delay(5000);
 
    daisysp::Phasor dacphs;
    dacphs.Init(1000);
    dacphs.SetFreq(0.5f);


    
    // Initialize test tone
    osc.Init(euro.patch.AudioSampleRate());
    euro.patch.StartAudio(AudioCallback);

    uint32_t now, dact, usbt, gatet, ledt;
    now = dact = usbt = ledt = System::GetNow();
    gatet             = now;
    // End section reference from DaisyExamples/patch_sm/HardwareTest/HardwareTest.cpp from electro-smith

    if(midiflag){
    MidiUsbHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = MidiUsbTransport::Config::EXTERNAL;
    midi.Init(midi_cfg);
    }
    bool led1_state = true;
    bool led2_state = false;

    while(1)
    {
        now = System::GetNow();
        // 500Hz samplerate for DAC output test


        if(now - dact > 2)
        {
            
            euro.WriteCvOutBipolar(CV_OUT_1, (dacphs.Process() * 10.f) - 5.f);
            euro.WriteCvOutBipolar(CV_OUT_2, (dacphs.Process() * 10.f) - 5.f);
            dact = now;
        }

        if(now - ledt > 2000)
        {
            euro.SetLed(1, led1_state);
            euro.SetLed(2, led2_state);
            led1_state = !led1_state;
            led2_state = !led2_state;
            ledt = now;
        }

        //This only prints if midiflag is false, since MIDI listening test disables logging.
        if(now - usbt > 100)
        {
            euro.PrintLine("Streaming Test Results");
            euro.PrintLine("######################");
            euro.PrintLine("Analog Inputs:");
            for(int i = 0; i < ADC_11; i++)
            {
                float val;
                if(i < euro.KNOB_LAST && i >= euro.KNOB_1)
                {
                    // KNOB inputs: use proper scaling
                    val = euro.GetKnobValue(i);
                }
                else
                {
                    // CV inputs: use standard GetCvIn
                    val = euro.GetCvIn(i);
                }
                const char* input_label = "ADC";
                int         input_idx   = i + 1;
                if(i < 4)
                {
                    input_label = "CV";
                    input_idx   = i + 1;
                }
                else if(i >= euro.KNOB_1 && i < euro.KNOB_LAST)
                {
                    input_label = "KNOB";
                    input_idx   = i - euro.KNOB_1 + 1;
                }
                
                euro.Print("%s%d: " FLT_FMT3, input_label, input_idx, FLT_VAR3(val));

                // Break lines bewteen groups of inputs for readability
                bool line_break = false;
                if(i < 4)
                    line_break = (i == 3);
                else if(i >= euro.KNOB_1 && i < euro.KNOB_LAST)
                    line_break = (i == euro.KNOB_1 + 2) || (i == euro.KNOB_1 + 5);

                if(line_break)
                    euro.Print("\n");
                else
                    euro.Print("\t");
            }
            euro.PrintLine("######################");
            euro.PrintLine("Digital Inputs:");
            euro.Print("Button: %s\t", euro.button.Pressed() ? "ON" : "OFF");
            euro.Print("Toggle1: %s\t", euro.toggle1.Pressed() ? "UP" : "DOWN");
            euro.Print("Toggle2: %s\t", euro.toggle2.Pressed() ? "UP" : "DOWN");
            euro.Print("Toggle3: %s\t", euro.toggle3.Pressed() ? "UP" : "DOWN");
            euro.Print("\nGATE_IN_1: %s\t",
                     euro.patch.gate_in_1.State() ? "HIGH" : "LOW");
            euro.PrintLine("GATE_IN_2: %s",
                         euro.patch.gate_in_2.State() ? "HIGH" : "LOW");
            euro.PrintLine("######################");
            usbt = now;
        }
        if(now - gatet > 1000)
        {
            dsy_gpio_toggle(&euro.patch.gate_out_1);
            dsy_gpio_toggle(&euro.patch.gate_out_2);
            gatet = now;
        }


        if(midiflag){
        /** Listen to MIDI for new changes */
            midi.Listen();

            /** When there are messages waiting in the queue... */
            while(midi.HasEvents())
            {
                /** Pull the oldest one from the list... */
                auto msg = midi.PopEvent();
                switch(msg.type)
                {
                    case NoteOn:
                    {
                        /** and change the frequency of the oscillator */
                        auto note_msg = msg.AsNoteOn();
                        euro.PrintLine("noteon: ch %d, note %d, vel %d",
                                    note_msg.channel,
                                    note_msg.note,
                                    note_msg.velocity);
                        if(note_msg.velocity != 0)
                            osc.SetFreq(mtof(note_msg.note));
                    }
                    break;
                }
            }
        }

        euro.BootloaderResetCheck();
    }
}
