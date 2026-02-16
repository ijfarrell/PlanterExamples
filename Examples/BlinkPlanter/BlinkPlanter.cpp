#include "planter.h"

using namespace daisy;
using namespace anachrome;

planter euro;

int main(void)
{
    bool led1_state, led2_state;

    // Configure and Initialize the Planter Module (patch_sm init called in planter init)
    euro.Init();

    led1_state = true;
    led2_state = false;

    uint32_t now, ledt;
    now = ledt = System::GetNow();

    // Loop forever
    while(1)
    {
        now = System::GetNow();
    
        //Switch LEDs every 500ms
        if(now - ledt > 500){
            // Set the onboard LED
            euro.SetLed(1,led1_state);
            euro.SetLed(2,led2_state);

            // Toggle the LED state for the next time around.
            led1_state = !led1_state;
            led2_state = !led2_state;
            ledt = now;
        }

        euro.button.Debounce(); // Update button state for bootloader check
        euro.BootloaderResetCheck(); //bootloader check
    }
}
