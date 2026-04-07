This is a WIP repo for the Anachrome Electronics Planter eurorack module. 

<img width="1971" height="2988" alt="frontmodule_" src="https://github.com/user-attachments/assets/8cdd2d58-4144-4556-8133-72134115d19e" /> <img width="2011" height="2941" alt="backmodule" src="https://github.com/user-attachments/assets/ef6ea12e-ad61-461d-879c-d10cff1f42af" />

This module is a eurorack interface and functionality extender for the electrosmith Daisy Patch SM.

Planter has the following interface features, configured within this repository in planter.h/planter.cpp and examples

- 14HP
- 6 control knobs
- Three On-On toggle switches
- momentary button
- 2 Gate inputs
- 2 Gate outputs
- 4 CV inputs
- 2 CV outputs, -5V to 5V bipolar CV output
- 2 Audio inputs
- 2 Audio Outputs

Module initialization requires the following steps:

- Initialize daisy development environment - use this link to ensure proper toolchain etc. https://daisy.audio/tutorials/cpp-dev-env/
- clone this repo:
  ```
  git clone https://github.com/ijfarrell/PlanterExamples.git
  ```
- Update and build libDaisy and DaisySP as submodules
  ```
  cd PlanterExamples
  git submodule update --init --recursive

  make -C libDaisy
  make -C DaisySP
  ```
- If using a new daisy patch SM, flash unit with v6.3 external bootloader at https://flash.daisy.audio/ , which allows the patchSM to connect via the front panel USB-C for bootloading
- connect the Planter to PC or mac via front panel USB-C
- Place Daisy Patch SM into bootloader mode with long press on RESET button, release, and quick press on BOOT button onboard the patch sm. The LED on board the patch SM should blink quickly once and then slowly blink
- Make and load the BlinkPlanter.cpp file
  ```
  cd PlanterExamples/Examples/BlinkPlanter

  make clean
  make
  make program-dfu
  ```
- Planter can be placed back into programming mode while running a program by holding the red button for 3 seconds
