# ConvReverb — Stereo Convolution Reverb

Real-time stereo convolution reverb for Daisy + Planter (Eurorack), using an
impulse response (IR) WAV file loaded from an external SD card.

## How it works

The algorithm uses **uniform partitioned convolution** (overlap-save) with
CMSIS-DSP `arm_rfft_fast_f32` for efficient FFT-based processing.  The IR is
split into 256-sample partitions, each pre-FFT'd at startup, and convolved with
the live audio input via a frequency-domain delay line.

## SD Card Setup

1. Format an SD card as **FAT32**.
2. Place a WAV file named **`ir.wav`** in the root directory.
   - **Stereo** (2-channel) or mono (will be duplicated to both channels).
   - **16-bit, 24-bit, or 32-bit** PCM, or 32-bit IEEE float.
   - **48 kHz** sample rate recommended (matching the hardware).
   - Maximum length: ~5 seconds.
3. Insert the SD card before powering on.

## Controls

| Knob   | Function         | Range                  |
|--------|------------------|------------------------|
| KNOB_1 | Dry / Wet mix    | Full CCW = dry, CW = wet |
| KNOB_2 | Reverb level     | Wet return attenuator  |
| KNOB_3 | Input gain       | 0.2 – 1.0             |
| KNOB_4 | Tone (LP filter) | 200 Hz – 18 kHz       |

## LED Indicators

- **LED 1 blinking** — IR loaded successfully, audio running.
- **LED 2 on** — IR load failed; audio passes through unprocessed.

## Building

```bash
cd Examples/ConvReverb
make clean && make
make program-dfu
```

## Dependencies

- libDaisy (FatFS, SdmmcHandler, WavParser, FileReader)
- DaisySP / DaisySP-LGPL
- CMSIS-DSP (TransformFunctions + CommonTables, compiled via Makefile)
- Planter hardware abstraction layer
