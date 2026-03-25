/*
 * ===========================================================================
 * ConvReverb.cpp
 *
 * Stereo Convolution Reverb for Daisy + Planter (Eurorack)
 *
 * Reads a stereo impulse response WAV file from an external SD card and
 * applies real-time FFT-based partitioned convolution (overlap-save) to
 * the stereo audio input.
 *
 * ---------------------------------------------------------------------------
 *  SD Card Setup
 * ---------------------------------------------------------------------------
 *  Place a stereo 16-bit or 24-bit PCM WAV file named "ir.wav" on the
 *  root of a FAT32-formatted SD card.  Sample rate should match the
 *  hardware (48 kHz).  Maximum IR length: ~5 seconds stereo.
 *
 * ---------------------------------------------------------------------------
 *  Controls (Planter)
 * ---------------------------------------------------------------------------
 *  KNOB_1  – Dry / Wet mix   (full CCW = dry only, full CW = wet only)
 *  KNOB_2  – Reverb level    (attenuates the wet return)
 *  KNOB_3  – Input gain      (pre-reverb send trim)
 *  KNOB_4  – Tone / LP freq  (low-pass filter on the wet signal)
 *
 * ---------------------------------------------------------------------------
 *  Algorithm: Uniform Partitioned Convolution (Overlap-Save)
 * ---------------------------------------------------------------------------
 *  1.  The IR is split into P partitions of PART_LEN samples each.
 *  2.  Each partition is zero-padded to FFT_LEN = 2*PART_LEN and
 *      transformed to the frequency domain via CMSIS arm_rfft_fast_f32.
 *  3.  At run-time, every PART_LEN new input samples are collected,
 *      concatenated with the previous PART_LEN samples (overlap-save),
 *      and FFT'd.
 *  4.  The input spectrum is multiplied with each IR partition spectrum
 *      and accumulated in a frequency-domain delay line (FDL).
 *  5.  The accumulated spectrum is inverse-FFT'd; the second half of the
 *      result (PART_LEN samples) is the output.
 *
 *  This gives O(N log N) per partition rather than O(N*M) time-domain.
 * ===========================================================================
 */

#include "daisy_patch_sm.h"
#include "daisysp-lgpl.h"
#include "planter.h"
#include <cmath>
#include <cstring>
#include <algorithm>

/* CMSIS-DSP for FFT */
#include "arm_math.h"

/* FatFS headers (enabled by USE_FATFS=1 in Makefile) */
#include "ff.h"
#include "util/WavParser.h"
#include "util/FileReader.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace anachrome;

/* ========================================================================
 *  Compile-time constants
 * ======================================================================== */

/** Partition length in samples — must be a power of two.
 *  256 gives 5.3 ms latency at 48 kHz and a good CPU/memory trade-off. */
static constexpr size_t PART_LEN = 512;

/** Late-stage partition length (processed at lower update rate). */
static constexpr size_t LATE_PART_LEN = 1024;

/** FFT length is twice partition length (overlap-save). */
static constexpr size_t FFT_LEN = PART_LEN * 2; // 512

/** Complex spectrum length in float elements (re,im interleaved). */
static constexpr size_t SPEC_LEN = FFT_LEN * 2; // 1024

/** Late-stage FFT/spectrum sizes. */
static constexpr size_t LATE_FFT_LEN = LATE_PART_LEN * 2;
static constexpr size_t LATE_SPEC_LEN = LATE_FFT_LEN * 2;

/** Maximum IR length in seconds. */
static constexpr float  MAX_IR_SEC = 5.0f;

/** Maximum number of samples per channel. */
static constexpr size_t MAX_IR_SAMPLES = static_cast<size_t>(48000 * MAX_IR_SEC);

/** Maximum number of partitions per stage. */
static constexpr size_t MAX_EARLY_PARTITIONS = 6;
static constexpr size_t MAX_LATE_PARTITIONS = (MAX_IR_SAMPLES + LATE_PART_LEN - 1) / LATE_PART_LEN;
static constexpr size_t MAX_ACTIVE_LATE_PARTITIONS = 2;

/** Runtime CPU guard: maximum active early partitions. */
static constexpr size_t MAX_ACTIVE_PARTITIONS = MAX_EARLY_PARTITIONS;

/** Output safety clamp */
static constexpr float OUTPUT_LIMIT = 0.98f;
static constexpr float MIX_DRY_BYPASS_THRESH = 0.001f;

/** Master debug switch for serial diagnostics and profiling. */
static constexpr bool DEBUG_MODE = false;
/** Enable verbose SD/FatFS diagnostics over serial */
static constexpr bool SD_DEBUG_LOG = DEBUG_MODE;
/** Enable runtime CPU/wet diagnostics over serial */
static constexpr bool CPU_DEBUG_LOG = DEBUG_MODE;

/** IR WAV filename on SD card */
static const char* IR_FILENAME = "ir.wav";

/* ========================================================================
 *  SDRAM-resident buffers
 *
 *  The STM32H750 on Daisy Patch SM has 64 MB external SDRAM.  We place
 *  all large convolution buffers there via the .sdram_bss section.
 * ======================================================================== */
#define DSY_SDRAM_BSS __attribute__((section(".sdram_bss")))

/** Raw IR samples read from SD (interleaved stereo, 16-bit → float).
 *  Temporary — only used during init, then freed conceptually. */
static float DSY_SDRAM_BSS ir_raw_L[MAX_IR_SAMPLES];

/** Pre-computed FFT of each IR partition, per channel.
 *  Layout: ir_spec_X[partition][SPEC_LEN] */
static float DSY_SDRAM_BSS ir_spec_L[MAX_EARLY_PARTITIONS][SPEC_LEN];
static float DSY_SDRAM_BSS ir_spec_late[MAX_LATE_PARTITIONS][LATE_SPEC_LEN];

/** Frequency-domain delay line (FDL) for the input, shared by both
 *  channels since both convolve with the same input on the respective
 *  IR channel.  If you want true stereo-in/stereo-out (L*IR_L, R*IR_R),
 *  you need two FDLs.  Here we do:
 *    out_L = in_L * IR_L
 *    out_R = in_R * IR_R
 *  so we need one FDL per input channel. */
static float DSY_SDRAM_BSS fdl_L[MAX_EARLY_PARTITIONS][SPEC_LEN];
static float DSY_SDRAM_BSS fdl_late[MAX_LATE_PARTITIONS][LATE_SPEC_LEN];

/** Accumulation spectrum buffer (one per channel, reused each block). */
static float DSY_SDRAM_BSS acc_spec_L[SPEC_LEN];
static float DSY_SDRAM_BSS acc_spec_late[LATE_SPEC_LEN];

/* ========================================================================
 *  Working buffers (DTCMRAM / normal RAM — small, fast-access)
 * ======================================================================== */

/** Overlap-save input ring: previous PART_LEN + current PART_LEN = FFT_LEN */
static float ols_buf_L[FFT_LEN];

/** FFT scratch buffer — used for complex transforms */
static float fft_buf[SPEC_LEN];
static float fft_buf_late[LATE_SPEC_LEN];

/** Output ring for overlap-save extraction */
static float out_buf_L[PART_LEN];
static float out_buf_late[LATE_PART_LEN];

/** Overlap-add tails (second half of previous IFFT block) */
static float ola_tail_L[PART_LEN];
static float ola_tail_late[LATE_PART_LEN];

/** Small collect buffer for incoming audio blocks (48 samples at a time). */
static float collect_L[PART_LEN];
static float collect_late[LATE_PART_LEN];

/* ========================================================================
 *  Globals
 * ======================================================================== */
static DaisyPatchSM hw_unused;          // not used directly — planter wraps it
static planter      euro;

/* CMSIS cfft instance */
static arm_cfft_instance_f32 cfft_inst;
static arm_cfft_instance_f32 cfft_inst_late;

/* SD / FatFS */
#define DSY_TEXT __attribute__((section(".text")))
DSY_TEXT FIL            sd_file;
DSY_TEXT FatFSInterface fsi;

/* Convolution state */
static size_t num_partitions = 0;       // actual number of IR partitions
static size_t active_partitions = 0;    // runtime partition count used by convolution
static size_t late_num_partitions = 0;
static size_t late_active_partitions = 0;
static size_t ir_samples     = 0;       // actual IR length per channel
static size_t collect_pos    = 0;       // write position into collect buffers
static size_t late_collect_pos = 0;
static size_t fdl_pos        = 0;       // current FDL write slot
static size_t late_fdl_pos   = 0;
static bool   ir_loaded      = false;   // true after successful IR load
static bool   late_stage_enabled = false;

/* Simple one-pole LP for tone control on wet signal */
static float lp_state_L = 0.f;

/* Smoothed control state (reduces callback-rate modulation artifacts) */
static float ctrl_mix       = 0.5f;
static float ctrl_wet_level = 0.5f;
static float ctrl_in_gain   = 0.6f;
static float ctrl_tone_knob = 0.5f;

/* Runtime debug/profile counters (written in audio callback, read in main loop) */
static volatile uint32_t dbg_cb_count            = 0;
static volatile uint32_t dbg_cb_us_accum         = 0;
static volatile uint32_t dbg_cb_us_max           = 0;
static volatile uint32_t dbg_cb_overrun_count    = 0;
static volatile uint32_t dbg_cb_budget_us_last   = 0;
static volatile uint32_t dbg_cb_timing_glitches  = 0;

static volatile uint32_t dbg_conv_count          = 0;
static volatile uint32_t dbg_conv_us_accum       = 0;
static volatile uint32_t dbg_conv_us_max         = 0;
static volatile uint32_t dbg_conv_overrun_count  = 0;
static volatile uint32_t dbg_conv_timing_glitches = 0;

static volatile float    dbg_wet_peak            = 0.f;
static volatile float    dbg_wet_rms_accum       = 0.f;
static volatile uint32_t dbg_wet_rms_count       = 0;

static constexpr uint32_t DBG_CONV_BUDGET_US = (PART_LEN * 1000000u) / 48000u; // ~5333us

/** Debug isolation mode: freeze controls to remove any zipper/modulation artifacts. */
static constexpr bool FREEZE_CONTROLS_FOR_DEBUG = DEBUG_MODE;
static constexpr float FIXED_MIX       = 0.5f;
static constexpr float FIXED_WET_LEVEL = 0.75f;
static constexpr float FIXED_IN_GAIN   = 0.6f;
static constexpr float FIXED_TONE_KNOB = 0.75f;

/* ========================================================================
 *  Helper: complex multiply-accumulate (interleaved re/im layout)
 * ======================================================================== */
static inline void spec_cmac(float* acc, const float* a, const float* b, size_t len)
{
    for(size_t i = 0; i < len; i += 2)
    {
        float ar = a[i], ai = a[i + 1];
        float br = b[i], bi = b[i + 1];
        acc[i]     += ar * br - ai * bi;
        acc[i + 1] += ar * bi + ai * br;
    }
}

/* ========================================================================
 *  Load IR from SD card
 * ======================================================================== */
static bool LoadIR()
{
    euro.PrintLine("ConvReverb: Mounting SD card...");

    /* ---- SD card init (same pattern as HardwareTestPlanter) ---- */
    SdmmcHandler::Config sd_config;
    SdmmcHandler         sdcard;
    sd_config.Defaults();
    sdcard.Init(sd_config);
    fsi.Init(FatFSInterface::Config::MEDIA_SD);

    if(f_mount(&fsi.GetSDFileSystem(), "/", 0) != FR_OK)
    {
        euro.PrintLine("  SD mount FAILED");
        return false;
    }
    euro.PrintLine("  SD mounted OK");

    /* ---- Open WAV file ---- */
    if(f_open(&sd_file, IR_FILENAME, (FA_OPEN_EXISTING | FA_READ)) != FR_OK)
    {
        euro.PrintLine("  Could not open %s", IR_FILENAME);
        return false;
    }
    euro.PrintLine("  Opened %s", IR_FILENAME);

    /* ---- Parse WAV header ---- */
    FileReader reader(&sd_file);
    WavParser  parser;
    if(!parser.parse(reader))
    {
        euro.PrintLine("  WAV parse error");
        f_close(&sd_file);
        return false;
    }

    const auto& info = parser.info();
    euro.PrintLine("  Format: %d-bit, %lu Hz, %d ch",
                   (int)info.bitsPerSample,
                   (unsigned long)info.sampleRate,
                   (int)info.numChannels);
    if(SD_DEBUG_LOG)
    {
        euro.PrintLine("  WAV info: fmt=%d blockAlign=%d byteRate=%lu",
                       (int)info.audioFormat,
                       (int)info.blockAlign,
                       (unsigned long)info.byteRate);
        euro.PrintLine("  WAV data: offset=%lu bytes=%lu",
                       (unsigned long)parser.dataOffset(),
                       (unsigned long)parser.dataSize());
    }

    if(info.numChannels < 1 || info.numChannels > 2)
    {
        euro.PrintLine("  Unsupported channel count");
        f_close(&sd_file);
        return false;
    }
    if(info.audioFormat != 1 && info.audioFormat != 3)
    {
        // 1 = PCM integer, 3 = IEEE float
        euro.PrintLine("  Unsupported audio format (%d)", (int)info.audioFormat);
        f_close(&sd_file);
        return false;
    }

    bool   is_stereo  = (info.numChannels == 2);
    bool   is_float   = (info.audioFormat == 3);
    size_t bps        = info.bitsPerSample;
    size_t frame_bytes = info.numChannels * (bps / 8);
    size_t data_bytes  = parser.dataSize();
    size_t total_frames = data_bytes / frame_bytes;

    if(total_frames > MAX_IR_SAMPLES)
    {
        euro.PrintLine("  IR too long (%lu frames, max %lu). Truncating.",
                       (unsigned long)total_frames,
                       (unsigned long)MAX_IR_SAMPLES);
        total_frames = MAX_IR_SAMPLES;
    }

    ir_samples = total_frames;
    euro.PrintLine("  Loading %lu frames...", (unsigned long)ir_samples);

    /* ---- Read raw audio data from SD ---- */
    FRESULT fres = f_lseek(&sd_file, parser.dataOffset());
    if(fres != FR_OK)
    {
        euro.PrintLine("  Seek error (fres=%d, dst=%lu)",
                       (int)fres,
                       (unsigned long)parser.dataOffset());
        f_close(&sd_file);
        return false;
    }
    if(SD_DEBUG_LOG)
    {
        euro.PrintLine("  Seek OK: f_tell=%lu", (unsigned long)f_tell(&sd_file));
    }

    /* Read in chunks to avoid huge stack buffers.
     * We read raw bytes and convert sample-by-sample.
     *
     * IMPORTANT: read_buf must be in a DMA-accessible region!
     * On STM32H7, the SDMMC IDMA cannot access DTCMRAM (where the stack
     * and .bss live). When FatFS reads full sectors, it DMAs directly into
     * the user buffer, bypassing the FIL internal sector buffer. If that
     * buffer is in DTCMRAM the transfer silently fails.
     * We place it in .sram1_bss → RAM_D2_DMA (0x30000000). */
    static constexpr size_t READ_CHUNK_FRAMES = 512;
    static uint8_t DMA_BUFFER_MEM_SECTION read_buf[READ_CHUNK_FRAMES * 8]; // max 2ch * 32bit = 8 bytes/frame
    size_t frames_read = 0;

    if(SD_DEBUG_LOG)
    {
        uintptr_t rb = reinterpret_cast<uintptr_t>(read_buf);
        euro.PrintLine("  read_buf addr=0x%08lx size=%lu",
                       (unsigned long)rb,
                       (unsigned long)sizeof(read_buf));
        euro.PrintLine("  read_buf in DMA RAM_D2=%s",
                       (rb >= 0x30000000UL && rb < 0x30008000UL) ? "YES" : "NO");
    }

    if(SD_DEBUG_LOG)
    {
        UINT  probe_br = 0;
        fres          = f_read(&sd_file, read_buf, 16, &probe_br);
        euro.PrintLine("  Probe read: fres=%d br=%lu tell=%lu",
                       (int)fres,
                       (unsigned long)probe_br,
                       (unsigned long)f_tell(&sd_file));
        fres = f_lseek(&sd_file, parser.dataOffset());
        euro.PrintLine("  Seek back after probe: fres=%d tell=%lu",
                       (int)fres,
                       (unsigned long)f_tell(&sd_file));
        if(fres != FR_OK)
        {
            f_close(&sd_file);
            return false;
        }
    }

    memset(ir_raw_L, 0, sizeof(float) * MAX_IR_SAMPLES);

    while(frames_read < ir_samples)
    {
        // Keep each f_read request strictly below one sector (512 bytes).
        // This avoids FatFS's direct/multi-sector transfer path that is failing
        // on this target after the first partial-sector read.
        size_t max_frames_subsector = (511u / frame_bytes);
        if(max_frames_subsector == 0)
            max_frames_subsector = 1;

        size_t frames_to_read
            = std::min(std::min(READ_CHUNK_FRAMES, ir_samples - frames_read),
                       max_frames_subsector);
        size_t bytes_to_read  = frames_to_read * frame_bytes;
        UINT   br = 0;
        dsy_dma_clear_cache_for_buffer(read_buf, bytes_to_read);
        fres = f_read(&sd_file, read_buf, bytes_to_read, &br);
        dsy_dma_invalidate_cache_for_buffer(read_buf, br);

        if(fres != FR_OK || br != bytes_to_read)
        {
            euro.PrintLine("  Read error at frame %lu (fres=%d req=%lu br=%lu tell=%lu)",
                           (unsigned long)frames_read,
                           (int)fres,
                           (unsigned long)bytes_to_read,
                           (unsigned long)br,
                           (unsigned long)f_tell(&sd_file));
            /* Use what we got */
            ir_samples = frames_read;
            break;
        }

        /* Decode samples */
        for(size_t f = 0; f < frames_to_read; f++)
        {
            size_t idx = frames_read + f;
            const uint8_t* p = read_buf + f * frame_bytes;

            float sL = 0.f, sR = 0.f;

            if(is_float && bps == 32)
            {
                /* IEEE 32-bit float */
                float raw;
                memcpy(&raw, p, sizeof(float));
                sL = raw;
                if(is_stereo)
                {
                    memcpy(&raw, p + 4, sizeof(float));
                    sR = raw;
                }
                else
                {
                    sR = sL;
                }
            }
            else if(bps == 16)
            {
                /* 16-bit signed PCM */
                int16_t s16;
                memcpy(&s16, p, 2);
                sL = static_cast<float>(s16) / 32768.f;
                if(is_stereo)
                {
                    memcpy(&s16, p + 2, 2);
                    sR = static_cast<float>(s16) / 32768.f;
                }
                else
                {
                    sR = sL;
                }
            }
            else if(bps == 24)
            {
                /* 24-bit signed PCM (little-endian) */
                int32_t s24 = (int32_t)(p[0]) | ((int32_t)(p[1]) << 8) | ((int32_t)(p[2]) << 16);
                if(s24 & 0x800000) s24 |= ~0xFFFFFF; // sign-extend
                sL = static_cast<float>(s24) / 8388608.f;
                if(is_stereo)
                {
                    s24 = (int32_t)(p[3]) | ((int32_t)(p[4]) << 8) | ((int32_t)(p[5]) << 16);
                    if(s24 & 0x800000) s24 |= ~0xFFFFFF;
                    sR = static_cast<float>(s24) / 8388608.f;
                }
                else
                {
                    sR = sL;
                }
            }
            else if(bps == 32 && !is_float)
            {
                /* 32-bit signed integer PCM */
                int32_t s32;
                memcpy(&s32, p, 4);
                sL = static_cast<float>(s32) / 2147483648.f;
                if(is_stereo)
                {
                    memcpy(&s32, p + 4, 4);
                    sR = static_cast<float>(s32) / 2147483648.f;
                }
                else
                {
                    sR = sL;
                }
            }

            float sM      = is_stereo ? 0.5f * (sL + sR) : sL;
            ir_raw_L[idx] = sM;
        }

        frames_read += frames_to_read;
    }

    f_close(&sd_file);

    if(ir_samples < PART_LEN)
    {
        euro.PrintLine("  IR too short (%lu samples)", (unsigned long)ir_samples);
        return false;
    }

    euro.PrintLine("  Loaded %lu IR samples per channel", (unsigned long)ir_samples);

    /* ---- Partition and FFT the IR into two stages ---- */
    size_t requested_early_partitions = (ir_samples + PART_LEN - 1) / PART_LEN;
    num_partitions                    = std::min(requested_early_partitions, MAX_EARLY_PARTITIONS);
    active_partitions                 = std::min(num_partitions, MAX_ACTIVE_PARTITIONS);

    size_t early_samples = num_partitions * PART_LEN;
    size_t late_samples  = (ir_samples > early_samples) ? (ir_samples - early_samples) : 0;
    late_num_partitions  = (late_samples + LATE_PART_LEN - 1) / LATE_PART_LEN;
    if(late_num_partitions > MAX_LATE_PARTITIONS)
        late_num_partitions = MAX_LATE_PARTITIONS;
    late_active_partitions = std::min(late_num_partitions, MAX_ACTIVE_LATE_PARTITIONS);

    euro.PrintLine("  Early partitions: loaded=%lu active=%lu (len=%lu)",
                   (unsigned long)num_partitions,
                   (unsigned long)active_partitions,
                   (unsigned long)PART_LEN);
    euro.PrintLine("  Late partitions: loaded=%lu active=%lu (len=%lu)",
                   (unsigned long)late_num_partitions,
                   (unsigned long)late_active_partitions,
                   (unsigned long)LATE_PART_LEN);
    if(late_num_partitions > late_active_partitions)
    {
        euro.PrintLine("  Late tail truncated for CPU: loaded=%lu active=%lu",
                       (unsigned long)late_num_partitions,
                       (unsigned long)late_active_partitions);
    }
    euro.PrintLine("  Active tail now: early only = %lu smp (~" FLT_FMT3 " ms)",
                   (unsigned long)(active_partitions * PART_LEN),
                   FLT_VAR3((1000.0f * static_cast<float>(active_partitions * PART_LEN))
                            / 48000.0f));
    if(late_active_partitions > 0)
    {
        euro.PrintLine("  Late tail avail: +%lu smp (~" FLT_FMT3 " ms) when toggle2=UP",
                       (unsigned long)(late_active_partitions * LATE_PART_LEN),
                       FLT_VAR3((1000.0f * static_cast<float>(late_active_partitions * LATE_PART_LEN))
                                / 48000.0f));
    }

    float pad_buf[SPEC_LEN];

    for(size_t p = 0; p < num_partitions; p++)
    {
        size_t offset = p * PART_LEN;
        size_t len    = std::min(PART_LEN, ir_samples - offset);

        /* --- Left channel partition --- */
        memset(pad_buf, 0, sizeof(pad_buf));
        for(size_t n = 0; n < len; n++)
            pad_buf[2 * n] = ir_raw_L[offset + n];
        arm_cfft_f32(&cfft_inst, pad_buf, 0, 1);
        memcpy(ir_spec_L[p], pad_buf, sizeof(float) * SPEC_LEN);

    }

    for(size_t p = 0; p < late_num_partitions; p++)
    {
        size_t offset = early_samples + p * LATE_PART_LEN;
        if(offset >= ir_samples)
            break;
        size_t len = std::min(LATE_PART_LEN, ir_samples - offset);

        memset(fft_buf_late, 0, sizeof(fft_buf_late));
        for(size_t n = 0; n < len; n++)
            fft_buf_late[2 * n] = ir_raw_L[offset + n];
        arm_cfft_f32(&cfft_inst_late, fft_buf_late, 0, 1);
        memcpy(ir_spec_late[p], fft_buf_late, sizeof(float) * LATE_SPEC_LEN);
    }

    euro.PrintLine("  IR FFT complete.");
    return true;
}

/* ========================================================================
 *  Process a full PART_LEN block of convolution
 * ======================================================================== */
static void ProcessConvBlock()
{
    if(active_partitions == 0)
    {
        memset(out_buf_L, 0, sizeof(out_buf_L));
        return;
    }
    if(fdl_pos >= active_partitions)
        fdl_pos = 0;

    uint32_t conv_start_us = 0;
    if(CPU_DEBUG_LOG)
        conv_start_us = System::GetUs();

    /* --- Build zero-padded current block for partitioned OLA ---
     * x_b = [current PART_LEN | 0...0] */
    memset(ols_buf_L, 0, sizeof(ols_buf_L));
    memcpy(ols_buf_L, collect_L, PART_LEN * sizeof(float));

    /* --- FORWARD FFT of the input blocks --- */
    float in_spec_L[SPEC_LEN];

    memset(fft_buf, 0, sizeof(fft_buf));
    for(size_t n = 0; n < FFT_LEN; n++)
        fft_buf[2 * n] = ols_buf_L[n];
    arm_cfft_f32(&cfft_inst, fft_buf, 0, 1); // forward
    memcpy(in_spec_L, fft_buf, sizeof(float) * SPEC_LEN);

    /* --- STORE into FDL at current position --- */
    memcpy(fdl_L[fdl_pos], in_spec_L, SPEC_LEN * sizeof(float));

    /* --- ACCUMULATE: multiply each FDL entry with corresponding IR partition --- */
    memset(acc_spec_L, 0, SPEC_LEN * sizeof(float));

    for(size_t p = 0; p < active_partitions; p++)
    {
        /* FDL is a circular buffer — partition 0 of the IR convolves
         * with the most recent input, partition 1 with the previous, etc. */
        size_t fdl_idx = (fdl_pos + active_partitions - p) % active_partitions;

        spec_cmac(acc_spec_L, fdl_L[fdl_idx], ir_spec_L[p], SPEC_LEN);
    }

    /* --- INVERSE FFT ---
     * arm_cfft_f32(ifft=1) applies 1/N scaling internally. */
    arm_cfft_f32(&cfft_inst, acc_spec_L, 1, 1);

    /* --- OVERLAP-ADD --- */
    for(size_t n = 0; n < PART_LEN; n++)
    {
        float yL0 = acc_spec_L[2 * n];
        float yL1 = acc_spec_L[2 * (n + PART_LEN)];

        out_buf_L[n]  = yL0 + ola_tail_L[n];
        ola_tail_L[n] = yL1;
    }

    /* --- ADVANCE FDL position --- */
    fdl_pos = (fdl_pos + 1) % active_partitions;

    if(CPU_DEBUG_LOG)
    {
        uint32_t conv_elapsed = System::GetUs() - conv_start_us;
        if(conv_elapsed > 1000000u)
        {
            dbg_conv_timing_glitches++;
            conv_elapsed = 0;
        }
        dbg_conv_count++;
        dbg_conv_us_accum += conv_elapsed;
        if(conv_elapsed > dbg_conv_us_max)
            dbg_conv_us_max = conv_elapsed;
        if(conv_elapsed > DBG_CONV_BUDGET_US)
            dbg_conv_overrun_count++;
    }

}

static void ProcessLateBlock()
{
    if(!late_stage_enabled || late_active_partitions == 0)
    {
        memset(out_buf_late, 0, sizeof(out_buf_late));
        return;
    }
    if(late_fdl_pos >= late_active_partitions)
        late_fdl_pos = 0;

    uint32_t conv_start_us = 0;
    if(CPU_DEBUG_LOG)
        conv_start_us = System::GetUs();

    memset(fft_buf_late, 0, sizeof(fft_buf_late));
    for(size_t n = 0; n < LATE_PART_LEN; n++)
        fft_buf_late[2 * n] = collect_late[n];
    arm_cfft_f32(&cfft_inst_late, fft_buf_late, 0, 1);

    memcpy(fdl_late[late_fdl_pos], fft_buf_late, sizeof(float) * LATE_SPEC_LEN);
    memset(acc_spec_late, 0, sizeof(float) * LATE_SPEC_LEN);

    for(size_t p = 0; p < late_active_partitions; p++)
    {
        size_t fdl_idx = (late_fdl_pos + late_active_partitions - p) % late_active_partitions;
        spec_cmac(acc_spec_late, fdl_late[fdl_idx], ir_spec_late[p], LATE_SPEC_LEN);
    }

    arm_cfft_f32(&cfft_inst_late, acc_spec_late, 1, 1);
    for(size_t n = 0; n < LATE_PART_LEN; n++)
    {
        float y0 = acc_spec_late[2 * n];
        float y1 = acc_spec_late[2 * (n + LATE_PART_LEN)];
        out_buf_late[n]  = y0 + ola_tail_late[n];
        ola_tail_late[n] = y1;
    }

    late_fdl_pos = (late_fdl_pos + 1) % late_active_partitions;

    if(CPU_DEBUG_LOG)
    {
        uint32_t conv_elapsed = System::GetUs() - conv_start_us;
        if(conv_elapsed > 1000000u)
        {
            dbg_conv_timing_glitches++;
            conv_elapsed = 0;
        }
        dbg_conv_count++;
        dbg_conv_us_accum += conv_elapsed;
        if(conv_elapsed > dbg_conv_us_max)
            dbg_conv_us_max = conv_elapsed;
        if(conv_elapsed > DBG_CONV_BUDGET_US)
            dbg_conv_overrun_count++;
    }
}

/* ========================================================================
 *  Audio callback
 * ======================================================================== */

/** Read position into out_buf for output playback */
static size_t out_read_pos = 0;
static size_t out_read_pos_late = 0;

static void AudioCallback(AudioHandle::InputBuffer  in,
                           AudioHandle::OutputBuffer out,
                           size_t                    size)
{
    uint32_t cb_start_us = 0;
    if(CPU_DEBUG_LOG)
        cb_start_us = System::GetUs();

    euro.patch.ProcessAllControls();
    euro.toggle1.Debounce();
    euro.toggle2.Debounce();
    euro.toggle3.Debounce();

    /* ---- Read/smooth knobs ---- */
    if(FREEZE_CONTROLS_FOR_DEBUG)
    {
        ctrl_mix       = FIXED_MIX;
        ctrl_wet_level = FIXED_WET_LEVEL;
        ctrl_in_gain   = FIXED_IN_GAIN;
        ctrl_tone_knob = FIXED_TONE_KNOB;
    }
    else
    {
        float raw_mix       = euro.GetKnobValue(planter::KNOB_1);
        float raw_wet_level = euro.GetKnobValue(planter::KNOB_2);
        float raw_in_gain   = 0.2f + euro.GetKnobValue(planter::KNOB_3) * 0.8f;
        float raw_tone_knob = euro.GetKnobValue(planter::KNOB_4);

        static constexpr float CTRL_SMOOTH = 0.003f;
        ctrl_mix       += CTRL_SMOOTH * (raw_mix - ctrl_mix);
        ctrl_wet_level += CTRL_SMOOTH * (raw_wet_level - ctrl_wet_level);
        ctrl_in_gain   += CTRL_SMOOTH * (raw_in_gain - ctrl_in_gain);
        ctrl_tone_knob += CTRL_SMOOTH * (raw_tone_knob - ctrl_tone_knob);
    }

    float mix       = ctrl_mix;
    float wet_level = ctrl_wet_level;
    float in_gain   = ctrl_in_gain;
    float tone_knob = ctrl_tone_knob;

    // Hardware override for debugging: toggle1 UP forces hard dry bypass.
    const bool force_dry = euro.toggle1.Pressed();
    if(force_dry)
        mix = 0.f;

    // True dry-bypass mode: skip all convolution work to remove DSP/CPU side effects.
    // This is intentionally direct pass-through (unity), for fault isolation.
    static bool prev_force_dry = false;
    if(force_dry)
    {
        if(!prev_force_dry)
        {
            collect_pos  = 0;
            late_collect_pos = 0;
            out_read_pos = 0;
            out_read_pos_late = 0;
            memset(collect_L, 0, sizeof(collect_L));
            memset(collect_late, 0, sizeof(collect_late));
            memset(out_buf_L, 0, sizeof(out_buf_L));
            memset(out_buf_late, 0, sizeof(out_buf_late));
            memset(ola_tail_late, 0, sizeof(ola_tail_late));
            memset(fdl_late, 0, sizeof(fdl_late));
            lp_state_L = 0.f;
        }
        prev_force_dry = true;

        for(size_t i = 0; i < size; i++)
        {
            out[0][i] = in[0][i];
            out[1][i] = in[0][i];
        }
        return;
    }
    prev_force_dry = false;

    const bool desired_late_enabled = euro.toggle2.Pressed();
    if(desired_late_enabled != late_stage_enabled)
    {
        late_stage_enabled = desired_late_enabled;
        late_collect_pos   = 0;
        late_fdl_pos       = 0;
        out_read_pos_late  = 0;
        memset(collect_late, 0, sizeof(collect_late));
        memset(out_buf_late, 0, sizeof(out_buf_late));
        memset(ola_tail_late, 0, sizeof(ola_tail_late));
        memset(fdl_late, 0, sizeof(fdl_late));
    }

    /* Tone: one-pole LP coefficient */
    float lp_freq = 200.f + tone_knob * 18000.f; // 200 Hz .. 18.2 kHz
    float lp_coeff = 1.f - expf(-2.f * 3.14159265f * lp_freq / 48000.f);

    if(!ir_loaded)
    {
        /* Pass-through if IR not loaded */
        for(size_t i = 0; i < size; i++)
        {
            out[0][i] = in[0][i];
            out[1][i] = in[0][i];
        }
        return;
    }

    for(size_t i = 0; i < size; i++)
    {
        float dry_mono = in[0][i] * in_gain;

        /* Read convolved output */
        float wet_mono = (out_buf_L[out_read_pos] + out_buf_late[out_read_pos_late]) * wet_level;
        out_read_pos++;
        out_read_pos_late++;

        if(!std::isfinite(wet_mono))
            wet_mono = 0.f;

        /* Tone filter on wet signal */
        lp_state_L += lp_coeff * (wet_mono - lp_state_L);
        wet_mono = lp_state_L;

        if(CPU_DEBUG_LOG)
        {
            float wet_abs = fabsf(wet_mono);
            if(wet_abs > dbg_wet_peak)
                dbg_wet_peak = wet_abs;
            dbg_wet_rms_accum += wet_mono * wet_mono;
            dbg_wet_rms_count++;
        }

        /* Mix dry + wet (with hard bypass near zero mix to avoid NaN propagation) */
        float out_L = 0.f;
        if(mix <= MIX_DRY_BYPASS_THRESH)
        {
            out_L = dry_mono;
        }
        else
        {
            out_L = dry_mono * (1.f - mix) + wet_mono * mix;
        }

        float out_R = out_L;

        if(!std::isfinite(out_L))
            out_L = 0.f;
        if(!std::isfinite(out_R))
            out_R = 0.f;

        /* Soft clamp */
        out_L = std::max(-OUTPUT_LIMIT, std::min(OUTPUT_LIMIT, out_L));
        out_R = std::max(-OUTPUT_LIMIT, std::min(OUTPUT_LIMIT, out_R));

        out[0][i] = out_L;
        out[1][i] = out_R;

        /* Collect input samples for the next convolution block */
        collect_L[collect_pos] = dry_mono;
        collect_pos++;
        collect_late[late_collect_pos] = dry_mono;
        late_collect_pos++;

        /* When we've collected PART_LEN samples, run convolution
         * for the NEXT output block and reset playback index. */
        if(collect_pos >= PART_LEN)
        {
            ProcessConvBlock();
            collect_pos  = 0;
            out_read_pos = 0;
        }
        if(late_collect_pos >= LATE_PART_LEN)
        {
            ProcessLateBlock();
            late_collect_pos  = 0;
            out_read_pos_late = 0;
        }
    }

    if(CPU_DEBUG_LOG)
    {
        uint32_t cb_elapsed = System::GetUs() - cb_start_us;
        if(cb_elapsed > 1000000u)
        {
            dbg_cb_timing_glitches++;
            cb_elapsed = 0;
        }
        uint32_t cb_budget_us = static_cast<uint32_t>((size * 1000000.0f) / 48000.0f);
        dbg_cb_count++;
        dbg_cb_us_accum += cb_elapsed;
        if(cb_elapsed > dbg_cb_us_max)
            dbg_cb_us_max = cb_elapsed;
        if(cb_elapsed > cb_budget_us)
            dbg_cb_overrun_count++;
        dbg_cb_budget_us_last = cb_budget_us;
    }
}

/* ========================================================================
 *  Main
 * ======================================================================== */
int main(void)
{
    /* ---- Hardware init ---- */
    euro.Init();
    euro.patch.SetAudioBlockSize(PART_LEN);
    euro.StartLog(true);  // block until serial terminal connects
    System::Delay(1000);

    euro.PrintLine("========================================");
    euro.PrintLine(" ConvReverb — Stereo Convolution Reverb");
    euro.PrintLine("========================================");
    if(DEBUG_MODE)
    {
        euro.PrintLine("CPU debug serial: %s", CPU_DEBUG_LOG ? "ON" : "OFF");
        euro.PrintLine("Controls frozen for debug: %s",
                       FREEZE_CONTROLS_FOR_DEBUG ? "YES (knobs ignored)" : "NO");
        euro.PrintLine("CPU probe: MAX_ACTIVE_PARTITIONS=%lu",
                       (unsigned long)MAX_ACTIVE_PARTITIONS);
        euro.PrintLine("CPU debug armed (expect 1 line/sec once audio starts)");
    }

    /* ---- Initialize CMSIS CFFT instance for FFT_LEN ---- */
    arm_cfft_init_f32(&cfft_inst, FFT_LEN);
    arm_cfft_init_f32(&cfft_inst_late, LATE_FFT_LEN);

    /* ---- Clear convolution state ---- */
    memset(ols_buf_L, 0, sizeof(ols_buf_L));
    memset(out_buf_L, 0, sizeof(out_buf_L));
    memset(out_buf_late, 0, sizeof(out_buf_late));
    memset(ola_tail_L, 0, sizeof(ola_tail_L));
    memset(ola_tail_late, 0, sizeof(ola_tail_late));
    memset(collect_L, 0, sizeof(collect_L));
    memset(collect_late, 0, sizeof(collect_late));
    memset(fdl_L, 0, sizeof(fdl_L));
    memset(fdl_late, 0, sizeof(fdl_late));
    collect_pos  = 0;
    late_collect_pos = 0;
    fdl_pos      = 0;
    late_fdl_pos = 0;
    out_read_pos = 0;
    out_read_pos_late = 0;

    /* ---- Load IR from SD card ---- */
    ir_loaded = LoadIR();

    if(ir_loaded)
    {
        euro.PrintLine("IR loaded successfully — starting audio.");
        euro.SetLed(planter::LED_1, true);
        euro.SetLed(planter::LED_2, false);
    }
    else
    {
        euro.PrintLine("IR load FAILED — running in pass-through mode.");
        euro.PrintLine("Ensure 'ir.wav' is on the SD card root.");
        euro.SetLed(planter::LED_1, false);
        euro.SetLed(planter::LED_2, true);
    }

    /* ---- Set audio block size (48 is the Planter default — fine here) ---- */
    euro.patch.StartAudio(AudioCallback);

    /* ---- Main loop (LED heartbeat) ---- */
    uint32_t last_led_time = System::GetNow();
    bool     led_state     = true;

    while(1)
    {
        uint32_t now = System::GetNow();
        if(now - last_led_time > 1000)
        {
            if(ir_loaded)
            {
                euro.SetLed(planter::LED_1, led_state);
                led_state = !led_state;
            }

            uint32_t cb_count         = dbg_cb_count;
            uint32_t cb_us_accum      = dbg_cb_us_accum;
            uint32_t cb_us_max        = dbg_cb_us_max;
            uint32_t cb_overruns      = dbg_cb_overrun_count;
            uint32_t cb_budget_us     = dbg_cb_budget_us_last;
            uint32_t cb_glitches      = dbg_cb_timing_glitches;

            uint32_t conv_count       = dbg_conv_count;
            uint32_t conv_us_accum    = dbg_conv_us_accum;
            uint32_t conv_us_max      = dbg_conv_us_max;
            uint32_t conv_overruns    = dbg_conv_overrun_count;
            uint32_t conv_glitches    = dbg_conv_timing_glitches;

            float wet_peak            = dbg_wet_peak;
            float wet_rms = (dbg_wet_rms_count > 0)
                                ? sqrtf(dbg_wet_rms_accum / static_cast<float>(dbg_wet_rms_count))
                                : 0.f;

            dbg_cb_count           = 0;
            dbg_cb_us_accum        = 0;
            dbg_cb_us_max          = 0;
            dbg_cb_overrun_count   = 0;
            dbg_cb_timing_glitches = 0;

            dbg_conv_count         = 0;
            dbg_conv_us_accum      = 0;
            dbg_conv_us_max        = 0;
            dbg_conv_overrun_count = 0;
            dbg_conv_timing_glitches = 0;

            dbg_wet_peak           = 0.f;
            dbg_wet_rms_accum      = 0.f;
            dbg_wet_rms_count      = 0;

            if(CPU_DEBUG_LOG)
            {
                uint32_t cb_us_avg   = (cb_count > 0) ? (cb_us_accum / cb_count) : 0;
                uint32_t conv_us_avg = (conv_count > 0) ? (conv_us_accum / conv_count) : 0;
                euro.PrintLine(
                    "DBG ir=%d cb(avg/max/bud/ovr/g): %lu/%lu/%lu/%lu/%lu/%lu | cv: %lu/%lu/%lu/%lu/%lu/%lu | wet(rms/peak): "
                    FLT_FMT3 "/" FLT_FMT3,
                    ir_loaded ? 1 : 0,
                    (unsigned long)cb_us_avg,
                    (unsigned long)cb_us_max,
                    (unsigned long)cb_budget_us,
                    (unsigned long)cb_overruns,
                    (unsigned long)cb_count,
                    (unsigned long)cb_glitches,
                    (unsigned long)conv_us_avg,
                    (unsigned long)conv_us_max,
                    (unsigned long)DBG_CONV_BUDGET_US,
                    (unsigned long)conv_overruns,
                    (unsigned long)conv_count,
                    (unsigned long)conv_glitches,
                    FLT_VAR3(wet_rms),
                    FLT_VAR3(wet_peak));
            }

            last_led_time = now;
        }
        euro.button.Debounce();
        euro.BootloaderResetCheck();
    }
}
