/**
 * PhysmodBar.cpp — Physical model of a stiff string / bar for the Anachrome Electronics Planter.
 *
 * Ported from Stefan Bilbao, Numerical Sound Synthesis, 2009 Stiff String code example (MATLAB),
 * with modifications for real-time control and performance, 
 * including polyphonic voice allocation for multiple simultaneous strikes.
 *
 * Trigger sources: Planter button (rising edge) or Gate In 1.
 * Knob map (values captured at the instant of each trigger):
 *   KNOB 1 – Pitch             (MIDI note 24–96, C1–C7)
 *   KNOB 2 – Strike Width      (0.1–1.0)
 *   KNOB 3 – Strike/Pluck blend (0=pure strike, 1=pure pluck)
 *   KNOB 4 – Decay             (T60 base 0.5–10 s)
 *   KNOB 5 – Fine tune         (-1 to +1 semitone)
 *   KNOB 6 – Brightness        (high-freq T60 ratio, 0.1–1.0)
 *
 * Toggle 1: Excitation position (up = 0.4, down = 0.05 along the bar)
 *
 * CV In 1 is read at trigger time and added as 1 V/oct pitch offset.
 *
 * LED 1 flashes on each new strike; LED 2 indicates voice activity.
 *
 * Stereo output uses two readout positions (0.30 / 0.70) to create stereo image from single strike.
 */

#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "planter.h"
#include <cmath>
#include <cstring>

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace anachrome;

/* ====================================================================
 *  Compile-time configuration
 * ==================================================================== */
static constexpr int   MAX_N       = 24;     // max spatial grid segments
static constexpr int   MAX_VOICES  = 4;       // max simultaneous strikes
static constexpr float MAX_TF      = 10.0f;   // max strike duration (s)
static constexpr float SILENCE_THR = 1e-7f;   // voice deactivation threshold
static constexpr int   ATTACK_SAMPS = 48;      // ~1 ms attack ramp at 48 kHz
static constexpr float OUTPUT_GAIN = 300.0f; // readout-to-audio scaling
// PI_F is already defined as a macro in DaisySP/Source/Utility/dsp.h
static constexpr float STEAL_DECAY = 0.99f;   // voice-steal fade ~2 ms at 48 kHz

/* Shared temp buffer - reused serially by each voice inside the callback */
static float tmp_rhs[MAX_N];

/* ====================================================================
 *  StiffStringVoice – one concurrent finite-difference strike
 *
 *  Implements an implicit FD scheme for the stiff string equation
 *  with clamped boundary conditions and two-parameter frequency-
 *  dependent loss.  All matrices are banded (tri- or penta-diagonal)
 *  and stored as scalar coefficients.  The tridiagonal solve uses a
 *  pre-factored Thomas algorithm (O(N) per sample).
 * ==================================================================== */
struct StiffStringVoice
{
    /* --- state --- */
    bool active     = false;
    int  N          = 0;      // grid segments
    int  n_int      = 0;      // interior points = N-1
    int  NF         = 0;      // max samples for this strike
    int  sample_idx = 0;

    /* Grid displacement (interior points only; boundaries = 0) */
    float u [MAX_N];          // current step
    float u1[MAX_N];          // previous step
    float u2[MAX_N];          // two steps back

    /* Pre-factored Thomas coefficients for tridiagonal A */
    float th_c   [MAX_N];    // modified super-diagonal
    float th_invm[MAX_N];    // 1 / modified main diagonal
    float A_off;              // off-diagonal value of A (symmetric)

    /* Tridiagonal C (loss complement) */
    float C_main, C_off;

    /* Pentadiagonal B (wave + stiffness) */
    float B_main, B_off1, B_off2;

    /* Stereo readout interpolation */
    int   rp_idx [2];
    float rp_frac[2];

    /* Attack ramp: eliminates click from pluck initial-condition discontinuity */
    float attack_env = 0.0f;
    float attack_inc = 0.0f;

    /* Voice-steal fade (applied externally in audio callback, not in Process) */
    float steal_env  = 1.0f;
    bool  fading_out = false;
    struct PendingParams {
        float SR, B_param, f0, TF, ctr, wid, u0, v0;
        float loss_f1, loss_t1, loss_f2, loss_t2, rp1, rp2;
    };
    PendingParams pending;

    void BeginFadeOut(float SR_, float B_, float f0_, float TF_,
                      float ctr_, float wid_, float u0_, float v0_,
                      float lf1_, float lt1_, float lf2_, float lt2_,
                      float rp1_, float rp2_)
    {
        pending = {SR_, B_, f0_, TF_, ctr_, wid_, u0_, v0_,
                   lf1_, lt1_, lf2_, lt2_, rp1_, rp2_};
        fading_out = true;
        // Don't reset steal_env — let an in-progress fade continue
    }

    /* ---------------------------------------------------------------
     *  Init – compute coefficients & initial conditions from params
     * ------------------------------------------------------------- */
    void Init(float SR,
              float B_param,   // inharmonicity (>0)
              float f0,        // fundamental (Hz)
              float TF,        // max duration (s)
              float ctr,       // excitation centre (0-1)
              float wid,       // excitation width  (0-1)
              float u0,        // initial displacement amplitude
              float v0,        // initial velocity amplitude
              float loss_f1, float loss_t1,  // low-freq loss
              float loss_f2, float loss_t2,  // high-freq loss
              float rp1,     float rp2)      // readout positions
    {
        const float k   = 1.0f / SR;
        NF              = static_cast<int>(SR * TF);
        const float gam = 2.0f * f0;
        const float Kv  = sqrtf(B_param) * gam / PI_F;

        /* Implicit-scheme free parameter (from MATLAB: theta = f0^1.25/1000+0.6) */
        const float theta = f0 * sqrtf(sqrtf(f0)) / 1000.0f + 0.6f;

        /* ---- Stability condition → grid spacing h ---- */
        const float g2k2      = gam * gam * k * k;
        const float two_th_m1 = 2.0f * theta - 1.0f;
        float h = sqrtf((g2k2
                         + sqrtf(g2k2 * g2k2
                                 + 16.0f * Kv * Kv * k * k * two_th_m1))
                        / (2.0f * two_th_m1));
        N = static_cast<int>(1.0f / h);
        if(N < 4)     N = 4;
        if(N > MAX_N) N = MAX_N;
        h     = 1.0f / static_cast<float>(N);
        n_int = N - 1;

        const float mu     = Kv * k / (h * h);
        const float lambda = gam * k / h;

        /* ---- Loss parameters (two-parameter freq-dependent) ---- */
        const float K2 = Kv * Kv;
        const float g2 = gam * gam;
        const float g4 = g2 * g2;
        const float w1 = 2.0f * PI_F * loss_f1;
        const float w2 = 2.0f * PI_F * loss_f2;

        const float zeta1 = (-g2 + sqrtf(g4 + 4.0f * K2 * w1 * w1))
                            / (2.0f * K2);
        const float zeta2 = (-g2 + sqrtf(g4 + 4.0f * K2 * w2 * w2))
                            / (2.0f * K2);
        const float ln10x6 = 6.0f * logf(10.0f);
        const float dz     = zeta1 - zeta2;
        const float sig0 = ln10x6
                            * (-zeta2 / loss_t1 + zeta1 / loss_t2) / dz;
        const float sig1 = ln10x6
                            * (1.0f / loss_t1 - 1.0f / loss_t2) / dz;

        /* ---- Matrix band coefficients ---- */
        const float h2 = h * h;

        /* A  (tridiagonal, implicit + loss) */
        float A_main = theta + sig1 * k / h2 + sig0 * k * 0.5f;
        A_off        = (1.0f - theta) * 0.5f - sig1 * k / (2.0f * h2);

        /* C  (tridiagonal, loss complement) */
        C_main = theta - sig1 * k / h2 - sig0 * k * 0.5f;
        C_off  = (1.0f - theta) * 0.5f + sig1 * k / (2.0f * h2);

        /* B  (pentadiagonal, wave + stiffness) */
        B_main = 2.0f * theta - 2.0f * lambda * lambda - 6.0f * mu * mu;
        B_off1 = (1.0f - theta) + lambda * lambda + 4.0f * mu * mu;
        B_off2 = -(mu * mu);

        /* ---- Pre-factor Thomas algorithm for A ---- */
        th_invm[0] = 1.0f / A_main;
        th_c[0]    = A_off * th_invm[0];
        for(int i = 1; i < n_int; i++)
        {
            float m    = A_main - A_off * th_c[i - 1];
            th_invm[i] = 1.0f / m;
            th_c[i]    = (i < n_int - 1) ? A_off * th_invm[i] : 0.0f;
        }

        /* ---- Readout interpolation (0-based indexing) ---- */
        for(int ch = 0; ch < 2; ch++)
        {
            float rv = (ch == 0) ? rp1 : rp2;
            int   ri = static_cast<int>(N * rv);
            float rf = N * rv - static_cast<float>(ri);
            if(ri >= n_int - 1) { ri = n_int - 2; rf = 1.0f; }
            if(ri < 0)          { ri = 0;          rf = 0.0f; }
            rp_idx[ch]  = ri;
            rp_frac[ch] = rf;
        }

        /* ---- Raised-cosine initial condition ---- */
        for(int i = 0; i < n_int; i++)
        {
            float x  = static_cast<float>(i + 1) * h;
            float d1 = x - ctr - wid * 0.5f;
            float d2 = x - ctr + wid * 0.5f;
            float rc = (-d1 * d2 > 0.0f)
                           ? 0.5f * (1.0f + cosf(2.0f * PI_F * (x - ctr) / wid))
                           : 0.0f;
            u2[i] = u0 * rc;
            u1[i] = (u0 + k * v0) * rc;
            u[i]  = 0.0f;
        }

        sample_idx  = 2;   // first Process() computes timestep 3 (matching MATLAB)
        active      = true;
        attack_env  = 0.0f;
        attack_inc  = 1.0f / static_cast<float>(ATTACK_SAMPS);
        steal_env   = 1.0f;
        fading_out  = false;
    }

    /* ---------------------------------------------------------------
     *  Process – advance one sample, return stereo output
     * ------------------------------------------------------------- */
    void Process(float& out_l, float& out_r)
    {
        if(!active) { out_l = out_r = 0.0f; return; }

        const int n = n_int;

        /* RHS = B·u1 − C·u2  (combined loop, no extra buffer) */
        for(int i = 0; i < n; i++)
        {
            float bv = B_main * u1[i];
            float cv = C_main * u2[i];
            if(i > 0)     { bv += B_off1 * u1[i - 1]; cv += C_off * u2[i - 1]; }
            if(i < n - 1) { bv += B_off1 * u1[i + 1]; cv += C_off * u2[i + 1]; }
            if(i > 1)       bv += B_off2 * u1[i - 2];
            if(i < n - 2)   bv += B_off2 * u1[i + 2];
            tmp_rhs[i] = bv - cv;
        }

        /* Solve A·u = rhs  (Thomas algorithm, pre-factored) */
        tmp_rhs[0] *= th_invm[0];
        for(int i = 1; i < n; i++)
            tmp_rhs[i] = (tmp_rhs[i] - A_off * tmp_rhs[i - 1]) * th_invm[i];
        for(int i = n - 2; i >= 0; i--)
            tmp_rhs[i] -= th_c[i] * tmp_rhs[i + 1];

        /* Copy solution into u */
        memcpy(u, tmp_rhs, n * sizeof(float));

        /* Stereo readout with linear interpolation */
        out_l = (1.0f - rp_frac[0]) * u[rp_idx[0]]
                + rp_frac[0] * u[rp_idx[0] + 1];
        out_r = (1.0f - rp_frac[1]) * u[rp_idx[1]]
                + rp_frac[1] * u[rp_idx[1] + 1];

        /* Attack ramp to avoid click from pluck displacement discontinuity */
        if(attack_env < 1.0f)
        {
            out_l *= attack_env;
            out_r *= attack_env;
            attack_env += attack_inc;
            if(attack_env > 1.0f) attack_env = 1.0f;
        }

        /* Shift state: u2 ← u1 ← u */
        memcpy(u2, u1, n * sizeof(float));
        memcpy(u1, u,  n * sizeof(float));

        sample_idx++;

        /* Deactivate on max duration */
        if(sample_idx >= NF) { active = false; return; }

        /* Periodic silence check (every ~4096 samples ≈ 85 ms) */
        if((sample_idx & 4095) == 0)
        {
            float peak = 0.0f;
            for(int i = 0; i < n; i++)
            {
                float a = fabsf(u[i]);
                if(a > peak) peak = a;
            }
            if(peak < SILENCE_THR) active = false;
        }
    }
};

/* ====================================================================
 *  Globals
 * ==================================================================== */
static StiffStringVoice voices[MAX_VOICES];
static planter          euro;

/* Trigger edge detection */
static bool prev_button = false;
static bool prev_gate1  = false;

/* LED flash counter (audio blocks) */
static int led_flash_ctr = 0;

/* ====================================================================
 *  TriggerStrike – sample knobs, allocate a voice, initialise it
 * ==================================================================== */
static void TriggerStrike()
{
    /* Find a free voice, or prefer one already fading, else steal oldest */
    int  idx      = -1;
    bool was_free = false;
    int  fading_idx = -1;
    int  oldest   = 0, oldest_age = -1;
    for(int i = 0; i < MAX_VOICES; i++)
    {
        if(!voices[i].active)
        {
            idx = i; was_free = true; break;
        }
        /* Prefer a voice already fading — it's about to be freed anyway */
        if(voices[i].fading_out && fading_idx < 0)
            fading_idx = i;
        if(voices[i].sample_idx > oldest_age)
        {
            oldest_age = voices[i].sample_idx;
            oldest     = i;
        }
    }
    if(idx < 0)
    {
        /* No free voice: reuse one already fading if possible, else steal oldest */
        idx = (fading_idx >= 0) ? fading_idx : oldest;
    }

    const float SR = euro.patch.AudioSampleRate();

    /* Read knob values (all return 0.0–1.0) */
    const float k1 = euro.GetKnobValue(planter::KNOB_1);
    const float k2 = euro.GetKnobValue(planter::KNOB_2);
    const float k3 = euro.GetKnobValue(planter::KNOB_3);
    const float k4 = euro.GetKnobValue(planter::KNOB_4);
    const float k5 = euro.GetKnobValue(planter::KNOB_5);
    const float k6 = euro.GetKnobValue(planter::KNOB_6);

    /* Read CV In 1 for 1 V/oct pitch offset (±5 V → ±60 semitones) */
    const float cv1_volts = euro.GetCvIn(0) * 5.0f;

    /* ---- Map to physical parameters ---- */
    // Knob 1: coarse pitch  C1–C7; KNOB 5: fine tune ±1 semitone; CV In 1: 1V/oct
    float midi_note = 24.0f + k1 * 72.0f;    // C1 (24) – C7 (96)
    midi_note      += (k5 * 2.0f - 1.0f);    // fine tune: -1 to +1 semitone
    midi_note      += cv1_volts * 12.0f;      // 1 V/oct
    float f0 = mtof(midi_note);

    //float B_param = 0.5f + k2 * 1.5f;    // inharmonicity  0.001–1.0
    float B_param = 1.0f;
    // Knob 3: crossfade between strike [v0>0, u0=0] and pluck [u0>0, v0=0].
    // v0 is scaled proportionally to f0 so that struck-string output energy
    // (which is ∝ v0/f0 per mode) stays perceptually level across pitch.
    // u0 displacement amplitude is frequency-independent and needs no scaling.
    const float f_ref  = 440.0f;           // reference pitch for v0 normalisation
    float v0_base = 7.0f - k3 * 7.0f;     // 7.0 (full strike) -> 0.0 (full pluck)
    float v0      = v0_base * (f0 / f_ref);// compensate: louder at high pitch, quieter at low
    float u0      = k3 * 0.001f;          // 0.0 (strike) -> 0.001 (pluck)
    float decay   = 0.5f   + k4 * 8.0f;     // T60 base        0.5–8 s
    // Toggle 1: excitation position (up=0.5, down=0.05)
    float ctr     = euro.toggle1.Pressed() ? 0.35f : 0.05f;
    float bright  = 0.1f   + k6 * 0.90f;     // hi-freq T60 ratio 0.1–1.0

    const float wid = 0.1f  + k2 * 0.9f;                  // excitation width 0.1 -> 1.0 (normalized to string length)

    /* Two-parameter frequency-dependent loss */
    const float loss_f1 = 100.0f;
    const float loss_t1 = decay;
    const float loss_f2 = 1000.0f;
    const float loss_t2 = decay * bright;

    /* Stereo readout positions */
    const float rp1 = 0.30f;
    const float rp2 = 0.70f;

    if(was_free)
    {
        voices[idx].Init(SR, B_param, f0, MAX_TF,
                         ctr, wid, u0, v0,
                         loss_f1, loss_t1, loss_f2, loss_t2,
                         rp1, rp2);
    }
    else
    {
        voices[idx].BeginFadeOut(SR, B_param, f0, MAX_TF,
                                 ctr, wid, u0, v0,
                                 loss_f1, loss_t1, loss_f2, loss_t2,
                                 rp1, rp2);
    }

    led_flash_ctr = 5;  // flash LED 1 for ~5 blocks
}

/* ====================================================================
 *  Audio Callback
 * ==================================================================== */
void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    euro.patch.ProcessAllControls();
    euro.button.Debounce();
    euro.toggle1.Debounce();

    /* ---- Trigger detection (rising edge) ---- */
    bool btn_now  = euro.button.Pressed();
    bool gate_now = euro.patch.gate_in_1.State();

    if(btn_now  && !prev_button) TriggerStrike();
    if(gate_now && !prev_gate1)  TriggerStrike();

    prev_button = btn_now;
    prev_gate1  = gate_now;

    /* ---- Process audio ---- */
    for(size_t i = 0; i < size; i++)
    {
        float mix_l = 0.0f, mix_r = 0.0f;

        for(int v = 0; v < MAX_VOICES; v++)
        {
            if(voices[v].active)
            {
                float l, r;
                voices[v].Process(l, r);

                /* Voice-steal fade-out envelope */
                if(voices[v].fading_out)
                {
                    l *= voices[v].steal_env;
                    r *= voices[v].steal_env;
                    voices[v].steal_env *= STEAL_DECAY;
                    if(voices[v].steal_env < 1e-4f)
                    {
                        auto& p = voices[v].pending;
                        voices[v].Init(p.SR, p.B_param, p.f0, p.TF,
                                       p.ctr, p.wid, p.u0, p.v0,
                                       p.loss_f1, p.loss_t1,
                                       p.loss_f2, p.loss_t2,
                                       p.rp1, p.rp2);
                        l = r = 0.0f;
                    }
                }

                mix_l += l;
                mix_r += r;
            }
        }

        /* Scale and hard-clip */
        float out_l = mix_l * OUTPUT_GAIN;
        float out_r = mix_r * OUTPUT_GAIN;
        if(out_l >  1.0f) out_l =  1.0f;
        if(out_l < -1.0f) out_l = -1.0f;
        if(out_r >  1.0f) out_r =  1.0f;
        if(out_r < -1.0f) out_r = -1.0f;

        out[0][i] = out_l;
        out[1][i] = out_r;
    }

    /* ---- LED feedback ---- */
    bool any_active = false;
    for(int v = 0; v < MAX_VOICES; v++)
        if(voices[v].active) { any_active = true; break; }

    euro.SetLed(2, any_active);            // LED 2: voice activity

    if(led_flash_ctr > 0)
    {
        euro.SetLed(1, true);              // LED 1: strike flash
        led_flash_ctr--;
    }
    else
    {
        euro.SetLed(1, false);
    }
}

/* ====================================================================
 *  Main
 * ==================================================================== */
int main(void)
{
    euro.Init();

    for(int i = 0; i < MAX_VOICES; i++)
        voices[i].active = false;

    euro.patch.StartAudio(AudioCallback);

    while(1)
    {
        euro.BootloaderResetCheck();
    }
}
