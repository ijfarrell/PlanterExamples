/**
 * VaDist.cpp - Virtual analog distortion effect for the Anachrome Electronics Planter.
 *
 * Ported from TubeScreamer.m MATLAB code example, using nodal K method for stateful
 * simulation of nonlinear analog overdrive circuit.
 *
 * Knob map:
 *   KNOB 1 – Drive (0–5)
 *   KNOB 2 – Tone Control (0-1)
 *   KNOB 3 – Master Volume (0-1)
 *   KNOB 4 – Clean Blend (0 = full wet, 1 = full dry)
 *   KNOB 5 – --
 *   KNOB 6 – --
 *
 * CV In 1: Drive CV (0-5 V → 0-100)
 * CV In 2: Tone CV (0-5 V → 0-1)
 * CV In 3: Volume CV (0-5 V → 0-1)
 * CV In 4: Clean Blend CV (0-5 V → 0-1)
 *
 * Toggle 1: Symmetric (up) vs asymmetric (down) diode clipping.
 *
 * CV Out 1 = Envelope follower output, Left channel (0-5 V)
 * CV Out 2 = Envelope follower output, Right channel (0-5 V)
 * True stereo: IN_L → OUT_L, IN_R → OUT_R with independent simulation state.
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
 *  Circuit Component Constants (from TubeScreamer.m)
 * ==================================================================== */
static constexpr float R1   = 10.0e3f;       // resistance R1 [Ohms]
static constexpr float R2   = 51.0e3f;       // resistance R2 [Ohms]
static constexpr float R3   = 4.7e3f;        // resistance R3 [Ohms]
static constexpr float C1   = 1.0e-6f;       // capacitance C1 [F]
static constexpr float C2   = 51.0e-12f;     // capacitance C2 [F]
static constexpr float C3   = 47.0e-9f;      // capacitance C3 [F]

// Diode parameters
static constexpr float Is   = 2.52e-9f;      // saturation current [A]
static constexpr float Vt   = 25.83e-3f;     // thermal voltage [V]
static constexpr float Ni   = 1.752f;        // ideality factor
static constexpr float NiVt = Ni * Vt;       // precomputed product

// Newton-Raphson solver limits
static constexpr float NR_TOL     = 1e-7f;
static constexpr int   NR_MAXITER = 16;       // reduced from MATLAB's 50 for RT budget

/* ====================================================================
 *  Globals
 * ==================================================================== */
static planter euro;

/* Pre-computed matrices (recomputed when drive changes) -------
 *   Q  = inv(H_min) * H_plus          (3x3)
 *   Qb = inv(H_min) * B               (3x1)
 *   Qc = inv(H_min) * C               (3x1)
 *   K_coeff = D * inv(H_min) * C      (scalar, = Qc[1])
 * These follow the K-method derivation in TubeScreamer.m:
 *   H_min   = (2·SR)·I − A
 *   H_plus  = (2·SR)·I + A
 *   with system matrices A, B, C, D as defined in the MATLAB.
 */
static float Q[3][3];
static float Qb[3];
static float Qc[3];
static float K_coeff;

/* Per-channel simulation state (allows stereo reuse of shared matrices)
 * NOTE: f_prev is intentionally absent. The MATLAB reference never updates
 * its f1 variable (it stays 0 for the entire simulation).  Feeding back
 * f_prev causes numerical divergence because the diode function produces
 * values ~1e20 that get amplified by Qc (~1e5), blowing up the state.  */
struct ChannelState
{
    float x_prev[3];   // state vector x_{n-1}
    float u_prev;      // previous input sample
    float v_prev;      // previous diode voltage (NR initial guess)
    float tone_lp;     // one-pole LPF tone filter state
    float env;         // envelope follower state
    float dc_x1;       // DC blocking filter: previous input
    float dc_y1;       // DC blocking filter: previous output
};

static ChannelState ch_L, ch_R;

/* Envelope follower coefficients (shared) */
static float env_attack;   // attack coefficient  (~5 ms)
static float env_release;  // release coefficient (~100 ms)

/* Cached control values */
static float cur_drive;
static float sample_rate;

/* LED flash counter */
static int led_flash_ctr = 0;

/* ====================================================================
 *  3×3 Matrix Helpers (inline, no heap)
 * ==================================================================== */
static void mat3_mul(float out[3][3],
                     const float a[3][3],
                     const float b[3][3])
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            out[i][j] = a[i][0]*b[0][j]
                      + a[i][1]*b[1][j]
                      + a[i][2]*b[2][j];
}

static void mat3_vec(float out[3],
                     const float m[3][3],
                     const float v[3])
{
    for(int i = 0; i < 3; i++)
        out[i] = m[i][0]*v[0] + m[i][1]*v[1] + m[i][2]*v[2];
}

static bool mat3_inv(float out[3][3], const float m[3][3])
{
    float det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
              - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
              + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
    if(fabsf(det) < 1e-30f) return false;
    float id = 1.0f / det;
    out[0][0] =  (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * id;
    out[0][1] = -(m[0][1]*m[2][2] - m[0][2]*m[2][1]) * id;
    out[0][2] =  (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * id;
    out[1][0] = -(m[1][0]*m[2][2] - m[1][2]*m[2][0]) * id;
    out[1][1] =  (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * id;
    out[1][2] = -(m[0][0]*m[1][2] - m[0][2]*m[1][0]) * id;
    out[2][0] =  (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * id;
    out[2][1] = -(m[0][0]*m[2][1] - m[0][1]*m[2][0]) * id;
    out[2][2] =  (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * id;
    return true;
}

/* ====================================================================
 *  RecomputeMatrices – call once per block when drive changes
 *
 *  Derives Q, Qb, Qc, K_coeff from the circuit component values
 *  and the current Rdist = drive * 50 kΩ  (drive ∈ [0, 100]).
 * ==================================================================== */
static void RecomputeMatrices(float drive, float sr)
{
    float rdist = drive * 50.0e3f;
    if(rdist < 1.0f) rdist = 1.0f;          // numerical safety

    /* System matrix  A  (negated in MATLAB source) */
    float A[3][3] = {
        { -1.0f/(R1*C1),   0.0f,                     0.0f           },
        { -1.0f/(R3*C2),  -1.0f/((R2 + rdist)*C2),  -1.0f/(R3*C2)  },
        { -1.0f/(R3*C3),   0.0f,                     -1.0f/(R3*C3)  }
    };

    float two_sr = 2.0f * sr;

    /* H_minus = (2/k)·I − A ,  H_plus = (2/k)·I + A */
    float Hm[3][3], Hp[3][3];
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
        {
            float eye = (i == j) ? two_sr : 0.0f;
            Hm[i][j] = eye - A[i][j];
            Hp[i][j] = eye + A[i][j];
        }

    float inv_Hm[3][3] = {};
    mat3_inv(inv_Hm, Hm);

    /* B and C vectors (from MATLAB) */
    float B[3] = { 1.0f/(R1*C1),  1.0f/(R3*C2),  1.0f/(R3*C3) };
    float Cv[3] = { 0.0f,         -1.0f/C2,       0.0f          };

    mat3_mul(Q,  inv_Hm, Hp);       // Q  = inv(Hm) · Hp
    mat3_vec(Qb, inv_Hm, B);        // Qb = inv(Hm) · B
    mat3_vec(Qc, inv_Hm, Cv);       // Qc = inv(Hm) · C

    K_coeff = Qc[1];                 // K = D · Qc , D=[0,1,0]
}

/* ====================================================================
 *  ProcessSample – one sample of the tube screamer K-method
 *
 *  Optimised for Cortex-M7:
 *   • sinh/cosh pair computed from a single expf() call
 *   • asymmetric pair computed from a single expf() call
 *   • diode voltage clamped to avoid FP overflow
 * ==================================================================== */
static inline float ProcessSample(float input, bool symmetric, ChannelState& st)
{
    float u_curr = input;
    float u_sum  = st.u_prev + u_curr;

    /* P_n = D · inv(Hm) · [Hp·x_prev + B·(u_prev+u_curr)]
     * f_prev term omitted (always 0 — see ChannelState comment).
     * Since D = [0,1,0], P_n is just row 1 of the bracketed expression.  */
    float P_n = Q[1][0]*st.x_prev[0] + Q[1][1]*st.x_prev[1] + Q[1][2]*st.x_prev[2]
              + Qb[1]*u_sum;

    /* --- Newton-Raphson for  g(v) = P_n + K·f(v) − v = 0 ---
     * Analytic initial guess: approximate v ≈ 0 in g(v) = 0 gives
     *   f(v) ≈ −P_n / K.  For symmetric diodes:
     *   v_guess = NiVt · asinh(−P_n / (2·Is·K))
     * This puts NR within 1-2 thermal voltages of the solution,
     * ensuring convergence in 3-4 iterations even on a cold start.  */
    float nr_target = -P_n / (2.0f * Is * K_coeff);
    float v_curr;
    if(symmetric)
    {
        v_curr = NiVt * asinhf(nr_target);
    }
    else
    {
        /* Asymmetric approximation: use dominant exponential branch */
        if(nr_target > 0.0f)
            v_curr = 2.0f * NiVt * logf(nr_target / Is + 1.0f);
        else
            v_curr = -NiVt * logf(-nr_target / Is + 1.0f);
    }

    for(int iter = 0; iter < NR_MAXITER; iter++)
    {
        /* Clamp to avoid expf overflow (|v| > 3 V is well beyond diode range) */
        if(v_curr >  3.0f) v_curr =  3.0f;
        if(v_curr < -3.0f) v_curr = -3.0f;

        float f_val, df_val;

        if(symmetric)
        {
            /* f(v) = 2·Is·sinh(v / NiVt)
             * Compute sinh & cosh from one expf():
             *   ex = exp(v/NiVt), emx = 1/ex
             *   f   = Is·(ex − emx)           [= 2·Is·sinh]
             *   df  = (Is/NiVt)·(ex + emx)    [= 2·Is/NiVt·cosh]            */
            float arg = v_curr / NiVt;
            float ex  = expf(arg);
            float emx = 1.0f / ex;
            f_val  = Is * (ex - emx);
            df_val = (Is / NiVt) * (ex + emx);
        }
        else
        {
            /* f(v) = Is·[exp(v/(2·NiVt)) − exp(−v/NiVt)]
             * Compute from one expf():
             *   e1 = exp(v/(2·NiVt)),  e2 = 1/e1² = exp(−v/NiVt)           */
            float arg1 = v_curr / (2.0f * NiVt);
            float e1   = expf(arg1);
            float e2   = 1.0f / (e1 * e1);
            f_val  = Is * (e1 - e2);
            df_val = Is * (e1 / (2.0f * NiVt) + e2 / NiVt);
        }

        float g    = P_n + K_coeff * f_val - v_curr;
        float J    = K_coeff * df_val - 1.0f;
        float step = g / J;
        v_curr -= step;

        if(fabsf(step) < NR_TOL) break;
    }

    /* Final nonlinear function value at converged v_curr */
    float f_curr;
    if(symmetric)
    {
        float ex  = expf(v_curr / NiVt);
        float emx = 1.0f / ex;
        f_curr = Is * (ex - emx);
    }
    else
    {
        float e1 = expf(v_curr / (2.0f * NiVt));
        float e2 = 1.0f / (e1 * e1);
        f_curr = Is * (e1 - e2);
    }

    /* x_curr = inv(Hm)·[Hp·x_prev + B·u_sum + C·f_curr]
     * Only f_curr is used (f_prev = 0, matching MATLAB reference).  */
    float x_curr[3];
    for(int i = 0; i < 3; i++)
        x_curr[i] = Q[i][0]*st.x_prev[0] + Q[i][1]*st.x_prev[1] + Q[i][2]*st.x_prev[2]
                   + Qb[i]*u_sum + Qc[i]*f_curr;

    /* Output: y = L·x + M·u ,  L=[-1,1,0], M=1 */
    float y = -x_curr[0] + x_curr[1] + u_curr;

    /* ---- state update ---- */
    st.x_prev[0] = x_curr[0];
    st.x_prev[1] = x_curr[1];
    st.x_prev[2] = x_curr[2];
    st.u_prev  = u_curr;
    st.v_prev  = v_curr;

    /* NaN guard: if any state is NaN, reset channel to avoid permanent corruption */
    if(!isfinite(x_curr[0]) || !isfinite(x_curr[1]) || !isfinite(x_curr[2]) || !isfinite(y))
    {
        st.x_prev[0] = st.x_prev[1] = st.x_prev[2] = 0.0f;
        st.u_prev = 0.0f;
        st.v_prev = 0.0f;
        st.tone_lp = 0.0f;
        return 0.0f;
    }

    return y;
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

    /* ---- Read controls ------------------------------------------------ */

    // KNOB 1 – Drive (0-100)
    float drive_knob = euro.GetKnobValue(planter::KNOB_1) * 5.0f;
    // CV In 1: adds 0-5 from 0-5 V (GetCvIn returns 0-1 for 0-5V)
    float drive_cv   = fmaxf(0.0f, euro.GetCvIn(0)) * 5.0f;
    float drive      = fminf(drive_knob + drive_cv, 5.0f);
    if(drive < 0.01f) drive = 0.01f;

    // KNOB 2 – Tone (0-1)
    float tone_knob = euro.GetKnobValue(planter::KNOB_2);
    float tone_cv   = fmaxf(0.0f, euro.GetCvIn(1));
    float tone      = fminf(tone_knob + tone_cv, 1.0f);

    // KNOB 3 – Master Volume (0-1)
    float vol_knob = euro.GetKnobValue(planter::KNOB_3);
    float vol_cv   = fmaxf(0.0f, euro.GetCvIn(2));
    float volume   = fminf(vol_knob + vol_cv, 1.0f);

    // KNOB 4 – Clean Blend (0 = full wet, 1 = full dry)
    float blend_knob = euro.GetKnobValue(planter::KNOB_4);
    float blend_cv   = fmaxf(0.0f, euro.GetCvIn(3));
    float blend      = fminf(blend_knob + blend_cv, 1.0f);

    // Toggle 1: UP = symmetric, DOWN = asymmetric
    bool symmetric = euro.toggle1.Pressed();

    /* ---- Recompute matrices if drive changed noticeably --------------- */
    if(fabsf(drive - cur_drive) > 0.1f)
    {
        cur_drive = drive;
        RecomputeMatrices(drive, sample_rate);
    }

    /* ---- Tone filter coefficient (log mapping 100 Hz – 10 kHz) ------- */
    float tone_freq  = 100.0f * powf(100.0f, tone);          // 100·100^tone
    float tone_coeff = 1.0f - expf(-6.2831853f * tone_freq / sample_rate);

    /* DC blocking filter coefficient: HPF at ~10 Hz */
    static constexpr float dc_R = 0.9987f;   // 1 - 2*pi*10/48000

    /* ---- Per-sample processing (true stereo) ------------------------ */
    bool bypass = (blend >= 0.99f);  // skip simulation at full clean blend

    for(size_t i = 0; i < size; i++)
    {
        float dry_L = IN_L[i];
        float dry_R = IN_R[i];
        float out_L, out_R;

        if(bypass)
        {
            /* Pure passthrough — no simulation, no CPU spent on expf() */
            out_L = dry_L * volume;
            out_R = dry_R * volume;
        }
        else
        {
            /* --- Left channel: simulation + tone + blend --- */
            float wet_L = ProcessSample(dry_L, symmetric, ch_L);
            ch_L.tone_lp += tone_coeff * (wet_L - ch_L.tone_lp);
            float mix_L = ch_L.tone_lp * (1.0f - blend) + dry_L * blend;
            out_L = mix_L * volume;

            /* --- Right channel: simulation + tone + blend --- */
            float wet_R = ProcessSample(dry_R, symmetric, ch_R);
            ch_R.tone_lp += tone_coeff * (wet_R - ch_R.tone_lp);
            float mix_R = ch_R.tone_lp * (1.0f - blend) + dry_R * blend;
            out_R = mix_R * volume;
        }

        /* Hard clip */
        if(out_L >  1.0f) out_L =  1.0f;
        if(out_L < -1.0f) out_L = -1.0f;
        if(out_R >  1.0f) out_R =  1.0f;
        if(out_R < -1.0f) out_R = -1.0f;

        /* DC blocking HPF: y[n] = x[n] - x[n-1] + R·y[n-1], ~10 Hz cutoff */
        {
            float dc_y_L = out_L - ch_L.dc_x1 + dc_R * ch_L.dc_y1;
            ch_L.dc_x1 = out_L;
            ch_L.dc_y1 = dc_y_L;
            out_L = dc_y_L;

            float dc_y_R = out_R - ch_R.dc_x1 + dc_R * ch_R.dc_y1;
            ch_R.dc_x1 = out_R;
            ch_R.dc_y1 = dc_y_R;
            out_R = dc_y_R;
        }

        /* Envelope follower (peak detector) */
        float abs_L = fabsf(out_L);
        if(abs_L > ch_L.env)
            ch_L.env = env_attack * ch_L.env + (1.0f - env_attack) * abs_L;
        else
            ch_L.env *= env_release;

        float abs_R = fabsf(out_R);
        if(abs_R > ch_R.env)
            ch_R.env = env_attack * ch_R.env + (1.0f - env_attack) * abs_R;
        else
            ch_R.env *= env_release;

        OUT_L[i] = out_L;
        OUT_R[i] = out_R;
    }

    /* ---- CV Out 1: L envelope, CV Out 2: R envelope (0-5 V) ---------- */
    euro.WriteCvOutBipolar(1, ch_L.env * 5.0f);
    euro.WriteCvOutBipolar(2, ch_R.env * 5.0f);

    /* ---- LED: blink on signal presence ------------------------------- */
    if(ch_L.env > 0.01f || ch_R.env > 0.01f)
        led_flash_ctr = 3;           // keep lit for ~3 blocks after signal
    if(led_flash_ctr > 0)
    {
        euro.SetLed(1, true);
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
    sample_rate = euro.patch.AudioSampleRate();

    /* Zero simulation state (both channels) */
    memset(&ch_L, 0, sizeof(ch_L));
    memset(&ch_R, 0, sizeof(ch_R));

    /* Envelope follower time constants */
    env_attack  = expf(-1.0f / (sample_rate * 0.005f));   // ~5 ms attack
    env_release = expf(-1.0f / (sample_rate * 0.100f));   // ~100 ms release

    /* Initial matrix computation (mid drive) */
    cur_drive = 50.0f;
    RecomputeMatrices(50.0f, sample_rate);

    euro.patch.StartAudio(AudioCallback);
    while(1)
    {
        euro.BootloaderResetCheck();
    }
}
