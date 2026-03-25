/*
 * ===========================================================================
 * PhysPlateReverb.cpp
 *
 * Physical Modeling Plate Reverb for Daisy + Planter (Eurorack)
 * Ported from plateReverbSRC.m (MATLAB) — B221438
 *
 * This replaces the DaisySP ReverbSc with a true modal plate reverb
 * derived from the physical wave equation on a rectangular plate. The
 * algorithm computes the eigenfrequencies (modes) of a tensioned plate,
 * prunes redundant/inaudible modes, then runs a second-order modal
 * filter bank in real time.
 *
 * Two output pickup positions are used for stereo (one per channel).
 *
 * ===========================================================================
 *
 * -------------------------------------------------------------------------
 *   MATLAB  →  C++  VECTOR MATH TRANSLATION GUIDE
 * -------------------------------------------------------------------------
 *
 * KEY CONCEPT: In MATLAB, most math operates on entire vectors/matrices
 * at once using "vectorized" operations. In C++, we don't have that —
 * we use plain arrays (float[]) and explicit for-loops to do the same
 * thing, one element at a time.
 *
 * Here's a side-by-side conversion reference used throughout this file:
 *
 * ╔═══════════════════════════════════╦════════════════════════════════════════╗
 * ║  MATLAB                           ║  C++ Equivalent                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  a = zeros(N,1)                   ║  float a[N];                           ║
 * ║  % Creates an Nx1 column vector   ║  memset(a, 0, N * sizeof(float));      ║
 * ║  % of all zeros.                  ║  // Declares a fixed-size array of N   ║
 * ║                                   ║  // floats, then fills with 0 bytes.   ║
 * ║                                   ║  // No dynamic sizing! N must be a     ║
 * ║                                   ║  // compile-time constant.             ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  c = a .* b                       ║  for (int i = 0; i < N; i++)           ║
 * ║  % Element-wise multiply. Each    ║      c[i] = a[i] * b[i];               ║
 * ║  % entry in c is the product of   ║  // Identical behavior: multiply each  ║
 * ║  % matching entries in a and b.   ║  // pair of elements and store result. ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  c = a + b                        ║  for (int i = 0; i < N; i++)           ║
 * ║  % Element-wise addition.         ║      c[i] = a[i] + b[i];               ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  s = sum(a .* b)                  ║  float s = 0.0f;                       ║
 * ║  % Dot product: multiply element- ║  for (int i = 0; i < N; i++)           ║
 * ║  % wise then sum all results into ║      s += a[i] * b[i];                 ║
 * ║  % a single scalar.               ║  // Accumulate products into one sum.  ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  c = a * scalar                   ║  for (int i = 0; i < N; i++)           ║
 * ║  % Multiply every element of a    ║      c[i] = a[i] * scalar;             ║
 * ║  % by a single number.            ║  // Same: scale each element.          ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  [MX,MY] = meshgrid(1:Mx, 1:My)   ║  for (int mx = 1; mx <= Mx; mx++)      ║
 * ║  % Creates 2D grids of all (mx,   ║    for (int my = 1; my <= My; my++)    ║
 * ║  % my) index combinations.        ║      // process pair (mx, my)          ║
 * ║  mx = reshape(MX, [Mx*My, 1])     ║  // A nested for-loop generates every  ║
 * ║  my = reshape(MY, [Mx*My, 1])     ║  // combination of (mx, my) exactly    ║
 * ║  % Flatten to column vectors.     ║  // like meshgrid+reshape does.        ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  idx = (a > thresh)               ║  bool idx[N];                          ║
 * ║  % Logical index: 1 where true.   ║  for (int i = 0; i < N; i++)           ║
 * ║                                   ║      idx[i] = (a[i] > thresh);         ║
 * ║                                   ║  // Builds array of true/false flags.  ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  b = nonzeros(a .* idx)           ║  int count = 0;                        ║
 * ║  % Remove zero entries, keeping   ║  for (int i = 0; i < N; i++)           ║
 * ║  % only values where idx == 1.    ║      if (idx[i])                       ║
 * ║  % Resulting vector is shorter.   ║          b[count++] = a[i];            ║
 * ║                                   ║  // "Compacts" the array: copies only  ║
 * ║                                   ║  // surviving elements into contiguous ║
 * ║                                   ║  // slots. count = new length.         ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  b = a.^2                         ║  for (int i = 0; i < N; i++)           ║
 * ║  % Element-wise squaring.         ║      b[i] = a[i] * a[i];               ║
 * ║                                   ║                                        ║
 * ╠═══════════════════════════════════╬════════════════════════════════════════╣
 * ║                                   ║                                        ║
 * ║  sqrt(x)  (double precision)      ║  sqrtf(x)  (single-precision float)    ║
 * ║  sin(x)                           ║  sinf(x)                               ║
 * ║  log(x)                           ║  logf(x)                               ║
 * ║  % MATLAB defaults to double.     ║  // Daisy's ARM Cortex-M7 has a        ║
 * ║  % C++ float is fine for audio.   ║  // hardware float FPU — always use    ║
 * ║                                   ║  // the 'f' suffix functions.          ║
 * ║                                   ║                                        ║
 * ╚═══════════════════════════════════╩════════════════════════════════════════╝
 *
 * OTHER KEY DIFFERENCES:
 *
 * 1) MATLAB arrays are 1-based, C++ arrays are 0-based.
 *    MATLAB: a(1) is the first element.
 *    C++:    a[0] is the first element.
 *
 * 2) MATLAB dynamically resizes vectors. C++ arrays are fixed-size.
 *    We pre-allocate MAX_MODES slots and track the actual count in
 *    num_modes_.
 *
 * 3) MATLAB 'find()' returns indices matching a condition.
 *    C++: we use a for-loop with an if-statement instead.
 *
 * 4) In MATLAB, the entire sample loop runs offline on a pre-built
 *    impulse. Here, we process ONE sample at a time inside an audio
 *    callback that the hardware calls 48000 times/second.
 *
 * ===========================================================================
 */

#include "daisy_patch_sm.h"
#include "daisysp-lgpl.h"       // provides fmap(), Mapping::LOG
#include "planter.h"
#include <cmath>                // sqrtf, sinf, logf, fabsf, powf
#include <cstring>              // memset

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace anachrome;

/* -----------------------------------------------------------------------
 * PI_F is already #defined in DaisySP/Source/Utility/dsp.h as
 * 3.1415927410125732421875f — we just use it directly, no redefinition.
 * -----------------------------------------------------------------------*/


/* ========================================================================
 * SECTION 1 — Physical Constants
 *
 * MATLAB equivalent: the block of scalar variable assignments at the
 * top of the script.  In C++ these are compile-time constants
 * (constexpr), which means they cost zero RAM and are baked straight
 * into the compiled machine code.
 * ========================================================================*/

/* --- Plate geometry & material ---
 * MATLAB:
 *   Lx = 1;  Ly = 2;  H = 50e-4;  T = 100;  rho = 7.87e3;
 *   E = 200e9;  v = 0.29;
 */
static constexpr float LX      = 1.0f;       // plate x dimension [m]
static constexpr float LY      = 2.0f;       // plate y dimension [m]
static constexpr float H       = 8e-4f;     // thickness [m]  (= 0.5 mm)
static constexpr float TENSION = 700.0f;     // tension [N/m]
static constexpr float RHO     = 7.87e3f;    // density [kg/m³]  (steel)
static constexpr float E_MOD   = 200e9f;     // Young's modulus [N/m²]
static constexpr float NU      = 0.29f;      // Poisson's ratio

/* --- Input / output positions on the plate surface ---
 * MATLAB:  xi = 0.5; yi = 0.5; xo = 0.75; yo = 0.75;
 *
 * For STEREO output we define TWO pickup positions — this is how real
 * plate reverbs work: one plate, one driver (input), two contact mics
 * (outputs) placed at different spots to produce natural stereo.
 */
static constexpr float XI   = 0.5f;    // input (driver) x position [m]
static constexpr float YI   = 0.5f;    // input y position [m]
static constexpr float XO_L = 0.3f;    // LEFT  pickup x position [m]
static constexpr float YO_L = 0.7f;    // LEFT  pickup y position [m]
static constexpr float XO_R = 0.8f;    // RIGHT pickup x position [m]
static constexpr float YO_R = 0.4f;    // RIGHT pickup y position [m]

/* --- Mode pruning thresholds ---
 * MATLAB:  cent_range = 5; damp_thresh = .5; phi_thresh = 1e-5;
 */
static constexpr float CENT_RANGE  = 8.0f;    // frequency window [cents] (larger = stronger pruning)
static constexpr float DAMP_THRESH = 0.5f;    // damping proximity fraction
static constexpr float PHI_THRESH  = 1e-5f;   // mode shape amplitude floor

/* --- Runtime stability / output scaling --- */
static constexpr float WET_TRIM     = 0.22f;   // global wet attenuation after modal sum
static constexpr float OUTPUT_LIMIT = 0.98f;   // safety clamp for Daisy audio output
static constexpr float OUT_NORM_BLEND = 0.20f; // 0=ignore normalization, 1=full normalization
static constexpr float WET_MAKEUP   = 1.0f;   // post-normalization wet makeup gain
static constexpr float MIX_HEADROOM = 0.85f;   // dry+wet pre-limiter headroom
static constexpr float INPUT_TRIM   = 0.55f;   // front-end trim for both dry and wet paths
static constexpr float DRY_MAX      = 0.75f;   // max dry contribution from knob
static constexpr float WET_SEND_MAX = 0.06f;   // conservative max excitation send
static constexpr float WET_MIX_MAX  = 0.85f;   // max wet return mix into output
static constexpr float WET_SEND_PRETRIM = 0.30f; // additional pre-reverb send trim
static constexpr float SEND_HPF_MIN_HZ = 10.0f;  // KNOB_5 minimum send HPF cutoff
static constexpr float SEND_HPF_MAX_HZ = 100.0f; // KNOB_5 maximum send HPF cutoff
static constexpr float WET_HPF_HZ   = 120.0f;  // remove sub buildup from wet return

/* --- Memory budgets ---
 * In MATLAB, vectors grow dynamically.  In C++ on embedded hardware we
 * must pre-allocate fixed-size arrays.  These constants set the limits.
 */
static constexpr int MAX_RAW_MODES = 12000;   // prior to pruning
static constexpr int MAX_MODES     = 400;    // final modes (CPU budget)

static inline float ClampAudio(float x)
{
    if(x > OUTPUT_LIMIT) return OUTPUT_LIMIT;
    if(x < -OUTPUT_LIMIT) return -OUTPUT_LIMIT;
    return x;
}


/* ========================================================================
 * SECTION 2 — Temporary Work Buffers (only used during Init)
 *
 * These live in external SDRAM (64 MB) via the DSY_SDRAM_BSS attribute.
 * In MATLAB all these would just be regular workspace variables; in C++
 * on the Daisy we manually choose where big arrays go.
 *
 * DSY_SDRAM_BSS tells the linker: "put this in the .sdram_bss section"
 * which maps to the external 64 MB SDRAM chip, not the smaller internal
 * SRAM (≈512 KB).  The arrays persist after Init() but that's fine —
 * SDRAM has plenty of room.
 * ========================================================================*/

/* MATLAB equivalents:
 *   omega          → raw_omega[]     (modal angular frequencies)
 *   beta_stable.^2 → raw_beta_sq[]   (squared wavenumbers)
 *   sigma          → raw_sigma[]     (per-mode damping)
 *   phi_i          → raw_phi_i[]     (input mode shapes)
 *   phi_o          → raw_phi_o_l/r[] (output mode shapes, L & R pickups)
 */
static float DSY_SDRAM_BSS raw_omega[MAX_RAW_MODES];
static float DSY_SDRAM_BSS raw_beta_sq[MAX_RAW_MODES];
static float DSY_SDRAM_BSS raw_sigma[MAX_RAW_MODES];
static float DSY_SDRAM_BSS raw_phi_i[MAX_RAW_MODES];
static float DSY_SDRAM_BSS raw_phi_o_l[MAX_RAW_MODES];
static float DSY_SDRAM_BSS raw_phi_o_r[MAX_RAW_MODES];

/* Pruning flag array — 1 = keep, 0 = pruned.
 * MATLAB uses omega_new ≠ 0 as the logical flag; here we use int. */
static int DSY_SDRAM_BSS raw_active[MAX_RAW_MODES];


/* ========================================================================
 * SECTION 3 — ModalPlateReverb Class
 *
 * This class encapsulates the entire physical model.  It owns:
 *   • the pruned modal parameter arrays (ω, β², φ_i, φ_o)
 *   • the state-update coefficient arrays (B, C, D)
 *   • the state vectors (p1, p2) that persist between samples
 *
 * In MATLAB these are all separate workspace variables.  Wrapping them
 * in a C++ class keeps everything organized and avoids name collisions.
 * ========================================================================*/
class ModalPlateReverb
{
public:
    /* -------------------------------------------------------------------
     * Init()
     *
     * One-time setup: compute modes, prune, build coefficients.
     * MATLAB equivalent: everything from "derived parameters" down to
     * the coefficient computation (B, C, D), NOT including the sample
     * loop.
     * -------------------------------------------------------------------*/
    void Init(float sample_rate);

    /* -------------------------------------------------------------------
     * UpdateDecay()
     *
     * Recalculate damping coefficients from new T60 values.
     * Called once per audio block when the decay knobs change.
     *
     * MATLAB equivalent: recomputing sig_0, sig_1, sigma, B, C, D with
     * new T60_low / T60_high values.
     * -------------------------------------------------------------------*/
    void UpdateDecay(float t60_low, float t60_high);

    /* -------------------------------------------------------------------
     * ProcessStereo()
     *
     * Process ONE audio sample through the modal plate.
     * Returns stereo output via pointers (two pickup positions).
     *
     * MATLAB equivalent: ONE iteration of the "for n = 1:Nf" loop:
     *   p0      = B.*p1 + C.*p2 + D*in1(n);
     *   out1(n) = sum(phi_o_new .* p0);
     *   p2 = p1;  p1 = p0;
     * -------------------------------------------------------------------*/
    void ProcessStereo(float input, float* out_l, float* out_r);

    void ProcessMono(float input, float* out_mono);

    /* How many modes survived pruning? (informational) */
    int GetModeCount() const { return num_modes_; }

private:
    int   num_modes_;   // actual mode count after pruning (≤ MAX_MODES)
    float k_;           // sample period = 1/SR                      [seconds]
    float k_sq_;        // k² (pre-computed, used in B coefficient)

    /* --- Per-mode data (FIXED after Init — geometry-dependent) ---
     *
     *  MATLAB variable  →  C++ array        meaning
     *  ───────────────     ──────────        ───────────────────────────
     *  omega_new         → omega_[]         modal angular frequencies [rad/s]
     *  beta_stable.^2    → beta_sq_[]       wavenumber² (for recomputing σ)
     *  phi_i_new         → phi_i_[]         mode shape at input position
     *  phi_o_new         → phi_o_l_[]       mode shape at LEFT  pickup
     *                      phi_o_r_[]       mode shape at RIGHT pickup
     */
    float omega_[MAX_MODES];
    float beta_sq_[MAX_MODES];
    float phi_i_[MAX_MODES];
    float phi_o_l_[MAX_MODES];
    float phi_o_r_[MAX_MODES];
    float phi_o_m_[MAX_MODES];

    /* --- Per-mode coefficients (RECOMPUTED when T60 knobs change) ---
     *
     *  MATLAB:                              C++ array:
     *  B = (2 - k²ω²) / (1+σk)          → B_[]
     *  C = (σk - 1)    / (1+σk)          → C_[]
     *  D = φ_i         / (1+σk)          → D_[]
     *
     *  These are the "weights" in the per-sample state-update equation.
     *  Recomputing them per audio block (~1 ms) lets us change the
     *  reverb time in real time without a full re-init.
     */
    float B_[MAX_MODES];
    float C_[MAX_MODES];
    float D_[MAX_MODES];

    /* --- Per-mode STATE vectors (persist between audio callbacks) ---
     *
     *  MATLAB: p1 = zeros(sp_len,1);  p2 = zeros(sp_len,1);
     *
     *  p1 = current  state (P^n)       — the "position" of each mode
     *  p2 = previous state (P^{n-1})   — needed for the 2nd-order update
     *
     *  In MATLAB these are Nx1 column vectors.  In C++ they are plain
     *  float arrays of length MAX_MODES.  Only the first num_modes_
     *  entries are used.
     */
    float p1_[MAX_MODES];
    float p2_[MAX_MODES];

    /* Cached beta² range for real-time sigma recomputation */
    float beta_sq_min_;
    float beta_sq_max_;

    /* Per-channel pickup normalization to reduce excessive modal sum gain */
    float out_norm_l_;
    float out_norm_r_;
    float out_norm_m_;
};


/* ========================================================================
 * SECTION 4 — Init() Implementation
 *
 * This function does ALL the pre-processing that the MATLAB script does
 * before its sample loop, in the same order:
 *   1. Derive physical constants (κ, c, ω_max, β_max)
 *   2. Determine max mode indices Mx, My
 *   3. Loop over all (mx, my) pairs → stability check → store raw modes
 *   4. Compute initial σ (damping) for the pruning step
 *   5. Prune modes (frequency/damping proximity + mode shape threshold)
 *   6. Compact surviving modes into final arrays ("nonzeros" in MATLAB)
 *   7. Zero the state vectors
 *   8. Compute initial B, C, D coefficients
 * ========================================================================*/
void ModalPlateReverb::Init(float sample_rate)
{
    num_modes_ = 0;
    out_norm_l_ = 1.0f;
    out_norm_r_ = 1.0f;
    out_norm_m_ = 1.0f;

    /* ===== Step 1: Derived constants =====================================
     *
     * MATLAB:
     *   k = 1/SR;
     *   K = sqrt( E*(H^2) / ( 12*rho*(1-v^2) ) );
     *   c = sqrt( T/(rho*H) );
     *   omega_max = 2/k;
     *   beta_max  = sqrt((-c^2 + sqrt(c^4 + 4*omega_max^2*K^2)) / (2*K^2));
     *
     * In C++ these are local variables computed once at startup.
     * Note: we use sqrtf() instead of sqrt() for single-precision.
     * ====================================================================*/

    // k = 1/SR  →  sample period in seconds
    k_    = 1.0f / sample_rate;
    k_sq_ = k_ * k_;

    // kappa (κ) — stiffness parameter
    // MATLAB: K = sqrt( E*(H^2) / (12*rho*(1-v^2)) )
    const float kappa = sqrtf(E_MOD * (H * H)
                              / (12.0f * RHO * (1.0f - NU * NU)));

    // c — wave speed on the plate
    // MATLAB: c = sqrt( T/(rho*H) )
    const float c = sqrtf(TENSION / (RHO * H));

    // omega_max — maximum modal frequency that can be represented
    // at this sample rate (Nyquist-related stability limit)
    // MATLAB: omega_max = 2/k
    const float omega_max = 2.0f / k_;

    // beta_max — maximum wavenumber for stable modes
    // MATLAB: beta_max = sqrt((-c^2 + sqrt(c^4 + 4*omega_max^2*K^2)) / (2*K^2))
    const float c2 = c * c;     // c squared
    const float K2 = kappa * kappa;  // κ squared
    const float beta_max = sqrtf(
        (-c2 + sqrtf(c2 * c2 + 4.0f * omega_max * omega_max * K2))
        / (2.0f * K2)
    );


    /* ===== Step 2: Maximum mode indices ==================================
     *
     * MATLAB:
     *   Mx = floor(sqrt( ((Lx*Ly*beta_max)^2 - (pi*Lx)^2) / (pi*Ly)^2 ));
     *   My = floor(sqrt( ((Lx*Ly*beta_max)^2 - (pi*Ly)^2) / (pi*Lx)^2 ));
     *
     * These determine how many modes to check in x and y.  Casting to
     * int is the C++ equivalent of floor() for positive values.
     * ====================================================================*/
    const float LxLy_beta_sq = (LX * LY * beta_max) * (LX * LY * beta_max);
    const float piLx_sq      = (PI_F * LX) * (PI_F * LX);
    const float piLy_sq      = (PI_F * LY) * (PI_F * LY);

    // (int) truncates toward zero = floor for positive numbers
    int Mx = (int)sqrtf((LxLy_beta_sq - piLx_sq) / piLy_sq);
    int My = (int)sqrtf((LxLy_beta_sq - piLy_sq) / piLx_sq);


    /* ===== Step 3: Generate all modes ====================================
     *
     * MATLAB:
     *   [MX,MY] = meshgrid(1:Mx, 1:My);
     *   mx = reshape(MX, [Mx*My, 1]);
     *   my = reshape(MY, [Mx*My, 1]);
     *   beta = sqrt( (mx*pi/Lx).^2 + (my*pi/Ly).^2 );
     *   stab_index = beta < beta_max;
     *   beta_stable = nonzeros(beta .* stab_index);
     *   omega = sqrt( c^2*beta_stable.^2 + K^2*beta_stable.^4 );
     *   phi_i = (2/sqrt(Lx*Ly)) * sin(mx*xi*pi/Lx) .* sin(my*yi*pi/Ly);
     *   phi_o = (2/sqrt(Lx*Ly)) * sin(mx*xo*pi/Lx) .* sin(my*yo*pi/Ly);
     *
     * C++ TRANSLATION:
     * ────────────────
     * meshgrid + reshape is replaced by a NESTED FOR LOOP.  Each (mx, my)
     * pair produces one mode.  We immediately check stability (beta < beta_max)
     * and skip unstable modes — this replaces the stab_index / nonzeros
     * pattern in MATLAB.
     *
     * The mode shape normalization factor 2/sqrt(Lx*Ly) is a scalar that
     * MATLAB broadcasts across the whole vector; in C++ we just multiply
     * it in for each element.
     *
     * We store surviving modes into the raw_* temporary arrays.
     * ====================================================================*/

    // MATLAB: 2/sqrt(Lx*Ly)  — mode shape normalization
    const float phi_norm = 2.0f / sqrtf(LX * LY);

    int raw_count = 0;  // number of modes that pass the stability check

    // ----- NESTED LOOP = meshgrid(1:Mx, 1:My) + reshape -----
    // mx is the outer loop (changes slowly)
    // my is the inner loop (changes quickly)
    // → same ordering as MATLAB's column-major reshape
    for (int mx = 1; mx <= Mx; mx++)
    {
        for (int my = 1; my <= My; my++)
        {
            // ----- Wavenumber (beta) -----
            // MATLAB: beta = sqrt( (mx*pi/Lx)^2 + (my*pi/Ly)^2 )
            //
            // In MATLAB this is one vectorized line operating on all modes.
            // In C++ we compute it for one (mx, my) pair per iteration.
            float bx   = (float)mx * PI_F / LX;   // x-direction component
            float by   = (float)my * PI_F / LY;   // y-direction component
            float beta = sqrtf(bx * bx + by * by); // total wavenumber

            // ----- Stability check -----
            // MATLAB: stab_index = beta < beta_max;
            // Only keep modes whose wavenumber is below the Nyquist limit.
            // In MATLAB this creates a logical vector; here we just skip.
            if (beta >= beta_max)
                continue;   // unstable → skip this (mx,my) pair

            // Guard: don't overflow our pre-allocated buffer
            if (raw_count >= MAX_RAW_MODES)
                break;

            float beta_sq = beta * beta;   // β² — reused several times

            // ----- Modal angular frequency -----
            // MATLAB: omega = sqrt( c^2 * beta.^2 + K^2 * beta.^4 )
            //
            // beta.^2 is element-wise squaring of the beta vector.
            // In C++ we already have beta_sq for this one mode.
            float omega = sqrtf(c2 * beta_sq + K2 * beta_sq * beta_sq);

            // ----- Mode shapes at input position -----
            // MATLAB: phi_i = (2/sqrt(Lx*Ly)) * sin(mx*xi*pi/Lx) .* sin(my*yi*pi/Ly)
            //
            // .* is element-wise multiply of two sin() vectors.
            // In C++ we compute the scalar product of two single sin values.
            float phi_i = phi_norm
                          * sinf((float)mx * XI * PI_F / LX)
                          * sinf((float)my * YI * PI_F / LY);

            // ----- Mode shapes at LEFT output position -----
            float phi_o_l = phi_norm
                            * sinf((float)mx * XO_L * PI_F / LX)
                            * sinf((float)my * YO_L * PI_F / LY);

            // ----- Mode shapes at RIGHT output position -----
            float phi_o_r = phi_norm
                            * sinf((float)mx * XO_R * PI_F / LX)
                            * sinf((float)my * YO_R * PI_F / LY);

            // Store this mode into the temporary arrays
            // (like building up the MATLAB vectors one element at a time)
            raw_omega[raw_count]   = omega;
            raw_beta_sq[raw_count] = beta_sq;
            raw_phi_i[raw_count]   = phi_i;
            raw_phi_o_l[raw_count] = phi_o_l;
            raw_phi_o_r[raw_count] = phi_o_r;
            raw_count++;
        }
        // Also guard the outer loop
        if (raw_count >= MAX_RAW_MODES)
            break;
    }


    /* ===== Step 4: Compute σ (damping) for the pruning step ==============
     *
     * MATLAB:
     *   sig_0 = ((6*log(10)) / (max(β²) - min(β²)))
     *           * ((max(β²)/T60_low) - (min(β²)/T60_high));
     *   sig_1 = ((6*log(10)) / (max(β²) - min(β²)))
     *           * (1/T60_high - 1/T60_low);
     *   sigma = sig_0 + sig_1 * beta_stable.^2;
     *
     * MATLAB: max() and min() scan the entire vector in one call.
     * C++ TRANSLATION: a for-loop that tracks the running min/max.
     *
     * We need sigma for the pruning comparisons.  After pruning, we'll
     * recompute sigma from the knob-controlled T60 values.
     * ====================================================================*/

    if(raw_count <= 0)
    {
        memset(p1_, 0, sizeof(p1_));
        memset(p2_, 0, sizeof(p2_));
        return;
    }

    // Find min and max of β²
    // MATLAB: max(beta_stable)^2 and min(beta_stable)^2
    float bsq_min = raw_beta_sq[0];
    float bsq_max = raw_beta_sq[0];
    for (int i = 1; i < raw_count; i++)
    {
        if (raw_beta_sq[i] < bsq_min) bsq_min = raw_beta_sq[i];
        if (raw_beta_sq[i] > bsq_max) bsq_max = raw_beta_sq[i];
    }

    // T60 values for the pruning pass (default — overridden later by knobs)
    const float t60_low_init  = 2.0f;    // T60 at lowest mode  [s]
    const float t60_high_init = 0.5f;    // T60 at highest mode [s]

    // 6 * ln(10) ≈ 13.8155 — appears in both sig_0 and sig_1
    const float six_ln10  = 6.0f * logf(10.0f);

    // Protect against degenerate case where all modes have same beta
    float bsq_range = bsq_max - bsq_min;
    if (bsq_range < 1e-10f) bsq_range = 1e-10f;

    // MATLAB: sig_0 = ((6*log(10))/(max(β²)-min(β²))) * ((max(β²)/T60_low) - (min(β²)/T60_high))
    float sig_0 = (six_ln10 / bsq_range)
                  * ((bsq_max / t60_low_init) - (bsq_min / t60_high_init));

    // MATLAB: sig_1 = ((6*log(10))/(max(β²)-min(β²))) * (1/T60_high - 1/T60_low)
    float sig_1 = (six_ln10 / bsq_range)
                  * (1.0f / t60_high_init - 1.0f / t60_low_init);

    // MATLAB: sigma = sig_0 + sig_1 * beta_stable.^2;
    //
    // This is a VECTOR operation in MATLAB — sig_0 is a scalar added to
    // every element, and sig_1 * beta.^2 is scalar-times-vector.
    // In C++ we loop and compute each element:
    for (int i = 0; i < raw_count; i++)
    {
        raw_sigma[i] = sig_0 + sig_1 * raw_beta_sq[i];
    }


    /* ===== Step 5: Mode Pruning ==========================================
     *
     * MATLAB:
     *   cent_thresh = cent_range * (2^(1/1200)-1);
     *   omega_new = omega;
     *   for m = 1:length(omega_new)
     *       if omega_new(m) == 0       ← skip already-pruned
     *       else
     *           if phi below threshold → omega_new(m) = 0
     *           else
     *               [row] = find(... modes within freq & damp window ...);
     *               omega_new(row) = 0;   ← prune neighbors
     *           end
     *       end
     *   end
     *   prunevec = omega_new ~= 0;
     *
     * C++ TRANSLATION:
     * ────────────────
     * MATLAB's find() returns a vector of indices matching a compound
     * boolean condition.  In C++ we replace it with a for-loop that
     * checks each condition with an if-statement.
     *
     * Instead of setting omega_new to 0, we maintain a separate integer
     * array raw_active[] where 1 = keep, 0 = pruned.  This is cleaner
     * than the MATLAB trick of zeroing out values then calling nonzeros().
     *
     * The pruning is "greedy": modes processed first get priority.
     * If mode m is still active, it prunes any NEIGHBOR modes that are
     * within both the frequency window AND the damping window.
     * ====================================================================*/

    // MATLAB: cent_thresh = cent_range * (2^(1/1200)-1)
    // This converts cents to a fractional frequency ratio.
    const float cent_thresh = CENT_RANGE * (powf(2.0f, 1.0f / 1200.0f) - 1.0f);

    // Initialize all modes as active (1 = keep)
    // MATLAB: omega_new = omega;  (copy — all nonzero = "active")
    for (int i = 0; i < raw_count; i++)
        raw_active[i] = 1;

    // ----- Main pruning loop -----
    // MATLAB: for m = 1:length(omega_new) ...
    //
    // NOTE: MATLAB is 1-indexed (starts at 1), C++ is 0-indexed (starts at 0).
    // The logic is identical; only the index origin differs.
    for (int m = 0; m < raw_count; m++)
    {
        // MATLAB: if omega_new(m) == 0 → skip
        // C++: check our active flag instead
        if (!raw_active[m])
            continue;   // already pruned by an earlier iteration — skip

        // ----- Check mode shape amplitudes -----
        // MATLAB: if abs(phi_o(m)) < phi_thresh || abs(phi_i(m)) < phi_thresh
        //             omega_new(m) = 0;
        //
        // If the mode has near-zero amplitude at either the input position
        // OR at BOTH output positions, it's inaudible → prune it.
        //
        // (For stereo: we keep the mode if it's audible on at least one
        //  output channel, which is why we use && for the two outputs.)
        if (fabsf(raw_phi_i[m]) < PHI_THRESH ||
            (fabsf(raw_phi_o_l[m]) < PHI_THRESH &&
             fabsf(raw_phi_o_r[m]) < PHI_THRESH))
        {
            raw_active[m] = 0;   // ← MATLAB: omega_new(m) = 0
            continue;
        }

        // ----- Find and prune close neighbors -----
        // MATLAB:
        //   [row] = find( omega > omega(m) - (cent_thresh/2)*omega(m)
        //               & omega < omega(m) + (cent_thresh/2)*omega(m)
        //               & omega ~= omega(m)
        //               & sigma > sigma(m) - (damp_thresh/2)*sigma(m)
        //               & sigma < sigma(m) + (damp_thresh/2)*sigma(m)
        //               & sigma ~= sigma(m) );
        //   omega_new(row) = 0;
        //
        // C++ TRANSLATION:
        // find() with multiple conditions → an inner for-loop with an
        // if-statement that checks ALL conditions simultaneously (&&).
        //
        // Compute the window bounds ONCE outside the inner loop:
        float half_freq_win = (cent_thresh * 0.5f) * raw_omega[m];
        float omega_lo = raw_omega[m] - half_freq_win;   // lower freq bound
        float omega_hi = raw_omega[m] + half_freq_win;   // upper freq bound

        float half_damp_win = (DAMP_THRESH * 0.5f) * raw_sigma[m];
        float sigma_lo = raw_sigma[m] - half_damp_win;   // lower damp bound
        float sigma_hi = raw_sigma[m] + half_damp_win;   // upper damp bound

        for (int j = 0; j < raw_count; j++)
        {
            if (j == m)         continue;   // skip self (MATLAB: omega~=omega(m))
            if (!raw_active[j]) continue;   // already pruned — no work to do

            // Check if mode j falls within BOTH the frequency AND damping
            // windows of mode m.  All conditions must be true (&&).
            if (raw_omega[j] > omega_lo && raw_omega[j] < omega_hi &&
                raw_sigma[j] > sigma_lo && raw_sigma[j] < sigma_hi)
            {
                // Mode j is too close to mode m → mark it as pruned
                raw_active[j] = 0;   // ← MATLAB: omega_new(row) = 0
            }
        }
    }


    /* ===== Step 6: Compact surviving modes ===============================
     *
     * MATLAB:
     *   prunevec    = omega_new ~= 0;
     *   omega_new   = nonzeros(omega_new);
     *   sigma_new   = nonzeros(sigma .* prunevec);
     *   phi_i_new   = nonzeros(phi_i .* prunevec);
     *   phi_o_new   = nonzeros(phi_o .* prunevec);
     *
     * C++ TRANSLATION:
     * ────────────────
     * MATLAB's nonzeros() returns a shorter vector with all the zero
     * entries removed.  In C++ we loop through the raw arrays and copy
     * only the "active" entries into the class's final arrays, packing
     * them contiguously.  A counter (num_modes_) tracks the new length.
     *
     * This is the canonical C++ pattern for "logical indexing + compact":
     *   int count = 0;
     *   for (i) if (active[i]) final[count++] = raw[i];
     *
     * We also cap at MAX_MODES to stay within our CPU budget.
     * ====================================================================*/
    num_modes_ = 0;
    for (int i = 0; i < raw_count; i++)
    {
        if (!raw_active[i])
            continue;   // pruned — skip

        if (num_modes_ >= MAX_MODES)
            break;      // hit our CPU budget — stop adding modes

        // Copy this mode's data from temporary → final arrays
        omega_[num_modes_]   = raw_omega[i];
        beta_sq_[num_modes_] = raw_beta_sq[i];
        phi_i_[num_modes_]   = raw_phi_i[i];
        phi_o_l_[num_modes_] = raw_phi_o_l[i];
        phi_o_r_[num_modes_] = raw_phi_o_r[i];
        phi_o_m_[num_modes_] = 0.5f * (raw_phi_o_l[i] + raw_phi_o_r[i]);
        num_modes_++;
    }

    if(num_modes_ <= 0)
    {
        memset(p1_, 0, sizeof(p1_));
        memset(p2_, 0, sizeof(p2_));
        return;
    }

    // Store β² extremes — needed later by UpdateDecay() to recompute σ
    beta_sq_min_ = bsq_min;
    beta_sq_max_ = bsq_max;

    // Compute pickup normalization so modal dot-product does not easily clip.
    float sum_sq_l = 0.0f;
    float sum_sq_r = 0.0f;
    float sum_sq_m = 0.0f;
    for(int i = 0; i < num_modes_; i++)
    {
        sum_sq_l += phi_o_l_[i] * phi_o_l_[i];
        sum_sq_r += phi_o_r_[i] * phi_o_r_[i];
        sum_sq_m += phi_o_m_[i] * phi_o_m_[i];
    }
    if(sum_sq_l > 1e-12f) out_norm_l_ = 1.0f / sqrtf(sum_sq_l);
    if(sum_sq_r > 1e-12f) out_norm_r_ = 1.0f / sqrtf(sum_sq_r);
    if(sum_sq_m > 1e-12f) out_norm_m_ = 1.0f / sqrtf(sum_sq_m);


    /* ===== Step 7: Zero the state vectors ================================
     *
     * MATLAB: p1 = zeros(sp_len,1);   p2 = zeros(sp_len,1);
     *
     * memset fills a block of memory with zero bytes.  Since IEEE 754
     * float 0.0 is all-zero-bits, this correctly initializes floats to 0.
     * ====================================================================*/
    memset(p1_, 0, sizeof(p1_));   // zero entire p1 array
    memset(p2_, 0, sizeof(p2_));   // zero entire p2 array


    /* ===== Step 8: Compute initial B, C, D coefficients ==================
     * Delegates to UpdateDecay() which applies the MATLAB coefficient
     * formulas using the initial T60 values.
     * ====================================================================*/
    UpdateDecay(t60_low_init, t60_high_init);
}


/* ========================================================================
 * SECTION 5 — UpdateDecay()
 *
 * Recomputes the per-mode state-update coefficients (B, C, D) from new
 * T60 values.  Called once per audio block (~every 1 ms) when the user
 * turns the decay knobs.
 *
 * MATLAB equivalent:
 *   sig_0     = ((6*log(10))/(max(β²)-min(β²))) * ((max(β²)/T60_low) - (min(β²)/T60_high));
 *   sig_1     = ((6*log(10))/(max(β²)-min(β²))) * (1/T60_high - 1/T60_low);
 *   sigma_new = sig_0 + sig_1 * beta_stable.^2;
 *   B = (2 - k²ω²)   ./ (1 + σk);
 *   C = (σk - 1)      ./ (1 + σk);
 *   D = φ_i           ./ (1 + σk);
 *
 * The ./ (element-wise divide) in MATLAB becomes computing a reciprocal
 * once per element and multiplying.  This is a standard optimization on
 * ARM: one division is slower than one reciprocal + several multiplies.
 * ========================================================================*/
void ModalPlateReverb::UpdateDecay(float t60_low, float t60_high)
{
    if(num_modes_ <= 0)
        return;

    // Clamp to safe range to avoid division by zero or negative damping
    if (t60_low  < 0.1f)  t60_low  = 0.1f;
    if (t60_high < 0.05f) t60_high = 0.05f;

    const float six_ln10 = 6.0f * logf(10.0f);   // ≈ 13.8155

    float bsq_range = beta_sq_max_ - beta_sq_min_;
    if (bsq_range < 1e-10f) bsq_range = 1e-10f;

    // MATLAB: sig_0 = ((6*log(10))/(max(β²)-min(β²))) * ((max(β²)/T60_low) - (min(β²)/T60_high))
    float sig_0 = (six_ln10 / bsq_range)
                  * ((beta_sq_max_ / t60_low) - (beta_sq_min_ / t60_high));

    // MATLAB: sig_1 = ((6*log(10))/(max(β²)-min(β²))) * (1/T60_high - 1/T60_low)
    float sig_1 = (six_ln10 / bsq_range)
                  * (1.0f / t60_high - 1.0f / t60_low);

    /* ----- Per-mode coefficient loop -----
     *
     * MATLAB does this with VECTOR operations:
     *   sigma_new = sig_0 + sig_1 * beta_stable.^2;   ← scalar + scalar*vector
     *   B = (2 - k^2*omega_new.^2) ./ (1+sigma_new*k);
     *   C = (sigma_new*k - 1)      ./ (1+sigma_new*k);
     *   D = phi_i_new              ./ (1+sigma_new*k);
     *
     * In C++ we iterate through each mode and compute the scalar
     * equivalent.  The "./" (element-wise divide) is implemented as
     * computing the reciprocal once (inv_denom) then multiplying.
     */
    for (int m = 0; m < num_modes_; m++)
    {
        // MATLAB: sigma_new(m) = sig_0 + sig_1 * beta_stable(m)^2
        // sig_0 + sig_1 * β²  gives the damping for this specific mode.
        // Low-β modes (low frequency) get less damping, high-β more.
        float sigma_m = sig_0 + sig_1 * beta_sq_[m];

        // Safety: ensure positive damping (physical requirement)
        if (sigma_m < 0.001f) sigma_m = 0.001f;

        // Denominator: (1 + σ·k)  — shared by B, C, D formulas
        float denom   = 1.0f + sigma_m * k_;

        // Compute reciprocal ONCE → multiply instead of divide 3 times
        // MATLAB's "./" does the division element-wise; here we do
        // 1/denom once and use it three times — same result, fewer ops.
        float inv_den = 1.0f / denom;

        // MATLAB: B = (2 - k^2 * omega_new.^2) ./ (1 + sigma_new*k)
        //
        // omega_new.^2 squares each element of the omega vector.
        // In C++ we just do omega_[m] * omega_[m] for one element.
        B_[m] = (2.0f - k_sq_ * omega_[m] * omega_[m]) * inv_den;

        // MATLAB: C = (sigma_new*k - 1) ./ (1 + sigma_new*k)
        C_[m] = (sigma_m * k_ - 1.0f) * inv_den;

        // MATLAB: D = phi_i_new ./ (1 + sigma_new*k)
        D_[m] = phi_i_[m] * inv_den;
    }
}


/* ========================================================================
 * SECTION 6 — ProcessStereo()
 *
 * This is the PERFORMANCE-CRITICAL inner loop.  It runs once per audio
 * sample — at 48 kHz that's 48000 calls per second.  It must finish
 * within ~20 µs (one sample period).
 *
 * MATLAB equivalent — ONE iteration of "for n = 1:Nf":
 *
 *   p0      = B.*p1 + C.*p2 + D*in1(n);     ← vector arithmetic
 *   p2      = p1;                            ← copy entire vector
 *   p1      = p0;                            ← copy entire vector
 *   out1(n) = sum(phi_o_new .* p0);          ← dot product → scalar
 *
 * C++ TRANSLATION:
 * ────────────────
 * In MATLAB, "B.*p1" multiplies every element of B with every element
 * of p1 in a single expression, and "sum(phi_o .* p0)" does an element-
 * wise multiply then sums — i.e., a dot product.
 *
 * In C++ we handle ALL of this in ONE for-loop over modes:
 *   for each mode m:
 *     1) compute p0 = B[m]*p1[m] + C[m]*p2[m] + D[m]*input
 *     2) accumulate sum_l += phi_o_l[m] * p0   (dot product, one term)
 *     3) accumulate sum_r += phi_o_r[m] * p0
 *     4) shift:   p2[m] = p1[m];  p1[m] = p0;
 *
 * This single-pass approach is BETTER than MATLAB's separate vector
 * operations because it keeps each mode's data together in CPU cache.
 * MATLAB has to read/write the full arrays 4+ times; we touch each
 * mode's data exactly once.
 *
 * For STEREO: the state update (step 1) is identical for both channels
 * — we just add one extra multiply-accumulate per channel (steps 2-3).
 * This costs almost nothing compared to running two separate reverbs.
 * ========================================================================*/
void ModalPlateReverb::ProcessStereo(float input, float* out_l, float* out_r)
{
    // These accumulators replace MATLAB's "sum()" operation.
    // MATLAB: out1(n) = sum(phi_o_new .* p0)
    //         ↑ sum() adds up all elements into one scalar.
    //
    // In C++ we start at 0 and add one term per iteration.
    float sum_l = 0.0f;   // left  channel accumulator
    float sum_r = 0.0f;   // right channel accumulator

    for (int m = 0; m < num_modes_; m++)
    {
        // ----- State update: p0 = B.*p1 + C.*p2 + D*input -----
        //
        // In MATLAB, B.*p1 is an element-wise multiply producing a
        // vector, then + C.*p2 adds another vector, then + D*input
        // adds a scalar-times-vector.  The result p0 is a vector.
        //
        // In C++ we compute ONE element of that vector per iteration.
        // p0 is a local scalar (not stored in an array), which lets the
        // compiler keep it in a CPU register — very fast.
        float p0 = B_[m] * p1_[m]      // B(m) * P^n(m)
                  + C_[m] * p2_[m]      // + C(m) * P^{n-1}(m)
                  + D_[m] * input;      // + D(m) * input_sample

        // ----- Output accumulation (dot product, one term at a time) -----
        //
        // MATLAB: out1(n) = sum(phi_o_new .* p0)
        //
        // The sum() wrapping phi_o .* p0 is a DOT PRODUCT.  In C++ we
        // build that dot product incrementally: += adds one (φ*p0) term
        // per loop iteration.  After the loop, sum_l and sum_r hold the
        // complete dot product result.
        sum_l += phi_o_l_[m] * p0;   // left  output pickup
        sum_r += phi_o_r_[m] * p0;   // right output pickup

        // ----- State shift -----
        //
        // MATLAB: p2 = p1;  p1 = p0;
        //
        // In MATLAB these copy entire vectors.  In C++ we do it one
        // element at a time inside the same loop — no separate copy pass.
        p2_[m] = p1_[m];    // old "current" becomes "previous"
        p1_[m] = p0;        // new value becomes "current"
    }

    // Write output with PARTIAL pickup normalization.
    // Full normalization made the wet signal too quiet on some settings.
    // Blend toward unity gain so reverb remains audible while still taming peaks.
    const float gain_l = (1.0f - OUT_NORM_BLEND) + OUT_NORM_BLEND * out_norm_l_;
    const float gain_r = (1.0f - OUT_NORM_BLEND) + OUT_NORM_BLEND * out_norm_r_;
    *out_l = sum_l * gain_l * (WET_TRIM * WET_MAKEUP);
    *out_r = sum_r * gain_r * (WET_TRIM * WET_MAKEUP);
}

void ModalPlateReverb::ProcessMono(float input, float* out_mono)
{
    float sum_m = 0.0f;

    for (int m = 0; m < num_modes_; m++)
    {
        float p0 = B_[m] * p1_[m]
                  + C_[m] * p2_[m]
                  + D_[m] * input;

        sum_m += phi_o_m_[m] * p0;

        p2_[m] = p1_[m];
        p1_[m] = p0;
    }

    const float gain_m = (1.0f - OUT_NORM_BLEND) + OUT_NORM_BLEND * out_norm_m_;
    *out_mono = sum_m * gain_m * (WET_TRIM * WET_MAKEUP);
}


/* ========================================================================
 * SECTION 7 — Daisy / Planter Boilerplate
 *
 * This section mirrors the structure of PlateReverb.cpp:
 *   1. Global hardware + DSP objects
 *   2. AudioCallback — called by hardware once per audio block
 *   3. main() — init + start
 *
 * The only change is replacing DaisySP's ReverbSc with our
 * ModalPlateReverb class.
 * ========================================================================*/

planter          euro;        // Planter hardware wrapper
ModalPlateReverb reverb;     // our physical modeling reverb

// Track previous T60 so we only recompute coefficients when knobs move
static float prev_t60_low  = -1.0f;
static float prev_t60_high = -1.0f;
static uint32_t last_control_update_ms = 0;
static constexpr uint32_t CONTROL_UPDATE_MS = 200;

// One-pole high-pass filter coefficients/states for send and wet return.
static float send_hpf_a = 0.0f;
static float wet_hpf_a  = 0.0f;
static float sample_dt  = 1.0f / 48000.0f;
static float send_x1 = 0.0f, send_y1 = 0.0f;
static float wet_x1 = 0.0f, wet_y1 = 0.0f;

// Smoothed runtime parameters to reduce zipper noise / coefficient jumps.
static float t60_low_sm  = 2.0f;
static float t60_high_sm = 0.5f;
static float dry_sm      = 0.5f;
static float wet_send_sm = 0.02f;
static float wet_mix_sm  = 0.10f;
static float send_hpf_hz_sm = 40.0f;

// Control targets are sampled at a slower control rate, then smoothed per callback.
static float t60_low_target_hold   = 2.0f;
static float t60_high_target_hold  = 0.5f;
static float dry_target_hold       = 0.5f;
static float wet_send_target_hold  = 0.02f;
static float wet_mix_target_hold   = 0.10f;
static float send_hpf_target_hold  = 40.0f;

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    euro.patch.ProcessAnalogControls();

    /* ---- Read knobs (same pattern as original PlateReverb.cpp) ----
     *
     * The four CV inputs map to:
     *   CV_1 → Decay time  (T60 at lowest mode, logarithmic)
     *   CV_2 → Damping     (T60 at highest mode; CW=bright, CCW=dark)
     *   CV_3 → Dry level   (pass-through of input signal)
     *   CV_4 → Wet level   (amount of reverb mixed in)
     */

    // Sample controls every 200ms (lower control-rate CPU), smoothing below
    // keeps transitions continuous between these updates.
    const uint32_t now_ms = System::GetNow();
    bool controls_refreshed = false;
    if(now_ms - last_control_update_ms >= CONTROL_UPDATE_MS)
    {
        last_control_update_ms = now_ms;
        controls_refreshed = true;

        float time_knob = euro.GetKnobValue(planter::KNOB_1);
        t60_low_target_hold = fmap(time_knob, 0.3f, 8.0f, Mapping::LOG);

        float damp_knob = euro.GetKnobValue(planter::KNOB_2);
        t60_high_target_hold = fmap(damp_knob, 0.05f, t60_low_target_hold);

        dry_target_hold = euro.GetKnobValue(planter::KNOB_3) * DRY_MAX;

        float wet_knob = euro.GetKnobValue(planter::KNOB_4);
        wet_send_target_hold = wet_knob * wet_knob * wet_knob * WET_SEND_MAX;
        wet_mix_target_hold  = wet_knob * wet_knob * WET_MIX_MAX;

        // KNOB_5 -> HPF cutoff for REVERB SEND ONLY (10 Hz .. 100 Hz).
        // Dry passthrough is intentionally unaffected.
        float hpf_knob = euro.GetKnobValue(planter::KNOB_5);
        send_hpf_target_hold = fmap(hpf_knob, SEND_HPF_MIN_HZ, SEND_HPF_MAX_HZ);
    }

    // Block-rate smoothing to reduce zippering artifacts from fast knob moves.
    // (One-pole low-pass at control rate.)
    constexpr float kControlSmooth = 0.08f;
    t60_low_sm  += kControlSmooth * (t60_low_target_hold  - t60_low_sm);
    t60_high_sm += kControlSmooth * (t60_high_target_hold - t60_high_sm);
    dry_sm      += kControlSmooth * (dry_target_hold      - dry_sm);
    wet_send_sm += kControlSmooth * (wet_send_target_hold - wet_send_sm);
    wet_mix_sm  += kControlSmooth * (wet_mix_target_hold  - wet_mix_sm);
    send_hpf_hz_sm += kControlSmooth * (send_hpf_target_hold - send_hpf_hz_sm);

    // Recompute send HPF coefficient at control rate from smoothed cutoff.
    const float rc_send = 1.0f / (2.0f * PI_F * send_hpf_hz_sm);
    send_hpf_a = rc_send / (rc_send + sample_dt);

    if(t60_high_sm > t60_low_sm)
        t60_high_sm = t60_low_sm;

    /* ---- Update decay coefficients if knobs changed ----
     *
     * UpdateDecay() recalculates B, C, D for ALL modes (a loop of
     * ~hundreds of iterations).  It's cheap enough to run every block,
     * but we only do it when the knobs actually moved to save a bit of CPU.
     */
    // Coefficient refresh at a reduced control rate to avoid audible artifacts
    // from frequent full-bank updates and to lower callback CPU spikes.
    if (controls_refreshed &&
        (fabsf(t60_low_sm  - prev_t60_low)  > 0.01f ||
         fabsf(t60_high_sm - prev_t60_high) > 0.01f))
    {
        reverb.UpdateDecay(t60_low_sm, t60_high_sm);
        prev_t60_low  = t60_low_sm;
        prev_t60_high = t60_high_sm;
    }

    /* ---- Per-sample audio processing ----
     *
     * MATLAB: for n = 1:Nf ... end
     *
     * On the Daisy, the hardware calls AudioCallback with a block of
     * `size` samples (typically 48).  We process one sample per iteration,
     * which mirrors one step of MATLAB's "for n = 1:Nf" loop.
     */
    const float send_gain = 0.5f * wet_send_sm * WET_SEND_PRETRIM;
    const float dry_gain  = dry_sm * MIX_HEADROOM;
    const float wet_gain  = wet_mix_sm * MIX_HEADROOM;

    for (size_t i = 0; i < size; i++)
    {

        // Sum stereo input to mono — a real plate has one driver (excitation
        // point).  Scale by 0.5 to maintain level, then apply wet level.
        const float in_l = IN_L[i] * INPUT_TRIM;
        const float in_r = IN_R[i] * INPUT_TRIM;

        // HPF the excitation send to prevent low-frequency energy pile-up.
        const float mono_in = (in_l + in_r) * send_gain;
        const float mono_send = send_hpf_a * (send_y1 + mono_in - send_x1);
        send_x1 = mono_in;
        send_y1 = mono_send;

        // Process one sample through the modal plate reverb.
        // Returns two outputs (two pickup positions = stereo).
        float wet_mono;
        reverb.ProcessMono(mono_send, &wet_mono);

        // Mix dry (original) + wet (reverb) for each channel.
        // This is identical to the original PlateReverb.cpp's mixing stage.
        // HPF wet returns to remove rumble/DC-like buildup that causes
        // early perceived clipping without useful audible reverb level.
        const float wet_hp = wet_hpf_a * (wet_y1 + wet_mono - wet_x1);
        wet_x1 = wet_mono;
        wet_y1 = wet_hp;

        OUT_L[i] = ClampAudio(in_l * dry_gain + wet_hp * wet_gain);
        OUT_R[i] = ClampAudio(in_r * dry_gain + wet_hp * wet_gain);
    }
}
  

int main(void)
{
    // Initialize full Planter hardware wrapper (includes button/toggle init,
    // LEDs, CV outputs, and internal patch setup).
    euro.Init();

    // Force 44.1kHz for predictable CPU budget and DSP tuning.
    euro.patch.SetAudioSampleRate(44100.f);

    // Larger audio block reduces callback-rate overhead and gives the CPU
    // more time per callback for heavy DSP like modal reverb.
    euro.patch.SetAudioBlockSize(512);

    // Initialize the modal plate reverb — this call computes all modes,
    // runs the pruning algorithm, and pre-computes coefficients.
    // It may take a fraction of a second (lots of math at startup).
    reverb.Init(euro.patch.AudioSampleRate());

    // Configure HPF coefficients from current sample rate.
    const float sr = euro.patch.AudioSampleRate();
    sample_dt = 1.0f / sr;
    const float rc_send = 1.0f / (2.0f * PI_F * send_hpf_hz_sm);
    const float rc_wet  = 1.0f / (2.0f * PI_F * WET_HPF_HZ);
    send_hpf_a = rc_send / (rc_send + sample_dt);
    wet_hpf_a  = rc_wet  / (rc_wet  + sample_dt);

    // Start the audio engine — AudioCallback will now be called
    // continuously by the hardware every ~1 ms with blocks of samples.
    euro.patch.StartAudio(AudioCallback);

    // Infinite loop — audio processing happens in the callback.
    // The main thread just idles.
    while (1) {
        // Match known-good example pattern for bootloader button handling.
        euro.button.Debounce();
        euro.BootloaderResetCheck();
    }
}
