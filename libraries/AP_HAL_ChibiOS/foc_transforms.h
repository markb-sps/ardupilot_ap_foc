#pragma once

#include <math.h>

namespace ChibiOS {
namespace FOC {

// Clarke: balanced 3-phase abc → αβ (amplitude-invariant)
// Precondition: ia + ib + ic = 0
inline void clarke(float ia, float ib, float &i_alpha, float &i_beta)
{
    constexpr float INV_SQRT3 = 0.57735026919f;
    i_alpha = ia;
    i_beta  = (ia + 2.0f * ib) * INV_SQRT3;
}

// Park: αβ → dq
inline void park(float i_alpha, float i_beta, float sin_t, float cos_t,
                 float &i_d, float &i_q)
{
    i_d =  i_alpha * cos_t + i_beta * sin_t;
    i_q = -i_alpha * sin_t + i_beta * cos_t;
}

// Inverse Park: dq → αβ
inline void inv_park(float v_d, float v_q, float sin_t, float cos_t,
                     float &v_alpha, float &v_beta)
{
    v_alpha = v_d * cos_t - v_q * sin_t;
    v_beta  = v_d * sin_t + v_q * cos_t;
}

// Inverse Clarke: αβ → balanced 3-phase abc
inline void inv_clarke(float v_alpha, float v_beta,
                       float &va, float &vb, float &vc)
{
    constexpr float SQRT3_OVER2 = 0.86602540378f;
    va =  v_alpha;
    vb = -0.5f * v_alpha + SQRT3_OVER2 * v_beta;
    vc = -0.5f * v_alpha - SQRT3_OVER2 * v_beta;
}

// SVPWM via min-max zero-sequence injection (equivalent to standard space-vector).
// Inputs: va,vb,vc normalized to Vdc/2 (range [-1,+1] at full SPWM; linear to 2/√3 with SVPWM)
// Outputs: da,db,dc duty cycles clamped to [0,1]
inline void svpwm(float va, float vb, float vc,
                  float &da, float &db, float &dc)
{
    const float v_max = fmaxf(va, fmaxf(vb, vc));
    const float v_min = fminf(va, fminf(vb, vc));
    const float v_zs  = -0.5f * (v_max + v_min);
    da = fmaxf(0.0f, fminf(1.0f, 0.5f + 0.5f * (va + v_zs)));
    db = fmaxf(0.0f, fminf(1.0f, 0.5f + 0.5f * (vb + v_zs)));
    dc = fmaxf(0.0f, fminf(1.0f, 0.5f + 0.5f * (vc + v_zs)));
}

} // namespace FOC
} // namespace ChibiOS
