#include "IIR_Filter.h"

#include <math.h>

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f // pi
#endif

IIR_Filter::~IIR_Filter() {}

// First Order Lowpass Filter
// Time continuous prototype: G(s) = 1 / (tau*s + 1)
// Discretization method: Tustin
void IIR_Filter::lowPass1Init(float tau, float Ts)
{
    // --- AUFGABE 2.4 ---
    // a1 = Ts + 2*tau;
    // b1 = Ts;
    // b0 = b1;
    // a0 = Ts - 2*tau;
    const float a1 = Ts + 2.0f * tau;
    const float b1 = Ts;
    const float b0 = b1;
    const float a0 = Ts - 2.0f * tau;

    m_b1 = b1 / a1;
    m_b0 = b0 / a1;
    m_b2 = 0.0f;
    m_a0 = a0 / a1;
    m_a1 = 0.0f;

    reset(0.0f, 0.0f);
}

// First Order Differentiating Lowpass Filter
// Time continuous prototype: G(s) =  s / (tau * s + 1)
// Discretization method: Tustin
void IIR_Filter::differentiatingLowPass1Init(float tau, float Ts)
{
    // a1 = Ts + 2*tau;
    // b1 = 2;
    // b0 = -b1;
    // a0 = Ts - 2*tau;
    const float a1 = Ts + 2.0f * tau;
    const float b1 = 2.0f;
    const float b0 = -b1;
    const float a0 = Ts - 2.0f * tau;

    m_b1 = b1 / a1;
    m_b0 = b0 / a1;
    m_b2 = 0.0f;
    m_a0 = a0 / a1;
    m_a1 = 0.0f;

    reset(0.0f, 0.0f);
}

// Second Order Notch Filter
// Time continuous prototype: G(s) = (s^2 + wcut^2) / (s^2 + 2*D*wcut*s + wcut^2)
// Discretization method: Tustin with prewarping, as in the original IIRFilter.
// fcut [Hz], D > 0, and 0 < fcut < 1/(2*Ts).
void IIR_Filter::notchInit(float fcut, float D, float Ts)
{
    const float omega = 2.0f * M_PIf * fcut * Ts;
    const float sn = sinf(omega);
    const float cs = cosf(omega);

    m_b1 = 1.0f / (1.0f + D * sn);
    m_b0 = -2.0f * cs * m_b1;
    m_b2 = m_b1;
    m_a0 = m_b0;
    m_a1 = (1.0f - D * sn) * m_b1;

    reset(0.0f, 0.0f);
}

// Second Order Lowpass Filter
// Time continuous prototype: G(s) = wcut^2 / (s^2 + 2*D*wcut*s + wcut^2)
// Discretization method: backward Euler, as in the original IIRFilter.
// fcut [Hz], D > 0, Ts > 0.
void IIR_Filter::lowPass2Init(float fcut, float D, float Ts)
{
    const float wcut = 2.0f * M_PIf * fcut;
    const float k1 = 2.0f * D * Ts * wcut;

    m_a1 = 1.0f / (Ts * Ts * wcut * wcut + k1 + 1.0f);
    m_b1 = 1.0f - m_a1 * (1.0f + k1);
    m_b0 = 0.0f;
    m_b2 = 0.0f;
    m_a0 = m_b1 - 1.0f - m_a1;

    reset(0.0f, 0.0f);
}

float IIR_Filter::evaluate(const float u_k)
{
    // --- AUFGABE 2.4 ---
    // update filter output
    const float y_k = m_b1 * u_k + m_b0 * m_u_kmin1 + m_b2 * m_u_kmin2 - m_a0 * m_y_kmin1 - m_a1 * m_y_kmin2;

    // store values for next iteration
    m_u_kmin2 = m_u_kmin1;
    m_u_kmin1 = u_k;
    m_y_kmin2 = m_y_kmin1;
    m_y_kmin1 = y_k;

    return y_k;
}

void IIR_Filter::reset(float u_kmin1, float y_kmin1)
{
    m_u_kmin1 = m_u_kmin2 = u_kmin1;
    m_y_kmin1 = m_y_kmin2 = y_kmin1;
}
