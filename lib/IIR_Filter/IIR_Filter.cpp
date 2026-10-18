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
    m_a0 = a0 / a1;

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
    m_a0 = a0 / a1;

    reset(0.0f, 0.0f);
}

float IIR_Filter::evaluate(const float u_k)
{
    // --- AUFGABE 2.4 ---
    // update filter output
    const float y_k = m_b1 * u_k + m_b0 * m_u_kmin1 - m_a0 * m_y_kmin1;

    // store values for next iteration
    m_u_kmin1 = u_k;
    m_y_kmin1 = y_k;

    return y_k;
}

void IIR_Filter::reset(float u_kmin1, float y_kmin1)
{
    m_u_kmin1 = u_kmin1;
    m_y_kmin1 = y_kmin1;
}
