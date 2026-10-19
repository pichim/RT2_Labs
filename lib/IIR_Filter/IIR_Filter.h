#pragma once

class IIR_Filter
{
public:
    IIR_Filter() {};       // default constructor
    virtual ~IIR_Filter(); // deconstructor

    float operator()(float u) { return evaluate(u); }

    void lowPass1Init(float, float);
    void differentiatingLowPass1Init(float, float);
    // Second-order filters used by fast_realtime_thread; fcut in Hz.
    void notchInit(float fcut, float D, float Ts);
    void lowPass2Init(float fcut, float D, float Ts);
    float evaluate(float);
    void reset(float, float);

private:
    // G(z) = (b1 + b0*z^-1 + b2*z^-2) / (1 + a0*z^-1 + a1*z^-2)
    // First-order filters set b2 and a1 to zero.
    float m_b1{0.0f};      // coefficient for u(k)
    float m_b0{0.0f};      // coefficient for u(k-1)
    float m_b2{0.0f};      // coefficient for u(k-2)
    float m_a0{0.0f};      // denominator coefficient for y(k-1)
    float m_a1{0.0f};      // denominator coefficient for y(k-2)
    float m_u_kmin1{0.0f}; // u(k-1)
    float m_u_kmin2{0.0f}; // u(k-2)
    float m_y_kmin1{0.0f}; // y(k-1)
    float m_y_kmin2{0.0f}; // y(k-2)
};
