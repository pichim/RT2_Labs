#pragma once

class IIR_Filter
{
public:
    IIR_Filter() {};       // default constructor
    virtual ~IIR_Filter(); // deconstructor

    float operator()(float u) { return evaluate(u); }

    void lowPass1Init(float, float);
    void differentiatingLowPass1Init(float, float);
    float evaluate(float);
    void reset(float, float);

private:
    // G(z) = Y(z)/U(z) = (b1*z + b0) / (z + a0)
    //      = (b1 + b0*z^-1) / (1 + a0*z^-1)
    // y(k) = b1*u(k) + b0*u(k-1) - a0*y(k-1)
    float m_b1{0.0f};      // coefficient for u(k)
    float m_b0{0.0f};      // coefficient for u(k-1)
    float m_a0{0.0f};      // denominator coefficient for y(k-1)
    float m_u_kmin1{0.0f}; // u(k-1)
    float m_y_kmin1{0.0f}; // y(k-1)
};
