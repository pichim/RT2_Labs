#include "observer.h"

// constructor
observer::observer(float Ts)
{
    m_Ts = Ts;
    init();
}

observer::~observer() {}

// calculate one step of the observer
Matrix<float, N, 1> observer::do_step(float u, float y)
{
    // --- P2, AUFGABE 2.3 ---
    // implement observer and apply time discrete integration step
    m_dxdt_hat = (m_A - m_H * m_C) * m_x_hat + m_B * u + m_H * y;
    integrate_states();
    return m_x_hat;
}

// get the observed states
Matrix<float, N, 1> observer::get_x_obsv() { return m_x_hat; }

void observer::init()
{
    // initialize all matrices with zeros
    m_A.setZero();
    m_B.setZero();
    m_C.setZero();
    m_H.setZero();
    m_dxdt_hat.setZero();
    m_x_hat.setZero();

    // --- P2, AUFGABE 2.2 ---
    // set A, B, C, H matrices of observer
    m_A << -905.387f, 452.694f, 452.694f, -452.694f;
    m_B << 452.694f, 0.000f;
    m_C << 0.000f, 1.000f;
    m_H << 6203.968f, 1641.919f;
}

void observer::integrate_states()
{
    // --- P2, AUFGABE 2.4 ---
    // implement time discrete integration step
    m_x_hat += m_Ts * m_dxdt_hat;
}
