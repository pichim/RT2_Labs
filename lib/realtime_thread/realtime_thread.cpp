#include "realtime_thread.h"

#include <chrono>
#include <cmath>
#include <cstdint>

#include "DataLogger.h"
#include "GPA.h"

extern DataLogger myDataLogger;
extern GPA myGPA;

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f // pi
#endif

// Uncomment to enable the existing 3 kHz current-controller roll-off.
// #define ENABLE_CURRENT_ROLLOFF

using namespace Eigen;
using namespace std::chrono;

// constructor for realtime_thread loop
realtime_thread::realtime_thread(IO_handler *io_handler, float Ts)
    : m_Thread(osPriorityHigh1, 4096)
{
    m_Ts = Ts;                 // the sampling time
    m_IO_handler = io_handler; // a pointer to the io handler
    m_state = INIT;
    m_IO_handler->set_enable_motor(false);
    m_IO_handler->write_pwm_motor(0.0f);
    m_IO_handler->set_user_led(false);
    m_Timer.reset();
    m_Timer.start();
}

realtime_thread::~realtime_thread() {}

// this is the main loop called every Ts with high priority
void realtime_thread::loop(void)
{
    // Current controller, matching matlab/inv_rot_pen_ini.m.
    const float supply_voltage = 24.0f;
    const float offset_voltage = 2.0f; // motor deadzone compensation
    const float kp_i = 2.5f;
    const float tn_i = 0.0013f / 4.5320f; // existing integral time constant (L / R)
    const float ki_i = kp_i / tn_i;
#ifdef ENABLE_CURRENT_ROLLOFF
    const float tau_ro_i = 1.0f / (2.0f * M_PIf * 3.0e3f);
#else
    const float tau_ro_i = 0.0f;
#endif
    PID_Cntrl pid_i;
    pid_i.setup(kp_i,
                ki_i,
                0.0f,
                0.0f,
                tau_ro_i,
                m_Ts,
                -supply_voltage + offset_voltage,
                supply_voltage - offset_voltage);

    // Encoder notch filters: same frequency/damping, recalculated for m_Ts.
    const float f_notch = 680.0f;
    const float d_notch = 0.6f;
    IIR_Filter fil_notch1, fil_notch2;
    fil_notch1.notchInit(f_notch, d_notch, m_Ts);
    fil_notch2.notchInit(f_notch, d_notch, m_Ts);

    // State feedback with current input, from matlab/inv_rot_pen_ini.m.
    // x = [theta1; shortangle(theta2-pi); omega1; omega2].
    Matrix<float, 1, 4> K(-0.447213595f, 3.437582984f, -0.155543856f, 0.303057827f);
    Matrix<float, 4, 1> x(0.0f, 0.0f, 0.0f, 0.0f);
    const float V = -0.447213595f;
    const float i_max = 0.5f;
    const float i_friction = 0.0205f;
    const float phi2_enable = 0.1745329252f;  // rad, 10 deg
    const float phi2_disable = 0.5235987756f; // rad, 30 deg

    // Same Tustin differentiating filters as G_diff in MATLAB, now at 10 kHz.
    const float Tf = 1.0f / (2.0f * M_PIf * 100.0f);
    IIR_Filter fil_diff1, fil_diff2;
    fil_diff1.differentiatingLowPass1Init(Tf, m_Ts);
    fil_diff2.differentiatingLowPass1Init(Tf, m_Ts);
    bool first_measurement = true;

    // User LED: 2 Hz blink while ARMED, solid while BALANCE, off in INIT.
    uint16_t led_counter = 0;
    const uint16_t led_half_period = static_cast<uint16_t>(0.25f / m_Ts + 0.5f);
    bool led_on = false;

    // GPA excitation placement will be finalized separately.
    // float exc = 0.0f;

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        const float time = 1e-6f * (float)(duration_cast<microseconds>(m_Timer.elapsed_time()).count());
        m_IO_handler->set_enable_rtt_do(true);
        // --------------------- THE LOOP ---------------------

        float w = myDataLogger.get_set_value(time); // get set values from the GUI [rad]

        // Update sensor readings once, as in mini_cuboid.
        m_IO_handler->update();
        const float current = m_IO_handler->get_current();
        const float theta1_raw = m_IO_handler->get_theta1();
        const float theta2_raw = m_IO_handler->get_theta2();
        if (first_measurement) {
            // Initialize notch histories at the measured angles (unity DC gain).
            fil_notch1.reset(theta1_raw, theta1_raw);
            fil_notch2.reset(theta2_raw, theta2_raw);
            fil_diff1.reset(theta1_raw, 0.0f);
            fil_diff2.reset(theta2_raw, 0.0f);
            first_measurement = false;
        }
        const float theta1 = fil_notch1(theta1_raw);
        const float theta2 = fil_notch2(theta2_raw);
        const float omega1 = fil_diff1(theta1);
        const float omega2 = fil_diff2(theta2);
        const float phi2 = shortangle(theta2 - M_PIf);
        x << theta1, phi2, omega1, omega2;

        // State machine: manually bring the pendulum upright; no automatic swing-up/down.
        // Rearming does not reset the encoders or their velocity filters.
        const bool do_transition = m_IO_handler->get_and_reset_button_state();
        switch (m_state) {
            case INIT: {
                // ------------------- INIT -------------------
                // Motor disabled; a fresh button press is required to arm.
                if (do_transition) {
                    m_state = ARMED;
                    led_on = true;
                }
                break;
            }
            case ARMED: {
                // ------------------- ARMED -------------------
                // Keep the motor disabled until the pendulum enters the capture band.
                if (do_transition)
                    m_state = INIT;
                else if (fabsf(phi2) <= phi2_enable)
                    m_state = BALANCE;
                break;
            }
            case BALANCE: {
                // ------------------- BALANCE ----------------
                // Disarm on button press or when leaving the outer angle band.
                if (do_transition || fabsf(phi2) >= phi2_disable)
                    m_state = INIT;
                break;
            }
            default:
                m_state = INIT;
                break;
        }

        float i_des = 0.0f;
        if (m_state == BALANCE) {
            // Control law from Simulink: current input is already accounted for in K/V.
            i_des = V * w - K * x;
            // i_des = V * (w + exc) - K * x; // GPA: finalize excitation placement later.
            i_des = saturate(i_des, -i_max, i_max);
            // Simulink adds friction compensation AFTER saturation: up to +/-0.5205 A.
            if (i_des > 0.0f)
                i_des += i_friction;
            else if (i_des < 0.0f)
                i_des -= i_friction;
        }

        // Current controller and motor output, in the same 10 kHz loop.
        m_IO_handler->set_enable_motor(m_state == BALANCE);
        if (m_state == BALANCE) {
            const float current_error = i_des - current;
            const float u = pid_i.update(current_error, current);
            m_IO_handler->set_dir(u > 0.0f ? 0 : 1);
            const float pwm = saturate((fabsf(u) + offset_voltage) / supply_voltage, 0.0f, 1.0f);
            m_IO_handler->write_pwm_motor(pwm);
        } else {
            m_IO_handler->write_pwm_motor(0.0f);
            pid_i.reset();
        }

        if (m_state == ARMED) {
            if (++led_counter >= led_half_period) {
                led_counter = 0;
                led_on = !led_on;
            }
        } else {
            led_counter = 0;
            led_on = m_state == BALANCE;
        }
        m_IO_handler->set_user_led(led_on);

        // Log the same six signals as the former Simulink scope.
        myDataLogger.write_to_log(time,
                                  i_des,   // 1: commanded current [A]
                                  current, // 2: measured current [A]
                                  theta1,  // 3: arm angle [rad]
                                  phi2,    // 4: pendulum angle relative to upright [rad]
                                  omega1,  // 5: arm angular velocity [rad/s]
                                  omega2); // 6: pendulum angular velocity [rad/s]

        // GPA preparation only; select excitation and measured transfer function first.
        // exc = myGPA.update(i_des, theta1);
        m_IO_handler->set_enable_rtt_do(false);
    }
}

float realtime_thread::saturate(float x, float ll, float ul)
{
    if (x > ul)
        return ul;
    else if (x < ll)
        return ll;
    return x;
}

float realtime_thread::shortangle(float angle) { return atan2f(sinf(angle), cosf(angle)); }

void realtime_thread::sendSignal() { m_Thread.flags_set(m_ThreadFlag); }

void realtime_thread::start_loop(void)
{
    m_Thread.start(callback(this, &realtime_thread::loop));
    m_Ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(m_Ts * 1e6f)});
}
