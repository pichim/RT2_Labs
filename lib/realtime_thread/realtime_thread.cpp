#include "realtime_thread.h"

#include <chrono>
#include <cstdint>

#include "DataLogger.h"
#include "GPA.h"
#include "swingup_trajectory.h"

extern DataLogger myDataLogger;
extern GPA myGPA;

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f // pi
#endif

using namespace Eigen;
using namespace std::chrono;

// contructor for realtime_thread loop
realtime_thread::realtime_thread(IO_handler *io_handler, float Ts)
    : m_Thread(osPriorityHigh, 4096)
{
    m_Ts = Ts;                 // the sampling time
    m_IO_handler = io_handler; // a pointer to the io handler
    m_state = INIT;
    m_IO_handler->disable_escon();
    m_Timer.reset();
    m_Timer.start();
}

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// this is the main loop called every Ts with high priority
void realtime_thread::loop(void)
{
    // motor constant
    const float km = 36.9e-3f;

    // // --- AUFGABE 5.2 ---
    // // variables for stabilizing state space controller for cube angle and angle velocity
    // Matrix<float, 1, 2> K2(-1.0557f, -0.0675f);
    // Matrix<float, 2, 1> x2(0.0f, 0.0f);

    // --- AUFGABE 6.5 ---
    // variables for state space controller with integrator for velocity error
    /*
    Matrix<float, 1, 4> K4(-2.1929f, -0.2016f, -0.0042f, 0.0100f);
    Matrix<float, 4, 1> x4(0.0f, 0.0f, 0.0f, 0.0f);
    float xi = 0.0f;
    */

    // --- AUFGABE 7.1 ---
    // simple P controller gain
    const float kp = 0.2f;
    const float spinup_time = 3.0f; // spin-up duration [s]
    float spinup_start_time = 0.0f;
    uint16_t swingup_sample = 0;
    m_IO_handler->write_current(0.0f); // zero current before enabling the drive

    // --- AUFGABE 7.5 ---
    // excitation signal for frequency response measurement
    // float exc = 0.0f;

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        const float time = 1e-6f * (float)(duration_cast<microseconds>(m_Timer.elapsed_time()).count());
        // --------------------- THE LOOP ---------------------

        float w = myDataLogger.get_set_value(time); // get set values from the GUI

        // update sensor readings and readout values from io handler
        m_IO_handler->update();
        const float phi_fw_vel = m_IO_handler->get_phi_fw_vel();
        const float gz = m_IO_handler->get_gz();
        const float phi_bd = m_IO_handler->get_phi_bd();

        // // --- AUFGABE 3.2 ---
        // log angle estimate from complementary filter
        // myDataLogger.write_to_log(time, phi_bd, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        // state machine
        float i_des = 0.0f;
        const bool do_transition = m_IO_handler->get_and_reset_button_state();
        switch (m_state) {
            case INIT: {
                // ------------------- INIT -------------------
                // disable motor and wait for button press to switch to FLAT
                m_IO_handler->disable_escon();

                // switch to FLAT
                if (do_transition) {
                    m_state = FLAT;
                    m_IO_handler->enable_escon();
                }
                break;
            }
            case FLAT: {
                // ------------------- FLAT -------------------

                // --- AUFGABE 7.1 ---
                // simple P controller
                i_des = -kp * phi_fw_vel; // zero wheel-speed setpoint while FLAT

                // // --- AUFGABE 7.5 ---
                // // use P controller to measure the frequency response
                // i_des = kp * (w - phi_fw_vel + (exc + 4.0f * M_PIf));

                // switch to SPIN_UP
                if (do_transition) {
                    spinup_start_time = time;
                    m_state = SPIN_UP;
                }
                break;
            }
            case SPIN_UP: {
                // Finish the ramp before accepting a nonzero GUI setpoint.
                const float w_spinup = swingup_omega2_0
                    * saturate((time - spinup_start_time) / spinup_time, 0.0f, 1.0f);
                i_des = kp * (w_spinup - phi_fw_vel);
                if (time - spinup_start_time >= spinup_time
                    && w != 0.0f) {
                    swingup_sample = 0;
                    m_state = SWING_UP;
                }
                break;
            }
            case SWING_UP: {
                // Apply one MATLAB torque per Ts tick; keep MATLAB and firmware Ts equal.
                if (swingup_sample < swingup_sample_count) {
                    const float M = swingup_torque_Nm[swingup_sample++];
                    i_des = M / km;
                } else {
                    // Balancing is disabled for now; stop after the last interval.
                    m_state = INIT;
                    m_IO_handler->disable_escon();
                }
                break;
            }
            // Uncomment BALANCE and its variables/measurements for later handover tests.
            /*
            case BALANCE: {
                // ------------------- BALANCE ----------------

                // // --- AUFGABE 5.2 ---
                // // stabilizing state space controller for cube angle and angle velocity
                // x2 << phi_bd, gz;
                // const float M = -K2 * x2;
                // i_des = M / km;

                // --- AUFGABE 6.5 ---
                // state space controller with integrator for velocity error
                const float err = w - phi_fw_vel;
                xi = saturate(xi + m_Ts * err, -5.0f * km / K4(3), 5.0f * km / K4(3));
                x4 << phi_bd, gz, phi_fw_vel, xi;
                const float M = -K4 * x4;
                i_des = M / km;

                // switch to FLAT
                if (do_transition) {
                    m_state = FLAT;
                    // --- AUFGABE 7.2 ---
                    // if we go to FLAT, reset integrator state
                    xi = 0.0f;
                }
                break;
            }
            */
            default:
                break;
        }

        // write current setpoint to motor
        // --- AUFGABE 5.2 ---
        // limit current to +/-15 A
        i_des = saturate(i_des, -15.0f, 15.0f);

        m_IO_handler->write_current(i_des);

        // --- AUFGABE 7.4 ---
        // log swing up and balancing data
        myDataLogger.write_to_log(time,
                                  w,           // 1: GUI setpoint (trigger)
                                  i_des,       // 2: commanded current [A]
                                  phi_bd,      // 3: body angle [rad]
                                  gz,          // 4: body angular velocity [rad/s]
                                  phi_fw_vel,  // 5: relative wheel velocity [rad/s]
                                  km * i_des); // 6: commanded torque [Nm]

        // --- AUFGABE 7.5 ---
        // measure the frequency response, GPA calculates future excitation exc(k+1)
        // exc = myGPA.update(i_des, phi_fw_vel);
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

void realtime_thread::sendSignal() { m_Thread.flags_set(m_ThreadFlag); }

void realtime_thread::start_loop(void)
{
    m_Thread.start(callback(this, &realtime_thread::loop));
    m_Ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(m_Ts * 1e6f)});
}
