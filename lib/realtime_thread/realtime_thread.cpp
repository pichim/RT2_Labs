#include "realtime_thread.h"

#include <chrono>
#include <cstdint>

#include "DataLogger.h"
#include "GPA.h"

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
    Matrix<float, 1, 4> K4(-2.1929f, -0.2016f, -0.0042f, 0.0100f);
    Matrix<float, 4, 1> x4(0.0f, 0.0f, 0.0f, 0.0f);
    float xi = 0.0f;

    // --- AUFGABE 7.1 ---
    // simple P controller gain
    const float kp = 0.2f;

    // --- AUFGABE 7.5 ---
    // excitation signal for frequency response measurement
    float exc = 0.0f;

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
                i_des = kp * (w - phi_fw_vel);

                // // --- AUFGABE 7.5 ---
                // // use P controller to measure the frequency response
                // i_des = kp * (w - phi_fw_vel + (exc + 4.0f * M_PIf));

                // switch to BALANCE
                if (do_transition) {
                    m_state = BALANCE;
                }
                break;
            }
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
                                  w,      // 1
                                  i_des,  // 2
                                  x4(0),  // 3
                                  x4(1),  // 4
                                  x4(2),  // 5
                                  x4(3)); // 6

        // --- AUFGABE 7.5 ---
        // measure the frequency response, GPA calculates future excitation exc(k+1)
        exc = myGPA.update(i_des, phi_fw_vel);
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
