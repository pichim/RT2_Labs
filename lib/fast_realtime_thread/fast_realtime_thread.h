#pragma once

#include <chrono>

#include "IIR_Filter.h"
#include "IO_handler.h"
#include "PID_Cntrl.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "rtos.h"

#define POWERSUPPLY_VOLTAGE 24.0f // Voltage of the power supply in Volts
#define OFFSET_VOLTAGE 2.0f       // Offset voltage to overcome motor deadzone in Volts

#define KP_I 2.5f                               // Proportional gain current controller
#define TN_I (0.0013f / 4.5320f)                // Integral time constant current controller (L / R)
#define KI_I (KP_I / TN_I)                      // Integral gain current controller
#define TAU_RO_I (1.f / (2.f * M_PIf * 3.0e3f)) // Time constant of first order rolloff filter in current controller

#define F_CUT_HZ_NOTCH 680.0f // Notch filter cutoff frequency in Hz
#define D_NOTCH 0.6f          // Notch filter damping

#define F_CUT_HZ 500.0f // Second order low-pass filter cutoff frequency in Hz
#define D 0.9f          // Second order low-pass filter damping ratio

using namespace std::chrono;

class fast_realtime_thread
{
public:
    fast_realtime_thread(IO_handler &io, float Ts);
    virtual ~fast_realtime_thread();
    void start_loop(void);

    // Coherent command update and measurement snapshot, each under one lock.
    void updateState(bool enable, float current_setpoint);
    void getMeasurements(float &current, float &motor_angle, float &pendulum_angle);

private:
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    float m_Ts;
    IO_handler &m_IO_handler;
    IIR_Filter m_notch_encoders[2];
    IIR_Filter m_low_pass2_current_setpoint;
    PID_Cntrl m_pid_cntrl;

    rtos::Mutex m_mutex;
    bool m_is_enabled{false};
    float m_current_setpoint{0.0f};
    float m_current{0.0f};
    float m_motor_angle{0.0f};
    float m_pendulum_angle{0.0f};

    void loop(void);
    void sendSignal();
    float clamp(float val, float min, float max);
};
