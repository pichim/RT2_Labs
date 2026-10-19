#include "fast_realtime_thread.h"

#include <chrono>
#include <cmath>
#include <cstdint>

using MutexLock = rtos::ScopedMutexLock;

fast_realtime_thread::fast_realtime_thread(IO_handler &io, float Ts)
    : m_Thread(osPriorityHigh2, (4 * OS_STACK_SIZE))
    , m_Ts(Ts)
    , m_IO_handler(io)
{
    m_notch_encoders[0].notchInit(F_CUT_HZ_NOTCH, D_NOTCH, m_Ts);
    m_notch_encoders[1].notchInit(F_CUT_HZ_NOTCH, D_NOTCH, m_Ts);

    m_low_pass2_current_setpoint.lowPass2Init(F_CUT_HZ, D, m_Ts);

    m_pid_cntrl.setup(KP_I,
                      KI_I,
                      0.0f,
                      0.0f,
                      TAU_RO_I,
                      m_Ts,
                      -POWERSUPPLY_VOLTAGE + OFFSET_VOLTAGE,
                      POWERSUPPLY_VOLTAGE - OFFSET_VOLTAGE);
}

fast_realtime_thread::~fast_realtime_thread() {}

void fast_realtime_thread::updateState(bool enable, float current_setpoint)
{
    MutexLock lock(m_mutex);
    m_is_enabled = enable;
    m_current_setpoint = current_setpoint;
}

void fast_realtime_thread::getMeasurements(float &current, float &motor_angle, float &pendulum_angle)
{
    MutexLock lock(m_mutex);
    current = m_current;
    motor_angle = m_motor_angle;
    pendulum_angle = m_pendulum_angle;
}

void fast_realtime_thread::loop(void)
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        m_IO_handler.set_enable_frtt_do(true);

        // Read current and filtered encoder values
        m_IO_handler.update();
        const float current = m_IO_handler.get_current();
        const float motor_angle = m_notch_encoders[0](m_IO_handler.get_theta1());
        const float pendulum_angle = m_notch_encoders[1](m_IO_handler.get_theta2());

        bool is_enabled;
        float current_setpoint;
        {
            MutexLock lock(m_mutex);
            is_enabled = m_is_enabled;
            current_setpoint = m_current_setpoint;
            m_current = current;
            m_motor_angle = motor_angle;
            m_pendulum_angle = pendulum_angle;
        }

        // Only the fast loop writes motor enable, direction and PWM.
        m_IO_handler.set_enable_motor(is_enabled);

        // Current controller
        if (is_enabled) {

            // Error
            float current_error = m_low_pass2_current_setpoint(current_setpoint) - current;
            // Controller
            const float u = m_pid_cntrl.update(current_error, current);

            // Calculate direction and PWM value
            if (u > 0.0f)
                m_IO_handler.set_dir(0);
            else
                m_IO_handler.set_dir(1);

            m_IO_handler.write_pwm_motor(clamp((fabs(u) + OFFSET_VOLTAGE) / POWERSUPPLY_VOLTAGE, 0.0f, 1.0f));
        } else {
            m_IO_handler.write_pwm_motor(0.0f);
            m_pid_cntrl.reset();
            m_low_pass2_current_setpoint.reset(0.0f, 0.0f);
        }

        m_IO_handler.set_enable_frtt_do(false);
    }
}

void fast_realtime_thread::sendSignal() { m_Thread.flags_set(m_ThreadFlag); }

void fast_realtime_thread::start_loop(void)
{
    m_Thread.start(callback(this, &fast_realtime_thread::loop));
    m_Ticker.attach(callback(this, &fast_realtime_thread::sendSignal), microseconds{static_cast<int64_t>(m_Ts * 1e6f)});
}

float fast_realtime_thread::clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}
