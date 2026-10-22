#include "IO_handler.h"

// constructors
IO_handler::IO_handler(void)
    : m_button(BUTTON1)
    , m_user_led(LED1, 0)
    , m_encoder_motor(PA_6, PC_7, ENCODER_MOTOR_COUNTS_PER_TURN)
    , m_encoder_pendulum(PB_6, PB_7, ENCODER_PENDULUM_COUNTS_PER_TURN)
    , m_pwm(PB_15)
    , m_pwm_val(0.0f)
    , m_dir(PB_14)
    , m_enable(PB_9)
    , m_current_sensor(PA_7)
    , m_fault(PB_8)
    , m_rtt_do(PB_4)
{
    // B1 is active low on NUCLEO-F446RE; use the existing 20 ms debounce.
    m_button.fall(callback(this, &IO_handler::button_pressed));

    m_pwm.write(0.0f); // ensure motor is off
    m_pwm.period_us(MOTOR_PWM_PERIOD_US);
    m_dir = 0;
    m_enable = 0;
}

IO_handler::~IO_handler() {}

void IO_handler::update(void)
{
    m_current.store(read_current(), std::memory_order_relaxed);
    m_theta1.store(read_encoder_motor(), std::memory_order_relaxed);
    m_theta2.store(read_encoder_pendulum(), std::memory_order_relaxed);
}

float IO_handler::read_encoder_motor(void) { return m_encoder_motor.getAngleRad(); }

// Positive encoder angle now matches the former Simulink output: remove both negations.
float IO_handler::read_encoder_pendulum(void) { return m_encoder_pendulum.getAngleRad(); }

void IO_handler::reset_encoders(void)
{
    m_encoder_motor.reset();
    m_encoder_pendulum.reset();
}

void IO_handler::write_pwm_motor(float val)
{
    m_pwm_val = val;
    m_pwm.write(m_pwm_val);
}

float IO_handler::get_set_value(void) const { return m_pwm_val; }
void IO_handler::set_enable_motor(bool en) { m_enable = en ? 1 : 0; }
float IO_handler::read_current(void) { return (m_current_sensor.read() * 3.3f - 1.5f) * -1.0f / (50.0f * 7.0e-3f); }
void IO_handler::set_dir(uint8_t d) { m_dir = d; }
void IO_handler::set_enable_rtt_do(bool en) { m_rtt_do = en ? 1 : 0; }
float IO_handler::get_current(void) const { return m_current.load(std::memory_order_relaxed); }
float IO_handler::get_theta1(void) const { return m_theta1.load(std::memory_order_relaxed); }
float IO_handler::get_theta2(void) const { return m_theta2.load(std::memory_order_relaxed); }
void IO_handler::set_user_led(bool on) { m_user_led = on ? 1 : 0; }

bool IO_handler::get_and_reset_button_state(void)
{
    // The flag is shared with the debounced interrupt callback.
    core_util_critical_section_enter();
    const bool pressed = m_button_pressed;
    m_button_pressed = false;
    core_util_critical_section_exit();
    return pressed;
}

void IO_handler::button_pressed(void) { m_button_pressed = true; }
