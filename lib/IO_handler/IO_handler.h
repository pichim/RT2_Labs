#pragma once

#include <atomic>
#include <cstdint>

#include "DebounceIn.h"
#include "Encoder.h"
#include "FastPWM.h"
#include "mbed.h"

//  1 Encoder Motor A            PA_6  - ok
//  2 Encoder Motor B            PC_7  - ok
//  3 Encoder Pendel A           PB_6  - ok
//  4 Encoder Pendel B           PB_7  - ok
//  5 PWM Motor                  PB_15 - ok
//  6 DIR Motor                  PB_14 - ok
//  7 Enable Motor               PB_9  - ok
//  8 Strommessung Motor         PA_7  - ok
//  9 Fehler Motor (Statusflag)  PB_8  - untested, TODO: Make use of this flag
// 10 Real-time thread DO        PB_4  - ok

#define ENCODER_MOTOR_COUNTS_PER_TURN (4 * 4096)    // 4096 PPR encoder with x4 decoding
#define ENCODER_PENDULUM_COUNTS_PER_TURN (4 * 1024) // 1024 PPR encoder with x4 decoding
#define MOTOR_PWM_PERIOD_US 50                      // 20 kHz PWM

class IO_handler
{
public:
    IO_handler();
    virtual ~IO_handler();
    void update(void); // Sample hardware once in realtime_thread.
    float read_encoder_motor(void);
    float read_encoder_pendulum(void);
    void reset_encoders(void);
    void write_pwm_motor(float val);
    float get_set_value(void) const;
    void set_enable_motor(bool en);
    float read_current(void);
    void set_dir(uint8_t d);
    void set_enable_rtt_do(bool en);

    // Cached telemetry; UART must not read/update the encoder counters itself.
    float get_current(void) const;
    float get_theta1(void) const;
    float get_theta2(void) const;
    void set_user_led(bool on);
    bool get_and_reset_button_state(void);

private:
    DebounceIn m_button;
    DigitalOut m_user_led;
    volatile bool m_button_pressed{false};
    // Individual UART telemetry values are shared with the UART sender.
    std::atomic<float> m_current{0.0f}, m_theta1{0.0f}, m_theta2{0.0f};
    void button_pressed(void);

    Encoder m_encoder_motor;    // Motor encoder
    Encoder m_encoder_pendulum; // Pendulum encoder
    FastPWM m_pwm;              // PWM for motor
    float m_pwm_val;            // stored PWM value
    DigitalOut m_dir;           // Direction for motor
    DigitalOut m_enable;        // Enable power h-bridge
    AnalogIn m_current_sensor;  // Current sensing
    DigitalIn m_fault;          // Fault status flag
    DigitalOut m_rtt_do;        // Real-time thread debug output
};
