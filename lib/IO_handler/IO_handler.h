#pragma once

#include <atomic>
#include <cstdint>

#include "DebounceIn.h"
#include "Encoder.h"
#include "IIR_Filter.h"
#include "LinearCharacteristics.h"
#include "mbed.h"
#include "mpu6500_spi.h"

class IO_handler
{
public:
    IO_handler(float Ts);  // default constructor
    virtual ~IO_handler(); // deconstructor

    void update();

    float get_phi_fw() const;
    float get_phi_bd() const;
    float get_phi_fw_vel() const;
    float get_ax() const;
    float get_ay() const;
    float get_gz() const;
    void write_current(float);
    void enable_escon();
    void disable_escon();
    bool get_and_reset_button_state();

private:
    Encoder m_encoder;
    AnalogOut m_a_out;
    DigitalOut m_d_out;
    DebounceIn m_button;
    SPI m_spi; // mosi, miso, sclk
    mpu6500_spi m_imu;
    LinearCharacteristics m_lc_i2u;
    LinearCharacteristics m_lc_ax2ax, m_lc_ay2ay, m_lc_gz2gz; // map imu raw values to m/s^2 and rad/s
    // sensor states
    float m_phi_fw = 0.0f;     // motor angle in rad
    float m_phi_bd = 0.0f;     // cube angle in rad
    float m_phi_fw_vel = 0.0f; // motor speed in rad/sec
    // Shared with the UART sender: accelerations [m/s^2] and gyroscope [rad/s].
    std::atomic<float> m_ax{0.0f}, m_ay{0.0f}, m_gz{0.0f};
    void button_pressed();
    volatile bool m_button_pressed = false;
    float m_tau = 0.0f;
    IIR_Filter m_fil_diff;
    IIR_Filter m_fil_ax, m_fil_ay, m_fil_gz;
};
