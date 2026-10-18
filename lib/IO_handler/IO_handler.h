#pragma once

#include <cstdint>

#include "DebounceIn.h"
#include "Encoder.h"
#include "IIR_Filter.h"
#include "LinearCharacteristics.h"
#include "mpu6500_spi.h"

class IO_handler
{
public:
    IO_handler(float Ts);  // default constructor
    virtual ~IO_handler(); // deconstructor

    void update(void);

    float get_phi_fw(void);
    float get_phi_bd(void);
    float get_phi_fw_vel(void);
    float get_ax(void);
    float get_ay(void);
    float get_gz(void);
    void write_current(float);
    void enable_escon();
    void disable_escon();
    bool get_and_reset_button_state(void);

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
    float m_phi_fw;         // motor angle in rad
    float m_phi_bd;         // cube angle in rad
    float m_phi_fw_vel;     // motor speed in rad/sec
    float m_ax, m_ay, m_gz; // accelerations x-y and gyroscope z in m/sec^2 and rad/sec
    void button_pressed(void);
    bool m_button_pressed;
    float m_tau = 0.0f;
    IIR_Filter m_fil_diff;
    IIR_Filter m_fil_ax, m_fil_ay, m_fil_gz;
};
