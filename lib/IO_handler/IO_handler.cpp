#include "IO_handler.h"

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f // pi
#endif

// constructor
IO_handler::IO_handler(float Ts)
    : m_encoder(PA_8, PA_9, 4 * 2048)
    , m_a_out(PA_4)
    , m_d_out(PB_1)
    , m_button(PA_10)
    , m_spi(PA_12, PA_11, PA_1)
    , m_imu(m_spi, PB_0)
{
    // attach debounced key press function
    m_button.rise(callback(this, &IO_handler::button_pressed));

    m_d_out = 0;

    // reset encoder
    m_encoder.reset();

    // initialize imu
    m_imu.init_inav();
    m_imu.configuration();

    // initialize linear characteristics
    m_lc_gz2gz.init(-32767.0f, 32768.0f, -1000.0f * M_PIf / 180.0f, 1000.0f * M_PIf / 180.0f);
    // --- AUFGABE 1.1, 1.2 & 1.3 ---
    m_lc_ax2ax.init(-16075.0f, 16813.0f, -9.81f, 9.81f);
    m_lc_ay2ay.init(-17050.0f, 15762.0f, -9.81f, 9.81f);
    m_lc_i2u.init(-15.0f, 15.0f, 0.0f, 1.0f);

    // differentiating low pass filter
    m_fil_diff.differentiatingLowPass1Init(1.0f / (2.0f * M_PIf * 40.0f), Ts);

    // --- AUFGABE 3.1 ---
    // low pass filters for complementary filter
    m_tau = 1.0f;
    m_fil_ax.lowPass1Init(m_tau, Ts);
    m_fil_ay.lowPass1Init(m_tau, Ts);
    m_fil_gz.lowPass1Init(m_tau, Ts);
}

IO_handler::~IO_handler() {}

void IO_handler::update()
{
    // Sample hardware and run filters; only the real-time thread calls update().
    // update encoder readings and calculate speed
    const float phi_fw = m_encoder.getAngleRad();
    const float phi_fw_vel = m_fil_diff(phi_fw);

    // --- AUFGABE 1.1, 1.2 & 1.3 ---
    // apply linear characteristics to imu readings
    const float ax = m_lc_ax2ax(m_imu.readAcc_raw(1));
    const float ay = m_lc_ay2ay(-m_imu.readAcc_raw(0));
    const float gz = m_lc_gz2gz(m_imu.readGyro_raw(2));

    // --- AUFGABE 3.2 ---
    // calculate complementary filter for wheel angle
    // const float phi_bd = atan2f(ax, ay) - M_PIf / 4.0f;
    // const float phi_bd = atan2f(m_fil_ax(ax), m_fil_ay(ay)) - M_PIf / 4.0f;
    const float phi_bd = atan2f(m_fil_ax(ax), m_fil_ay(ay)) - M_PIf / 4.0f + m_tau * m_fil_gz(gz);

    m_phi_fw = phi_fw;
    m_phi_fw_vel = phi_fw_vel;
    m_phi_bd = phi_bd;
    m_ax.store(ax, std::memory_order_relaxed);
    m_ay.store(ay, std::memory_order_relaxed);
    m_gz.store(gz, std::memory_order_relaxed);
}

float IO_handler::get_phi_fw() const { return m_phi_fw; }

float IO_handler::get_phi_bd() const { return m_phi_bd; }

float IO_handler::get_phi_fw_vel() const { return m_phi_fw_vel; }

float IO_handler::get_ax() const { return m_ax.load(std::memory_order_relaxed); }

float IO_handler::get_ay() const { return m_ay.load(std::memory_order_relaxed); }

float IO_handler::get_gz() const { return m_gz.load(std::memory_order_relaxed); }

void IO_handler::write_current(float i_des) { m_a_out = m_lc_i2u(i_des); }

void IO_handler::enable_escon() { m_d_out = 1; }

void IO_handler::disable_escon() { m_d_out = 0; }

bool IO_handler::get_and_reset_button_state()
{
    // The flag is shared with the debounced interrupt callback.
    core_util_critical_section_enter();
    const bool pressed = m_button_pressed;
    m_button_pressed = false;
    core_util_critical_section_exit();
    return pressed;
}

void IO_handler::button_pressed() { m_button_pressed = true; }
