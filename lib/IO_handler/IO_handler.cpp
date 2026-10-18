#include "IO_handler.h"

#include <cstdint>

// constructors
IO_handler::IO_handler(void)
    : m_a_out(PA_5)
    , m_a_in1(PA_6)
    , m_a_in2(PA_7)
    , m_d_out(PC_8)
{
    m_a_out.write(0.0f);
    m_set_value = 0.0f;
    m_lc_in.init(0.0f, 1.0f, -1.0f, 1.0f);
    m_lc_out.init(-1.0f, 1.0f, 0.0f, 1.0f);
}

IO_handler::~IO_handler() {}

// --- P1, AUFGABE 1.7 ---
// apply linear characteristics to the read values of the analog inputs
// float IO_handler::read_ain1(void) { return m_a_in1.read(); }

// float IO_handler::read_ain2(void) { return m_a_in2.read(); }

float IO_handler::read_ain1(void) { return m_lc_in(m_a_in1.read()); }

float IO_handler::read_ain2(void) { return m_lc_in(m_a_in2.read()); }

void IO_handler::write_aout(float output)
{
    m_set_value = output;

    // --- P1, AUFGABE 1.6 ---
    // apply linear characteristics to the output value before writing it to the analog output
    // m_a_out.write(m_set_value);
    m_a_out.write(m_lc_out(m_set_value));
}

float IO_handler::get_set_value() { return m_set_value; }

void IO_handler::write_dout(bool val) { m_d_out = val; }
