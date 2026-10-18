#pragma once

#include <cstdint>

#include "LinearCharacteristics.h"
#include "mbed.h"

class IO_handler
{
public:
    IO_handler();          // default constructor
    virtual ~IO_handler(); // deconstructor

    float read_ain1(void);  // read analog input 1
    float read_ain2(void);  // read analog input 2
    void write_aout(float); // write analog output
    float get_set_value();
    void write_dout(bool);

private:
    AnalogOut m_a_out;
    AnalogIn m_a_in1;
    AnalogIn m_a_in2;
    DigitalOut m_d_out;
    float m_set_value;
    // --- P1, AUFGABE 1.6 & 1.7 ---
    // add linear characteristics for input and output
    LinearCharacteristics m_lc_out, m_lc_in;
};
