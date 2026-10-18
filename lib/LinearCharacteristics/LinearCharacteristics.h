#pragma once

class LinearCharacteristics
{
public:
    LinearCharacteristics() {};          // default constructor
    LinearCharacteristics(float, float); // constructor with gain and offset
    // --- P1, AUFGABE 1.5 ---
    // constructor with min/max values
    LinearCharacteristics(float, float, float, float); // constructor with min/max values
    virtual ~LinearCharacteristics();                  // deconstructor

    float operator()(float x) { return evaluate(x); }

    void init(float, float);               // set gain and offset
    void init(float, float, float, float); // set gain and offset based on min/max values
    float evaluate(float);                 // calculate y(x)
    void set_limits(float, float);

private:
    float m_gain{0.0f};
    float m_offset{0.0f};
    float m_ulim{0.0f};
    float m_llim{0.0f};
};
