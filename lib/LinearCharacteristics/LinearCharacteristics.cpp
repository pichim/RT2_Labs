#include "LinearCharacteristics.h"

using namespace std;

LinearCharacteristics::LinearCharacteristics(float gain, float offset)
{
    init(gain, offset);
}

LinearCharacteristics::LinearCharacteristics(float xmin, float xmax, float ymin, float ymax)
{
    init(xmin, xmax, ymin, ymax);
}

LinearCharacteristics::~LinearCharacteristics() {}

void LinearCharacteristics::init(float gain, float offset)
{
    m_gain = gain;
    m_offset = offset;
    m_ulim = 999999.0f;  // a large number
    m_llim = -999999.0f; // a large neg. number
}

void LinearCharacteristics::init(float xmin, float xmax, float ymin, float ymax)
{
    m_gain = (ymax - ymin) / (xmax - xmin);
    m_offset = xmin - ymin / m_gain;
    m_ulim = 999999.0f;  // a large number
    m_llim = -999999.0f; // a large neg. number
}

float LinearCharacteristics::evaluate(float x)
{
    // calculate result as y(x) = gain * (x - offset)
    float ret_val = m_gain * (x - m_offset);

    // limit result to upper and lower limits
    if (ret_val > m_ulim)
        return m_ulim;
    else if (ret_val < m_llim)
        return m_llim;
    return ret_val;
}

void LinearCharacteristics::set_limits(float ll, float ul)
{
    m_llim = ll;
    m_ulim = ul;
}
