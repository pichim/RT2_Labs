#include "DataLogger.h"

#include "math.h"
#define PI 3.14159265358979323846f

DataLogger::DataLogger(uint8_t dum)
{
    this->N_row = N_ROW;
    this->N_col = N_COL;
    this->Amp = 0.0f;
    this->omega = 0.0f;
    this->offset = 0.0f;
    this->ds_count = 0;
    this->downsamp = 1;
    init_data();
}

DataLogger::~DataLogger() {}

void DataLogger::init_data(void)
{
    for (uint16_t k = 0; k < this->N_row; k++)
        for (uint16_t j = 0; j < N_col; j++)
            this->log_data[k * N_col + j] = 0.0f; //(float)k*.1f;
    this->log_status = 1; // 0 = IDLE, 1 = ready, wait for trigger, 2 = logging, 3 = init_sending, 4 = sending
    this->input_type = 1;
    this->count = 0;
    this->ds_count = 0;
    new_data_available = false;
    ti_offset = 0.0f;
    packet = 0;
}

void DataLogger::reset_data(void)
{
    for (uint16_t k = 0; k < N_row; k++)
        for (uint16_t j = 0; j < N_col; j++)
            this->log_data[k * N_col + j] = 0.0f; // 1000*(float)j+(float)k;
    this->count = 0;
    this->ds_count = 0;
    new_data_available = false;
}

void DataLogger::write_to_log(float ti, float val1, float val2, float val3, float val4, float val5, float val6)
{
    if (log_status == 2) {
        if (++ds_count == downsamp) {
            ds_count = 0;
            log_data[count * N_col + 0] = ti - ti_offset;
            log_data[count * N_col + 1] = val1;
            log_data[count * N_col + 2] = val2;
            log_data[count * N_col + 3] = val3;
            log_data[count * N_col + 4] = val4;
            log_data[count * N_col + 5] = val5;
            log_data[count * N_col + 6] = val6;
            count++;
            if (count >= N_row) {
                log_status = 3;
                new_data_available = true;
                packet = 0;
            }
        }
    }
}

float DataLogger::get_set_value(float ti)
{
    // float output = 0;
    if (log_status == 2) // run here, if logging is active
    {
        if (count == 0)
            ti_offset = ti;
        switch (input_type) {
            case 1: // return a sequence of steps (see few lines below)
                return StepSeq(ti - ti_offset);
                break;
            case 2: // return sine wave
                return Amp * sinf((ti - ti_offset) * omega) + offset;
                break;
            case 3:
                return ZigZag(ti - ti_offset); // return zigzag
                break;
            default:
                return 0.0f;
                break;
        }
    }
    return 0.0f;
}

float DataLogger::StepSeq(float ti)
{
    float phi = ti * omega / PI;
    uint16_t fphi = (uint16_t)floor(phi) % 2;
    if (fphi == 0)
        return Amp + offset;
    else
        return -Amp + offset;
}

// return a zig-zag like set value, the parameters are Amplitude, frequency and offset, these are set from the uart
// communication
float DataLogger::ZigZag(float ti)
{
    float phi = ti * omega / (2.0f * PI);
    float fphi = (float)floor(phi);
    float phi_ = phi - fphi;
    // Create a symmetric triangle wave in [-1, 1] and center it at offset
    float tri = (phi_ <= 0.5f) ? (4.0f * phi_ - 1.0f) : (-4.0f * phi_ + 3.0f);
    return offset + Amp * tri;
}
