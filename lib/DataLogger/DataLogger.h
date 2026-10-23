#pragma once
#include <cstdint>

#define N_ROW 1000
#define N_COL 7

class DataLogger
{
public:
    // Keep the documented state values; these names are shared by control and UART code.
    enum class State : uint8_t {
        IDLE = 0,
        READY = 1,
        LOGGING = 2,
        DATA_READY = 3,
        SENDING = 4
    };

    DataLogger(uint8_t);
    DataLogger() {};

    virtual ~DataLogger();

    uint16_t N_row;
    uint16_t N_col;
    uint16_t packet;
    float log_data[N_ROW * N_COL];
    State log_status;
    uint8_t input_type; // 1 = step, 2 = sine
    void reset_data();
    bool new_data_available;
    void write_to_log(float ti, float val1, float val2, float val3, float val4, float val5, float val6);
    float get_set_value(float);
    float Amp, omega, offset;
    uint8_t downsamp;

private:
    void init_data();
    float ti_offset;
    uint16_t count;
    float step_sequence(float ti);
    float zigzag(float);
    uint8_t ds_count;
};
