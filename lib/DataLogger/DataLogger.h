#pragma once
#include <stdint.h>

#define N_ROW 1000
#define N_COL 7

class DataLogger
{
public:
    DataLogger(uint8_t);
    DataLogger() {};

    virtual ~DataLogger();

    void start_logging(void);
    uint16_t N_row;
    uint16_t N_col;
    uint16_t packet;
    float log_data[N_ROW * N_COL];
    uint8_t log_status; // 0 = IDLE, 1 = ready, wait for trigger, 2 = start_logging, 3 = init_sending, 4 = sending
    uint8_t input_type; // 1 = step, 2 = sine
    void reset_data(void);
    bool new_data_available;
    void write_to_log(float ti, float val1, float val2, float val3, float val4, float val5, float val6);
    float get_set_value(float);
    float Amp, omega, offset;
    uint8_t downsamp;

private:
    void init_data(void);
    float ti_offset;
    uint16_t count;
    float StepSeq(float ti);
    float ZigZag(float);
    uint8_t ds_count;
};
