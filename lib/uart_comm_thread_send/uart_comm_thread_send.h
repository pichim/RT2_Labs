#pragma once

#include <cstdint>

#include "AvgFilter.h"
#include "IO_handler.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "uartProtocolDefinition.h"

#define Navg 100

class uart_comm_thread_send
{
public:
    // public members
    uart_comm_thread_send(IO_handler *, BufferedSerial *, float);
    virtual ~uart_comm_thread_send();
    void loop(); // runs the statemachine, call this function periodicly, returns true when new data ready (only
                 // true for 1 cycle)
    void start_uart();
    void send_text(const char *);

private:
    float Ts;
    void send(uint8_t, uint8_t, uint16_t, const char *);
    void send_slow_data();
    void send_gpa_data();
    bool gpa_stop_sent;
    char buffer_tx[20];
    uint16_t send_state_slow;

    // -------------------
    BufferedSerial *uart; // pointer to uart for communication with device
    ThreadFlag threadFlag;
    Thread thread;
    Ticker ticker;
    void send_thread_flag();
    IO_handler *m_io;
    AvgFilter avg_filter[3];
    bool is_first_avg = true;
};
