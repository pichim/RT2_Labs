#pragma once

#include <cstdint>
#include <stdint.h>

#include "AvgFilter.h"
#include "IO_handler.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "uartProtocolDefinition.h"

#define Navg 100

using namespace std;

class uart_comm_thread_send
{
public:
    // public members
    uart_comm_thread_send(IO_handler *, BufferedSerial *, float);
    virtual ~uart_comm_thread_send();
    void loop(void); // runs the statemachine, call this function periodicly, returns true when new data ready (only
                     // true for 1 cycle)
    void start_uart(void);
    void send_text(const char *);

private:
    void sendCmd(char); // sends comand to device
    float Ts;
    void send(uint8_t, uint8_t, uint16_t, char *);
    void send_slow_data(void);
    void send_essential_fast_data(void);
    void send_maxspeed_fast_data(void);
    void send_debug_data(void);
    void send_gpa_data(void);
    bool gpa_stop_sent;
    uint8_t subsamp_1;
    char buffer_tx[20];
    uint16_t send_state_slow;

    // -------------------
    BufferedSerial *uart; // pointer to uart for communication with device
    ThreadFlag threadFlag;
    Thread thread;
    Ticker ticker;
    void sendThreadFlag();
    IO_handler *m_io;
    AvgFilter avg_filter[3];
    bool is_first_avg = true;
};
