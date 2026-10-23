#pragma once

#include <Eigen/Dense>

#include "IIR_Filter.h"
#include "IO_handler.h"
#include "ThreadFlag.h"
#include "mbed.h"

class realtime_thread
{
public:
    realtime_thread(IO_handler *, float Ts);
    virtual ~realtime_thread();
    void start_loop();

private:
    enum ControlState {
        INIT,
        FLAT,
        BALANCE
    };

    IO_handler *m_IO_handler;
    void loop();
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    Timer m_Timer;
    float m_Ts;
    ControlState m_state;
    void send_thread_flag();
    float saturate(float, float, float);
};
