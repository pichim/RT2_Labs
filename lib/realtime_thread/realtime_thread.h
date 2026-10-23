#pragma once

#include <Eigen/Dense>

#include "IO_handler.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "observer.h"

class realtime_thread
{
public:
    realtime_thread(IO_handler *, float Ts);
    virtual ~realtime_thread();
    void start_loop();

private:
    IO_handler *m_IO_handler;
    void loop();
    Timer m_Timer;
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    void send_thread_flag();
    float saturate(float, float, float);
    float m_Ts;
};
