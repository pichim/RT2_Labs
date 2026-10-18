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
    void start_loop(void);

private:
    IO_handler *m_IO_handler;
    void loop(void);
    Timer m_Timer;
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    void sendSignal();
    float saturate(float, float, float);
    float m_Ts;
};
