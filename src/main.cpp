#include "DataLogger.h"
#include "GPA.h"
#include "IO_handler.h"
#include "mbed.h"
#include "realtime_thread.h"
#include "uart_comm_thread_receive.h"
#include "uart_comm_thread_send.h"

// IMPORTANT:
// - Do NOT set the BufferedSerial to non-blocking mode, as this will break proper communication!
// - Switching from Mbed Studio to PlatformIO while running Ts = 200.0e-6f:
//   - observer.cpp: x_hat += Ts / 2.0f * (dxdt + dxdt_old); was unstable, using x_hat += Ts * dxdt; fixed this

// float Ts = 1.0f / 500.0f;
// float Ts = 1.0f / 5.0e3f;
float Ts = 1.0f / 10.0e3f;                    // 10 kHz, 20 kHz was not working with GPA + UART communication
GPA myGPA(1.0f, 1000.0f, 30, 0.1f, 0.2f, Ts); // setup here does not affect the actual used parameters, they are set via
                                              // the UART communication via MATLAB
DataLogger myDataLogger(1);

int main()
{
    // Input/Output handling
    IO_handler io_handler;

    // Communication
    BufferedSerial uart_serial(USBTX, USBRX, 115200);                   // leave this blocking!
    uart_comm_thread_send uart_com_send(&io_handler, &uart_serial, .01f); // send communication thread
    uart_comm_thread_receive uart_com_receive(&uart_serial, .01f);      // receive communication thread

    // Real-Time Thread
    realtime_thread rt_thread(&io_handler, Ts);

    // Start the three threads
    uart_com_receive.start_uart();
    uart_com_send.start_uart();
    rt_thread.start_loop();

    while (true) {
        ThisThread::sleep_for(500ms);
    }
}
