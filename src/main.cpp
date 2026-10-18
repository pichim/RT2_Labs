#include "DataLogger.h"
#include "GPA.h"
#include "IO_handler.h"
#include "mbed.h"
#include "realtime_thread.h"
#include "uart_comm_thread_receive.h"
#include "uart_comm_thread_send.h"

// IMPORTANT:
// - Do NOT set the BufferedSerial to non-blocking mode, as this will break proper communication!

float Ts = 1.0f / 1.0e3f;
GPA myGPA(1.0f, 245.0f, 30, 0.1f, 0.2f, Ts); // setup here does not affect the actual used parameters, they are set via
                                             // the UART communication via MATLAB
DataLogger myDataLogger(1);

int main()
{
    // Input/Output handling
    IO_handler io_handler(Ts);

    // Communication
    BufferedSerial uart_serial(USBTX, USBRX, 115200);                     // leave this blocking!
    uart_comm_thread_send uart_com_send(&io_handler, &uart_serial, .01f); // send communication thread
    uart_comm_thread_receive uart_com_receive(&uart_serial, .01f);        // receive communication thread

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
