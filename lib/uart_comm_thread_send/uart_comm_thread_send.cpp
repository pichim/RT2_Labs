// includes
#include "uart_comm_thread_send.h"

#include <cstdint>

#include "DataLogger.h"
#include "GPA.h"

extern DataLogger myDataLogger;
extern GPA myGPA;

using namespace std::chrono;

// #### constructor
uart_comm_thread_send::uart_comm_thread_send(IO_handler *io, BufferedSerial *com, float Ts)
    : thread(osPriorityBelowNormal, 2 * 512)
{
    // init serial
    this->uart = com;
    this->Ts = Ts;
    gpa_stop_sent = false;
    this->m_io = io;
}

// #### destructor
uart_comm_thread_send::~uart_comm_thread_send() {}

// #### run the statemachine
void uart_comm_thread_send::loop(void)
{
    send_state_slow = 100;

    while (true) // loop, latest Ts = 1ms
    {
        ThisThread::flags_wait_any(threadFlag);
        //---  The LOOP --------------------------------------------------------
        send_slow_data();

    } // loop
}

void uart_comm_thread_send::send_gpa_data(void)
{
    float dum[8];
    if (myGPA.new_data_available) {
        myGPA.getGPAdata(dum);
        send(250, 1, 32, (char *)&(dum[0])); // send new values (8 floats)
    } else if (myGPA.start_now) {
        char dum = 0;
        send(250, 2, 1, &dum); // send start flag
        myGPA.start_now = false;
        gpa_stop_sent = false;
    } else if (myGPA.meas_is_finished && !gpa_stop_sent && !myGPA.new_data_available) {
        char dum = 0;
        send(250, 255, 1, &dum); // send stop flag
        gpa_stop_sent = true;
    } else {
        char dum = myGPA.status;
        send(250, 3, 1, &dum); // send start flag
    }
}

void uart_comm_thread_send::send_slow_data(void)
{
    // char local_buffer[5] = {0, 0, 0, 0, 0};
    float buf[3];
    // char str[30];
    switch (send_state_slow) {
        case 100: // only at startup
            send_text((char *)"RCRC Mbed firmware started");
            send_state_slow = 115;
            break;
        case 115:
            buf[0] = m_io->get_set_value();
            buf[1] = m_io->read_ain1();
            buf[2] = m_io->read_ain2();
            send(115, 1, 12, (char *)&buf[0]);
            send_state_slow = 210;
            break;
        case 210: // stream datalogger content in chunks
            if (myDataLogger.new_data_available) {
                const uint32_t totalBytes = 4 * myDataLogger.N_col * myDataLogger.N_row;
                const uint32_t sentBytes = myDataLogger.packet * PACK_SIZE;
                uint32_t remaining = (sentBytes < totalBytes) ? (totalBytes - sentBytes) : 0;

                if (remaining == 0) {
                    myDataLogger.log_status = 1;
                    myDataLogger.new_data_available = false;
                    send_state_slow = 211; // send terminator next
                    break;
                }

                const uint16_t chunk = (remaining > PACK_SIZE) ? PACK_SIZE : (uint16_t)remaining;
                uint8_t id2 = 1 + myDataLogger.packet;
                if (id2 == 99) // reserve 99 for terminator
                    id2 = 100;
                else if (id2 > 99)
                    id2 += 1; // shift up once we passed the gap

                send(210, id2, chunk, (char *)&(myDataLogger.log_data[sentBytes / 4]));

                ++myDataLogger.packet;

                if (remaining <= PACK_SIZE) {
                    myDataLogger.log_status = 1;
                    myDataLogger.new_data_available = false;
                    send_state_slow = 211; // send terminator next
                }
            } else
                send_state_slow = 250;
            break;
        case 211:
            send(210, 99, 0, buffer_tx);
            send_state_slow = 115;
            break;
        case 250: // send GPA values
            send_gpa_data();
            send_state_slow = 115;
            break;
        default:
            send_state_slow = 115;
            break;
    }
}

// ------------------- start uart ----------------
void uart_comm_thread_send::start_uart(void)
{

    thread.start(callback(this, &uart_comm_thread_send::loop));
    ticker.attach(callback(this, &uart_comm_thread_send::sendThreadFlag), microseconds{static_cast<int64_t>(Ts * 1e6f)});
}

// this is for realtime OS
void uart_comm_thread_send::sendThreadFlag() { thread.flags_set(threadFlag); }

// ---------------------  send N char data --------------------------------------
void uart_comm_thread_send::send(uint8_t id1, uint8_t id2, uint16_t N, char *m)
{
    char buffer[7], csm_tail[3] = {0, '\r', '\n'};
    /* Add header */
    buffer[0] = 254;
    buffer[1] = 1;
    buffer[2] = 255;
    /* Add message IDs*/
    buffer[3] = id1;
    buffer[4] = id2;
    /* Add number of bytes*/
    *(uint16_t *)&buffer[5] = N; // cast targt to appropriate data type
                                 /* send header */
    uart->write(buffer, 7);
    for (int i = 0; i < 7; ++i)
        csm_tail[0] += buffer[i];
    /* send data */
    if (N > 0)
        uart->write(m, N);
    for (uint16_t i = 0; i < N; ++i)
        csm_tail[0] += m[i];
    uart->write(&csm_tail, 3);
}

void uart_comm_thread_send::send_text(const char *txt)
{
    uint16_t N = 0;
    char buffer[40];
    while (txt[N] != 0) // get length of text
        N++;
    buffer[0] = 254;
    buffer[1] = 1;
    buffer[2] = 255; // standard pattern
    buffer[3] = 241;
    buffer[4] = 1;
    buffer[5] = N % 256;
    buffer[6] = N / 256;
    uart->write(buffer, 7);
    uart->write(txt, N);
    char dum = 0;
    uart->write(&dum, 1); // line end
}
