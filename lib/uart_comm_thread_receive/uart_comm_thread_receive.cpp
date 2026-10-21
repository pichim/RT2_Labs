// includes
#include "uart_comm_thread_receive.h"

#include <cstdint>
#include <cstring>
#include <cmath>

extern DataLogger myDataLogger;
extern GPA myGPA;

using namespace std::chrono;

// #### constructor
uart_comm_thread_receive::uart_comm_thread_receive(BufferedSerial *com, float Ts)
    : thread(osPriorityBelowNormal1, 512 * 2) // thread(osPriorityHigh1, 1024)//
{
    // init serial
    this->uart = com;
    this->Ts = Ts;
}

uart_comm_thread_receive::~uart_comm_thread_receive() {}

void uart_comm_thread_receive::start_uart(void)
{
    thread.start(callback(this, &uart_comm_thread_receive::loop));
    ticker.attach(callback(this, &uart_comm_thread_receive::sendThreadFlag), microseconds{static_cast<int64_t>(Ts * 1e6f)});
}

void uart_comm_thread_receive::sendThreadFlag() { thread.flags_set(threadFlag); }

void uart_comm_thread_receive::loop(void)
{
    while (true) {
        ThisThread::flags_wait_any(threadFlag);
        readUartIntoSeparateMessageBuffer();
    }
}

void uart_comm_thread_receive::recoverFromReadError(char newByte)
{
    if (isCsmErrorFlag) {
        recovery_count++;
        char prevByte0 = errBuffer[errIx];
        bool isTerminatorReceived = (prevByte0 == UART_TERM_BY0 && newByte == UART_TERM_BY1);
        if (isTerminatorReceived) {
            messageBufIndex = 0;
            currentMessageDataSize = -1;
            isCsmErrorFlag = false;
            recovery_count = 0;
        } else {
            char prevByte1 = errBuffer[(errIx + UART_ERR_LEN - 1) % UART_ERR_LEN];
            bool isStartReceived =
                (prevByte1 == UART_HEAD_BY0 && prevByte0 == UART_HEAD_BY1 && newByte == UART_HEAD_BY2);
            if (isStartReceived) {
                messageBuffer[0] = UART_HEAD_BY0;
                messageBuffer[1] = UART_HEAD_BY1;
                messageBuffer[2] = UART_HEAD_BY2;
                messageBufIndex = 3;
                currentMessageDataSize = -1;
                isCsmErrorFlag = false;
                recovery_count = 0;
            }
        }
    }
    errIx = (errIx + 1) % UART_ERR_LEN;
    errBuffer[errIx] = newByte;
}

void uart_comm_thread_receive::readUartIntoSeparateMessageBuffer()
{
    while (uart->readable()) {
        bufLengthLLL = bufLengthLL;
        bufLengthLL = bufLengthLast;
        bufLengthLast = bufLength;
        bufLength = uart->read(buffer_rx, sizeof(buffer_rx));
        // if(bufLength==30){
        // 	bool fault = buffer_rx[0] == UART_HEAD_BY0 &&
        // 					buffer_rx[1] == UART_HEAD_BY1 &&
        // 					buffer_rx[2] == UART_HEAD_BY2 &&
        // 					buffer_rx[26] == 0 &&
        // 					buffer_rx[27] == 253;
        // 	if(fault){
        // 		m_data->debug_var[4] = msgCount;
        // 	}
        // }
        for (int i = 0; i < bufLength; i++) {
            messageBuffer[messageBufIndex] = buffer_rx[i]; // copy to separate buffer
            byte_count++;
            if (!isCsmErrorFlag) {
                // Read Header
                if (messageBufIndex == 2) {
                    checkMessageBufferHeader();
                } else if (messageBufIndex == 6) {
                    currentMessageDataSize = parseMessageBufferDataSize();
                }

                // Read Completed Message
                bool isMessageLengthReached =
                    messageBufIndex >= (currentMessageDataSize + UART_HEAD_LEN + UART_TAIL_LEN - 1) &&
                    currentMessageDataSize >= 0;
                if (isMessageLengthReached) {
                    parseMessageBuffer(i); // parse message
                    msgCount++;
                    messageBufIndex =
                        messageBufLen - 1; // reset message buffer index to start (waiting for next message)
                    currentMessageDataSize = -1;
                }
            }
            messageBufIndex = (messageBufIndex + 1) % messageBufLen;
            // Recovery
            recoverFromReadError(buffer_rx[i]);
        }
    }
}

void uart_comm_thread_receive::checkMessageBufferHeader()
{
    bool headerValid =
        messageBuffer[0] == UART_HEAD_BY0 && messageBuffer[1] == UART_HEAD_BY1 && messageBuffer[2] == UART_HEAD_BY2;
    isCsmErrorFlag = !headerValid;
    head_err_cnt += !headerValid;
    if (!headerValid) {
        // m_data->debug_var[2] = (float) receive_cnt;
    }
}

int uart_comm_thread_receive::parseMessageBufferDataSize()
{
    uint16_t dataSize = (static_cast<uint8_t>(messageBuffer[6]) << 8)
        | static_cast<uint8_t>(messageBuffer[5]);
    if (dataSize > messageBufLen - UART_HEAD_LEN - UART_TAIL_LEN) {
        isCsmErrorFlag = true;
        return -1;
    }
    return dataSize;
}

bool uart_comm_thread_receive::verifyChecksumValid(uint16_t dataSize)
{
    char csm_calc = 0;
    bool csm_valid = false;
    if (dataSize > 0 && dataSize <= messageBufLen - UART_HEAD_LEN - UART_TAIL_LEN) {
        uint32_t csm_len = UART_HEAD_LEN + dataSize;
        if (csm_len < UART_BUF_LEN) {
            for (uint32_t i = 0; i < csm_len; i++) {
                csm_calc += messageBuffer[i];
            }
            csm_valid = csm_calc == messageBuffer[csm_len]
                && messageBuffer[csm_len + 1] == UART_TERM_BY0
                && messageBuffer[csm_len + 2] == UART_TERM_BY1;
        }
    }
    return csm_valid;
}

// -----------------------------------------------------------------------------
// analyse data, see comments at top of this file for numbering
bool uart_comm_thread_receive::parseMessageBuffer(int i)
{
    receive_cnt++;
    uint8_t msg_id1 = static_cast<uint8_t>(messageBuffer[3]);
    uint8_t msg_id2 = static_cast<uint8_t>(messageBuffer[4]);
    uint16_t N = (static_cast<uint8_t>(messageBuffer[6]) << 8)
        | static_cast<uint8_t>(messageBuffer[5]);
    float f0, f1, A0, A1;
    float amp, omega, offset;
    // char stri[20];
    uint8_t Nmeas;
    if (!verifyChecksumValid(N)) {
        isCsmErrorFlag = true;
        csm_err_cnt++;
        // m_data->debug_var[0] = (float) buffer_rx[0];
        // m_data->debug_var[1] = (float) messageBuffer[0];
        return false;
    }
    switch (msg_id1) {
        case 210:
            switch (msg_id2) {
                case 101: // receive start signal and Amp, Freq, offset values
                    if (N != 14 || static_cast<uint8_t>(messageBuffer[7]) < 1
                        || static_cast<uint8_t>(messageBuffer[7]) > 3 || static_cast<uint8_t>(messageBuffer[20]) == 0)
                        return false;
                    memcpy(&amp, &messageBuffer[8], sizeof(float));
                    memcpy(&omega, &messageBuffer[12], sizeof(float));
                    memcpy(&offset, &messageBuffer[16], sizeof(float));
                    if (!std::isfinite(amp) || !std::isfinite(omega) || !std::isfinite(offset) || omega <= 0.0f)
                        return false;
                    if (myDataLogger.log_status == 1) {
                        myDataLogger.reset_data();
                        myDataLogger.input_type = (uint8_t)messageBuffer[7];
                        myDataLogger.Amp = amp;
                        myDataLogger.omega = omega;
                        myDataLogger.offset = offset;
                        myDataLogger.downsamp = (uint8_t)messageBuffer[20];
                        myDataLogger.log_status = 2;
                    }
                    break;
            }
            break; // case 210
        case 250:
            switch (msg_id2) {
                case 101: // four floats and number of frequency points; no motor/type selection
                    if (N != 17)
                        return false;
                    memcpy(&f0, &messageBuffer[7], sizeof(float));
                    memcpy(&f1, &messageBuffer[11], sizeof(float));
                    memcpy(&A0, &messageBuffer[15], sizeof(float));
                    memcpy(&A1, &messageBuffer[19], sizeof(float));
                    Nmeas = static_cast<uint8_t>(messageBuffer[23]);
                    if (myGPA.setup(f0, f1, (int)Nmeas, A0, A1, myGPA.get_Ts()))
                        myGPA.status = 2;
                    break;
            }
            break; // case 250
    }
    return false;
}
