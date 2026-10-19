// includes
#include "uart_comm_thread_receive.h"

#include <cstdint>

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
    isCsmErrorFlag = true;
    head_err_cnt += !headerValid;
    if (!headerValid) {
        // m_data->debug_var[2] = (float) receive_cnt;
    }
}

int uart_comm_thread_receive::parseMessageBufferDataSize()
{
    uint16_t dataSize = 256 * messageBuffer[6] + messageBuffer[5];
    dataSize = min(dataSize, (uint16_t)UART_DATA_LEN);
    return dataSize;
}

bool uart_comm_thread_receive::verifyChecksumValid(uint16_t dataSize)
{
    char csm_calc = 0;
    bool csm_valid = false;
    if (dataSize > 0 && dataSize <= UART_DATA_LEN) {
        uint32_t csm_len = UART_HEAD_LEN + dataSize;
        if (csm_len < UART_BUF_LEN) {
            for (uint32_t i = 0; i < csm_len; i++) {
                csm_calc += messageBuffer[i];
            }
            csm_valid = csm_calc == messageBuffer[csm_len];
        }
    }
    return csm_valid;
}

// -----------------------------------------------------------------------------
// analyse data, see comments at top of this file for numbering
bool uart_comm_thread_receive::parseMessageBuffer(int i)
{
    receive_cnt++;
    char msg_id1 = messageBuffer[3];
    char msg_id2 = messageBuffer[4];
    uint16_t N = 256 * messageBuffer[6] + messageBuffer[5];
    float f0, f1, A0, A1;
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
                    if (myDataLogger.log_status == 1) {
                        myDataLogger.reset_data();
                        myDataLogger.input_type = (uint8_t)messageBuffer[7];
                        myDataLogger.Amp = *(float *)&messageBuffer[8];
                        myDataLogger.omega = *(float *)&messageBuffer[12];
                        myDataLogger.offset = *(float *)&messageBuffer[16];
                        myDataLogger.downsamp = (uint8_t)messageBuffer[20];
                        myDataLogger.log_status = 2;
                    }
                    break;
            }
            break; // case 210
        case 250:
            switch (msg_id2) {
                case 101: // start GPA xternally
                    if (true) {
                        myGPA.reset();
                        f0 = *(float *)&messageBuffer[7];
                        f1 = *(float *)&messageBuffer[11];
                        A0 = *(float *)&messageBuffer[15];
                        A1 = *(float *)&messageBuffer[19];
                        Nmeas = (uint8_t)messageBuffer[23];
                        myGPA.setup(f0, f1, (int)Nmeas, A0, A1, myGPA.get_Ts());
                        // sprintf (stri, "Start GPA, N: %d",Nmeas);
                        // send_text((char *)"Started GPA");
                        myGPA.status = 2;
                    }
                    break;
            }
            break; // case 250
    }
    return false;
}
