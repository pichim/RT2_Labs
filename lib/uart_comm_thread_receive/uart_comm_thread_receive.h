#pragma once

#include <cstdint>
#include <stdint.h>

#include "DataLogger.h"
#include "GPA.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "uartProtocolDefinition.h"
#include "uart_comm_thread_send.h"

using namespace std;

class uart_comm_thread_receive
{
public:
    uart_comm_thread_receive(BufferedSerial *, float);
    virtual ~uart_comm_thread_receive();
    void start_uart(void);
    uart_comm_thread_send *m_se;

private:
    void loop(void);
    void readUartIntoSeparateMessageBuffer(void);
    void recoverFromReadError(char newByte);
    void checkMessageBufferHeader(void);
    int parseMessageBufferDataSize(void);
    bool parseMessageBuffer(int i);
    bool verifyChecksumValid(uint16_t dataSize);

    float Ts;
    char buffer_rx[20]; // RX buffer
    static const uint32_t messageBufLen = 4 * 20;
    char messageBuffer[messageBufLen];
    uint16_t messageBufIndex = 0;

    uint32_t receive_cnt = 0;
    uint32_t csm_err_cnt = 0;
    uint32_t head_err_cnt = 0;
    uint32_t msgCount = 0;
    int16_t currentMessageDataSize = -1;
    int bufLength = 0;
    int bufLengthLast = 0;
    int bufLengthLL = 0;
    int bufLengthLLL = 0;
    uint32_t byte_count = 0;
    bool isCsmErrorFlag = false;
    uint32_t recovery_count = 0;
    uint8_t errIx = (UART_ERR_LEN - 1);
    char errBuffer[UART_ERR_LEN];

    // -------------------
    BufferedSerial *uart; // pointer to uart for communication with device
    ThreadFlag threadFlag;
    Thread thread;
    Ticker ticker;
    void sendThreadFlag();
};
