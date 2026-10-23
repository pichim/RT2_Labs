#pragma once

#include <cstdint>

#include "ThreadFlag.h"
#include "mbed.h"
#include "uartProtocolDefinition.h"

class uart_comm_thread_receive
{
public:
    uart_comm_thread_receive(BufferedSerial *, float);
    virtual ~uart_comm_thread_receive();
    void start_uart();

private:
    void loop();
    void read_uart_into_message_buffer();
    void recover_from_read_error(char newByte);
    void check_message_buffer_header();
    int parse_message_buffer_data_size();
    bool parse_message_buffer();
    bool verify_checksum_valid(uint16_t dataSize);

    float Ts;
    char buffer_rx[20]; // RX buffer
    static const uint32_t messageBufLen = 4 * 20;
    char messageBuffer[messageBufLen];
    uint16_t messageBufIndex = 0;

    int16_t currentMessageDataSize = -1;
    bool isCsmErrorFlag = false;
    uint8_t errIx = (UART_ERR_LEN - 1);
    char errBuffer[UART_ERR_LEN];

    // -------------------
    BufferedSerial *uart; // pointer to uart for communication with device
    ThreadFlag threadFlag;
    Thread thread;
    Ticker ticker;
    void send_thread_flag();
};
