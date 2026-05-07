#ifndef _SERIAL_TRIPLE_H_
#define _SERIAL_TRIPLE_H_

#include <stdint.h>
#include "hardware/uart.h"

#define SERIAL_TRIPLE_MAX_PAYLOAD_SIZE 512
#define SERIAL_TRIPLE_BUFFER_SIZE 512

// Default pins for channel 1 (UART1) — same as original serial.h
#ifndef SERIAL_TX_PIN
#define SERIAL_TX_PIN 20
#endif
#ifndef SERIAL_RX_PIN
#define SERIAL_RX_PIN 21
#endif
#ifndef SERIAL_CTS_PIN
#define SERIAL_CTS_PIN 26
#endif
#ifndef SERIAL_RTS_PIN
#define SERIAL_RTS_PIN 27
#endif

// Default pins for channel 2 (UART0)
#ifndef SERIAL2_TX_PIN
#define SERIAL2_TX_PIN 12
#endif
#ifndef SERIAL2_RX_PIN
#define SERIAL2_RX_PIN 13
#endif
#ifndef SERIAL2_CTS_PIN
#define SERIAL2_CTS_PIN 14
#endif
#ifndef SERIAL2_RTS_PIN
#define SERIAL2_RTS_PIN 15
#endif

typedef bool (*serial_triple_msg_recv_cb_t)(const uint8_t* data, uint16_t len);

struct serial_triple_state_t {
    uart_inst_t* uart;
    char outgoing_buffer[SERIAL_TRIPLE_BUFFER_SIZE];
    uint16_t buf_head;
    uint16_t buf_tail;
    uint16_t buf_items;
    uint8_t rx_buffer[SERIAL_TRIPLE_MAX_PAYLOAD_SIZE + 32];
    uint16_t bytes_read;
    bool escaped;
};

void serial_triple_init_channel(serial_triple_state_t* s, uart_inst_t* uart, unsigned int tx, unsigned int rx, unsigned int cts, unsigned int rts);
bool serial_triple_read_channel(serial_triple_state_t* s, serial_triple_msg_recv_cb_t callback);
bool serial_triple_write_channel(serial_triple_state_t* s, const uint8_t* data, uint16_t len, bool drop_if_blocking = false);
bool serial_triple_write_nonblocking_channel(serial_triple_state_t* s, const uint8_t* data, uint16_t len);

#endif
