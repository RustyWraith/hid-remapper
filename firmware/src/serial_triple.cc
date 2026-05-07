#include "hardware/gpio.h"
#include "hardware/uart.h"

#include "pico/stdio.h"
#include "stdio.h"

#include "crc.h"
#include "serial_triple.h"

#define SERIAL_TRIPLE_BAUDRATE 4000000

#define END 0300     /* indicates end of packet */
#define ESC 0333     /* indicates byte stuffing */
#define ESC_END 0334 /* ESC ESC_END means END data byte */
#define ESC_ESC 0335 /* ESC ESC_ESC means ESC data byte */

void serial_triple_init_channel(serial_triple_state_t* s, uart_inst_t* uart, unsigned int tx, unsigned int rx, unsigned int cts, unsigned int rts) {
    s->uart = uart;
    s->buf_head = 0;
    s->buf_tail = 0;
    s->buf_items = 0;
    s->bytes_read = 0;
    s->escaped = false;
    uart_inst_t* u = uart;
    uart_init(u, SERIAL_TRIPLE_BAUDRATE);
    uart_set_hw_flow(u, true, true);
    uart_set_translate_crlf(u, false);
    gpio_set_function(tx, GPIO_FUNC_UART);
    gpio_set_function(rx, GPIO_FUNC_UART);
    gpio_set_function(cts, GPIO_FUNC_UART);
    gpio_set_function(rts, GPIO_FUNC_UART);
}

bool serial_triple_read_channel(serial_triple_state_t* s, serial_triple_msg_recv_cb_t callback) {
    uart_inst_t* u = s->uart;
    while ((s->buf_items > 0) && uart_is_writable(u)) {
        uart_putc_raw(u, s->outgoing_buffer[s->buf_head]);
        s->buf_head = (s->buf_head + 1) % SERIAL_TRIPLE_BUFFER_SIZE;
        s->buf_items--;
    }

    while (uart_is_readable(u)) {
        s->bytes_read %= sizeof(s->rx_buffer);

        char c = uart_getc(u);

        if (s->escaped) {
            switch (c) {
                case ESC_END:
                    s->rx_buffer[s->bytes_read++] = END;
                    break;
                case ESC_ESC:
                    s->rx_buffer[s->bytes_read++] = ESC;
                    break;
                default:
                    // this shouldn't happen
                    s->rx_buffer[s->bytes_read++] = c;
                    break;
            }
            s->escaped = false;
        } else {
            switch (c) {
                case END:
                    if (s->bytes_read > 4) {
                        uint32_t crc = crc32(s->rx_buffer, s->bytes_read - 4);
                        uint32_t received_crc = 0;
                        for (int i = 0; i < 4; i++) {
                            received_crc = (received_crc << 8) | s->rx_buffer[s->bytes_read - 1 - i];
                        }
                        if (crc == received_crc) {
                            bool ret = callback(s->rx_buffer, s->bytes_read - 4);
                            s->bytes_read = 0;
                            return ret;
                        } else {
                            printf("CRC error\n");
                        }
                    }
                    s->bytes_read = 0;
                    break;
                case ESC:
                    s->escaped = true;
                    break;
                default:
                    s->rx_buffer[s->bytes_read++] = c;
                    break;
            }
        }
    }

    return false;
}

static void my_putc_triple(serial_triple_state_t* s, char c) {
    uart_inst_t* u = s->uart;
    if ((s->buf_items == 0) && uart_is_writable(u)) {
        uart_putc_raw(u, c);
    } else {
        if (s->buf_items < SERIAL_TRIPLE_BUFFER_SIZE) {
            s->outgoing_buffer[s->buf_tail] = c;
            s->buf_tail = (s->buf_tail + 1) % SERIAL_TRIPLE_BUFFER_SIZE;
            s->buf_items++;
        } else {
            uart_putc_raw(u, s->outgoing_buffer[s->buf_head]);  // blocks
            s->buf_head = (s->buf_head + 1) % SERIAL_TRIPLE_BUFFER_SIZE;
            s->outgoing_buffer[s->buf_tail] = c;
            s->buf_tail = (s->buf_tail + 1) % SERIAL_TRIPLE_BUFFER_SIZE;
        }
    }
}

static void send_escaped_byte_triple(serial_triple_state_t* s, uint8_t b) {
    switch (b) {
        case END:
            my_putc_triple(s, ESC);
            my_putc_triple(s, ESC_END);
            break;
        case ESC:
            my_putc_triple(s, ESC);
            my_putc_triple(s, ESC_ESC);
            break;
        default:
            my_putc_triple(s, b);
    }
}

bool serial_triple_write_channel(serial_triple_state_t* s, const uint8_t* data, uint16_t len, bool drop_if_blocking) {
    uint32_t crc = crc32(data, len);

    if (drop_if_blocking) {
        uint16_t bytes_to_send = len + 2 + 4;
        for (uint16_t i = 0; i < len; i++) {
            if ((data[i] == END) || (data[i] == ESC)) {
                bytes_to_send++;
            }
        }
        for (uint16_t i = 0; i < 4; i++) {
            uint8_t b = crc >> (i * 8);
            if ((b == END) || (b == ESC)) {
                bytes_to_send++;
            }
        }
        if (bytes_to_send > SERIAL_TRIPLE_BUFFER_SIZE - s->buf_items) {
            return false;
        }
    }

    my_putc_triple(s, END);

    for (int i = 0; i < len; i++) {
        send_escaped_byte_triple(s, data[i]);
    }

    for (int i = 0; i < 4; i++) {
        send_escaped_byte_triple(s, (crc >> (i * 8)) & 0xFF);
    }

    my_putc_triple(s, END);

    return true;
}

bool serial_triple_write_nonblocking_channel(serial_triple_state_t* s, const uint8_t* data, uint16_t len) {
    return serial_triple_write_channel(s, data, len, true);
}
