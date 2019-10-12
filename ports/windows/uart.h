/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 // windows version derived from micropython/ports/stm32/uart.h
#ifndef MICROPY_INCLUDED_WINDOWS_UART_H
#define MICROPY_INCLUDED_WINDOWS_UART_H

#include "windows.h"
struct _mp_irq_obj_t;

typedef struct _pyb_uart_obj_t {
    mp_obj_base_t base;
    HANDLE hComm;
    uint16_t uart_id;
    bool is_enabled : 1;
    uint16_t timeout;                   // timeout waiting for first char
    uint16_t timeout_char;              // timeout waiting between chars
    uint16_t read_buf_len;              // len in chars; buf can hold len-1 chars
} pyb_uart_obj_t;

extern const mp_obj_type_t pyb_uart_type;

bool uart_exists(int uart_id);
bool uart_init(pyb_uart_obj_t *uart_obj,
    uint32_t baudrate, uint32_t bits, uint32_t parity, uint32_t stop, uint32_t flow);
void uart_deinit(pyb_uart_obj_t *uart_obj);
bool uart_exists(int uart_id);
mp_uint_t uart_rx_any(pyb_uart_obj_t *uart_obj);
size_t uart_rx_data(pyb_uart_obj_t *self, void *src_out, size_t num_chars, int *errcode);
bool uart_sendbreak(pyb_uart_obj_t *self);
bool uart_timeouts(pyb_uart_obj_t *self, uint16_t timeout, uint16_t timeout_char);
size_t uart_tx_data(pyb_uart_obj_t *self, const void *src_in, size_t num_chars, int *errcode);

static inline bool uart_tx_avail(pyb_uart_obj_t *self) {
    return true;
}

#endif // MICROPY_INCLUDED_WINDOWS_UART_H
