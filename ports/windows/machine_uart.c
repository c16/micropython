/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2018 Damien P. George
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

// windows version derived from micropython/ports/stm32/machine_uart.c
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "uart.h"
#include "windows.h"

#define FLOWCONTROL_RTS 1U
#define FLOWCONTROL_CTS 2U

/// \moduleref pyb
/// \class UART - duplex serial communication bus
///
/// UART implements the standard UART/USART duplex serial communications protocol.  At
/// the physical level it consists of 2 lines: RX and TX.  The unit of communication
/// is a character (not to be confused with a string character) which can be 8 or 9
/// bits wide.
///
/// UART objects can be created and initialised using:
///
///     from pyb import UART
///
///     uart = UART(1, 9600)                         # init with given baudrate
///     uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters
///
/// Bits can be 7 or 8.  Parity can be None, 0 (even) or 1 (odd).  Stop can be 1 or 2.
///
/// A UART object acts like a stream object and reading and writing is done
/// using the standard stream methods:
///
///     uart.read(10)       # read 10 characters, returns a bytes object
///     uart.read()         # read all available characters
///     uart.readline()     # read a line
///     uart.readinto(buf)  # read and store into the given buffer
///     uart.write('abc')   # write the 3 characters
///
/// Individual characters can be read/written using:
///
///     uart.readchar()     # read 1 character and returns it as an integer
///     uart.writechar(42)  # write 1 character
///
/// To check if there is anything to be read, use:
///
///     uart.any()               # returns True if any characters waiting

STATIC void pyb_uart_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (!self->is_enabled) {
        mp_printf(print, "UART(%u)", self->uart_id);
    } else {
        DCB dcb;
        GetCommState(self->hComm, &dcb);

        mp_printf(print, "UART(%u, baudrate=%u, bits=%u, parity=",
            self->uart_id, dcb.BaudRate, dcb.ByteSize);
        if (!(dcb.fParity)) {
            mp_print_str(print, "None");
        } else {
            mp_printf(print, "%u", (dcb.Parity == EVENPARITY) ? 0:1);
        }

        mp_printf(print, ", stop=%u, flow=",
            (((dcb.StopBits) & 3) == 0) ? 1 : 2);

        if ((dcb.fRtsControl != RTS_CONTROL_HANDSHAKE) && (dcb.fOutxCtsFlow == FALSE)) {
            mp_print_str(print, "0");
        }
        else {
            if (dcb.fRtsControl == RTS_CONTROL_HANDSHAKE) {
                mp_print_str(print, "RTS");
                if (dcb.fOutxCtsFlow) {
                   mp_print_str(print, "|");
                }
            }
            if (dcb.fOutxCtsFlow) {
                mp_print_str(print, "CTS");
            }
        }

        mp_printf(print, ", timeout=%u, timeout_char=%u, rxbuf=%u",
            self->timeout, self->timeout_char,
            self->read_buf_len == 0 ? 0 : self->read_buf_len - 1); // -1 to adjust for usable length of buffer

        mp_print_str(print, ")");
    }
}

/// \method init(baudrate, bits=8, parity=None, stop=1, *, timeout=1000, timeout_char=0, flow=0, read_buf_len=64)
///
/// Initialise the UART bus with the given parameters:
///
///   - `baudrate` is the clock rate.
///   - `bits` is the number of bits per byte, 7, 8.
///   - `parity` is the parity, `None`, 0 (even) or 1 (odd).
///   - `stop` is the number of stop bits, 1 or 2.
///   - `timeout` is the timeout in milliseconds to wait for the first character.
///   - `timeout_char` is the timeout in milliseconds to wait between characters.
///   - `flow` is RTS | CTS where RTS == 1, CTS == 2
///   - `read_buf_len` is the character length of the read buffer (0 to disable).
STATIC mp_obj_t pyb_uart_init_helper(pyb_uart_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_baudrate, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 9600} },
        { MP_QSTR_bits, MP_ARG_INT, {.u_int = 8} },
        { MP_QSTR_parity, MP_ARG_OBJ, {.u_rom_obj = MP_ROM_PTR(&mp_const_none_obj)} },
        { MP_QSTR_stop, MP_ARG_INT, {.u_int = 1} },
        { MP_QSTR_flow, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_timeout_char, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_rxbuf, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_read_buf_len, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 64} }, // legacy
    };

    // parse args
    struct {
        mp_arg_val_t baudrate, bits, parity, stop, flow, timeout, timeout_char, rxbuf, read_buf_len;
    } args;
    mp_arg_parse_all(n_args, pos_args, kw_args,
        MP_ARRAY_SIZE(allowed_args), allowed_args, (mp_arg_val_t*)&args);

    uint32_t parity;
    if (args.parity.u_obj == mp_const_none) {
        parity = 0U;
    }
    else {
        mp_int_t p = mp_obj_get_int(args.parity.u_obj);
        parity = (p & 1) ? ODDPARITY : EVENPARITY;
    }

    self->is_enabled = uart_init(self, args.baudrate.u_int, args.bits.u_int, parity, args.stop.u_int, args.flow.u_int);

    // set timeout
    self->timeout = args.timeout.u_int;
    self->timeout_char = args.timeout_char.u_int;

    self->is_enabled  &= uart_timeouts(self, self->timeout, self->timeout_char);

    if (!self->is_enabled) {
        uart_deinit(self);
    }
    
    return mp_const_none;
}

/// \classmethod \constructor(comport, ...)
///
/// Construct a UART object on the given comport; 1 onwards
/// With no additional parameters, the UART object is created but not
/// initialised (it has the settings from the last initialisation of
/// the bus, if any).  If extra arguments are given, the bus is initialised.
/// See `init` for parameters of initialisation.

STATIC mp_obj_t pyb_uart_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // work out port
    int uart_id = 0;
    if (mp_obj_is_str(args[0])) {
        const char *szPort = mp_obj_str_get_str(args[0]);
        if (strlen(szPort) > 3) {
            uart_id = atoi(&szPort[3]);
        }
        else {
            uart_id = atoi(szPort);
        }
    }
    else {
        uart_id = mp_obj_get_int(args[0]);
    }

    if (!uart_exists(uart_id)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "UART(%d) doesn't exist", uart_id));
    }
    
    pyb_uart_obj_t *self = m_new0(pyb_uart_obj_t, 1);
    self->base.type = &pyb_uart_type;
    self->uart_id = uart_id;
    self->is_enabled = false;

    if (n_args > 1 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        pyb_uart_init_helper(self, n_args - 1, args + 1, &kw_args);
    }

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t pyb_uart_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return pyb_uart_init_helper(MP_OBJ_TO_PTR(args[0]), n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_uart_init_obj, 1, pyb_uart_init);

/// \method deinit()
/// Turn off the UART bus.
STATIC mp_obj_t pyb_uart_deinit(mp_obj_t self_in) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uart_deinit(self);
    self->is_enabled = FALSE;
   
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_uart_deinit_obj, pyb_uart_deinit);

/// \method any()
/// Return `True` if any characters waiting, else `False`.
STATIC mp_obj_t pyb_uart_any(mp_obj_t self_in) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return MP_OBJ_NEW_SMALL_INT(uart_rx_any(self));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_uart_any_obj, pyb_uart_any);

/// \method writechar(char)
/// Write a single character on the bus.  `char` is an integer to write.
/// Return value: `None`.
STATIC mp_obj_t pyb_uart_writechar(mp_obj_t self_in, mp_obj_t char_in) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    byte ch = mp_obj_get_int(char_in);
    int errcode;
    uart_tx_data(self, &ch, 1U, &errcode);
  
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_uart_writechar_obj, pyb_uart_writechar);

/// \method readchar()
/// Receive a single character on the bus.
/// Return value: The character read, as an integer.  Returns -1 on timeout.
STATIC mp_obj_t pyb_uart_readchar(mp_obj_t self_in) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    byte buf;
    int errcode = 0;
    size_t numread = uart_rx_data(self, &buf, 1U, &errcode);

    return (numread != 0 )? MP_OBJ_NEW_SMALL_INT(buf): MP_OBJ_NEW_SMALL_INT(-1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_uart_readchar_obj, pyb_uart_readchar);

// uart.sendbreak()
STATIC mp_obj_t pyb_uart_sendbreak(mp_obj_t self_in) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uart_sendbreak(self);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_uart_sendbreak_obj, pyb_uart_sendbreak);


STATIC const mp_rom_map_elem_t pyb_uart_locals_dict_table[] = {
    // instance methods

    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&pyb_uart_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&pyb_uart_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_any), MP_ROM_PTR(&pyb_uart_any_obj) },

    /// \method read([nbytes])
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    /// \method readline()
    { MP_ROM_QSTR(MP_QSTR_readline), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj)},
    /// \method readinto(buf[, nbytes])
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_stream_readinto_obj) },
    /// \method write(buf)
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_stream_write_obj) },

    { MP_ROM_QSTR(MP_QSTR_writechar), MP_ROM_PTR(&pyb_uart_writechar_obj) },
    { MP_ROM_QSTR(MP_QSTR_readchar), MP_ROM_PTR(&pyb_uart_readchar_obj) },
    { MP_ROM_QSTR(MP_QSTR_sendbreak), MP_ROM_PTR(&pyb_uart_sendbreak_obj) },

    // class constants
    { MP_ROM_QSTR(MP_QSTR_RTS), MP_ROM_INT(FLOWCONTROL_RTS) },
    { MP_ROM_QSTR(MP_QSTR_CTS), MP_ROM_INT(FLOWCONTROL_CTS) },

};

STATIC MP_DEFINE_CONST_DICT(pyb_uart_locals_dict, pyb_uart_locals_dict_table);

STATIC mp_uint_t pyb_uart_read(mp_obj_t self_in, void *buf_in, mp_uint_t size, int *errcode) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return uart_rx_data(self, buf_in, size, errcode);
}

STATIC mp_uint_t pyb_uart_write(mp_obj_t self_in, const void *buf_in, mp_uint_t size, int *errcode) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return uart_tx_data(self, buf_in, size, errcode);
}

STATIC mp_uint_t pyb_uart_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    pyb_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_uint_t ret;
    if (request == MP_STREAM_POLL) {
        uintptr_t flags = arg;
        ret = 0;
        if ((flags & MP_STREAM_POLL_RD) && uart_rx_any(self)) {
            ret |= MP_STREAM_POLL_RD;
        }
        if ((flags & MP_STREAM_POLL_WR) && uart_tx_avail(self)) {
            ret |= MP_STREAM_POLL_WR;
        }
    } else {
        *errcode = MP_EINVAL;
        ret = MP_STREAM_ERROR;
    }
    return ret;
}

STATIC const mp_stream_p_t uart_stream_p = {
    .read = pyb_uart_read,
    .write = pyb_uart_write,
    .ioctl = pyb_uart_ioctl,
    .is_text = false,
};

const mp_obj_type_t pyb_uart_type = {
    { &mp_type_type },
    .name = MP_QSTR_UART,
    .print = pyb_uart_print,
    .make_new = pyb_uart_make_new,
    .getiter = mp_identity_getiter,
    .iternext = mp_stream_unbuffered_iter,
    .protocol = &uart_stream_p,
    .locals_dict = (mp_obj_dict_t*)&pyb_uart_locals_dict,
};

// UART driver
void uart_deinit(pyb_uart_obj_t *uart_obj)
{
    CloseHandle(uart_obj->hComm);
}

bool uart_exists(int uart_id)
{
    char szPort[10];
    sprintf(szPort, "\\\\.\\com%d", uart_id); 
    HANDLE hComm = CreateFileA(szPort,
                               GENERIC_READ | GENERIC_WRITE,
                               0,
                               0,
                               OPEN_EXISTING,
                               0,
                               0);

    if (hComm != INVALID_HANDLE_VALUE) {
        CloseHandle(hComm);
    }

    return hComm != INVALID_HANDLE_VALUE;
}

bool uart_init(pyb_uart_obj_t *uart_obj,
               uint32_t baudrate, 
               uint32_t bits,                                                   // 7 or 8
               uint32_t parity,                                                 // 0 (None), 2 (even) or 1 (odd)
               uint32_t stop,                                                   // 1 or 2
               uint32_t flow)
{
    char szPort[10];
    sprintf(szPort, "\\\\.\\com%d", uart_obj->uart_id); //@@ replace wiht mp version of sprintf when I find it
    uart_obj->hComm = CreateFileA(szPort,
                              GENERIC_READ | GENERIC_WRITE,
                              0,
                              0,
                              OPEN_EXISTING,
                              0,
                              0);

    if (uart_obj->hComm == INVALID_HANDLE_VALUE) {
        uart_obj->hComm = NULL;
    }

    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);

    GetCommState(uart_obj->hComm, &dcb);

    // baudrate
    dcb.BaudRate = baudrate;

    // number of bits 7,8
    if (bits == 8) {
        dcb.ByteSize = 8;
    }
    else if (bits == 7) {
        dcb.ByteSize = 7;
    }
    else {
        uart_deinit(uart_obj);
        mp_raise_ValueError("unsupported bits");
    }

    // parity  0 (None), 2 (even) or 1 (odd)
    if (parity == 0U) {
        dcb.fParity = FALSE;
    }
    else if ((parity == 1U) || (parity == 2U)) {
        dcb.fParity = TRUE;
        dcb.Parity = parity;
    }
    else {
        uart_deinit(uart_obj);
        mp_raise_ValueError("unsupported parity");
    }

    // stop bits 1 or 2
    if (stop == 1U) {
        dcb.StopBits = ONESTOPBIT;
    }
    else if (stop == 2U) {
        dcb.StopBits = TWOSTOPBITS;
    }
    else {
        uart_deinit(uart_obj);
        mp_raise_ValueError("unsupported stopbits");
    }


    // flow control 0, 1 | 2
    if (flow == 0U) {
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        dcb.fOutxCtsFlow = FALSE;

    }
    else if ((flow & FLOWCONTROL_RTS) || (flow & FLOWCONTROL_CTS)) {
        if (flow & FLOWCONTROL_RTS) {
            dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;

        }

        if (flow & FLOWCONTROL_CTS) {
            dcb.fOutxCtsFlow = TRUE;
        }
    }
    else {
        uart_deinit(uart_obj);
        mp_raise_ValueError("unsupported flow");
    }

    // init UART (if it fails, it's because the port doesn't exist)
    if (!SetCommState(uart_obj->hComm, &dcb)) {
        uart_deinit(uart_obj);
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "UART(%d) doesn't exist", uart_obj->uart_id));
    }

    return uart_obj->hComm != NULL;
}

mp_uint_t uart_rx_any(pyb_uart_obj_t *uart_obj)
{
    COMSTAT comstat;
    uint32_t errors;
    ClearCommError(uart_obj->hComm, &errors, &comstat);
    return comstat.cbInQue;
}

size_t uart_rx_data(pyb_uart_obj_t *self, void *src_out, size_t num_chars, int *errcode)
{
    byte *buf = src_out;

    uint32_t read;
    BOOL  bSuccess = ReadFile(self->hComm, buf, num_chars, &read, NULL);

    if (!bSuccess) {
        *errcode = MP_EIO;
        return MP_STREAM_ERROR;
    }

    return read;
}

bool uart_sendbreak(pyb_uart_obj_t *self)
{
    HANDLE hComm = self->hComm;
    uint32_t mask = 0U;

    BOOL bSuccess = GetCommMask(hComm, &mask);
    bSuccess &= SetCommMask(hComm, mask | EV_TXEMPTY);

    bSuccess &= SetCommBreak(hComm);
    uint32_t evt = 0U;
    while ((evt & EV_TXEMPTY) == 0U) {
        WaitCommEvent(hComm, &evt, NULL);
    }

    bSuccess &= ClearCommBreak(hComm);
    bSuccess &= SetCommMask(hComm, mask);

    return bSuccess;
}

bool uart_timeouts(pyb_uart_obj_t *self, uint16_t timeout, uint16_t timeout_char)
{
    COMMTIMEOUTS commtimeouts;
    BOOL bSuccess = GetCommTimeouts(self->hComm, &commtimeouts);
    if (timeout || timeout_char) {
        commtimeouts.ReadTotalTimeoutConstant = timeout;
        commtimeouts.ReadIntervalTimeout = timeout_char;
    }
    else {
        commtimeouts.ReadIntervalTimeout = MAXDWORD;
        commtimeouts.ReadTotalTimeoutMultiplier = 0U;
        commtimeouts.ReadTotalTimeoutConstant = 0U;
    }

    bSuccess &= SetCommTimeouts(self->hComm, &commtimeouts);

    return bSuccess;
}

size_t uart_tx_data(pyb_uart_obj_t *self, const void *src_in, size_t num_chars, int *errcode)
{
    const byte *buf = src_in;

    uint32_t written;
    uint32_t length = num_chars;
    BOOL bSuccess = WriteFile(self->hComm, buf, length, &written, NULL);

    if (!bSuccess) {
        *errcode = MP_EIO;
        return MP_STREAM_ERROR;
    }

    return written;
}