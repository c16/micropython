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

#include <windows.h>
#include <errno.h>
#include <limits.h>
#include "py/mpconfig.h"

#if MICROPY_PY_THREAD == 0
HANDLE waitTimer = NULL;
#endif

void init_sleep(void) {
#if MICROPY_PY_THREAD == 0
    waitTimer = CreateWaitableTimer(NULL, TRUE, NULL);
#endif
}

void deinit_sleep(void) {
#if MICROPY_PY_THREAD == 0
    if (waitTimer != NULL) {
        CloseHandle(waitTimer);
        waitTimer = NULL;
    }
#endif
}

int usleep_impl(__int64 usec) {
#if MICROPY_PY_THREAD
    HANDLE waitTimer = CreateWaitableTimer(NULL, TRUE, NULL);
#endif
    
    if (waitTimer == NULL) {
        errno = EAGAIN;
    } else if (usec < 0 || usec > LLONG_MAX / 10) {
        errno = EINVAL;
    } else {
        LARGE_INTEGER ft;
        ft.QuadPart = -10 * usec; // 100 nanosecond interval, negative value = relative time
        if (SetWaitableTimer(waitTimer, &ft, 0, NULL, NULL, 0) == 0) {
            errno = EINVAL;
        } else if (WaitForSingleObject(waitTimer, INFINITE) != WAIT_OBJECT_0) {
            errno = EAGAIN;
        } else {
            // ok
        }
    }

#if MICROPY_PY_THREAD
    CloseHandle(waitTimer);
#endif

    return (errno != EAGAIN) ? 0:-1;
}

#ifdef _MSC_VER // mingw and the likes provide their own usleep()
int usleep(__int64 usec) {
    return usleep_impl(usec);
}
#endif

void msec_sleep(double msec) {
    const double usec = msec * 1000.0;
    usleep_impl(usec > (double)LLONG_MAX ? LLONG_MAX : (__int64)usec);
}
