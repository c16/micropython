/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George on behalf of Pycom Ltd
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
 // based on ports/unix/mpthreadport.h

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <memory.h>
#include <windows.h>

#include "py/runtime.h"
#include "py/mpthread.h"
#include "py/gc.h"


#if MICROPY_PY_THREAD


#include <assert.h>
#include <signal.h>
typedef HANDLE sem_t;



// this structure forms a linked list, one node per active thread
typedef struct _thread_t {
    HANDLE hThread;         // handle of thread
    HANDLE id;              // system id of thread
    int ready;              // whether the thread is ready and running
    void *arg;              // thread Python args, a GC root pointer
    struct _thread_t *next;
} thread_t;

STATIC DWORD tls_key;


// the mutex controls access to the linked list
STATIC HANDLE thread_mutex = NULL;
STATIC thread_t *thread;

// this is used to synchronise the signal handler of the thread
// it's needed because we can't use any pthread calls in a signal handler
STATIC sem_t thread_signal_done;

// this signal handler is used to scan the regs and stack of a thread
STATIC void mp_thread_gc(int signo) {
    if (signo == SIGTERM) {
        void gc_collect_regs_and_stack(void);
        gc_collect_regs_and_stack();
        // We have access to the context (regs, stack) of the thread but it seems
        // that we don't need the extra information, enough is captured by the
        // gc_collect_regs_and_stack function above
        //gc_collect_root((void**)context, sizeof(ucontext_t) / sizeof(uintptr_t));
        #if MICROPY_ENABLE_PYSTACK
        void **ptrs = (void**)(void*)MP_STATE_THREAD(pystack_start);
        gc_collect_root(ptrs, (MP_STATE_THREAD(pystack_cur) - MP_STATE_THREAD(pystack_start)) / sizeof(void*));
        #endif
       
        ReleaseSemaphore(thread_signal_done, 1, NULL);
    }

}


void mp_thread_init(void) {
    tls_key = TlsAlloc();
    TlsSetValue(tls_key, &mp_state_ctx.thread);

    // create first entry in linked list of all threads
    thread = (thread_t *)malloc(sizeof(thread_t));
    thread->id = GetCurrentThread();
    thread->ready = 1;
    thread->arg = NULL;
    thread->next = NULL;

    thread_signal_done = CreateSemaphore(NULL, 0, 100, NULL);
    mp_thread_mutex_init(&thread_mutex);
}

void mp_thread_deinit(void) {

    mp_thread_mutex_lock(&thread_mutex, 1);
    while (thread->next != NULL) {
        thread_t *th = thread;
        thread = thread->next;
        CloseHandle(th->hThread);
        free(th);
    }
    mp_thread_mutex_unlock(&thread_mutex);
    
    assert(thread->id == GetCurrentThread());
    free(thread);

}

// This function scans all pointers that are external to the current thread.
// It does this by signalling all other threads and getting them to scan their
// own registers and stack.  Note that there may still be some edge cases left
// with race conditions and root-pointer scanning: a given thread may manipulate
// the global root pointers (in mp_state_ctx) while another thread is doing a
// garbage collection and tracing these pointers.
void mp_thread_gc_others(void) {

    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        gc_collect_root(&th->arg, 1);
        if (th->id == GetCurrentThread()) {
            continue;
        }
        if (!th->ready) {
            continue;
        }
        
        signal(SIGTERM, mp_thread_gc);
        WaitForSingleObject(
            thread_signal_done,   // handle to semaphore
            INFINITE);
        
    }
    mp_thread_mutex_unlock(&thread_mutex);

}

mp_state_thread_t *mp_thread_get_state(void) {
    return (mp_state_thread_t *)TlsGetValue(tls_key);
}

void mp_thread_set_state(void *state) {
    TlsSetValue(tls_key, state);
}

void mp_thread_start(void) {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == GetCurrentThread()) {
            th->ready = 1;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

#define STACK_MIN (8192 * BYTES_PER_WORD)

void mp_thread_create(void *(*entry)(void*), void *arg, size_t *stack_size) {

    // default stack size is 8k machine-words
    if (*stack_size == 0) {
        *stack_size = STACK_MIN;
    }

    // minimum stack size is set by pthreads
    if (*stack_size < STACK_MIN) {
        *stack_size = STACK_MIN;
    }

    mp_thread_mutex_lock(&thread_mutex,1);

    // create thread
    HANDLE hThread = CreateThread(NULL, *stack_size, (LPTHREAD_START_ROUTINE)entry, arg, 0, NULL);
    int ret = (hThread != NULL) ? 0 : -1;
    if (ret != 0) {
        mp_thread_mutex_unlock(&thread_mutex);
        goto er;
    }

    // adjust stack_size to provide room to recover from hitting the limit
    // this value seems to be about right for both 32-bit and 64-bit builds
    *stack_size -= 8192;

    // add thread to linked list of all threads
    thread_t *th = (thread_t *)malloc(sizeof(thread_t));
    th->hThread = hThread;
    th->id = GetCurrentThread();
    th->ready = 0;
    th->arg = arg;
    th->next = thread;
    thread = th;

    mp_thread_mutex_unlock(&thread_mutex);

    return;

er:
    mp_raise_OSError(ret);

}

void mp_thread_finish(void) {
    mp_thread_mutex_lock(&thread_mutex, 1);

    thread_t *prev = NULL;
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == GetCurrentThread()) {
            if (prev == NULL) {
                thread = th->next;
            } else {
                prev->next = th->next;
            }
            free(th);
            break;
        }
        prev = th;
    }

    mp_thread_mutex_unlock(&thread_mutex);
}

void mp_thread_mutex_init(mp_thread_mutex_t *mutex) {
    *mutex = CreateMutex(NULL,              // default security attributes
                         FALSE,             // initially not owned
                         NULL);             // unnamed mutex
}

int mp_thread_mutex_lock(mp_thread_mutex_t *mutex, int wait) {
    DWORD waitresult = WaitForSingleObject(*mutex,    // handle to mutex
                                           wait ? INFINITE:0);


    return (waitresult == WAIT_OBJECT_0) ? 1:(waitresult == WAIT_TIMEOUT) ? 0:-1;
}

void mp_thread_mutex_unlock(mp_thread_mutex_t *mutex) {
    if (!ReleaseMutex(*mutex)) {
        // Handle error.
    }
}

#endif // MICROPY_PY_THREAD
