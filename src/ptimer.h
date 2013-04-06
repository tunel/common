/*
 * ptimer.h
 * A timer using POSIX API (Header file)
 *
 * Copyright (c) 2005-2008, Colomban Wendling <ban@herbesfolles.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the  nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <sys/time.h>


#ifndef PTIMER_H_DEFINED
#define PTIMER_H_DEFINED


/* timer struct, simply timeval */
typedef struct timeval PTimer;

/* timer precisions, values must be the real precision (see usage in ptimer.c) */
#define PTIMER_SEC    1L       /**< seconds (s) */
#define PTIMER_MSEC   1000L    /**< milliseconds (ms) */
#define PTIMER_USEC   1000000L /**< microseconds (µs) */

/* prototypes */
extern unsigned long  ptimer_get_tick_count (long prec);

extern int            ptimer_init           (PTimer* timer);
extern int            ptimer_reset          (PTimer* timer);
extern unsigned long  ptimer_get_elapsed    (const PTimer* timer,
                                             long prec);
extern unsigned long  ptimer_diff           (const PTimer* old,
                                             const PTimer* new,
                                             long prec);


#endif /* guard (PTIMER_H_DEFINED) */
