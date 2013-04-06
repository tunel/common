/*
 * ptimer.c
 * A timer using POSIX API
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

/*
Example usage:

  PTimer timer;
  // starting timer
  ptimer_init (&timer);

  // do some stuff

  printf ("µs elapsed: %lu\n", ptimer_get_elapsed (&timer, PTIMER_USEC));

*/


#include <sys/time.h>
#include "ptimer.h"


/**
 * @typedef PTimer
 * 
 * An structure representing a timer.  No assumption should be made on what
 * the fields are, and it is only public to be allocatable on the stack.
 */


/** Gets the number of seconds, milliseconds or microseconds since 1970/01/01
 * @param prec  the precision to get, from PTIMER_SEC to PTIMER_USEC
 * @return The number of seconds, milliseconds or microseconds since 1970/01/01
 */
unsigned long ptimer_get_tick_count (long prec)
{
    /* return number of s, ms or us since 1970/1/1 */
    struct timeval ctime;
    gettimeofday (&ctime, 0);
    return (ctime.tv_sec * prec) + (ctime.tv_usec / (PTIMER_USEC / prec));
}

/** Initializes a timer
 * @param timer   a timer to initialize
 * @return 0 on success, -1 otherwise
 * 
 * This function initializes a timer, e.g. start count for it.
 */
int ptimer_init (PTimer* timer)
{
    return gettimeofday (timer, 0);
}

/** Resets a timer
 * @param timer   a timer to reset
 * @return 0 on success, -1 otherwise
 */
int ptimer_reset (PTimer* timer)
{
    return ptimer_init (timer);
}

/** Gets the number of milliseconds or microseconds elapsed since given timer
 *  initialization
 * @param timer  an initialized timer
 * @param prec   the precision to get, from PTIMER_SEC to PTIMER_USEC
 * @return The number of milliseconds or microseconds since timer initialization
 */
unsigned long ptimer_get_elapsed (const PTimer* timer, long prec)
{
    PTimer ctimer;

    ptimer_init (&ctimer);

    return ptimer_diff (timer, &ctimer, prec);
}

/** Compares two timers
 * @param old   older timer
 * @param new   newer timer
 * @param prec  precision of the result, can be one from PTIMER_SEC to
 *              PTIMER_USEC. This value MUST be positive and not 0.
 * 
 * @warning if old and new are swapped, the result should be something like
 *          (((unsigned long)~0UL) - diff) because the negative value will be
 *          implicitly cast to an unsigned one.
 */
/* the third argument is long and not unsigned because we need the computing to
 * be done with signed values; then, this remove an explicit cast. */
unsigned long ptimer_diff (const PTimer* old, const PTimer* new, long prec)
{
    return ((new->tv_sec - old->tv_sec) * prec)
        + ((new->tv_usec - old->tv_usec) / (PTIMER_USEC / prec));
}
