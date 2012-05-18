/*------------------------------------------------------------------------------
    Tune Land - Sandbox RPG
    Copyright (C) 2012-2012
        Antony Martin <antony(dot)martin(at)scengine(dot)org>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 -----------------------------------------------------------------------------*/

#include <SCE/utils/SCEUtils.h>
#include "circbuf.h"

void Cirb_Init (Circbuf *cb)
{
    cb->buf = NULL;
    cb->size = 0;
    cb->start = 0;
    cb->end = 0;
    cb->full = SCE_FALSE;
}
Circbuf* Cirb_New (size_t size)
{
    Circbuf *cb = SCE_malloc (sizeof *cb + size);
    if (!cb)
        SCEE_LogSrc ();
    else {
        Cirb_Init (cb);
        cb->size = size;
        cb->buf = (char*)&cb[1];
        memset (cb->buf, 0, size);
    }
    return cb;
}
void Cirb_Free (Circbuf *cb)
{
    SCE_free (cb);
}

void Cirb_SetBuffer (Circbuf *cb, char *buf, size_t size)
{
    cb->buf = buf;
    cb->size = size;
}

void Cirb_SeekRead (Circbuf *buf, long offset)
{
    long sum = (long)buf->start + offset;
    if (sum < 0)
        buf->start = sum + buf->size;
    else if ((unsigned long)sum >= buf->size)
        buf->start = sum - buf->size;
    else
        buf->start += offset;
}
void Cirb_SeekWrite (Circbuf *buf, long offset)
{
    long sum = (long)buf->end + offset;
    if (sum < 0)
        buf->end = sum + buf->size;
    else if ((unsigned long)sum >= buf->size)
        buf->end = sum - buf->size;
    else
        buf->end += offset;
}

void Cirb_Putc (Circbuf *cb, char c)
{
    cb->buf[cb->end] = c;
    cb->end = (cb->end + 1) % cb->size;
    if (cb->end == cb->start)   /* end encountered the head by looping */
        cb->full = SCE_TRUE;
    if (cb->full)
        cb->start = cb->end;
}
void Cirb_Puts (Circbuf *cb, const char *str)
{
    while (*str)
        Cirb_Putc (cb, *str++);
}
void Cirb_Put (Circbuf *cb, const char *str, size_t size)
{
    size_t i = 0;
    while (i < size)
        Cirb_Putc (cb, str[i++]);
}
size_t Cirb_PrintfArg (Circbuf *cb, const char *fmt, va_list args)
{
    char buf[4096] = {0};       /* hoho! */
    size_t size;
    size = vsnprintf (buf, 4096, fmt, args);
    Cirb_Puts (cb, buf);
    return size;
}
size_t Cirb_Printf (Circbuf *cb, const char *fmt, ...)
{
    va_list args;
    size_t size;
    va_start (args, fmt);
    size = Cirb_PrintfArg (cb, fmt, args);
    va_end (args);
    return size;
}


int Cirb_Getc (Circbuf *cb)
{
    int c = EOF;
    if (cb->start != cb->end || cb->full) {
        c = (unsigned char)cb->buf[cb->start];
        cb->start = (cb->start + 1) % cb->size;
        cb->full = SCE_FALSE;
    }
    return c;
}
size_t Cirb_Gets (Circbuf *cb, char *str, size_t size)
{
    size_t i;
    for (i = 0; i < size; i++) {
        str[i] = Cirb_Getc (cb);
        if (str[i] == '\n')
            break;
        else if (str[i] == EOF) {
            str[i] = 0;
            break;
        }
    }
    return i + 1;
}
size_t Cirb_Get (Circbuf *cb, char *str, size_t size)
{
    size_t read = 0;
    while ((cb->full || cb->start != cb->end) && read < size) {
        *str = Cirb_Getc (cb);
        read++;
        str++;
    }
    return read;
}

size_t Cirb_GetLineLength (Circbuf *cb)
{
    int c;
    size_t length = 0;
    size_t prev = cb->start;
    do {
        c = Cirb_Getc (cb);
        length++;
    } while (c != EOF && c != '\n');
    cb->start = prev;
    return length;
}
size_t Cirb_GetOccupedSize (Circbuf *cb)
{
    if (cb->full)
        return cb->size;
    else if (cb->end < cb->start)
        return cb->size + cb->end - cb->start;
    else
        return cb->end - cb->start;
}
size_t Cirb_GetAvailableSize (Circbuf *cb)
{
    return cb->size - Cirb_GetOccupedSize (cb);
}
size_t Cirb_GetSize (Circbuf *cb)
{
    return cb->size;
}
