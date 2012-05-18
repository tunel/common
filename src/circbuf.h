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

#ifndef H_CIRCBUF
#define H_CIRCBUF

#include <stdio.h>
#include <stdarg.h>

typedef struct {
    char *buf;
    size_t size;
    size_t start;
    size_t end;
    int full;
} Circbuf;

void Cirb_Init (Circbuf*);
Circbuf* Cirb_New (size_t);
void Cirb_Free (Circbuf*);

void Cirb_SetBuffer (Circbuf*, char*, size_t);
void Cirb_SeekRead (Circbuf*, long);
void Cirb_SeekWrite (Circbuf*, long);

/* put functions */
void Cirb_Putc (Circbuf*, char);
void Cirb_Puts (Circbuf*, const char*);
void Cirb_Put (Circbuf*, const char*, size_t);
size_t Cirb_PrintfArg (Circbuf*, const char*, va_list);
size_t Cirb_Printf (Circbuf*, const char*, ...);

/* get functions */
int Cirb_Getc (Circbuf*);
size_t Cirb_Gets (Circbuf*, char*, size_t);
size_t Cirb_Get (Circbuf*, char*, size_t);

size_t Cirb_GetLineLength (Circbuf*);
size_t Cirb_GetOccupedSize (Circbuf*);
size_t Cirb_GetAvailableSize (Circbuf*);
size_t Cirb_GetSize (Circbuf*);

#endif /* guard */
