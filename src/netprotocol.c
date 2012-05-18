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

#include <time.h>
#include <SCE/utils/SCEUtils.h> /* SCEError */
#include "netprotocol.h"

/* Tune Land Protocol */

#if 0
const char* Tlp_CmdString (TlpCmd cmd)
{
#define TLP_CMD(c) case c: return #c

    switch (cmd) {
    TLP_CMD (42);
    default: return "Unknown command";
    }
#undef TLP_CMD
}

void Tlp_SendMsg (const char *prefix, TlpCmd cmd)
{
    if (prefix)
        SCEE_SendMsg ("%ld <== %s: %s\n", (long)time (NULL), prefix,
                      Tlp_CmdString (cmd));
    else
        SCEE_SendMsg ("%ld <== %s\n", (long)time (NULL), Tlp_CmdString (cmd));
}
void Tlp_RecvMsg (const char *prefix, TlpCmd cmd)
{
    if (prefix)
        SCEE_SendMsg ("%ld ==> %s: %s\n", (long)time (NULL), prefix,
                      Tlp_CmdString (cmd));
    else
        SCEE_SendMsg ("%ld ==> %s\n", (long)time (NULL), Tlp_CmdString (cmd));
}
#endif
