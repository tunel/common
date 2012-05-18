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

#ifndef H_NETPROTOCOL
#define H_NETPROTOCOL

#include "netclient.h"           /* NETWORK_FIRST_ID_OFFSET */

enum TlpCommand {
    /* do not need to be logged in */
    /* state/query */
    TLP_SAY_HI = NETWORK_FIRST_ID_OFFSET,
    TLP_GET_CLIENT_NUM,
    TLP_GET_CLIENT_LIST,
    TLP_GET_CLIENT_NICK,

    /* connect/disconnect to a server */
    TLP_CONNECT,
    TLP_CONNECT_ACCEPTED,
    TLP_CONNECT_REFUSED,
    TLP_DISCONNECT,

    /* chat commands */
    TLP_CHAT_MSG,

    /* asynchronous commands */
    TLP_PLAYER_KEYSTATE,
    TLP_PLAYER_POSITION,
    TLP_OBJECT_POSITION,

    TLP_LAST_COMMAND
};

typedef enum TlpCommand TlpCmd;

#define TLP_NUM_COMMANDS (TLP_LAST_COMMAND - NETWORK_FIRST_ID_OFFSET)

#if 0

const char* Tlp_CmdString (TlpCmd);

void Tlp_SendMsg (const char*, TlpCmd);
void Tlp_RecvMsg (const char*, TlpCmd);

#define Tlp_Send Tlp_SendMsg
#define Tlp_Recv Tlp_RecvMsg

#else

#define Tlp_Send(a, b)
#define Tlp_Recv(a, b)

#endif

#endif /* guard */
