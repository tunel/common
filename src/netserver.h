/*------------------------------------------------------------------------------
    Tune Land - Sandbox RPG
    Copyright (C) 2012-2013
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

#ifndef H_NETSERVER
#define H_NETSERVER

#include "netclient.h"

#define NETSERVER_DEFAULT_TIMEOUT 30 /* default ping time out */

typedef struct netserver NetServer;

typedef void (*NetServerCmdFunc)(NetServer*, NetClient*, void*,
                                 const char*, size_t);

typedef struct netservercmd NetServerCmd;
struct netservercmd {
    SockID id;
    NetServerCmdFunc command;
    void *data;                 /* user-defined data */
    SCE_SListIterator it;
};

struct netserver {
    Sock tcpsock;
    Sock udpsock;
    SOCKADDR_IN infos;
    SCE_SList clients;
    SCE_SList selected;
    pthread_mutex_t clients_mutex;
    int opened;
    int port;
    in_addr_t ip;
    int timeout;
    SCE_SList tcp_cmds;
    SCE_SList udp_cmds;
    NetServerCmd tcp_unknowncmd;
    NetServerCmd udp_unknowncmd;
    NetServerCmd discocmd;
    NetServerCmd pingcmd;
    NetServerCmd timeoutcmd;
    void *udata;                /* user-defined data */
};

void NetServer_InitCmd (NetServerCmd*);
NetServerCmd* NetServer_NewCmd (int, NetServerCmdFunc, void*);
void NetServer_FreeCmd (NetServerCmd*);

void NetServer_Init (NetServer*);
void NetServer_Clear (NetServer*);
NetServer* NetServer_New (void);
void NetServer_Free (NetServer*);

void NetServer_SetCmdID (NetServerCmd*, int);
void NetServer_SetCmdCallback (NetServerCmd*, NetServerCmdFunc);
void NetServer_SetCmdData (NetServerCmd*, void*);
void NetServer_CallCmd (NetServerCmd*, NetServer*, NetClient*, const char*,
                        size_t);

SCE_SList* NetServer_GetNetClients (NetServer*);

void NetServer_AddTCPCmd (NetServer*, NetServerCmd*);
void NetServer_AddUDPCmd (NetServer*, NetServerCmd*);
/* specific commands */
/* TODO: no callback for client connection? */
void NetServer_SetTCPUnknownCmd (NetServer*, NetServerCmd*);
void NetServer_SetUDPUnknownCmd (NetServer*, NetServerCmd*);
void NetServer_SetDisconnectCommand (NetServer*, NetServerCmd*);
void NetServer_SetPingCommand (NetServer*, NetServerCmd*);
void NetServer_SetPingTimeoutCommand (NetServer*, NetServerCmd*);

void NetServer_SetData (NetServer*, void*);
void* NetServer_GetData (NetServer*);
void NetServer_SetTimeOut (NetServer*, int);
int NetServer_GetTimeOut (NetServer*);

int NetServer_Open (NetServer*, int);
void NetServer_Launch (NetServer*);
void NetServer_Stop (NetServer*);
int NetServer_Wait (NetServer*, long, long);
int NetServer_Poll (NetServer*);
void NetServer_Step (NetServer*);
void NetServer_Close (NetServer*);

void NetServer_KickClient (NetServer*, NetClient*, const char*);
int NetServer_SendUDP (NetServer*, NetClient*, int, const char*, size_t);
int NetServer_SendTCP (NetServer*, NetClient*, int, const char*, size_t);
int NetServer_StreamTCP (NetServer*, NetClient*, int, const char*,
                         size_t, size_t);
int NetServer_SendUDPString (NetServer*, NetClient*, int, const char*);
int NetServer_SendTCPString (NetServer*, NetClient*, int, const char*);

#endif /* guard */
