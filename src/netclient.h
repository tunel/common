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

#ifndef H_NETCLIENT
#define H_NETCLIENT

#include <pthread.h>
#include <SCE/utils/SCEUtils.h>
#include "socket.h"

typedef struct netclient NetClient;

typedef void (*NetClientCmdFunc)(NetClient*, void*, const char*, size_t);

typedef struct netclientcmd NetClientCmd;
struct netclientcmd {
    SockID id;
    NetClientCmdFunc command;
    void *data;                 /* user-defined data */
    SCE_SListIterator it;
};

/* pre-defined commands */
enum CommandsID {
    NETCLIENT_CONNECTION_DATA = NETWORK_CONTINUE_STREAM + 1,
    NETCLIENT_UDP_CONNECTION,
    NETCLIENT_PING,
    NETCLIENT_PONG,
    NETCLIENT_NUM_PREDEFINED_COMMANDS
};

/* user shall use it to make his own ID */
#define NETWORK_FIRST_ID_OFFSET NETCLIENT_NUM_PREDEFINED_COMMANDS

typedef void (*NetClientFreeDataFunc)(NetClient*, void*);

struct netclient {
    Sock tcpsock;
    Sock udpsock;
    int udp_id;
    void *serv;                 /* hack (used by server.c) */
    SOCKADDR_IN server_addr;
    pthread_t tcp_thread;
    pthread_t udp_thread;
    in_addr_t ip;
    int port;
    int connected;
    int listening;
    time_t tm, timeout;
    SCE_SListIterator it;
    SCE_SList tcp_cmds;
    SCE_SList udp_cmds;
    NetClientCmd tcp_unknowncmd;
    NetClientCmd udp_unknowncmd;
    NetClientCmd discocmd;
    NetClientCmd pingcmd;
    NetClientCmd timeoutcmd;
    void *udata;                /* user-defined data */
    NetClientFreeDataFunc freedata;
};

void NetClient_InitCmd (NetClientCmd*);
/*void NetClient_ClearCmd (NetClientCmd*);*/
NetClientCmd* NetClient_NewCmd (int, NetClientCmdFunc, void*);
void NetClient_FreeCmd (NetClientCmd*);

void NetClient_Init (NetClient*);
void NetClient_Clear (NetClient*);
NetClient* NetClient_New (void);
void NetClient_Free (NetClient*);

void NetClient_SetCmdID (NetClientCmd*, SockID);
void NetClient_SetCmdCallback (NetClientCmd*, NetClientCmdFunc);
void NetClient_SetCmdData (NetClientCmd*, void*);
void NetClient_CallCmd (NetClientCmd*, NetClient*, const char*, size_t);

void NetClient_AddTCPCmd (NetClient*, NetClientCmd*);
void NetClient_AddUDPCmd (NetClient*, NetClientCmd*);
/* specific commands */
void NetClient_SetTCPUnknownCmd (NetClient*, NetClientCmd*);
void NetClient_SetUDPUnknownCmd (NetClient*, NetClientCmd*);
void NetClient_SetDisconnectCommand (NetClient*, NetClientCmd*);
void NetClient_SetPingCommand (NetClient*, NetClientCmd*);
void NetClient_SetPingTimeoutCommand (NetClient*, NetClientCmd*);

void NetClient_SetData (NetClient*, void*);
void* NetClient_GetData (NetClient*);
void NetClient_SetFreeDataFunc (NetClient*, NetClientFreeDataFunc);

int NetClient_Connect (NetClient*, in_addr_t, int);
void NetClient_Disconnect (NetClient*);

int NetClient_SendTCP (NetClient*, SockID, const char*, size_t);
int NetClient_SendUDP (NetClient*, SockID, const char*, size_t);
int NetClient_SendTCPString (NetClient*, SockID, const char*);
int NetClient_SendUDPString (NetClient*, SockID, const char*);

time_t NetClient_LastPacket (NetClient*);

void NetClient_Listen (NetClient*);
int NetClient_PollTCP (NetClient*);
int NetClient_PollUDP (NetClient*);
int NetClient_WaitTCP (NetClient*, long, long);
int NetClient_WaitUDP (NetClient*, long, long);
int NetClient_TCPStep (NetClient*, SockID*);
SockID NetClient_UDPStep (NetClient*);
void NetClient_StopListening (NetClient*);

long NetClient_WaitTCPPacket (NetClient*, SockID, time_t);
long NetClient_WaitUDPPacket (NetClient*, SockID, time_t);

#endif /* guard */
