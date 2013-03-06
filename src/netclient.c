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

#include <errno.h>
#include <ctype.h>

#include "netclient.h"

void NetClient_InitCmd (NetClientCmd *cmd)
{
    cmd->id = 0;
    cmd->command = NULL;
    cmd->data = NULL;
    SCE_List_InitIt (&cmd->it);
    SCE_List_SetData (&cmd->it, cmd);
}
#if 0
void NetClient_ClearCmd (NetClientCmd *cmd)
{
}
#endif
NetClientCmd* NetClient_NewCmd (int id, NetClientCmdFunc f, void *data)
{
    NetClientCmd *cmd = SCE_malloc (sizeof *cmd);
    if (!cmd) goto failure;
    NetClient_InitCmd (cmd);
    cmd->id = id;
    cmd->command = f;
    cmd->data = data;
    return cmd;
failure:
    NetClient_FreeCmd (cmd), cmd = NULL;
    SCEE_LogSrc ();
    return NULL;
}

void NetClient_FreeCmd (NetClientCmd *cmd)
{
    SCE_free (cmd);
}


static void NetClient_DisconnectSockets (NetClient *client)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("called NetClient_DisconnectSockets()\n");
#endif
    client->listening = SCE_FALSE;
    client->connected = SCE_FALSE;
    /* TODO: shutdown may fail */
    Socket_Shutdown (&client->tcpsock);
    Socket_Close (&client->tcpsock);
    Socket_Close (&client->udpsock);
}

/* default commands functions */
static void gotunknowntcp (NetClient *client, void *a, const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("client: unknown TCP command received: %ld.\n",
                  SCE_Decode_Long (b));
#endif
    (void)a; (void)b; (void)c; (void)client;
}
static void gotunknownudp (NetClient *client, void *a, const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("client: unknown UDP command received: %ld.\n",
                  SCE_Decode_Long (b));
#endif
    (void)a; (void)b; (void)c; (void)client;
}
static void gotkick (NetClient *client, void *a, const char *b, size_t c)
{
#ifdef TL_DEBUG
    /* TODO: warning: b may be "null" (on what occasion?) */
    SCEE_SendMsg ("client: disconnected from server: %s.\n",
                  (c > 0 ? b : "unknown reason"));
#endif
    (void)a; (void)b; (void)c; (void)client;
}
static void gotping (NetClient *client, void *a, const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("client: ping query from server, sending response.\n");
#endif
    (void)a; (void)b; (void)c; (void)client;
}
static void gotpingtimeout (NetClient *client, void *a, const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("client: ping timeout (%lus).\n",
                  (unsigned long)(time (NULL) - client->tm));
#endif
    (void)client; (void)a; (void)b; (void)c;
}


static void NetClient_InitDefaultCommands (NetClient *client)
{
    NetClient_InitCmd (&client->tcp_unknowncmd);
    NetClient_InitCmd (&client->udp_unknowncmd);
    NetClient_InitCmd (&client->discocmd);
    NetClient_InitCmd (&client->pingcmd);
    NetClient_InitCmd (&client->timeoutcmd);

    client->tcp_unknowncmd.command = gotunknowntcp;
    client->udp_unknowncmd.command = gotunknownudp;
    client->discocmd.command = gotkick;
    client->pingcmd.command = gotping;
    client->timeoutcmd.command = gotpingtimeout;
}
void NetClient_Init (NetClient *client)
{
    Socket_Init (&client->tcpsock);
    Socket_Init (&client->udpsock);
    Socket_SetData (&client->tcpsock, client);
    Socket_SetData (&client->udpsock, client);
    client->serv = NULL;
    (void)client->server_addr;  /* lol */
    (void)client->tcp_thread;   /* re-lol */
    (void)client->udp_thread;   /* re-re-lol */
    client->ip = 0;
    client->port = 0;
    client->connected = SCE_FALSE;
    client->listening = SCE_FALSE;
    client->tm = 0;
    client->timeout = 30;       /* TODO: much like in server.h */
    SCE_List_InitIt (&client->it);
    SCE_List_SetData (&client->it, client);
    SCE_List_Init (&client->tcp_cmds);
    SCE_List_Init (&client->udp_cmds);
    NetClient_InitDefaultCommands (client);
    client->udata = NULL;
    client->freedata = NULL;
}
void NetClient_Clear (NetClient *client)
{
    if (client->listening)
        NetClient_StopListening (client);
    if (client->connected)
        NetClient_Disconnect (client);
    SCE_List_Remove (&client->it);
    if (client->freedata)
        client->freedata (client, client->udata);
    SCE_List_Clear (&client->tcp_cmds);
    SCE_List_Clear (&client->udp_cmds);
}

NetClient* NetClient_New (void)
{
    NetClient *client = SCE_malloc (sizeof *client);
    if (client) {
        NetClient_Init (client);
    } else {
        SCEE_LogSrc ();
    }
    return client;
}
void NetClient_Free (NetClient *client)
{
    if (client) {
        NetClient_Clear (client);
        SCE_free (client);
    }
}


void NetClient_SetCmdID (NetClientCmd *cmd, SockID id)
{
    cmd->id = id;
}
void NetClient_SetCmdCallback (NetClientCmd *cmd, NetClientCmdFunc f)
{
    cmd->command = f;
}
void NetClient_SetCmdData (NetClientCmd *cmd, void *data)
{
    cmd->data = data;
}
void NetClient_CallCmd (NetClientCmd *cmd, NetClient *client,
                        const char *data, size_t size)
{
    cmd->command (client, cmd->data, data, size);
}


static NetClientCmd* locatecmd (SCE_SList *cmds, SockID id)
{
    NetClientCmd *cmd;
    SCE_SListIterator *it;
    SCE_List_ForEach (it, cmds) {
        cmd = SCE_List_GetData (it);
        if (cmd->id == id)
            return cmd;
    }
    return NULL;
}
static void NetClient_AddCmd (SCE_SList *cmds, NetClientCmd *cmd)
{
    NetClientCmd *old = locatecmd (cmds, cmd->id);
    if (old)
        SCE_List_Remove (&old->it);
    SCE_List_Appendl (cmds, &cmd->it);
}
void NetClient_AddTCPCmd (NetClient *client, NetClientCmd *cmd)
{
    NetClient_AddCmd (&client->tcp_cmds, cmd);
}
void NetClient_AddUDPCmd (NetClient *client, NetClientCmd *cmd)
{
    NetClient_AddCmd (&client->udp_cmds, cmd);
}
/* here is a lot of ugly func */
void NetClient_SetTCPUnknownCmd (NetClient *client, NetClientCmd *cmd)
{
    if (cmd)
        client->tcp_unknowncmd = *cmd;
    else {
        NetClient_InitCmd (&client->tcp_unknowncmd);
        client->tcp_unknowncmd.command = gotunknowntcp;
    }
}
void NetClient_SetUDPUnknownCmd (NetClient *client, NetClientCmd *cmd)
{
    if (cmd)
        client->udp_unknowncmd = *cmd;
    else {
        NetClient_InitCmd (&client->udp_unknowncmd);
        client->udp_unknowncmd.command = gotunknownudp;
    }
}
void NetClient_SetDisconnectCommand (NetClient *client, NetClientCmd *cmd)
{
    if (cmd)
        client->discocmd = *cmd;
    else {
        NetClient_InitCmd (&client->discocmd);
        client->discocmd.command = gotkick;
    }
}
void NetClient_SetPingCommand (NetClient *client, NetClientCmd *cmd)
{
    if (cmd)
        client->pingcmd = *cmd;
    else {
        NetClient_InitCmd (&client->pingcmd);
        client->pingcmd.command = gotping;
    }
}
void NetClient_SetPingTimeoutCommand (NetClient *client, NetClientCmd *cmd)
{
    if (cmd)
        client->timeoutcmd = *cmd;
    else {
        NetClient_InitCmd (&client->timeoutcmd);
        client->timeoutcmd.command = gotpingtimeout;
    }
}


void NetClient_SetData (NetClient *client, void *data)
{
    client->udata = data;
}
void* NetClient_GetData (NetClient *client)
{
    return client->udata;
}
void NetClient_SetFreeDataFunc (NetClient *client, NetClientFreeDataFunc f)
{
    client->freedata = f;
}


static int NetClient_InitSocket (NetClient *client, in_addr_t ip, int port)
{
    if (Socket_InitUDP (&client->udpsock, SCE_FALSE) < 0)
        goto fail;
    if (Socket_InitTCP (&client->tcpsock, SCE_FALSE) < 0)
        goto fail;
    Socket_SetupAddr (&client->server_addr, ip, port);
    return SCE_OK;
fail:
    SCEE_LogSrc ();
    return SCE_ERROR;
}
/* TODO: use WaitTCPPacket() */
static int NetClient_ReceiveConnectionData (NetClient *client)
{
    size_t size = 0;
    SockID id;
    const char *data = NULL;
    int r;

#define TIMEOUT 10

    /* waiting for the first packet that is the server time for timeout */
    if ((r = Socket_NbReceiveTCP (&client->tcpsock, TIMEOUT, 0)) < 0) {
        SCEE_LogSrc();
        return SCE_ERROR;
    } else if (r == NETWORK_DISCONNECTED) {
        SCEE_Log (42);
        SCEE_LogMsg ("connection to server failed: server closed connection."
                     " maybe you should try later?");
        return SCE_ERROR;
    } else if (r == NETWORK_TIMEOUT) {
        SCEE_Log (SOCKET_PING_TIMEOUT);
        SCEE_LogMsg ("connection to server timed out (%ds)", TIMEOUT);
        return SCE_ERROR;
    }

    data = Socket_GetPacket (&client->tcpsock, &id, &size);
    if (data) {
        if (id == NETCLIENT_CONNECTION_DATA) {
            client->timeout = Socket_GetID (data);
            client->udp_id = (int)Socket_GetID (&data[SOCKET_ID_SIZE]);
            Socket_PopPacket (&client->tcpsock);
        } else {
            SCEE_Log (43);
            SCEE_LogMsg ("wrong packet id for the first-connection-packet");
            Socket_PopPacket (&client->tcpsock);
            return SCE_ERROR;
        }
    }
    return SCE_OK;
}
#if 0
static int NetClient_ConnectUDP (NetClient *client)
{
    char data[4] = {0};
    int id, size, i;

#define NETCLIENT_UDP_CONNECTION_TIMEOUT 2
#define NETCLIENT_NUM_UDP_TRIES 5

    for (i = 0; i < NETCLIENT_NUM_UDP_TRIES; i++) {
        Socket_SetID (data, client->udp_id);
        NetClient_SendUDP (client, NETCLIENT_UDP_CONNECTION, data, 2);
        id = Socket_NbReceiveTCP (&client->tcpsock, data, &size,
                                  NETCLIENT_UDP_CONNECTION_TIMEOUT, 0);
        if (id != NETWORK_TIMEOUT)
            break;
    }
    switch (id) {
    case SCE_ERROR:
        SCEE_LogSrc();
        return SCE_ERROR;
    case NETWORK_TIMEOUT:
        SCEE_Log (SOCKET_PING_TIMEOUT);
        SCEE_LogMsg ("connection to server timed out (%ds)",
                     NETCLIENT_UDP_CONNECTION_TIMEOUT *NETCLIENT_NUM_UDP_TRIES);
        return SCE_ERROR;
    case NETWORK_DISCONNECTED:
        SCEE_Log (42);
        SCEE_LogMsg ("connection to server failed: server closed connection."
                     " maybe you should try later?");
        return SCE_ERROR;
    default:
        if (id == NETCLIENT_UDP_CONNECTION) {
            /* it's all right! */
        } else {
            SCEE_Log (44);
            SCEE_LogMsg ("expected UDP connection from server but received %d",
                         id);
            return SCE_ERROR;
        }
    }
    return SCE_OK;
}
#endif
int NetClient_Connect (NetClient *client, in_addr_t ip, int port)
{
    int err;

    if (client->connected) {
#ifdef TL_DEBUG
        SCEE_SendMsg ("client: warning: client already connected.\n");
#endif
        return SCE_OK;
    }
    if (NetClient_InitSocket (client, ip, port) < 0)
        goto fail;
    if ((err = Socket_Connect (&client->tcpsock, &client->server_addr)) < 0)
        goto fail;
    if (NetClient_ReceiveConnectionData (client) < 0) {
        Socket_Close (&client->tcpsock);
        goto fail;
    }
#if 0
    /* achieving UDP connection */
    if (NetClient_ConnectUDP (client) < 0) {
        Socket_Close (&client->tcpsock);
        goto fail;
    }
#endif

    client->connected = SCE_TRUE;
    client->tm = time (NULL);
#if TL_DEBUG
    SCEE_SendMsg ("client connected.\n");
#endif
    return SCE_OK;
fail:
    SCEE_LogSrc ();
    return SCE_ERROR;
}

void NetClient_Disconnect (NetClient *client)
{
    if (!client->connected) {
#ifdef TL_DEBUG
        SCEE_SendMsg ("client: can't disconnect: not connected.\n");
#endif
        return;
    }
    NetClient_StopListening (client);
    NetClient_DisconnectSockets (client);
#if TL_DEBUG
    SCEE_SendMsg ("client disconnected.\n");
#endif
}


int NetClient_SendTCP (NetClient *client, SockID id, const char *data,
                       size_t size)
{
    int err;
    err = Socket_SendTCP (&client->tcpsock, id, data, size);
#ifdef TL_DEBUG
    if (err < 0) {
        SCEE_SendMsg ("client: failed to send TCP packet.\n");
    }
#endif
    return err;
}
int NetClient_SendUDP (NetClient *client, SockID id, const char *data,
                       size_t size)
{
    char buf[512] = {0};        /* TODO: sucks. */
    int err;
    Socket_SetID (buf, client->udp_id);
    memcpy (&buf[SOCKET_ID_SIZE], data, size);
    err = Socket_SendUDP (&client->udpsock, id, buf, size + SOCKET_ID_SIZE,
                          &client->server_addr);
#ifdef TL_DEBUG
    if (err < 0) {
        SCEE_SendMsg ("client: failed to send UDP packet.\n");
    }
#endif
    return err;
}
int NetClient_SendTCPString (NetClient *client, SockID id, const char *data)
{
    if (!data)
        return NetClient_SendTCP (client, id, NULL, 0);
    else
        return NetClient_SendTCP (client, id, data, strlen (data) + 1);
}
int NetClient_SendUDPString (NetClient *client, SockID id, const char *data)
{
    if (!data)
        return NetClient_SendUDP (client, id, NULL, 0);
    else
        return NetClient_SendUDP (client, id, data, strlen (data) + 1);
}

/* suxxor function */
static void NetClient_CheckTimeout (NetClient *client)
{
    if ((time (NULL) - client->tm) >= client->timeout) {
        /* client ping timeout! */
        NetClient_DisconnectSockets (client);
        NetClient_CallCmd (&client->timeoutcmd, client, NULL, 0);
        client->listening = client->connected = SCE_FALSE;
    }
}

static void
NetClient_TreatUDPPacket (NetClient *client, SockID id, const char *data,
                          size_t size)
{
    NetClientCmd *cmd = NULL;
    client->tm = time (NULL);
    cmd = locatecmd (&client->udp_cmds, id);
    if (cmd) {
        NetClient_CallCmd (cmd, client, &data[2 * SOCKET_ID_SIZE],(size_t)size);
    } else {
        /* unknown command */
        NetClient_CallCmd (&client->udp_unknowncmd, client, data, (size_t)size);
    }
}
static void NetClient_TreatTCPPacket (NetClient *client, SockID id,
                                      const char *data, size_t size)
{
    NetClientCmd *cmd = NULL;
    client->tm = time (NULL);
    /* unmodifiable behavior commands */
    switch (id) {
    case NETCLIENT_PONG:
        /* store exact time */
        /* client->pongtm_s = time (NULL), client->pongtm_us = foo (); */
        break;
    case NETCLIENT_PING:
        NetClient_SendTCP (client, NETCLIENT_PONG, NULL, 0);
        NetClient_CallCmd (&client->pingcmd, client, data, size);
        break;
    default:
        cmd = locatecmd (&client->tcp_cmds, id);
        if (cmd) {
            NetClient_CallCmd (cmd, client, data, size);
        } else {
            unsigned char buf[4] = {0};
            SCE_Encode_Long (id, buf);
            /* NOTE: well we dont have any access to the data in the
               callback but who cares. */
            NetClient_CallCmd (&client->tcp_unknowncmd, client, buf, size);
        }
    }
}

static void* NetClient_TCPLoop (void *clnt)
{
    SockID id;
    NetClient *client = clnt;
    while (client->listening) {
        NetClient_TCPStep (client, &id);
    }
    return NULL;
}
static void* NetClient_UDPLoop (void *clnt)
{
    NetClient *client = clnt;
    while (client->listening) {
        NetClient_UDPStep (client);
    }
    return NULL;
}


void NetClient_Listen (NetClient *client)
{
    if (!client->listening && client->connected) {
        client->listening = SCE_TRUE;
        pthread_create (&client->tcp_thread, NULL, NetClient_TCPLoop, client);
        if (Socket_IsValidSocket (&client->udpsock))
            pthread_create (&client->udp_thread, NULL, NetClient_UDPLoop, client);
    }
}

int NetClient_PollTCP (NetClient *client)
{
    return Socket_Wait (&client->tcpsock, SOCKET_READ, 0, 0);
}
int NetClient_PollUDP (NetClient *client)
{
    return Socket_Wait (&client->udpsock, SOCKET_READ, 0, 0);
}

int NetClient_WaitTCP (NetClient *client, long sec, long usec)
{
    return Socket_Wait (&client->tcpsock, SOCKET_READ, sec, usec);
}
int NetClient_WaitUDP (NetClient *client, long sec, long usec)
{
    return Socket_Wait (&client->udpsock, SOCKET_READ, sec, usec);
}

int NetClient_TCPStep (NetClient *client, SockID *id)
{
    size_t size;
    const char *data = NULL;
    int r;
    SockID derp;

    if (!id)
        id = &derp;

#define LOOP_TIMEOUT 1
    if ((r = Socket_NbReceiveTCP (&client->tcpsock, LOOP_TIMEOUT, 0)) < 0) {
        if (SCE_Error_GetCode () == SOCKET_PACKET_LOST)
            SCEE_Clear ();      /* ignore packet loss */
        else {
            SCEE_LogSrc ();
            return SCE_ERROR;
        }
    } else if (r == NETWORK_DISCONNECTED) {
        /* it does like NetClient_Disconnect(), except that this function
           waits for TCPLoop() thread to finish */
        NetClient_DisconnectSockets (client);
        NetClient_CallCmd (&client->discocmd, client, NULL, 0);
        client->listening = SCE_FALSE;
        r = SCE_OK;
    } else if (r == NETWORK_TIMEOUT) {
        r = SCE_OK;
    } else {
        data = Socket_GetPacket (&client->tcpsock, id, &size);
        if (data) {
            NetClient_TreatTCPPacket (client, *id, data, size);
            /* TODO: PopPacket() can fail */
            Socket_PopPacket (&client->tcpsock);
        }
    }

    /* check for ping timeout */
    NetClient_CheckTimeout (client);

    return r;
}

SockID NetClient_UDPStep (NetClient *client)
{
    size_t size;
    SockID id;
    char data[SOCKET_UDP_PACKET_SIZE] = {0};
    int r;

    r = Socket_NbReceiveUDP (&client->udpsock, data, sizeof data, &id, &size,
                             &client->server_addr, LOOP_TIMEOUT, 0);
    if (r < 0) {
        if (SCE_Error_GetCode () == SOCKET_PACKET_LOST)
            SCEE_Clear ();      /* ignore packet loss */
        else
            SCEE_LogSrc ();
    } else
        NetClient_TreatUDPPacket (client, id, data, size);

    return r;
}

void NetClient_StopListening (NetClient *client)
{
    if (client->listening) {
        client->listening = SCE_FALSE;
        pthread_join (client->tcp_thread, NULL);
        if (Socket_IsValidSocket (&client->udpsock))
            pthread_join (client->udp_thread, NULL);
    }
}


/* TODO: does not account for the time needed to download the packet */
long NetClient_WaitTCPPacket (NetClient *client, SockID id, time_t delay)
{
    time_t start, current;
    SockID id_;
    unsigned int count = 0;

    id_ = id + 1;
    current = start = time (NULL);

    while (current - start < delay) {
        if (NetClient_WaitTCP (client, delay - (current - start), 0)) {
            if (NetClient_TCPStep (client, &id_) < 0) {
                SCEE_LogSrc ();
                SCEE_Out ();
                return -1;
            }
            if (id_ == id)
                return (long)count;
            else
                count++;
        }
        current = time (NULL);
    }

    return -1;
}
long NetClient_WaitUDPPacket (NetClient *client, SockID id, time_t delay)
{
    time_t start, current;
    SockID id_;
    unsigned int count = 0;

    current = start = time (NULL);

    while (current - start < delay) {
        if (NetClient_WaitUDP (client, delay - (current - start), 0)) {
            id_ = NetClient_UDPStep (client);
            if (id_ == id)
                return (long)count;
            else
                count++;
        }
        current = time (NULL);
    }

    return -1;
}
