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

/* TODO: mark reentrant/nonreentrant functions */

#include <pthread.h>
#include "netserver.h"

void NetServer_InitCmd (NetServerCmd *cmd)
{
    cmd->id = 0;
    cmd->command = NULL;
    cmd->data = NULL;
    SCE_List_InitIt (&cmd->it);
    SCE_List_SetData (&cmd->it, cmd);
}

NetServerCmd* NetServer_NewCmd (int id, NetServerCmdFunc f, void *data)
{
    NetServerCmd *cmd = SCE_malloc (sizeof *cmd);
    if (!cmd) goto failure;
    NetServer_InitCmd (cmd);
    cmd->id = id;
    cmd->command = f;
    cmd->data = data;
    return cmd;
failure:
    NetServer_FreeCmd (cmd), cmd = NULL;
    SCEE_LogSrc ();
    return NULL;
}

void NetServer_FreeCmd (NetServerCmd *cmd)
{
    SCE_free (cmd);
}



static void NetServer_CmdUnknownTCP (NetServer *server, NetClient *client,
                                     void *a, const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("server: unknown TCP command received: %ld\n",
                  Socket_GetID (b));
#endif
    (void)a; (void)b; (void)c; (void)server; (void)client;
}
static void NetServer_CmdUnknownUDP (NetServer *server, NetClient *client,
                                     void *a, const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("server: unknown UDP command received: %ld\n",
                  Socket_GetID (b));
#endif
    (void)a; (void)b; (void)c; (void)server; (void)client;
}
static void NetServer_CmdDisconnect (NetServer *server, NetClient *client,
                                     void *a, const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("server: received disconnect signal from %d.\n", client->ip);
#endif
    (void)a; (void)b; (void)c; (void)server;
}
static void NetServer_CmdPing (NetServer *server, NetClient *client, void *a,
                               const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("server: ping query from %d, sending response.\n",
                  client->ip);
#endif
    (void)a; (void)b; (void)c; (void)server;
}
static void NetServer_CmdTimeout (NetServer *server, NetClient *client, void *a,
                                  const char *b, size_t c)
{
#ifdef TL_DEBUG
    SCEE_SendMsg ("server: client %d timeout (%ds).\n",
                  client->ip, server->timeout);
#endif
    (void)a; (void)b; (void)c; (void)server;
}


static void NetServer_InitDefaultCommands (NetServer *server)
{
    NetServer_InitCmd (&server->tcp_unknowncmd);
    NetServer_InitCmd (&server->udp_unknowncmd);
    NetServer_InitCmd (&server->discocmd);
    NetServer_InitCmd (&server->pingcmd);
    NetServer_InitCmd (&server->timeoutcmd);

    server->tcp_unknowncmd.command = NetServer_CmdUnknownTCP;
    server->udp_unknowncmd.command = NetServer_CmdUnknownUDP;
    server->discocmd.command = NetServer_CmdDisconnect;
    server->pingcmd.command = NetServer_CmdPing;
    server->timeoutcmd.command = NetServer_CmdTimeout;
}
static void NetServer_FreeNetClient (void *client)
{
    NetClient_Free (client);
}
void NetServer_Init (NetServer *server)
{
    Socket_Init (&server->tcpsock);
    Socket_Init (&server->udpsock);
    Socket_SetData (&server->tcpsock, server);
    Socket_SetData (&server->udpsock, server);
    (void)server->infos;
    SCE_List_Init (&server->clients);
    SCE_List_SetFreeFunc (&server->clients, NetServer_FreeNetClient);
    SCE_List_Init (&server->selected);
    pthread_mutex_init (&server->clients_mutex, NULL);
    server->opened = SCE_FALSE;
    server->port = 0;
    server->ip = 0;
    server->timeout = NETSERVER_DEFAULT_TIMEOUT;
    SCE_List_Init (&server->tcp_cmds);
    SCE_List_Init (&server->udp_cmds);
    NetServer_InitDefaultCommands (server);
    server->udata = NULL;
}
void NetServer_Clear (NetServer *server)
{
    NetServer_Close (server);
    pthread_mutex_destroy (&server->clients_mutex);
    SCE_List_Clear (&server->clients);
    SCE_List_Clear (&server->tcp_cmds);
    SCE_List_Clear (&server->udp_cmds);
}

NetServer* NetServer_New (void)
{
    NetServer *server = SCE_malloc (sizeof *server);
    if (server) {
        NetServer_Init (server);
    } else {
        SCEE_LogSrc ();
    }
    return server;
}
void NetServer_Free (NetServer *server)
{
    if (server) {
        NetServer_Clear (server);
        SCE_free (server);
    }
}


void NetServer_SetCmdID (NetServerCmd *cmd, int id)
{
    cmd->id = id;
}
void NetServer_SetCmdCallback (NetServerCmd *cmd, NetServerCmdFunc f)
{
    cmd->command = f;
}
void NetServer_SetCmdData (NetServerCmd *cmd, void *data)
{
    cmd->data = data;
}
void NetServer_CallCmd (NetServerCmd *cmd, NetServer *server, NetClient *client,
                        const char *data, size_t size)
{
    cmd->command (server, client, cmd->data, data, size);
}

SCE_SList* NetServer_GetNetClients (NetServer *server)
{
    return &server->clients;
}

static NetServerCmd* locatecmd (SCE_SList *cmds, SockID id)
{
    NetServerCmd *cmd;
    SCE_SListIterator *it;
    SCE_List_ForEach (it, cmds) {
        cmd = SCE_List_GetData (it);
        if (cmd->id == id)
            return cmd;
    }
    return NULL;
}
static void NetServer_AddCmd (SCE_SList *cmds, NetServerCmd *cmd)
{
    NetServerCmd *old = locatecmd (cmds, cmd->id);
    if (old) {
        SCE_List_Remove (&old->it);
    }
    SCE_List_Appendl (cmds, &cmd->it);
}
void NetServer_AddTCPCmd (NetServer *server, NetServerCmd *cmd)
{
    NetServer_AddCmd (&server->tcp_cmds, cmd);
}
void NetServer_AddUDPCmd (NetServer *server, NetServerCmd *cmd)
{
    NetServer_AddCmd (&server->udp_cmds, cmd);
}

void NetServer_SetTCPUnknownCmd (NetServer *server, NetServerCmd *cmd)
{
    if (cmd)
        server->tcp_unknowncmd = *cmd;
    else {
        NetServer_InitCmd (&server->tcp_unknowncmd);
        server->tcp_unknowncmd.command = NetServer_CmdUnknownTCP;
    }
}
void NetServer_SetUDPUnknownCmd (NetServer *server, NetServerCmd *cmd)
{
    if (cmd)
        server->udp_unknowncmd = *cmd;
    else {
        NetServer_InitCmd (&server->udp_unknowncmd);
        server->udp_unknowncmd.command = NetServer_CmdUnknownUDP;
    }
}
void NetServer_SetDisconnectCommand (NetServer *server, NetServerCmd *cmd)
{
    if (cmd)
        server->discocmd = *cmd;
    else {
        NetServer_InitCmd (&server->discocmd);
        server->discocmd.command = NetServer_CmdDisconnect;
    }
}
void NetServer_SetPingCommand (NetServer *server, NetServerCmd *cmd)
{
    if (cmd)
        server->pingcmd = *cmd;
    else {
        NetServer_InitCmd (&server->pingcmd);
        server->pingcmd.command = NetServer_CmdPing;
    }
}
void NetServer_SetPingTimeoutCommand (NetServer *server, NetServerCmd *cmd)
{
    if (cmd)
        server->timeoutcmd = *cmd;
    else {
        NetServer_InitCmd (&server->timeoutcmd);
        server->timeoutcmd.command = NetServer_CmdTimeout;
    }
}

static NetClient* NetServer_LocateNetClient (NetServer *server, SockID udp_id)
{
    NetClient *client;
    SCE_SListIterator *it;
    SCE_List_ForEach (it, &server->clients) {
        client = SCE_List_GetData (it);
        if (client->udp_id == udp_id)
            return client;
    }
    return NULL;
}

#if 0
static void NetServer_CheckTimeout (NetServer *server, NetClient *client)
{
    if (time (NULL) - client->tm >= server->timeout && client->connected) {
        /* ping timeout ! */
        NetServer_CallCmd (&server->timeoutcmd, server, client, NULL, 0);
        client->connected = SCE_FALSE;
    }
}
#endif

static void
NetServer_TreatTCPPacket (NetServer *server, NetClient *client, SockID id,
                          const char *data, size_t size)
{
    NetServerCmd *cmd = NULL;

    client->tm = time (NULL);

    switch (id) {
    case NETCLIENT_PONG:
        /* TODO: store exact time */
        /* client->pongtm_s = time (NULL), client->pongtm_us = foo (); */
        break;
    case NETCLIENT_PING:
        NetServer_SendTCP (server, client, NETCLIENT_PONG, NULL, 0);
        NetServer_CallCmd (&server->pingcmd, server, client, data, size);
        break;
    default:
        cmd = locatecmd (&server->tcp_cmds, id);
        if (cmd) {
            NetServer_CallCmd (cmd, server, client, data, size);
        } else {
            unsigned char buf[4] = {0};
            SCE_Encode_Long (id, buf);
            NetServer_CallCmd (&server->tcp_unknowncmd, server, client,
                               buf, size);
        }
    }
}

static void NetServer_TreatUDPPacket (NetServer *server, NetClient *client,
                                      SockID id, const char *data, size_t size)
{
    NetServerCmd *cmd = NULL;
    client->tm = time (NULL);
    cmd = locatecmd (&server->udp_cmds, id);
    if (!cmd) {
        NetServer_CallCmd (cmd, server, client, &data[3 * SOCKET_ID_SIZE],size);
    } else {
        unsigned char buf[4] = {0};
        SCE_Encode_Long (id, buf);
        NetServer_CallCmd (&server->udp_unknowncmd, server, client,
                           buf, size);
    }
}

static void NetServer_UDPStep (NetServer *server, SockID id, const char *data,
                               size_t size, SOCKADDR_IN *addr)
{
    SockID udp_id;
    NetClient *client = NULL;

    udp_id = Socket_GetID (&data[2 * SOCKET_ID_SIZE]);
    client = NetServer_LocateNetClient (server, udp_id);
    if (!client) {
        /* a motherfucker is trying to send crap on my VIP server */
        /* TODO: get his IP and IP BAN HAHAHA §§§§ */
#ifdef TL_DEBUG
        SCEE_SendMsg ("server: warning: received UDP packet from an "
                      "unauthentified UDP client. id: %ld; ip: %d.\n",
                      udp_id, addr->sin_addr.s_addr);
#endif
        return;
    }
    /* check if IP address is valid */
    /* TODO: check fail */
#if 0
    if (Socket_CompareAddress (&client->server_addr, addr) ==
        (NETWORK_ADDRESS | NETWORK_PORT))
#else
    if (1)
#endif
    {
        /* TODO: wtf, sending UDP packets doesn't work otherwise */
        /* TODO: update CompareAddress() function and fix that. */
        client->server_addr = *addr;
        NetServer_TreatUDPPacket (server, client, id, data, size);
    } else {
        /* HOLY SHIT! a fucking bastard stole the UDP ID of one of
           my clients!!11one or the client just changed his IP ? */
#ifdef TL_DEBUG
        SCEE_SendMsg ("server: fucking bastard stole UDP id %ld.\n",
                      udp_id);
#endif
    }
}

static int NetServer_IsUDPIDUsed (NetServer *server, int id)
{
    SCE_SListIterator *it = NULL;
    SCE_List_ForEach (it, &server->clients) {
        NetClient *client = SCE_List_GetData (it);
        if (client->udp_id == id)
            return SCE_TRUE;
    }
    return SCE_FALSE;
}
/* TODO: update dat lol */
static int Rand_Range (int a, int b) { return a + b / 2; } /* xxd */
static int NetServer_GetUDPID (NetServer *server)
{
    static int id = 42;
#if 0
    do {
        id = Rand_Range (0, 2000);
    } while (NetServer_IsUDPIDUsed (server, id));
#else
    id++;
#endif
    return id;
}

static NetClient* NetServer_AddNewNetClient (NetServer *server, Sock *sock,
                                             SOCKADDR_IN *client_addr)
{
    NetClient *client = NetClient_New ();
    if (!client) {
        SCEE_LogSrc ();
        return NULL;
    }
    Socket_Copy (&client->tcpsock, sock);
    client->server_addr = *client_addr;
    client->serv = server;
    client->ip = Socket_GetAddressFromSockAddr (client_addr);
    client->port = Socket_GetPortFromSockAddr (client_addr);
    pthread_mutex_lock (&server->clients_mutex);
    /* full server side UDP ID generation: unpredictable for clients */
    client->udp_id = NetServer_GetUDPID (server);
    SCE_List_Appendl (&server->clients, &client->it);
    pthread_mutex_unlock (&server->clients_mutex);
    return client;
}

/* accept new connection */
static int NetServer_Accept (NetServer *server)
{
    int err;
    Sock newsock;
    SOCKADDR_IN addr;
    NetClient *client = NULL;

    Socket_Init (&newsock);
    err = Socket_Accept (&server->tcpsock, &newsock, &addr);
    if (err < 0) {
#ifdef TL_DEBUG
        SCEE_LogSrc (), SCEE_Out ();
#endif
        SCEE_Clear ();
    } else if (err > 0) {
        char data[2 * SOCKET_ID_SIZE] = {0};
        if (!(client = NetServer_AddNewNetClient (server, &newsock, &addr))) {
            SCEE_LogSrc ();
            Socket_Clear (&newsock);
            return SCE_ERROR;
        }
        Socket_SetID (data, (int)server->timeout);
        Socket_SetID (&data[SOCKET_ID_SIZE], client->udp_id);
        NetServer_SendTCP (server, client, NETCLIENT_CONNECTION_DATA, data,
                           2 * SOCKET_ID_SIZE);
        client->connected = SCE_TRUE;
        client->tm = time (NULL);
#ifdef TL_DEBUG
        SCEE_SendMsg ("server: new connection: %d\n", client->ip);
#endif
    }
    Socket_Clear (&newsock);
    return SCE_OK;
}

/* remove disconnected clients */
static void NetServer_RemoveNetClients (NetServer *server)
{
    SCE_SListIterator *it = NULL, *pro = NULL;
    NetClient *client = NULL;
    SCE_List_ForEachProtected (pro, it, &server->clients) {
        client = SCE_List_GetData (it);
        if (!client->connected)
            NetClient_Free (client);
    }
}

static int NetServer_ReceiveTCP (NetServer *server, NetClient *client)
{
    int r;

    r = Socket_ReceiveTCP (&client->tcpsock);
    switch (r) {
    case SCE_ERROR:
        SCEE_LogSrc(), SCEE_Out ();
        SCEE_Clear ();
        /* kick client */
    case NETWORK_DISCONNECTED:
        /* no need to close sockets, NetClient_Free() will ensure that,
           see TreatSelected() */
        NetServer_CallCmd (&server->discocmd, server, client, NULL, 0);
        client->connected = SCE_FALSE;
    case NETWORK_TIMEOUT:
        /* I'm not sure but I feel like Socket_ReceiveTCP() cannot
           return NETWORK_TIMEOUT. */
        r = SCE_OK;
        break;
    }
    return r;
}

static void NetServer_TreatSelected (NetServer *server)
{
    char data[SOCKET_UDP_PACKET_SIZE] = {0};
    size_t size;
    SockID id;
    SCE_SListIterator *it = NULL, *pro = NULL;
    NetClient *client = NULL;
    int r;

    /* treat selected sockets */
    /* protected, just in case a client gets mysteriously removed */
    SCE_List_ForEachProtected (pro, it, &server->selected) {
        /* check for specific cases */
        if (it == Socket_GetIterator (&server->tcpsock)) {
            /* new connection */
            /* TODO: hope that this socket wont be used for something else */
            NetServer_Accept (server);
        } else if (it == Socket_GetIterator (&server->udpsock)) {
            /* UDP packet received */
            SOCKADDR_IN addr;
            if ((r = Socket_ReceiveUDP (&server->udpsock, data, sizeof data,
                                        &id, &size, &addr)) < 0) {
                SCEE_LogSrc(), SCEE_Out ();
                SCEE_Clear ();
            } else if (r == SCE_OK)
                NetServer_UDPStep (server, id, data, size, &addr);
            /* NOTE: r cannot be something else than SCE_ERROR, SCE_OK or
               NETWORK_TIMEOUT, and we do nothing in case of timeout */
        } else {
            const char *ptr = NULL;
            client = Socket_GetData (SCE_List_GetData (it));
            NetServer_ReceiveTCP (server, client);
            ptr = Socket_GetPacket (&client->tcpsock, &id, &size);
            if (ptr) {
                NetServer_TreatTCPPacket (server, client, id, ptr, size);
                Socket_PopPacket (&client->tcpsock);
            }
        }
    }

    /* flush before remove clients */
    SCE_List_Flush (&server->selected);

    /* remove disconnected clients */
    NetServer_RemoveNetClients (server);
}


void NetServer_SetData (NetServer *server, void *data)
{
    server->udata = data;
}
void* NetServer_GetData (NetServer *server)
{
    return server->udata;
}

void NetServer_SetTimeOut (NetServer *server, int timeout)
{
    server->timeout = timeout;
}
int NetServer_GetTimeOut (NetServer *server)
{
    return server->timeout;
}


int NetServer_Open (NetServer *server, int port)
{
    if (server->opened) {
#ifdef TL_DEBUG
        SCEE_SendMsg ("warning: server already opened.\n");
#endif
        return SCE_OK;
    }
    server->opened = SCE_TRUE;
    if (Socket_InitTCP (&server->tcpsock, SCE_TRUE) < 0) {
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
    if (Socket_InitUDP (&server->udpsock, SCE_FALSE) < 0) {
        Socket_Close (&server->tcpsock);
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
    server->port = port;
    Socket_SetupAddr (&server->infos, htonl (INADDR_ANY), port);
    if (Socket_Bind (&server->tcpsock, &server->infos) < 0 ||
        Socket_Bind (&server->udpsock, &server->infos) < 0) {
        Socket_Close (&server->tcpsock);
        Socket_Close (&server->udpsock);
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
#define LISTEN_QUEUE_LENGTH 8
    Socket_Listen (&server->tcpsock, LISTEN_QUEUE_LENGTH);
#ifdef TL_DEBUG
    SCEE_SendMsg ("server opened.\n");
#endif
    return SCE_OK;
}

/**
 * \brief Waits until a packet is available for reading,
 * may also be a connection request
 * \returns a boolean indicating if either a packet has been received or not
 * SCE_ERROR on error :>
 */
int NetServer_Wait (NetServer *server, long sec, long usec)
{
    SCE_SListIterator *it = NULL;

    SCE_List_Init (&server->selected);

    /* fill list (O(n) bouh) */
    /* put TCP socket */
    SCE_List_Appendl (&server->selected, Socket_GetIterator (&server->tcpsock));
    /* put UDP socket */
    SCE_List_Appendl (&server->selected, Socket_GetIterator (&server->udpsock));
    /* fill TCP client sockets */
    SCE_List_ForEach (it, &server->clients) {
        NetClient *client = SCE_List_GetData (it);
        SCE_List_Appendl (&server->selected, Socket_GetIterator (&client->tcpsock));
    }

    /* proceed select */
    if (Socket_SelectRead (&server->selected, sec, usec) < 0) {
        SCEE_LogSrc ();
        return SCE_ERROR;
    }

    return SCE_List_HasElements (&server->selected);
}
/**
 * \brief Calls NetServer_Wait(\p server, 0, 0)
 */
int NetServer_Poll (NetServer *server)
{
    return NetServer_Wait (server, 0, 0);
}
/**
 * \brief Does one server step, it flushes all pending packets
 */
void NetServer_Step (NetServer *server)
{
    NetServer_TreatSelected (server);
}

static void NetServer_KickClients (NetServer *server, const char *reason)
{
    SCE_SListIterator *it, *pro;
    SCE_List_ForEachProtected (pro, it, &server->clients)
        NetServer_KickClient (server, SCE_List_GetData (it), reason);
}

/**
 * \brief Close the server, unbind sockets and free the port
 * \sa NetServer_Open(), NetServer_Stop()
 */
void NetServer_Close (NetServer *server)
{
    if (server->opened) {
        NetServer_KickClients (server, "server closed");
        NetServer_RemoveNetClients (server);
        /* close sockets */
        Socket_Shutdown (&server->tcpsock);
        Socket_Close (&server->tcpsock);
        Socket_Close (&server->udpsock);
#ifdef TL_DEBUG
        SCEE_SendMsg ("server closed.\n");
#endif
    }
}

void NetServer_KickClient (NetServer *server, NetClient *client,
                           const char *reason)
{
    NetServer_SendTCP (server, client, NETWORK_DISCONNECTED, (char*)reason,
                       (reason ? strlen (reason) + 1 : 0));
    client->connected = SCE_FALSE;
}

int NetServer_SendUDP (NetServer *server, NetClient *client, int id,
                       const char *data, size_t size)
{
    int err;
    err = Socket_SendUDP (&server->udpsock, id, data, size,
                          &client->server_addr);
    /* client->server_addr used as the address of the client */
    return err;
}
int NetServer_SendTCP (NetServer *server, NetClient *client, int id,
                       const char *data, size_t size)
{
    int err;
    (void)server;
    err = Socket_SendTCP (&client->tcpsock, id, data, size);
    return err;
}
int NetServer_StreamTCP (NetServer *server, NetClient *client, int id,
                         const char *data, size_t size, size_t total_size)
{
    int err;
    (void)server;
    err = Socket_StreamTCP (&client->tcpsock, id, data, size, total_size);
    return err;
}

int NetServer_SendUDPString (NetServer *server, NetClient *client, int id,
                             const char *data)
{
    if (!data)
        return NetServer_SendUDP (server, client, id, NULL, 0);
    else
        return NetServer_SendUDP (server, client, id, data, strlen (data) + 1);
}
int NetServer_SendTCPString (NetServer *server, NetClient *client, int id,
                             const char *data)
{
    if (!data)
        return NetServer_SendTCP (server, client, id, NULL, 0);
    else
        return NetServer_SendTCP (server, client, id, data, strlen (data) + 1);
}
