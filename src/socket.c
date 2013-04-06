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

#include <string.h>
#include <ctype.h>
#include <curl/curl.h>
#include <SCE/utils/SCEUtils.h>
#include "socket.h"

#include "netprotocol.h"        /* debug msg */

int Init_Socket (void)
{
    if (curl_global_init (CURL_GLOBAL_WIN32) != 0) {
        /* TODO: curl error managment */
        SCEE_Log (35);
        SCEE_LogMsg ("cURL has failed: %s\n", "but i dunno why");
        return SCE_ERROR;
    }
    return SCE_OK;
}
void Quit_Socket (void)
{
    curl_global_cleanup ();
}



void Socket_Init (Sock *sock)
{
    sock->buffer = NULL;
    sock->bufsize = 0;
    SCE_Array_Init (&sock->stream);
    sock->udata = NULL;
    SCE_List_InitIt (&sock->it);
    SCE_List_SetData (&sock->it, sock);
}
void Socket_Clear (Sock *sock)
{
    SCE_free (sock->buffer);
    SCE_Array_Clear (&sock->stream);
    SCE_List_Remove (&sock->it);
}

int Socket_InitUDP (Sock *sock, int nonblock)
{
    int flags = SOCK_DGRAM;
    sock->sock = socket (AF_INET, flags, 0/*IPPROTO_UDP*/);
    if (sock->sock == INVALID_SOCKET) {
        SCEE_LogErrno ("socket() returned an invalid UDP/IP socket");
        return SCE_ERROR;
    }
    if (nonblock)
        fcntl (sock->sock, F_SETFL, O_NONBLOCK | fcntl (sock->sock, F_GETFL));
    return SCE_OK;
}
int Socket_InitTCP (Sock *sock, int nonblock)
{
    int flags = SOCK_STREAM;
    sock->sock = socket (AF_INET, flags, 0/*IPPROTO_TCP*/);
    if (sock->sock == INVALID_SOCKET) {
        SCEE_LogErrno ("socket() returned an invalid TCP/IP socket");
        return SCE_ERROR;
    }
    if (nonblock)
        fcntl (sock->sock, F_SETFL, O_NONBLOCK | fcntl (sock->sock, F_GETFL));
    return SCE_OK;
}
int Socket_IsValidSocket (Sock *sock)
{
    return (sock->sock != INVALID_SOCKET);
}
void Socket_Copy (Sock *dst, Sock *src)
{
    dst->sock = src->sock;
#if 0
    dst->buffer = src->buffer;
    dst->bufsize = src->bufsize;
    dst->udata = src->udata;
#endif
}


int Socket_Close (Sock *sock)
{
    if (closesocket (sock->sock) < 0) {
        SCEE_LogErrno ("closesocket() failed");
        return SCE_ERROR;
    }
    return SCE_OK;
}
int Socket_Shutdown (Sock *sock)
{
    if (shutdown (sock->sock, SHUT_RDWR) < 0) {
        SCEE_LogErrno ("shutdown() failed");
        return SCE_ERROR;
    }
    return SCE_OK;
}


static int verifyaddressipv4 (const char *address_)
{
    size_t i, n_nums;
    const char *address = address_;
    for (n_nums = 0; n_nums < 4; address++, n_nums++) {
        for (i = 0; i < 3; i++, address++)
            if (!isdigit (*address)) break;
        if (i == 0) goto err;
        if (*address == 0) break;
        if (*address != '.') goto err;
    }
    if (n_nums != 3) goto err;
    goto ok;
err:
    SCEE_Log (SCE_INVALID_ARG);
    SCEE_LogMsg ("invalid IPv4 address '%s'", address_);
    return SCE_ERROR;
ok:
    return SCE_OK;
}

int Socket_GetAddressFromStringv (const char *s, in_addr_t *addr)
{
    struct in_addr a;
    if (verifyaddressipv4 (s) < 0) {
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
    if (inet_aton (s, &a) == 0) {
        char b[256] = {0};
        snprintf (b, sizeof b, "inet_aton() failed to convert '%s'", s);
        SCEE_LogErrno (b);
        return SCE_ERROR;
    }
    *addr = a.s_addr;
    return SCE_OK;
}
int Socket_GetPortFromString (const char *s)
{
    long port;
    errno = 0;
    port = strtol (s, NULL, 10);
    if (errno != 0 && port <= 0) {
        SCEE_Log (errno);
        SCEE_LogMsg ("failed to convert string '%s' to a long", s);
        return SCE_ERROR;
    }
    return (int)port;
}
int Socket_GetAddressAndPortFromStringv (const char *str, in_addr_t *addr,
                                         int *port)
{
    in_addr_t a;
    int p = 0;                  /* = 0 otherwise it may be used uninitialized */
    char *c = NULL;
    char *s = SCE_malloc (strlen (str) + 1);
    if (!s) {
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
    strcpy (s, str);
    c = strchr (s, ':');
    if (c) {
        if ((p = Socket_GetPortFromString (&c[1])) < 0) {
            SCE_free (s);
            SCEE_LogSrc ();
            return SCE_ERROR;
        }
        *c = '\0';
    }
    if (Socket_GetAddressFromStringv (s, &a) < 0) {
        SCE_free (s);
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
    SCE_free (s);
    *addr = a;
    if (c)
        *port = p;
    return SCE_OK;
}

void Socket_SetupAddr (SOCKADDR_IN *addr, in_addr_t ip, int port)
{
    addr->sin_family = AF_INET;  /* TODO: add IPv6 */
    addr->sin_port = htons (port);
    addr->sin_addr.s_addr = ip;
}
int Socket_SetupAddrFromString (SOCKADDR_IN *addr, const char *ip, int port)
{
    in_addr_t inaddr;
    if (Socket_GetAddressFromStringv (ip, &inaddr) < 0) {
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
    Socket_SetupAddr (addr, inaddr, port);
    return SCE_OK;
}
int Socket_SetupAddrFullFromString (SOCKADDR_IN *addr, const char *ipport)
{
    in_addr_t inaddr;
    int port;
    if (Socket_GetAddressAndPortFromStringv (ipport, &inaddr, &port) < 0) {
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
    Socket_SetupAddr (addr, inaddr, port);
    return SCE_OK;
}

in_addr_t Socket_GetAddressFromSockAddr (SOCKADDR_IN *addr)
{
    return addr->sin_addr.s_addr;
}
int Socket_GetPortFromSockAddr (SOCKADDR_IN *addr)
{
    return htons (addr->sin_port); /* TODO: wat hton. */
    /* TODO: use ntohs tkt */
}
int Socket_CompareAddress (SOCKADDR_IN *a, SOCKADDR_IN *b)
{
    int r = 0;
#if 0
    return (!memcmp (a, b, sizeof *b) ? NETWORK_ADDRESS | NETWORK_PORT : 0);
#else
    if (Socket_GetAddressFromSockAddr (a) == Socket_GetAddressFromSockAddr (b))
        r |= NETWORK_ADDRESS;
    if (Socket_GetPortFromSockAddr (a) == Socket_GetPortFromSockAddr (b))
        r |= NETWORK_PORT;
#endif
    return r;
}
char* Socket_GetAddressStringFromSockAddr (SOCKADDR_IN *addr)
{
    return inet_ntoa (addr->sin_addr);
}

SockID Socket_GetID (const char *data)
{
    return SCE_Decode_Long ((const unsigned char*)data);
}
void Socket_SetID (char *data, SockID id)
{
    SCE_Encode_Long (id, (unsigned char*)data);
}


void Socket_SetData (Sock *sock, void *data)
{
    sock->udata = data;
}
void* Socket_GetData (Sock *sock)
{
    return sock->udata;
}
SCE_SListIterator* Socket_GetIterator (Sock *sock)
{
    return &sock->it;
}


int Socket_Bind (Sock *sock, SOCKADDR_IN *addr)
{
    int err = bind (sock->sock, (SOCKADDR*)addr, sizeof *addr);
    if (err != 0) {
        char str[128] = {0};
        sprintf (str, "bind() failed to bind to port %d",
                 Socket_GetPortFromSockAddr (addr));
        SCEE_LogErrno (str);
        return SCE_ERROR;
    }
    return SCE_OK;
}
int Socket_Listen (Sock *sock, int num)
{
    int err = listen (sock->sock, num);
    if (err != 0) {
        SCEE_LogErrno ("listen() failed");
        return SCE_ERROR;
    }
    return SCE_OK;
}
int Socket_Connect (Sock *sock, SOCKADDR_IN *addr)
{
    int err = connect (sock->sock, (SOCKADDR*)addr, sizeof *addr);
    if (err != 0) {
        char str[128] = {0};
        /* TODO: ntoa() is posix only */
        sprintf (str, "connect() failed to connect to server %s",
                 Socket_GetAddressStringFromSockAddr (addr));
        SCEE_LogErrno (str);
        return SCE_ERROR;
    }
    return SCE_OK;
}
/**
 * \brief Bound to accept(2).
 * \returns 1: connection has been received, 0: no connection, -1: error
 */
int Socket_Accept (Sock *sock, Sock *sock2, SOCKADDR_IN *addr)
{
    socklen_t len = sizeof *addr;
    sock2->sock = accept (sock->sock, (SOCKADDR*)addr, &len);
    if (sock2->sock == INVALID_SOCKET) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;
        /* TODO: handle ECONNABORTED */
        SCEE_LogErrno ("accept() returned an invalid socket");
        return SCE_ERROR;
    }
    return 1;
}

int Socket_SendTCP (Sock *sock, SockID id, const char *data, size_t size)
{
    ssize_t n_bytes;
    char buffer[2 * SOCKET_ID_SIZE] = {0};

    Socket_SetID (buffer, (SockID)(size + 2 * SOCKET_ID_SIZE));
    Socket_SetID (&buffer[SOCKET_ID_SIZE], id);
    n_bytes  = send (sock->sock, buffer, 2 * SOCKET_ID_SIZE, 0);
    n_bytes += send (sock->sock, data, size, 0);
    if (n_bytes == (ssize_t)(size + 2 * SOCKET_ID_SIZE)) {
#ifdef TL_DEBUG
        Tlp_Send (NULL, id);
#endif
        return SCE_OK;
    } else if (n_bytes < 0) {
        SCEE_LogErrno ("send()");
        return SCE_ERROR;
    } else
        return /*SCE_ERROR*/SCE_OK; /* ptdr */
}
int Socket_StreamTCP (Sock *sock, SockID id, const char *data, size_t size,
                      size_t total_size)
{
    ssize_t n_bytes = 0, theoretical;

    theoretical = size;
    if (id != NETWORK_CONTINUE_STREAM) {
        char buffer[2 * SOCKET_ID_SIZE] = {0};
        Socket_SetID (buffer, (SockID)(total_size + 2 * SOCKET_ID_SIZE));
        Socket_SetID (&buffer[SOCKET_ID_SIZE], id);
        n_bytes = send (sock->sock, buffer, 2 * SOCKET_ID_SIZE, 0);
        theoretical += 2 * SOCKET_ID_SIZE;
    }

    n_bytes += send (sock->sock, data, size, 0);
    if (n_bytes == theoretical) {
#ifdef TL_DEBUG
        Tlp_Send (NULL, id);
#endif
        return SCE_OK;
    } else if (n_bytes < 0) {
        SCEE_LogErrno ("send()");
        return SCE_ERROR;
    } else
        return /*SCE_ERROR*/SCE_OK; /* ptdr */
}
int Socket_SendUDP (Sock *sock, SockID id, const char *data, size_t size,
                    SOCKADDR_IN *info)
{
    ssize_t n_bytes;
    /* reallocate if necessary */
    if (size + 2 * SOCKET_ID_SIZE > sock->bufsize) {
        SCE_free (sock->buffer);
        sock->bufsize = size + 2 * SOCKET_ID_SIZE;
        sock->buffer = SCE_malloc (sock->bufsize);
        if (!sock->buffer) {
            SCEE_LogSrc ();
            return SCE_ERROR;
        }
    }
    memset (sock->buffer, 0, sock->bufsize);
    Socket_SetID (sock->buffer, (SockID)(size + 2 * SOCKET_ID_SIZE));
    Socket_SetID (&sock->buffer[SOCKET_ID_SIZE], id);
    memcpy (&sock->buffer[2 * SOCKET_ID_SIZE], data, size);
    n_bytes = sendto (sock->sock, sock->buffer, size + 2 * SOCKET_ID_SIZE, 0,
                      (SOCKADDR*)info, sizeof *info);
    if (n_bytes == (ssize_t)(size + 2 * SOCKET_ID_SIZE)) {
        return SCE_OK;
    } else if (n_bytes < 0) {
        SCEE_LogErrno ("sendto()");
        return SCE_ERROR;
    } else
        return /*SCE_ERROR*/SCE_OK; /* ptdr */
}

SocketState Socket_ReceiveTCP (Sock *sock)
{
#define SOCKET_BUFFER_SIZE 256
    if (sock->nonblock) {
        char buf[SOCKET_BUFFER_SIZE] = {0};
        ssize_t n_bytes;

        n_bytes = recv (sock->sock, buf, sizeof buf, 0);
        sock->nonblock = SCE_FALSE;
        if (n_bytes < 0) {
            SCEE_LogErrno ("recv()");
            return SCE_ERROR;
        } else if (n_bytes == 0) {
            return NETWORK_DISCONNECTED;
        } else if (SCE_Array_Append (&sock->stream, buf, (size_t)n_bytes) < 0) {
            SCEE_LogSrc ();
            return SCE_ERROR;
        }
    }
    return SCE_OK;
}
SocketState Socket_ReceiveUDP (Sock *sock, char *data, size_t size,
                               SockID *id, size_t *rcvsize, SOCKADDR_IN *info)
{
    int err = SCE_ERROR;
    ssize_t n_bytes;
    socklen_t addr_size = sizeof *info;
    n_bytes = recvfrom (sock->sock, data, size, 0,
                        (SOCKADDR*)info, &addr_size);
    if (n_bytes < 0) {
        SCEE_LogErrno ("recvfrom()");
    } else if (n_bytes >= 2 * SOCKET_ID_SIZE &&
               n_bytes >= Socket_GetID (data)) {
        *id = (int)Socket_GetID (&data[SOCKET_ID_SIZE]);
        if (rcvsize)
            *rcvsize = (size_t)Socket_GetID (data) - 2 * SOCKET_ID_SIZE;
        err = SCE_OK;
    } else {
        SCEE_Log (SOCKET_PACKET_LOST);
        SCEE_LogMsg ("packet lost.");
    }
    return err;
}


const void* Socket_GetPacket (const Sock *sock, SockID *id, size_t *size)
{
    size_t ssize;

    ssize = SCE_Array_GetSize (&sock->stream);
    if (ssize >= SOCKET_ID_SIZE) {
        SockID s;
        char *buf = NULL;

        buf = SCE_Array_Get (&sock->stream);
        s = Socket_GetID (buf);
        if (ssize >= (size_t)s) {
            *size = (size_t)s - 2 * SOCKET_ID_SIZE;
            *id = Socket_GetID (&buf[SOCKET_ID_SIZE]);
            return &buf[2 * SOCKET_ID_SIZE];
        }
    }

    return NULL;
}

int Socket_PopPacket (Sock *sock)
{
    const void *data = NULL;
    size_t size;
    SockID id;

    if ((data = Socket_GetPacket (sock, &id, &size))) {
        if (SCE_Array_PopFront (&sock->stream, size + 2 * SOCKET_ID_SIZE) < 0) {
            SCEE_LogSrc ();
            return SCE_ERROR;
        }
    }
    return SCE_OK;
}

static int Socket_PacketAvailable (Sock *sock)
{
    SockID id;
    size_t size;
    return Socket_GetPacket (sock, &id, &size) == NULL ? SCE_FALSE : SCE_TRUE;
}


int Socket_Wait (Sock *so, int flags, long sec, long usec)
{
    fd_set readfds, writefds, exceptfds;
    fd_set *readfds_p = NULL, *writefds_p = NULL, *exceptfds_p = NULL;
    struct timeval outime;
    int err;
    SOCKET sock = so->sock;
    /* there is something to read */
    if (Socket_PacketAvailable (so))
        return 1;
    FD_ZERO (&readfds); FD_SET (sock, &readfds);
    FD_ZERO (&writefds); FD_SET (sock, &writefds);
    FD_ZERO (&exceptfds); FD_SET (sock, &exceptfds);
    outime.tv_sec = sec;
    outime.tv_usec = usec;
    if (flags & SOCKET_READ) readfds_p = &readfds;
    if (flags & SOCKET_WRITE) writefds_p = &writefds;
    if (flags & SOCKET_EXCEPT) exceptfds_p = &exceptfds;
    err = select (sock + 1, readfds_p, writefds_p, exceptfds_p, &outime);
    if (err < 0)
        SCEE_LogErrno ("select()");
    if (err == 1 && flags & SOCKET_READ)
        so->nonblock = SCE_TRUE;
    return err;
}

int Socket_SelectRead (SCE_SList *socks, long sec, long usec)
{
    fd_set readfds, exceptfds;
    struct timeval outime;
    int err;
    SOCKET sockmax = 0;                /* max */
    SCE_SListIterator *it = NULL, *pro = NULL;

    FD_ZERO (&readfds);
    FD_ZERO (&exceptfds);
    SCE_List_ForEach (it, socks) {
        Sock *sock = SCE_List_GetData (it);
        FD_SET (sock->sock, &readfds);
        FD_SET (sock->sock, &exceptfds);
        if (sock->sock > sockmax)
            sockmax = sock->sock;
    }
    outime.tv_sec = sec;
    outime.tv_usec = usec;
    err = select (sockmax + 1, &readfds, NULL, &exceptfds, &outime);
    if (err < 0) {
        SCEE_LogErrno ("select()");
        return SCE_ERROR;
    }

    /* remove from the list those who have no packet available */
    SCE_List_ForEachProtected (pro, it, socks) {
        Sock *sock = SCE_List_GetData (it);
        if (FD_ISSET (sock->sock, &readfds))
            sock->nonblock = SCE_TRUE;
        else if (!Socket_PacketAvailable (sock))
            SCE_List_Removel (it);

        if (FD_ISSET (sock->sock, &exceptfds)) {
            SCEE_Log (84554);
            SCEE_LogMsg ("socket error pending");
            return SCE_ERROR;
        }
    }
    return SCE_OK;
}

#if 0
int Socket_PollRead (SCE_SList *socks, int flags, long sec, long usec)
{
    /* using poll()/epoll() */
}
#endif

/* non blocking versions of Receive() */
SocketState Socket_NbReceiveTCP (Sock *so, long sec, long usec)
{
    int err;

    err = Socket_Wait (so, SOCKET_READ, sec, usec);
    if (err == 0) {
        /* timeout: no packet received */
        err = NETWORK_TIMEOUT;
    } else if (err < 0) {
        /* error occured! */
        SCEE_LogSrc ();
    } else {
        err = Socket_ReceiveTCP (so);
        if (err < 0)
            SCEE_LogSrc ();
    }
    return err;
}
SocketState Socket_NbReceiveUDP (Sock *so, char *data, size_t size,
                                 SockID *id, size_t *rcvsize, SOCKADDR_IN *addr,
                                 long sec, long usec)
{
    int err;

    err = Socket_Wait (so, SOCKET_READ, sec, usec);
    if (err == 0) {
        /* timeout: no packet received */
        err = NETWORK_TIMEOUT;
    } else if (err < 0) {
        /* error occured! */
        SCEE_LogSrc ();
    } else {
        memset (data, 0, size); /* hmm.. */
        err = Socket_ReceiveUDP (so, data, size, id, rcvsize, addr);
        if (err < 0)
            SCEE_LogSrc ();
    }
    return err;
}


static int Socket_PrepareDownload (const char *url, const char *dst,
                                   const char **fname, const char **path)
{
    *fname = strrchr (url, '/');
    if (!*fname)
        *fname = url;            /* hihi */
    if (!(*path = SCE_String_CombinePaths (dst, *fname)))
        return SCE_ERROR;
    return SCE_OK;
}


static size_t Socket_OwnWrite (const void *a, size_t b, size_t c, FILE *d)
{
    (void)a; (void)d;
    return b * c;
}
static int Socket_GetLenghtFunc (void *len, double size, double b,
                                 double c, double d)
{
    if (size > 0.1) {
        int *length = len;
        *length = (int)size;
        return 42;              /* stop the download process */
    }
    return 0;
}
/**
 * \brief Gives the size of a distant file
 * \param url URL of the file
 * \returns the size in bytes
 */
int Socket_DownloadLength (const char *url)
{
    char curlerr[CURL_ERROR_SIZE] = {0};
    CURL *handle = NULL;
    int length = 0, i;

    handle = curl_easy_init ();
    curl_easy_setopt (handle, CURLOPT_ERRORBUFFER, curlerr);
    curl_easy_setopt (handle, CURLOPT_URL, url);
    curl_easy_setopt (handle, CURLOPT_WRITEFUNCTION, Socket_OwnWrite);
    curl_easy_setopt (handle, CURLOPT_WRITEDATA, (FILE*)((size_t)NULL + 42));
    curl_easy_setopt (handle, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt (handle, CURLOPT_PROGRESSDATA, (void*)&length);
    curl_easy_setopt (handle, CURLOPT_PROGRESSFUNCTION, Socket_GetLenghtFunc);
    /* do 3 tries */
    for (i = 0; i < 3; i++) {
        int err = curl_easy_perform (handle);
        if (err == 0 || err == CURLE_ABORTED_BY_CALLBACK)
            break;
    }
    curl_easy_cleanup (handle);

    return length;
}


int Socket_Download (const char *url, const char *dst, int tries, void *udata,
                     Socket_CurlFunc fun)
{
    char curlerr[CURL_ERROR_SIZE] = {0};
    CURL *handle = NULL;
    FILE *fp = NULL;
    const char *fname = NULL;
    const char *path = NULL;
    int num = 0, err;

    if (Socket_PrepareDownload (url, dst, &fname, &path) < 0) goto fail;
    if ((fp = fopen (path, "r"))) {
        fclose (fp);
        SCEE_Log (87);
        SCEE_LogMsg ("cannot download %s in %s: file already exists",
                     fname, dst);
        goto fail;
    }
    if (!(fp = fopen (path, "wb"))) {
        SCEE_LogErrno ("fopen() failed");
        goto fail;
    }

    handle = curl_easy_init ();
    curl_easy_setopt (handle, CURLOPT_ERRORBUFFER, curlerr);
    curl_easy_setopt (handle, CURLOPT_URL, url);
    curl_easy_setopt (handle, CURLOPT_WRITEFUNCTION, fwrite);
    curl_easy_setopt (handle, CURLOPT_WRITEDATA, fp);
    if (fun) {
        /* libcurl expects a long */
        curl_easy_setopt (handle, CURLOPT_NOPROGRESS, 0L);
        curl_easy_setopt (handle, CURLOPT_PROGRESSDATA, udata);
        curl_easy_setopt (handle, CURLOPT_PROGRESSFUNCTION, fun);
    }

    /* download */
    do {
        err = curl_easy_perform (handle);
        num++;
    } while (num < tries && err != 0);

    fclose (fp);                /* does curl do that? */
    curl_easy_cleanup (handle);

    switch (err) {
    case 0: break;
    case CURLE_ABORTED_BY_CALLBACK:
        remove (path);
        SCEE_Log (8655);
        SCEE_LogMsg ("download aborted");
        return SCE_ERROR;
    default:
        remove (path);
        SCEE_Log (763);
        SCEE_LogMsg ("cURL failed to download %s: %s", url, curlerr);
        return SCE_ERROR;
    }

    return SCE_OK;
fail:
    SCEE_LogSrc ();
    SCEE_LogSrcMsg ("failed to download %s", url);
    return SCE_ERROR;
}
