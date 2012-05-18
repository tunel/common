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

#ifndef H_SOCKET
#define H_SOCKET

/* TODO: suxxor. */
#ifdef WIN32
 #include <winsock2.h>
#else
 #include <sys/types.h>
 #include <sys/time.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>
 #include <unistd.h>
 #include <fcntl.h>
 #define INVALID_SOCKET -1
 #define SOCKET_ERROR -1
 #define closesocket(s) close (s)
 typedef int SOCKET;
 typedef struct sockaddr_in SOCKADDR_IN;
 typedef struct sockaddr SOCKADDR;
#endif

#include <SCE/utils/SCEUtils.h>

/* error codes */
enum SocketErrorCodes {
    SOCKET_PACKET_LOST = SCE_NUM_ERRORS,
    SOCKET_PING_TIMEOUT
};

/* NOTE: this one has multiple usage, it's a bit ugly. */
typedef enum {
    NETWORK_ERROR = SCE_ERROR,
    NETWORK_OK = SCE_OK,
    NETWORK_DISCONNECTED,
    NETWORK_TIMEOUT
} SocketState;

#define NETWORK_ADDRESS (1)
#define NETWORK_PORT (1<<1)

/* size of what Socket_{Set,Get}ID() writes/reads */
#define SOCKET_ID_SIZE 4

#define SOCKET_UDP_PACKET_SIZE 1024

typedef long SockID;

typedef struct {
    SOCKET sock;
    char *buffer;                        /* for sendto() */
    size_t bufsize;

    SCE_SArray stream;

    int nonblock;                        /* will recv() blocks? */
    void *udata;
    SCE_SListIterator it;
} Sock;

/* TODO: does libcurl define any type for this? */
typedef int (*Socket_CurlFunc)(void*, double, double, double, double);

int Init_Socket (void);
void Quit_Socket (void);

void Socket_Init (Sock*);
void Socket_Clear (Sock*);

int Socket_InitUDP (Sock*, int);
int Socket_InitTCP (Sock*, int);
int Socket_IsValidSocket (Sock*);
void Socket_Copy (Sock*, Sock*);

int Socket_Close (Sock*);
int Socket_Shutdown (Sock*);

int Socket_GetAddressFromStringv (const char*, in_addr_t*);
int Socket_GetPortFromString (const char*);
int Socket_GetAddressAndPortFromStringv (const char*, in_addr_t*, int*);

void Socket_SetupAddr (SOCKADDR_IN*, in_addr_t, int);
int Socket_SetupAddrFromString (SOCKADDR_IN*, const char*, int);
int Socket_SetupAddrFullFromString (SOCKADDR_IN*, const char*);

in_addr_t Socket_GetAddressFromSockAddr (SOCKADDR_IN*);
int Socket_GetPortFromSockAddr (SOCKADDR_IN*);
int Socket_CompareAddress (SOCKADDR_IN*, SOCKADDR_IN*);
char* Socket_GetAddressStringFromSockAddr (SOCKADDR_IN*);

SockID Socket_GetID (const char*);
void Socket_SetID (char*, SockID);

void Socket_SetData (Sock*, void*);
void* Socket_GetData (Sock*);
SCE_SListIterator* Socket_GetIterator (Sock*);

int Socket_Bind (Sock*, SOCKADDR_IN*);
int Socket_Listen (Sock*, int);
int Socket_Connect (Sock*, SOCKADDR_IN*);
int Socket_Accept (Sock*, Sock*, SOCKADDR_IN*);

int Socket_SendTCP (Sock*, SockID, const char*, size_t);
int Socket_SendUDP (Sock*, SockID, const char*, size_t, SOCKADDR_IN*);

SocketState Socket_ReceiveTCP (Sock*);
SocketState Socket_ReceiveUDP (Sock*, char*, size_t, SockID*, size_t*,
                               SOCKADDR_IN*);

const void* Socket_GetPacket (const Sock*, SockID*, size_t*);
int Socket_PopPacket (Sock*);

#define SOCKET_READ 1
#define SOCKET_WRITE (SOCKET_READ << 1)
#define SOCKET_EXCEPT (SOCKET_WRITE << 1)

int Socket_Wait (Sock*, int, long, long);
int Socket_SelectRead (SCE_SList*, long, long);

SocketState Socket_NbReceiveTCP (Sock*, long, long);
SocketState Socket_NbReceiveUDP (Sock*, char*, size_t, SockID*, size_t*,
                                 SOCKADDR_IN*, long, long);

/* http and (?) ftp downloads via libcurl */

int Socket_DownloadLength (const char*);
int Socket_Download (const char*, const char*, int, void*, Socket_CurlFunc);

#endif /* guard */
