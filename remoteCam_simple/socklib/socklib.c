/*
 *  Socket Wrapper Code
 */


#ifdef WIN32
#include <WS2tcpip.h>
#include <winsock.h>
#include <process.h>
#include <io.h>
#include <stdio.h>

#else
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <sys/poll.h>
#include <arpa/inet.h>
#include <pthread.h>

#endif

#include "socklib.h"

#define POLL_TIMEOUT  3000

#ifdef WIN32
static struct {
    int errorNumber;
    char *errorString;
} sockerrlist [] = {
    {10013, "Permission denied"},
    {10048, "Address already in use"},
    {10049, "Cannot assign requested address"},
    {10047, "Address family not supported by protocol family"},
    {10037, "Operation already in progress"},
    {10053, "Software caused connection abort"},
    {10061, "Connection refused"},
    {10054, "Connection reset by peer"},
    {10039, "Destination address required"},
    {10014, "Bad address"},
    {10064, "Host is down"},
    {10065, "No route to host"},
    {10036, "Operation now in progress"},
    {10004, "Interrupted function call"},
    {10022, "Invalid argument"},
    {10056, "Socket is already connected"},
    {10024, "Too many open files"},
    {10040, "Message too long"},
    {10050, "Network is down"},
    {10052, "Network dropped connection on reset"},
    {10051, "Network is unreachable"},
    {10055, "No buffer space available"},
    {10042, "Bad protocol option"},
    {10057, "Socket is not connected"},
    {10038, "Socket operation on non-socket"},
    {10045, "Operation not supported"},
    {10046, "Protocol family not supported"},
    {10067, "Too many processes"},
    {10043, "Protocol not supported"},
    {10041, "Protocol wrong type for socket"},
    {10058, "Cannot send after socket shutdown"},
    {10044, "Socket type not supported"},
    {10060, "Connection timed out"},
    {10109, "Class type not found"},
    {10035, "Resource temporarily unavailable"},
    {11001, "Host not found"},
    {10093, "Successful WSAStartup not yet performed"},
    {11004, "Valid name; no data record of requested type"},
    {11003, "Non-recoverable error"},
    {10091, "Network subsystem is unavailable"},
    {11002, "Non-authoritative host not found"},
    {10092, "WINSOCK.DLL version out of range"},
    {10094, "Graceful shutdown in progress"},
    {0, NULL}
}; 
#endif

void sockerror(char msg[]) {
#ifdef WIN32
    int err = WSAGetLastError();
    int i;
    for (i = 0; sockerrlist[i].errorNumber; i++)
        if (sockerrlist[i].errorNumber == err) {
            fprintf(stderr, "%s: (%d) %s\n", msg, err, sockerrlist[i].errorString);
            return;
        }
#endif
    perror(msg);
}

/*
 * This is a dual purpose routine.
 * - if host is not NULL, connect as a client to the given host and port.
 * - if host is NULL, bind a socket to the given port.
 * In either case, return a valid socket or -1.
 */
int sockopen(char *host, int port)
{
    int sd;
    static struct sockaddr_in sin;
    struct hostent *hp;

#ifdef WIN32
    static int started = 0;
    if (!started) {
        short wVersionRequested = 0x101;
        WSADATA wsaData;
        if (WSAStartup( wVersionRequested, &wsaData ) == -1) {
            sockerror("sockopen");
            exit(0);
        }
        if (wsaData.wVersion != 0x101) {
            fprintf(stderr, "Incorrect winsock version\n");
            exit(0);
        }
        started = 1;
    }
#endif

    /* get an internet domain socket */
    if ((sd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
        return -1;

        /* complete the socket structure */
    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = htonl(INADDR_ANY);
    sin.sin_port = htons((short)port);
    if (host != NULL) {
        /* get the IP address of the requested host */
        if ((hp = gethostbyname(host)) == 0)
            return -1;
        sin.sin_addr.s_addr = ((struct in_addr *) (hp->h_addr))->s_addr;



        
        if (connect(sd, (struct sockaddr *) &sin, sizeof(sin)) == -1)
            return -1;
    } else {                    /* server */
        int one = 1;
        struct linger l;
        l.l_onoff = 0;
        l.l_linger = 0;
        
        /* avoid "bind: socket already in use" msg */
        if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR,
                       (char *) &one, sizeof(one)) < 0)
            return -1;

        /* Linger for a second before closing the */
        if (setsockopt(sd, SOL_SOCKET, SO_LINGER,
                       (char *) &l, sizeof(l)) < 0)
            return -1;

        
        /* bind the socket to the port number */
        if (bind(sd, (struct sockaddr *) &sin, sizeof(sin)) == -1)
            return -1;
    }

    return sd;
}

int sockinfo(int sd, char *info)
{
    struct sockaddr_in sin;
    socklen_t len = sizeof(sin);
    memset(&sin, 0, sizeof(sin));
    if (getsockname(sd, (struct sockaddr *) &sin, &len) == -1) {
        sockerror("sockinfo");
        return -1;
    }
    memcpy(info, &sin.sin_addr, 4);
    memcpy(info + 4, &sin.sin_port, 2);
    return 0;
}

int sockclose(int sock)
{
#ifdef WIN32
    return closesocket(sock);
#else
    return close(sock);
#endif
    return 0;
}



typedef struct {
    PF_SOCKETHANDLER handler;
    void *handler_param;
    int socket;
#ifndef WIN32
    pthread_t thread;
#endif
} socketHandlerInfo;

#ifdef WIN32
void handlerThread(void *param) {
#else
void *handlerThread(void *param) {
#endif
    socketHandlerInfo *info = (socketHandlerInfo *)param;
    info->handler(info->socket, info->handler_param);
//    while(1)
    sockclose(info->socket);
    free(info);
    return NULL;
        /* implicit exit here */
}


void starthandler(void *handler_param, PF_SOCKETHANDLER handler, int socket) 
{
    socketHandlerInfo *info = (socketHandlerInfo *)malloc(sizeof(socketHandlerInfo));

    info->handler = handler;
    info->socket = socket;
    info->handler_param = handler_param;

#ifdef WIN32    
    _beginthread(handlerThread, 0, (void *)info);
#else
    pthread_create( &info->thread, NULL, handlerThread, (void*)info);
#endif
    return;
}

void socklisten(int sock, void *handler_param, PF_SOCKETHANDLER handler)
{
    struct sockaddr sockaddr;
#if WIN32
    int addrlen = sizeof(sockaddr);
#else
    socklen_t addrlen = sizeof(sockaddr); 
#endif

    int newsock;

    if (listen(sock, SOMAXCONN) == -1) {
        sockerror("socklisten(listen)");
        exit(0);
    }

    
    while (1) {
        memset(&sockaddr, 0, sizeof(sockaddr));

        newsock = accept(sock, &sockaddr, &addrlen);
        //printf("Socket accept %d\n", newsock);        
        if (newsock != -1) {
            starthandler(handler_param, handler, newsock);
        } else {
            sockerror("socklisten(accept)");
            exit(0);
        }
        //printf("Socket run\n");        
    }
}

int
sendraw(int socket, const char *buffer, int length)
{
    return send(socket, buffer, length, 0);
}

int
recvraw(int socket, char *data, int length){

    int ret;
#ifndef WIN32
    struct pollfd ufd;
    ufd.fd = socket;
    ufd.events = POLLIN;

//    fd_set rfds;
//    struct timeval tv;
//    FD_ZERO(&rfds);
//    FD_SET(socket, &rfds);
//    tv.tv_sec = 5;
//    tv.tv_usec = 0;

    while(1) {
        ret = poll(&ufd, 1, POLL_TIMEOUT);
//        ret = select(1, &rfds, NULL, NULL, &tv);
        if (ret == -1) {
//            DEBUGMSG(1, "Error in sockets\n");
            perror("select()");
            return 0;
        } else if (ret == 0) {
            // Time out
            continue;
        } else if (ufd.revents & POLLIN) {
        //        else {
            ret = recv(socket, data, length, 0);
            return ret;
        }
    }
#else
    ret = recv(socket, data, length, 0);
#endif
    return ret;
}

int
broadcast(int socket, int port, char *buffer, int length) {

    // XXX-broadcasting part is not done yet.
    struct sockaddr_in sin;
    int sin_len = sizeof(sin);
    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = htonl(INADDR_BROADCAST);
//    sin.sin_addr.s_addr = inet_addr("192.168.0.2");
    sin.sin_port = htons((short)port);
    return sendto(socket, buffer, length, 0, (struct sockaddr*)&sin, sin_len);
}

int
broadcast_recv(int socket, struct sockaddr *sin, char *buffer, int maxsize) 
{
#if WIN32
    int fromlen = sizeof(*sin);
#else
    socklen_t fromlen = sizeof(*sin);
#endif
    return recvfrom(socket, buffer, maxsize, 0, sin, &fromlen);
}

int
broadcast_open(int port) 
{
    int sd, ret;
    int so_broadcast;
    struct sockaddr_in si_me;
    struct ip_mreq imreq;

    unsigned char one = 1;
    struct sockaddr_in saddr;
    struct in_addr iaddr;

    
    if ((sd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        return -1;

    so_broadcast = 1;
    ret = setsockopt(sd,
                     SOL_SOCKET,
                     SO_BROADCAST,
                     &so_broadcast,
                     sizeof so_broadcast);
    if (ret !=0) {
        perror("Set broadcast opt:");
    }

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sd, (struct sockaddr*)&si_me, sizeof(si_me))==-1) {
        sockclose(sd);
        return -1;
    }

    // set content of struct saddr and imreq to zero
    memset(&saddr, 0, sizeof(struct sockaddr_in));
    memset(&iaddr, 0, sizeof(struct in_addr));
    
    imreq.imr_multiaddr.s_addr = inet_addr("226.0.0.1");
    imreq.imr_interface.s_addr = htonl(INADDR_ANY); // use DEFAULT interface

    iaddr.s_addr = INADDR_ANY; // use DEFAULT interface
    
    // send multicast to itself
    setsockopt(sd, IPPROTO_IP, IP_MULTICAST_LOOP,
               &one, sizeof(unsigned char));
    
    // Set the outgoing interface to DEFAULT
    ret = setsockopt(sd, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
              sizeof(struct in_addr));
    
    // JOIN multicast group on default interface
    ret = setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, 
                     (const void *)&imreq, sizeof(struct ip_mreq));    
    if (ret !=0) {
        perror("Set broadcast opt:");
    }
    
   return sd;
}

int issockopen(int sock) {
    int optval;
#if WIN32
    int optlen = sizeof(optval);
#else
    socklen_t optlen = sizeof(optval);
#endif
    int ret = getsockopt(sock, SOL_SOCKET, SO_BROADCAST, &optval, &optlen);
    return ret ==0;
}
