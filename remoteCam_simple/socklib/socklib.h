#ifndef __SOCKLIB__
#define __SOCKLIB__

#ifdef WIN32
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
#include <arpa/inet.h>
#endif


#ifdef __cplusplus
extern "C" {
#endif 

int sockopen(char *host, int port);
int sockinfo(int sock, char *info);
int sockclose(int sock);
int issockopen(int sock);

typedef void (*PF_SOCKETHANDLER)(int socket, void *);
void socklisten(int sock, void *handler_param, PF_SOCKETHANDLER handler_func);
void sockerror(char msg[]);

int broadcast(int socket, int port, char *buffer, int length);
int broadcast_recv(int socket, struct sockaddr *sin, char *buffer, int maxsize);
int broadcast_open(int port);

int sendraw(int socket, const char *buffer, int length);
int recvraw(int socket, char *buffer, int length);
    
typedef struct {
    char *recvbuffer;
    char *bufp;
    char *bufe;
    int   size;
} recvbuffer_t;

recvbuffer_t *recvBufferCreate(int size);
void recvBufferDestroy(recvbuffer_t *rb);
int recvstring(int socket, char *buffer, int maxsize, recvbuffer_t *r);
int sendstring(int socket, char *string);



#ifdef __cplusplus
}
#endif 


#endif
