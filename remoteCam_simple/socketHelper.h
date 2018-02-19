#include <stdlib.h>
#include <stdio.h>
#include "socklib/socklib.h"
#include <stdint.h>


int recvBytes(int socket, char* recvBuff, int numBytes);
int32_t recvHeader(int socket);
int32_t recvHeader_nowait(int socket, int32_t* header);
int32_t peekHeader_nowait(int socket, int32_t* header);
int sendBytes(int socket, char* sendBuff, int numBytes);
void sendHeader(int socket, int32_t header);
