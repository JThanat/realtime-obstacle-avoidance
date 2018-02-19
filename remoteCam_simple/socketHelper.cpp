#include <stdlib.h>
#include <stdio.h>
#include "socklib/socklib.h"
#include "socketHelper.h"
#include "remoteCam.h"
#include <stdint.h>

int recvBytes(int socket, char* recvBuff, int numBytes){
	int totalBytes = 0;
	int newBytes = -1;
	while(totalBytes < numBytes){
		newBytes = recvraw(socket, (char*)recvBuff+totalBytes, numBytes - totalBytes);
		if(newBytes<=0){
			fprintf(stderr, "socket closed, exiting\n");
			exit(0);
		}
		totalBytes += newBytes;
	}
	
	return totalBytes;
}


int32_t recvHeader(int socket){
	int32_t header= HEADER_INVALID;
	recvBytes(socket, (char*)&header, sizeof(header));	
	return header;
}



#ifndef WIN32

//there is no MSG_DONTWAIT in windows socket API
//we will need to deal with this later...
//not sure if this is a good idea...

//nah, just don't use it in Windows...
int peekHeader_nowait(int socket, int32_t* header, int peek){
	int flags = MSG_DONTWAIT;
	if(peek) flags |= MSG_PEEK;
	header[0] = HEADER_INVALID;
	int totalBytes = 0;
	int newBytes = recv(socket, (char*)header, sizeof(int32_t), flags);
	totalBytes += newBytes<0?0:newBytes;
	if(totalBytes<=0) return 0;
	//If the header is fragmented, peek until we can recieve whole header...
	while(totalBytes<sizeof(int32_t)){
		newBytes = recv(socket, (char*)header, sizeof(int32_t), flags);
		totalBytes = newBytes<0?0:newBytes;
	}
	return totalBytes;
}

int32_t recvHeader_nowait(int socket, int32_t* header){
	return peekHeader_nowait(socket, header, 0);
}

int32_t peekHeader_nowait(int socket, int32_t* header){
	return peekHeader_nowait(socket, header, 1);
}

#endif

int sendBytes(int socket, char* sendBuff, int numBytes){
	int totalBytes = 0;
	int newBytes = -1;
	
	while(totalBytes < numBytes){
		newBytes = sendraw(socket, (char*)sendBuff+totalBytes, numBytes - totalBytes);
		totalBytes += newBytes<0?0:newBytes;
	}
	
	return totalBytes;
}

void sendHeader(int socket, int32_t header){
	sendBytes(socket, (char*)&header, sizeof(header));	
	return;
}
