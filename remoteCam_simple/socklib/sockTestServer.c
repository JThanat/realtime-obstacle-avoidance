#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
#include "socklib.h"


struct threadData{
	int sc;
};


void recvCall(int sc){
	int buffLength=400*1024*1024/sizeof(int);
	int buffSize = buffLength*sizeof(int);
	int* buffer = malloc(buffSize);
	int iteration = 1000;
	int i;
	for(i=0;i<iteration;i++){
		int s=0;
		int r=0;
		while(s<buffSize){
			r = recvraw(sc, (char*)buffer+s, buffSize-s);
			s += r<0?0:r;
		}
		fprintf(stderr, "send finish\n"); fflush(stderr);
		//verification
		int j;
		#pragma omp parallel for
		for(j=0;j<buffLength;j++){
			if(buffer[j]!=j){
				fprintf(stderr, "transfer error\n"); fflush(stderr);
				exit(0);
			}
		}
		fprintf(stderr, "iteration:%d\n", i); fflush(stderr);
	}
	free(buffer);
	exit(0);
}


int main(){
	int port = 41000;
	char* serv_addr = NULL;
	int sc = sockopen(serv_addr, port);
	fprintf(stderr, "sc:%d\n", sc);
	struct threadData r1 = {sc};
	socklisten(sc, (void*)&r1, (sc, (void*)recvCall));
	
	sockclose(sc);
	
	return 0;
}
