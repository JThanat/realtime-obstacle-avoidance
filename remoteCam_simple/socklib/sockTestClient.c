#include <stdio.h>
#include <stdlib.h>
#include "socklib.h"
struct threadData{
	char* buffer;
	int length;
	int sc;
};


int main(){
	int port = 41000;
	char* serv_addr = "192.168.1.42";
	int sc = sockopen(serv_addr, port);
	if(sc<0){
		fprintf(stderr, "cannot connect\n");
		exit(0);
	}
	int element = 400*1024*1024/sizeof(int);
	int size = sizeof(int)*element;
	int* buffer = malloc(size);
	//generate input
	int i;
	for(i=0;i<element;i++){
		buffer[i]=i;		
	}	
	int iteration = 1000;
	time_t t1, t2;
	t1 = time(NULL);
	for(i=0;i<iteration;i++){
		int r=0;
		int s=0;
		while(r<size){
			s = sendraw(sc, (char*)buffer+r, size-r);
			r+=s<0?0:s;
		}
		t2 = time(NULL);
		if((t2-t1)>20){
			fprintf(stderr, "error\n");fflush(stderr);
			exit(0);
		}
		t1=t2;
	}
	sockclose(sc);
	
	return 0;
}
