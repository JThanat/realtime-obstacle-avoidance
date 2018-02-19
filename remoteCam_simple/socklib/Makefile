all: sockTestClient sockTestServer

sockTestClient: sockTestClient.c socklib.c
	gcc sockTestClient.c socklib.c -lpthread -O3 -o sockTestClient

sockTestServer: sockTestServer.c socklib.c
	gcc sockTestServer.c socklib.c -lpthread -fopenmp -O3 -o sockTestServer

clean:
	rm sockTestServer sockTestClient
