
#include <MQTTInterface.h>
#include <cstdio>
#include <cstring>

#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h> // strerror()
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h> // random(), exit
#include <errno.h>

#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <netinet/in.h> // inet_ntoa
#include <arpa/inet.h> // inet_ntoa
#include <fcntl.h> // fcntl
#include <limits.h> // INT_MAX
#include "inetlib.h"

/* mqtt_bridge - bridge between a TCP hub server and an MQTT broker

	 Messages in on the TCP connection are published to the MQTT broker
	 Messages from the MQTT broker are sent out on the TCP connection(s)

	 TCP Protocol
	 <topic>,<message>

	 For now, it only subscribes to Vision topic PI/CV/SHOOT/DAT

	 The TCP server receives connections from clients.  The connections to the
	 clients are kept open (up to a max MAX_BROWSER connections).
	 
	 A new connection received when there are already MAX_BROWSER
	 connections open will cause the oldest connection to be closed.
	 
	 This process defaults to port 9122.  If a port is provided on
	 the command line, this port will be used for all connections.
	 
*/





int MQTTInterface::GetString(int insock)
{
	// returns true for a good character read, otherwise return false
	// Really dumb, look at calling loop for full functionality
	int numread;
	char c;
	int fGood = false;

	
	// printf("waiting to read ...");
	numread = read(insock, &c, 1);
	// printf("(%2.2X) %c\n", c, c);


	if(numread < 0) {
		printf("error reading from input socket: %d\n", numread);
	} else {
		// Ok, read a character
		if((c == CR) || (c == LF)) {
			fGood = true;
		} else {
			// Good character
			rxbuf[rxbuf_index] = c;
			rxbuf_index++;
			// Need to ensure buffer does not overflow (which shouldnt happen anyway, but oh well)
			if(rxbuf_index >= RXBUF_MAX) {
				rxbuf_index = RXBUF_MAX - 1;
			}
		}
	}

	return fGood;
}

void MQTTInterface::PutString(char *s)
{
	int i, j;
	char sout[MAXLEN + 2];
	int ret;
	int max_age_reached = 0;

	// printf("Send string to [%d:%d]: *%s*\n", i, sock, s);

	// append a CRLF
	strcpy(sout, s);
	i = strlen(s);
	sout[i] = CR;
	sout[i + 1] = LF;
	sout[i + 2] = 0;

	// Now send to mqtt bridge
	ret = write(sock, sout, strlen(sout));
	if(ret < 0) {
		printf("error(%d) writing to socket: %s\n", ret, strerror(errno));
	}
}


void MQTTInterface::SignalHandler(int signum)
{
	printf("Caught signal %d\n", signum);
	fflush(stdout);
}

// Build string based on MQTT message and send to TCP client
void MQTTInterface::bot2tcp(char *topic, char *msg)
{
	char s1[255];

	sprintf(s1, "%s,%s", topic, msg);
	
	printf("Sending '%s' to TCP client\n", s1);
	PutString(s1);
}

// Parse string into topic and msg and sent to mqtt broker
void MQTTInterface::tcpReceived(char * smsg)
{
	char topic[255];
	char msg[255];
	char *p1;
	int rc;

	// parse smsg, breaking into topic and msg at the ","
	p1 = strtok(smsg, ",");
	strcpy(topic, p1);
	p1 = strtok(NULL, ",");
	strcpy(msg, p1);
	printf("From mqtt bridge: topic: %s  msg: %s\n", topic, msg);
	
	VisionMessageFilter(topic, msg);


}

void MQTTInterface::VisionMessageFilter(char* topic, char* msg) 
{
	char buf[255];

	if(!strcmp(topic, "PI/CV/SHOOT/DATA")){
		memset(buf, 0, 255*sizeof(char));
		/* Copy N-1 bytes to ensure always 0 terminated. */
		memcpy(buf, msg, 254*sizeof(char));
		float distance = 0;
		float angle = 0;
		sscanf(buf, "%f %f", &distance, &angle);
		targetDistance = distance;
		targetAngle = angle;
		printf("vision data is %f in and %f degrees", distance, angle);
	}
}

void MQTTInterface::cleanup(void)
{	
	if(sock) {
		close(sock);
		sock = 0;
	}
}



MQTTInterface::MQTTInterface(const char *host, int port)
{
	Init();
}


MQTTInterface::~MQTTInterface()
{
	cleanup();
}

void MQTTInterface::Init()
{
	clientport = DEFAULT_PORT;
	char stemp[256];

	// Clear out buffers
	rxbuf_index = 0;
	for(int i = 0; i < RXBUF_MAX; i++) {
		rxbuf[i] = 0;
	}

	/* 
	// We pray we never need this
	// atexit(cleanup);

	// signal(SIGINT, handle_signal);
	// signal(SIGTERM, handle_signal);

	if(signal(SIGPIPE, &SignalHandler) == SIG_ERR) {
		printf("Couldn't register signal handler for SIGPIPE\n");
	}

	if(signal(SIGHUP, &SignalHandler) == SIG_ERR) {
		printf("Couldn't register signal handler for SIGHUP\n");
	}

	*/

	// initialize socket so that cleanup() can determine if they are assigned
	sock = 0;

	// Initialize TCP connection
	sprintf(stemp, "%d", clientport);

	sock = connectTCP(bridge_host, stemp);
	if(sock) {
		// make it async so that select() return that is not from an outstanding connection can 
		// be ignored
		fcntl(sock, F_SETFL, O_NONBLOCK);
	}	
}

void MQTTInterface::Update()
{
	
	int alen; // from address length
	char scmd[MAXLEN]; // string from client
	int i, j;
	char stemp[256];
	int ret;
	char *pstr;
	struct timeval timeout;
	
	fd_set orig_fdset, fdset;
	int nfds = getdtablesize(); // number of file descriptors

	if(sock == 0){
		// If no socket, try an open cnnection
		// Initialize TCP connection
		sprintf(stemp, "%d", clientport);

		sock = connectTCP(bridge_host, stemp);
		if(sock) {
			// make it async so that select() return that is not from an outstanding connection can 
			// be ignored
			fcntl(sock, F_SETFL, O_NONBLOCK);
		}	else {
			return;
		}
	}

	FD_ZERO(&orig_fdset);
	FD_SET(sock, &orig_fdset);
	while(1) {


		/* Restore watch set as appropriate. */
		bcopy(&orig_fdset, &fdset, sizeof(orig_fdset));

		// need to sleep sometime, so just set timeout to 100 usec
		timeout.tv_sec = 0;
		timeout.tv_usec = 100;
		ret = select(nfds, &fdset, NULL, NULL, &timeout);
		if(ret != 0) {
			printf("select returned %d\n", ret);
		}
		if(ret > 0) {
			// process input from TCP connection
			if(sock && FD_ISSET(sock, &fdset)) {
				printf("Client has data - fd: %d\n", sock);
				ret = GetString(sock);
				// Did we find a full message?
				if(ret && (rxbuf_index > 0)) {
					// Parse and interpret message from MQTT Bridge
					tcpReceived(scmd);
					// Finished message, reset buffer for next message
					rxbuf_index = 0;
				} 
			}
		}	


	}
} 

float MQTTInterface::GetDistance()
{
	return targetDistance;
}

float MQTTInterface::GetAngle()
{
	return targetAngle;
}





