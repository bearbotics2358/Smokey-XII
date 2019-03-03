
#ifndef SRC_MQTTINTERFACE_H_
#define SRC_MQTTINTERFACE_H_

// for mqtt bridge
#define bridge_host "localhost"
#define DEFAULT_PORT 9122
#define bridge_port DEFAUT_PORT

#define CR 0x0d
#define LF 0x0a

#define MAXLEN 255

#define RXBUF_MAX 1024

class MQTTInterface
{
	public:
		MQTTInterface(const char *host, int port);
		~MQTTInterface();

		float GetDistance();
		float GetAngle();
	
	private:
		void cleanup(void);
		void Init();
		void Update();
		void VisionMessageFilter(char* topic, char* msg);
		void tcpReceived(char * smsg);
		void bot2tcp(char *topic, char *msg);
		void SignalHandler(int signum);
		void PutString(char *s);
		int GetString(int insock);

	private:
		float targetDistance = 0;
		float targetAngle = 0;
		// sockets are global so cleanup can close them
		int sock; // socket for communicating with mqtt bridge
		int clientport;
		char rxbuf[RXBUF_MAX];
		int rxbuf_index;

};




#endif
