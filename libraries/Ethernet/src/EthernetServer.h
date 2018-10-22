#ifndef ethernetserver_h
#define ethernetserver_h

#include "Server.h"

#define MAX_CLIENTS 8


class EthernetClient;

class EthernetServer : public Server {
private:
        static int  _socketHandle;
        int8_t _lastServicedClient;
	uint16_t _port;
public:
	EthernetServer(uint16_t);
	EthernetClient available();
	virtual void begin();
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buf, size_t size);
	static void acceptor(uint32_t arg0, uint32_t arg1);
	using Print::write;
};

#endif
