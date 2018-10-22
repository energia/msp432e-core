#include "Ethernet.h"
#include "EthernetClient.h"
#include "EthernetServer.h"

#include <ti/net/bsd/sys/socket.h>
#include <ti/net/bsd/netinet/in.h>
#include <ti/net/bsd/arpa/inet.h>
#include <ti/posix/gcc/pthread.h>
#include <ti/net/bsd/netdb.h>
#include <errno.h>

extern "C" void *TaskCreate(void (*pFun)(), char *Name, int Priority,
        uint32_t StackSize, uintptr_t Arg1, uintptr_t Arg2, uintptr_t Arg3);

extern "C" int    fdOpenSession( void *hTask );

int  EthernetServer::_socketHandle = NO_SOCKET_AVAIL;

EthernetServer::EthernetServer(uint16_t port) {
    _socketHandle = NO_SOCKET_AVAIL;
    _port = port;
    _lastServicedClient = 0;
}


void EthernetServer::acceptor(uint32_t arg0, uint32_t arg1) {
    struct sockaddr_in clientAddr;
    struct sockaddr_in localAddr;
    socklen_t          addrlen = sizeof(clientAddr);
    int                clientSocketIndex;
    int                clientHandle;
    int                status;
    int                optval;
    int                optlen = sizeof(optval);
    int                port;
    int                enableOption = 1;

    port = (int)arg0;

    _socketHandle = socket(AF_INET, SOCK_STREAM, 0);
    if (_socketHandle == -1) {
        Serial.println("tcpHandler: socket failed");
    }

    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port = htons(port);

    status = bind(_socketHandle, (struct sockaddr *)&localAddr, sizeof(localAddr));
    if (status == -1) {
        Serial.println("tcpHandler: bind failed");
        return;
    }

    status = listen(_socketHandle, 3);
    if (status == -1) {
        Serial.println("tcpHandler: listen failed");
        return;
    }

    optval = 1;
    status = setsockopt(_socketHandle, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen);
    if (status == -1) {
        Serial.print("tcpHandler: setsockopt failed");
    }

    while((clientHandle = accept(_socketHandle, (struct sockaddr *)&clientAddr, &addrlen)) != -1) {
        if (clientHandle > 0) {
            clientSocketIndex = EthernetClass::getSocket();
            if (clientSocketIndex == NO_SOCKET_AVAIL) {
                close(clientHandle);
                continue;
            }

            setsockopt(clientHandle, SOL_SOCKET, SO_NONBLOCKING, &enableOption, sizeof(enableOption));

            EthernetClass::_handleArray[clientSocketIndex] = clientHandle;
            EthernetClass::_typeArray[clientSocketIndex] = TYPE_TCP_CONNECTED_CLIENT;
            EthernetClass::_portArray[clientSocketIndex] = htons(clientAddr.sin_port);
            EthernetClass::_serverPortArray[clientSocketIndex] = port;
            EthernetClass::clients[clientSocketIndex] = EthernetClient(clientSocketIndex);
            EthernetClass::clients[clientSocketIndex].magic = 0xaabb;
            Serial.println("New client");
        } else {
            Serial.println("accept failed");
        }
    }
}

void EthernetServer::begin() {
    void *thread = NULL;
    thread = TaskCreate((void (*)())EthernetServer::acceptor, NULL, 1, 4096, _port, 0, 0);
}

EthernetClient EthernetServer::available() {
    for(uint8_t i = 0; i < MAXSOCKETS; i++) {
        if (++_lastServicedClient >= MAXSOCKETS) {
            _lastServicedClient = 0;
        }

        if(EthernetClass::_handleArray[_lastServicedClient] != -1 && EthernetClass::_typeArray[_lastServicedClient] == TYPE_TCP_CONNECTED_CLIENT && EthernetClass::_serverPortArray[_lastServicedClient] == _port) {
            return EthernetClass::clients[_lastServicedClient];
        }
    }

    return EthernetClient(255);
}

size_t EthernetServer::write(uint8_t b) {
	return write(&b, 1);
}

size_t EthernetServer::write(const uint8_t *buffer, size_t size) {
    for(uint8_t i = 0; i < MAXSOCKETS; i++) {
        if(EthernetClass::_typeArray[i] == TYPE_TCP_CONNECTED_CLIENT && EthernetClass::_serverPortArray[i] == _port) {
            send(EthernetClass::_handleArray[i], buffer, size, 0);
        }
    }
}
