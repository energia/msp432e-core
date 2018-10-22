#include "Ethernet.h"
#include "EthernetClient.h"

#include <ti/net/bsd/sys/socket.h>
#include <ti/net/bsd/netinet/in.h>
#include <ti/net/bsd/arpa/inet.h>
#include <ti/net/bsd/netdb.h>
#include <ti/posix/gcc/pthread.h>
#include <errno.h>

#define SO_ERROR        0x1007

extern "C" int fdError();
extern "C" int    fdOpenSession( void *hTask );
extern "C" void   fdCloseSession( void *hTask );

EthernetClient::EthernetClient() {
  _socketIndex = NO_SOCKET_AVAIL;
  rx_currentIndex = 0;
  rx_fillLevel = 0;
}

EthernetClient::EthernetClient(uint8_t socketIndex) {
    rx_currentIndex = 0;
    rx_fillLevel = 0;
    _socketIndex = socketIndex;
}

EthernetClient::~EthernetClient()
{
    if(EthernetClass::_typeArray[_socketIndex] != TYPE_TCP_CONNECTED_CLIENT) {
        return;
    }
    if(_socketIndex == NO_SOCKET_AVAIL) {
        return;
    }

    EthernetClient *client = &EthernetClass::clients[_socketIndex];
    memcpy(&client->rx_buffer, &rx_buffer, TCP_RX_BUFF_MAX_SIZE);
    client->rx_currentIndex = rx_currentIndex;
    client->rx_fillLevel = rx_fillLevel;
}

int EthernetClient::connect(const char *host, uint16_t port) {
  struct addrinfo hints;
  struct addrinfo *addrs;
  int ret;
  struct sockaddr_in *h;
  IPAddress ip_address;

  memset(&hints, 0, sizeof(struct addrinfo));

  //fdOpenSession((void *)pthread_self());

  hints.ai_family = AF_INET; // AF_INET means IPv4 only addresses
  ret = getaddrinfo(host, "0", &hints, &addrs);

  if (ret != 0) {
    return false;
  }

  h = (struct sockaddr_in *) addrs->ai_addr;
  ip_address = IPAddress(h->sin_addr.s_addr);
  freeaddrinfo(addrs);

  return connect(ip_address, port);
}


int EthernetClient::connect(IPAddress ip, uint16_t port)
{
  sockaddr_in sa;
  int sopt = 0;
  int ret;

  if (_socketIndex != NO_SOCKET_AVAIL) {
    return false;
  }

  int socketIndex = EthernetClass::getSocket();
  if (socketIndex == NO_SOCKET_AVAIL) {
    return false;
  }

  //fdOpenSession((void *)pthread_self());
  
  int socketHandle = socket(AF_INET, SOCK_STREAM, sopt);
  Serial.print("Socket returned: ");
  Serial.println(socketHandle);
  if (socketHandle < 0) {
    Serial.print("socket() failed, error = ");
    Serial.println(fdError());
    return false;
  }

  sa.sin_family = SLNETSOCK_AF_INET;
  sa.sin_port = htons(port);
  sa.sin_addr.s_addr = ip;
  ret = ::connect(socketHandle, (SlNetSock_Addr_t *)&sa, (int)sizeof(sockaddr_in));

  if (ret < 0) {
    close(socketHandle);
    return false;
  }

  int enableOption = 1;
  setsockopt(socketHandle, SOL_SOCKET, SO_NONBLOCKING, &enableOption, sizeof(enableOption));
//  setsockopt(socketHandle, SOL_SOCKET, SO_KEEPALIVE, &enableOption, sizeof(enableOption));

  //  struct timeval tv;
  //  tv.tv_sec = 5;
  //  tv.tv_usec = 0;
  //  setsockopt(socketHandle, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  _socketIndex = socketIndex;
  EthernetClass::_handleArray[socketIndex] = socketHandle;
  EthernetClass::_typeArray[socketIndex] = TYPE_TCP_CLIENT;
  EthernetClass::_portArray[socketIndex] = port;
  return true;
}


size_t EthernetClient::write(uint8_t b) {
  return write(&b, 1);
}

size_t EthernetClient::write(const uint8_t *buf, size_t len) {

  //
  //don't do anything if not properly set up
  //
  if (_socketIndex == NO_SOCKET_AVAIL) {
    return 0;
  }

  ssize_t nbytes;
  ssize_t status = -1;

  //fdOpenSession((void *)pthread_self());
  while (len > 0) {
    nbytes = send(EthernetClass::_handleArray[_socketIndex], (void *)buf, len, 0);

    if (nbytes >= 0) {
      len -= nbytes;
      buf = (uint8_t *)buf + nbytes;
      status = (status == -1) ? nbytes : (status + nbytes);
    }  else {
      this->stop();
      return 0;
    }
  }

  return (status);
}

int EthernetClient::available() {
  if (_socketIndex == NO_SOCKET_AVAIL) {
    return 0;
  }

  int bytesLeft = rx_fillLevel - rx_currentIndex;

  if (bytesLeft > 0) {
    return bytesLeft;
  }

  //fdOpenSession((void *)pthread_self());
  int numRead = recv(EthernetClass::_handleArray[_socketIndex], rx_buffer, TCP_RX_BUFF_MAX_SIZE, 0);

  if ((numRead == -1)) {
    return 0;
  }

  // If 0 is returned, remote host closed the connection.
  if (numRead == 0) {
    this->stop();
    return 0;
  }

  rx_currentIndex = 0;
  rx_fillLevel = numRead;

  return numRead;
}

int EthernetClient::read() {
  if ( available() ) {
    return rx_buffer[rx_currentIndex++];
  } else {
    return -1;
  }
}

int EthernetClient::read(uint8_t *buf, size_t size)
{
  if (!available()) {
    return -1;
  }

  int len = rx_fillLevel - rx_currentIndex;
  if (len > size) {
    len = size;
  }

  memcpy(buf, &rx_buffer[rx_currentIndex], len);
  rx_currentIndex += len;

  return len;
}

int EthernetClient::peek()
{
  if (!available()) {
    return -1;
  }

  return rx_buffer[rx_currentIndex];
}

void EthernetClient::flush() {
  // TODO
}

void EthernetClient::stop() {
  if (_socketIndex == NO_SOCKET_AVAIL) {
    return;
  }

  close(EthernetClass::_handleArray[_socketIndex]);

  EthernetClass::_portArray[_socketIndex] = -1;
  EthernetClass::_handleArray[_socketIndex] = -1;
  EthernetClass::_typeArray[_socketIndex] = -1;
  _socketIndex = NO_SOCKET_AVAIL;

  memset(rx_buffer, 0, TCP_RX_BUFF_MAX_SIZE);
}

uint8_t EthernetClient::connected()
{
  available();

  if (_socketIndex == NO_SOCKET_AVAIL) {
    return false;
  }

  return true;
}

EthernetClient::operator bool() {
  return(_socketIndex != NO_SOCKET_AVAIL);
}
