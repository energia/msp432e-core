#include <Energia.h>
#include <IPAddress.h>
#include <Client.h>

#ifndef _ETHERNETCLIENT_H_
#define _ETHERNETCLIENT_H_
#define TCP_RX_BUFF_MAX_SIZE 1500

class EthernetClient : public Client {
  private:
    int _socketIndex;
    uint8_t rx_buffer[TCP_RX_BUFF_MAX_SIZE];
    int rx_currentIndex;
    int rx_fillLevel;
    bool server_socket;
  public:
    EthernetClient();
    ~EthernetClient();
    EthernetClient(uint8_t socketIndex);
    uint16_t magic;
    virtual int connect(IPAddress ip, uint16_t port);
    virtual int connect(const char *host, uint16_t port);

    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buf, size_t size);

    virtual int available();
    virtual int read();
    virtual int read(uint8_t *buf, size_t size);
    virtual int peek();
    virtual void flush();
    virtual void stop();
    virtual uint8_t connected();
    virtual operator bool();
};

#endif
