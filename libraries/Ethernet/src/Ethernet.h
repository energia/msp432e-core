#include <stdint.h>
#include <IPAddress.h>
#include <MACAddress.h>
#include "EthernetClient.h"
#include "ipconfig.h"

#ifndef _ETHERNET_H_
#define _ETHERNET_H_

#define MAXSOCKETS 10
#define NO_SOCKET_AVAIL 255

#define TYPE_TCP_CONNECTED_CLIENT (0) //socket acting as server to this client
#define TYPE_TCP_CLIENT (1)           //socket acting as a client
#define TYPE_TCP_SERVER (2)
#define TYPE_UDP_PORT (3)

static _IPCONFIG ipconfig;

class EthernetClass {
  private:
    static EthernetClient clients[MAXSOCKETS];
    static void netIPAddrHook(uint32_t IPAddr, unsigned int IfIdx, unsigned int fAdd);
  public:
    static int16_t _handleArray[MAXSOCKETS];
    static int16_t _portArray[MAXSOCKETS];
    static int16_t _serverPortArray[MAXSOCKETS];
    static int16_t _typeArray[MAXSOCKETS];
    static uint8_t getSocket();

    void begin(uint8_t *mac_address);
    void begin(IPAddress local_ip);
    void begin(IPAddress local_ip, IPAddress dns_server);
    void begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
    void begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);
//    void begin(uint8_t *mac_address, IPAddress local_ip);
//    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server);
//    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
//    void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);

    /* Start ethernet in DHCP mode and use internal MAC */
    void begin();

    int maintain();
    IPAddress nslookup(const char *host);
    EthernetClass();
    IPAddress localIP();
    IPAddress subnetMask();
    IPAddress dnsServer();
    static IPAddress gatewayIP();
    friend class EthernetClient;
    friend class EthernetServer;
};

extern EthernetClass Ethernet;
#endif
