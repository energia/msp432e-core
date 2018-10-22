#include <Energia.h>
#include "Ethernet.h"

#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/os/oskern.h>
#include <ti/posix/gcc/pthread.h>
#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/stack/inc/resif.h>
#include <ti/ndk/inc/stack/inc/routeif.h>
#include <ti/ndk/slnetif/slnetifndk.h>
#include <ti/net/slnetif.h>
#include <ti/posix/gcc/pthread.h>

extern "C" int fdError();
extern "C" int    fdOpenSession( void *hTask );
extern "C" void   fdCloseSession( void *hTask );

int16_t EthernetClass::_handleArray[MAXSOCKETS];
int16_t EthernetClass::_portArray[MAXSOCKETS];
int16_t EthernetClass::_typeArray[MAXSOCKETS];
int16_t EthernetClass::_serverPortArray[MAXSOCKETS];
EthernetClient EthernetClass::clients[MAXSOCKETS];

extern "C" void ti_ndk_config_Global_startupFxn(_IPCONFIG *config);
//extern "C" void ti_ndk_config_Global_startupFxn();
bool stack_is_up = false;

EthernetClass::EthernetClass()
{

  //Initialize the WiFi socket state arrays
  //
  int i;
  for (i = 0; i < MAXSOCKETS; i++) {
    _handleArray[i] = _portArray[i] = _typeArray[i] = _serverPortArray[i] = -1;
  }
}

void EthernetClass::begin()
{
    ti_ndk_config_Global_startupFxn(NULL);
    while(!stack_is_up) Task_yield();
    fdOpenSession((void *)TaskSelf());
}

void EthernetClass::begin(IPAddress local_ip)
{
    ipconfig.address = (uint32_t) local_ip;
    //ti_ndk_config_Global_startupFxn(&ipconfig);
    while(!stack_is_up);
    fdOpenSession((void *)pthread_self());
}

void EthernetClass::begin(IPAddress local_ip, IPAddress dns_server)
{
    ipconfig.address = (uint32_t) local_ip;
    ipconfig.dns = (uint32_t) dns_server;
    //ti_ndk_config_Global_startupFxn(&ipconfig);
    while(!stack_is_up);
    fdOpenSession((void *)pthread_self());
}

void EthernetClass::begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway)
{
    ipconfig.address = (uint32_t) local_ip;
    ipconfig.dns = (uint32_t) dns_server;
    ipconfig.gateway = (uint32_t) gateway;
    //ti_ndk_config_Global_startupFxn(&ipconfig);
    while(!stack_is_up);
    fdOpenSession((void *)pthread_self());
}

void EthernetClass::begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
{
    ipconfig.address = (uint32_t) local_ip;
    ipconfig.dns = (uint32_t) dns_server;
    ipconfig.gateway = (uint32_t) gateway;
    ipconfig.subnet = (uint32_t) subnet;
    //ti_ndk_config_Global_startupFxn(&ipconfig);
    while(!stack_is_up);
    fdOpenSession((void *)pthread_self());
}

IPAddress EthernetClass::localIP()
{
    CI_IPNET    NA;

    if (CfgGetImmediate( 0, CFGTAG_IPNET, 1, 1, sizeof(NA), (unsigned char *)&NA) == sizeof(NA)) {
        IPAddress ip = NA.IPAddr;
        return ip;
    }
    return INADDR_NONE;
}

IPAddress EthernetClass::subnetMask()
{
    CI_IPNET    NA;

    if (CfgGetImmediate( 0, CFGTAG_IPNET, 1, 1, sizeof(NA), (unsigned char *)&NA) == sizeof(NA)) {
        IPAddress subnet = NA.IPMask;
        return subnet;
    }
        return INADDR_NONE;
}

IPAddress EthernetClass::gatewayIP()
{
    uint32_t wFlags;
    void *hRt;
    uint32_t IPAddr;

    IPAddress route = INADDR_NONE;

    llEnter();
    hRt = RtWalkBegin();
    llExit();

    while ( hRt ) {
        wFlags = RtGetFlags( hRt );

        if ( wFlags & FLG_RTE_GATEWAY ) {
            llEnter();
            route = RtGetGateIP( hRt );
            llExit();
        }

        llEnter();
        hRt = RtWalkNext( hRt );
        llExit();
    }

    llEnter();
    RtWalkEnd( 0 );
    llExit();

    return route;
}

IPAddress EthernetClass::dnsServer()
{
    uint32_t IPTmp;
    CfgGetImmediate(0, CFGTAG_SYSINFO, CFGITEM_DHCP_DOMAINNAMESERVER,
                 1, 4, (uint8_t *)&IPTmp );
    return IPTmp; 
}
uint8_t EthernetClass::getSocket()
{
    for (uint8_t i = 0; i < MAXSOCKETS; ++i)
    {
        if (_handleArray[i] == -1)
        {
            return i;
        }
    }
    return NO_SOCKET_AVAIL;
}

int EthernetClass::maintain() {
  return 0;
}

IPAddress EthernetClass::nslookup(const char *host)
{
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
    return INADDR_NONE;
  }

  h = (struct sockaddr_in *) addrs->ai_addr;
  ip_address = IPAddress(h->sin_addr.s_addr);
  freeaddrinfo(addrs);

  return(ip_address);
}

EthernetClass Ethernet;
