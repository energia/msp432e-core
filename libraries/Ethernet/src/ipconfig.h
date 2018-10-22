#ifndef _IPCONFIG_H_
#define _IPCONFIG_H_
#ifdef __cplusplus
extern "C" {
#endif
typedef struct _ip_config {
    uint32_t address;
    uint32_t subnet;
    uint32_t gateway;
    uint32_t dns;
} _IPCONFIG;
#ifdef __cplusplus
}
#endif
#endif
