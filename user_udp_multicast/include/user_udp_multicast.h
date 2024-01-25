#ifndef __USER_UDP_MULTICAST_H__
#define __USER_UDP_MULTICAST_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>

struct user_udp_multicast_info {

};

char * user_udp_multicast_rcv(void);
int user_udp_multicast_send(char * pbuf, int len);
int user_udp_multicast_init(void);

#endif