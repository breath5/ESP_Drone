
#ifndef COMPONENTS_EXTERNAL_COMMUNICATION_UDP_TCP_INCLUDE_UDP_TCP_H_
#define COMPONENTS_EXTERNAL_COMMUNICATION_UDP_TCP_INCLUDE_UDP_TCP_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

void udp_client_init(void);
void udp_rc_send(uint8_t *data,uint8_t len);//UDP发送

#endif /* COMPONENTS_EXTERNAL_COMMUNICATION_UDP_TCP_INCLUDE_UDP_TCP_H_ */
