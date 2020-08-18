// code from 《LwIP应用开发实战指南—基于野火 STM32 全系列（M4-M7）开发板》, lwip_iperf-70Mbps

#include <stdint.h>
#include <stdio.h>

#include <lwip/sockets.h>
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"

#define TCP_SERVER_THREAD_NAME            "iperf_server"
#define TCP_SERVER_THREAD_STACKSIZE        1024
#define TCP_SERVER_THREAD_PRIO             4

#define IPERF_PORT          5001
#define IPERF_BUFSZ         (64 * 1024)
#define TICK_RATE_HZ		1000

static uint8_t iperf_buff[IPERF_BUFSZ];

void iperf_server(void *thread_param)
{
    uint8_t *recv_data;
    socklen_t sin_size;
    uint32_t tick1, tick2;
    int sock = -1, connected, bytes_received;
    uint64_t recvlen;
    struct sockaddr_in server_addr, client_addr;
    fd_set readset;
    struct timeval timeout;

    recv_data = iperf_buff;
    if (recv_data == NULL)
    {
        PRINTF("No memory\n");
        goto __exit;
    }

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
    	PRINTF("Socket error\n");
        goto __exit;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(IPERF_PORT);
    memset(&(server_addr.sin_zero), 0x0, sizeof(server_addr.sin_zero));

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
    {
    	PRINTF("Unable to bind\n");
        goto __exit;
    }

    if (listen(sock, 5) == -1)
    {
    	PRINTF("Listen error\n");
        goto __exit;
    }

    timeout.tv_sec = 3;
    timeout.tv_usec = 0;

    PRINTF("iperf_server\n");
    while (1)
    {
        FD_ZERO(&readset);
        FD_SET(sock, &readset);
      
        if (select(sock + 1, &readset, NULL, NULL, &timeout) == 0)
            continue;
        
        PRINTF("iperf_server\n");
        
        sin_size = sizeof(struct sockaddr_in);

        connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);

        PRINTF("new client connected from (%s, %d)\n",
                   inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        {
            int flag = 1;

            setsockopt(connected,
                       IPPROTO_TCP,     /* set option at TCP level */
                       TCP_NODELAY,     /* name of option */
                       (void *) &flag,  /* the cast is historical cruft */
                       sizeof(int));    /* length of option value */
        }

        recvlen = 0;
        tick1 = OSTimeGet();
        while (1)
        {
            bytes_received = recv(connected, recv_data, IPERF_BUFSZ, 0);
            if (bytes_received <= 0) break;

            recvlen += bytes_received;

            tick2 = OSTimeGet();
            if (tick2 - tick1 >= TICK_RATE_HZ * 5)
            {
                float f;
                f = (float)(recvlen * TICK_RATE_HZ / 125 / (tick2 - tick1));
                f /= 1000.0f;
                //snprintf(speed, sizeof(speed), "%.4f Mbps!\n", f);
                //printf("%s", speed);
                tick1 = tick2;
                recvlen = 0;
            }
        }

        if (connected >= 0) closesocket(connected);
        connected = -1;
    }

__exit:
    if (sock >= 0) closesocket(sock);
}

void iperf_server_init(void)
{
  sys_thread_new("iperf_server", iperf_server, NULL, 2048, 4);
}

