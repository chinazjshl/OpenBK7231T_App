#include "lwip/sockets.h"
#include "wifi.h"
#include "string.h"

#define UDP_LISTEN_PORT 8899
static int udp_sock = -1;

void udp_relay_task(void *pvParameters)
{
    struct sockaddr_in server_addr;
    char buf[64];
    int len;

    // 循环等待Wi‑Fi获取IP
    while (1)
    {
        uint32_t ip = WIFI_GetIPAddress();
        if (ip != 0)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // 创建UDP Socket
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0)
    {
        vTaskDelete(NULL);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_LISTEN_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(udp_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        close(udp_sock);
        vTaskDelete(NULL);
    }

    while (1)
    {
        struct sockaddr_in client;
        socklen_t cli_len = sizeof(client);
        len = recvfrom(udp_sock, buf, sizeof(buf)-1, 0,
                       (struct sockaddr *)&client, &cli_len);

        if (len > 0)
        {
            buf[len] = '\0';
            if(strstr(buf, "ON"))
            {
                RELAY_SetState(1);
            }
            else if(strstr(buf, "OFF"))
            {
                RELAY_SetState(0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void user_init(void)
{
    xTaskCreate(udp_relay_task, "udp_relay", 2048, NULL, 5, NULL);
}
