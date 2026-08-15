#include "lwip/sockets.h"
#include "new_pins.h"
#include "logging/logging.h"
#include "udp_iot.h"

#define UDP_LISTEN_PORT 8899
// 假设你的继电器在 Web UI 中映射到了 Channel 1
#define RELAY_CHANNEL 1 

static void udp_server_thread(void *arg) {
  // 1. 阻塞等待直到 WiFi 连接成功
    while (Main_IsConnectedToWiFi() == 0) {
        rtos_delay_milliseconds(1000); // 没连上就每秒检查一次，不占用 CPU
    }

    //ADDLOGF_INFO("WiFi Connected! Starting UDP Server...");
  
    int sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char rx_buffer[128];
    char tx_buffer[64]; // 用于构造回复的字符串

    while (1) {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            //ADDLOGF_ERROR("UDP Server: socket creation failed");
            rtos_delay_milliseconds(1000);
            continue;
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(UDP_LISTEN_PORT);

        if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            //ADDLOGF_ERROR("UDP Server: bind failed on port %d", UDP_LISTEN_PORT);
            close(sock);
            rtos_delay_milliseconds(1000);
            continue;
        }

        //ADDLOGF_INFO("UDP Server listening on port %d", UDP_LISTEN_PORT);

        while (1) {
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, 
                               (struct sockaddr *)&client_addr, &client_addr_len);
            if (len > 0) {
                rx_buffer[len] = '\0'; // 确保字符串截断
                
                // 忽略可能存在的换行符，比对前缀指令
                if (strncmp(rx_buffer, "ON", 2) == 0) {
                    CHANNEL_Set(RELAY_CHANNEL, 1, 0);
                    //ADDLOGF_INFO("UDP command: ON -> Relay %d closed", RELAY_CHANNEL);
                } else if (strncmp(rx_buffer, "OFF", 3) == 0) {
                    CHANNEL_Set(RELAY_CHANNEL, 0, 0);
                    //ADDLOGF_INFO("UDP command: OFF -> Relay %d opened", RELAY_CHANNEL);
                }
                // 新增：处理 STATUS 查询命令
                else if (strncmp(rx_buffer, "STATUS", 6) == 0) {
                    // 读取 Channel 当前的实际状态 (返回 1 为开，0 为关)
                    int current_state = CHANNEL_Get(RELAY_CHANNEL);
                    
                    // 构造回复文本，比如 "STATUS:ON" 或 "STATUS:OFF"
                    snprintf(tx_buffer, sizeof(tx_buffer), "STATUS:%s", current_state ? "ON" : "OFF");
                    
                    // 使用 sendto 将数据原路返回给 client_addr
                    sendto(sock, tx_buffer, strlen(tx_buffer), 0, 
                           (struct sockaddr *)&client_addr, client_addr_len);
                           
                    //ADDLOGF_INFO("UDP command: STATUS -> Replied: %s", tx_buffer);
                }
            } else if (len < 0) {
                // Socket 异常，跳出内层循环以重新创建 Socket
                break; 
            }
        }
        close(sock);
    }
}

void UDP_Server_Start() {
    beken_thread_t udp_thread;
    // 分配 1024 字节栈空间对于简单的 UDP recv 已经足够
    rtos_create_thread(&udp_thread, BEKEN_APPLICATION_PRIORITY, 
                       "udp_svr", udp_server_thread, 1024, NULL);
}
