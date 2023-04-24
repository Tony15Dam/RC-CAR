#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#define HIGH 1
#define LOW 0

#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD" 
#define WIFI_SUCCESS 1
#define WIFI_FAIL 0
#define MAXIMUM_RETRY 10 
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "RC CAR";

static int s_retry_num = 0;

//event group to handle wifi communication
static EventGroupHandle_t s_wifi_event_group;

char* webPage = 
"<!DOCTYPE html>"  
"<head>"
"<title>RC car</title>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<style> body{font-family: Arial, Helvetica, sans-serif;}"
".button{background-color: #4CAF50;"
"border:2px solid black;"
"border-radius: 12px;"
"color:white;"
"padding: 15px 32px;"
"text-align: center;"
"text-decoration: none;"
"display: inline_block;"
"font_size: 16px;"
"margin: 4px 2px"
"cursor: pointer;}"

"button:hover{background-color:#90EE90}"
".button2{background-color: #ff0000;}"
".button3{background-color: #0000FF;}"

".button:active{background-color: #90EE90;"
"box-shadow : 0 5px #666;"
"transform: translateY(4px);}"
".button2:active{background-color: #FFCCCE;}"

"</style>"
"</head>"
"<body> <h1><center>RC CONTROL</center></h1>"
"<p><center><a href='/manual'><button class='button'>MANUAL MODE</button></a></center></p>"
"<p><center><a href='/fwd/'><button class='button'>FORWARD</button></a></center></p>"
"<p><center><a href='/rleft/'><button class='button'>LEFT</button></a>"
"<a href='/rright/'><button class='button'>RIGHT</button></a></center></p>"
"<p><center><a href='/bck'><button class='button'>BACKWARDS</button></a></center></p>"
"<p><center><a href='/stop'><button class='button button2'>STOP</button></a></center></p>"
"<p><center><a href='/Auto'><button class='button button3'>AUTOMATIC MODE</button></a></center></p>"

"</body> </html>";

//UART buffer to be allocated on the heap
char* uart_data;

//Uart config
uart_config_t uart_config = {
    .baud_rate = 74880,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };

    if (strlen((char *)wifi_config.sta.password)) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    //Wait until connection succeeds or fails the maximum amount of times
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

void TCP_connect(){
    struct sockaddr_in server_info = {0};
    char readBuffer[1024] = {0};  
    uart_data = (char*) malloc(4);

    server_info.sin_family = AF_INET;
    server_info.sin_addr.s_addr = 0x0;  
    server_info.sin_port = htons(80);

    int sock = socket(AF_INET, SOCK_STREAM, 0);

    bool manual = true;

    if(sock < 0){
        ESP_LOGE(TAG, "Failed to create a socket.");
        return;
    }
    else
        ESP_LOGI(TAG, "Socket created successfully!");


    if(bind(sock, (struct sockaddr* )&server_info, sizeof(server_info)) != 0){
        ESP_LOGE(TAG, "Failed to bind socket to port");
        close(sock);
        return;
    }

    if(listen(sock, 3) != 0){ 
        ESP_LOGE(TAG, "Failed to listen in on the port");
        close(sock);
        return;
    }
    
    ESP_LOGI(TAG, "Connection to TCP server established at address :%s", inet_ntoa(server_info.sin_addr.s_addr));

    while(1){

        struct sockaddr c_info = {0};
        uint32_t c_size = 0;
        int cfd = accept(sock, &c_info, &c_size);

        if(cfd < 0){
            ESP_LOGE(TAG, "Error accepting client");
            close(sock);
            return;
        }

        int gate = 1;

        //Read buffer inputs
        while(gate > 0){  

            gate = read(cfd, readBuffer, sizeof(readBuffer));

            if(manual == true){
                if(strstr(readBuffer, "/stop") > 0){

                    uart_data = "Stop";
                }

                else if(strstr(readBuffer, "/fwd") > 0){  

                    uart_data = "Forw";
                }

                else if(strstr(readBuffer, "/bck") > 0){

                    uart_data = "Back";
                }

                else if(strstr(readBuffer, "/rleft") > 0){

                    uart_data = "Left";
                }

                else if(strstr(readBuffer, "/rright") > 0){

                    uart_data = "Righ";
                }
            }

            if(strstr(readBuffer, "/Auto") > 0){

                //Set gpio high for a few ms so pico can read the wave change
                gpio_set_level(2, HIGH); 
                vTaskDelay(1/portTICK_PERIOD_MS);
                gpio_set_level(2, LOW);
                manual = false;
            }

            else if(strstr(readBuffer, "/manual") > 0){

                gpio_set_level(4, HIGH);
                vTaskDelay(1/portTICK_PERIOD_MS);
                gpio_set_level(4, LOW);
                manual = true;
            }

            ESP_LOGI(TAG, uart_data);

            write(cfd, webPage, strlen(webPage));
            uart_write_bytes(UART_NUM_0, uart_data, strlen(uart_data));
            gate = 0;
        }
        //Close socket and add delay to allow client time to load the page
        vTaskDelay(20/portTICK_PERIOD_MS);
        close(cfd);
    }
    close(sock);
    free(uart_data);
}

void app_main()
{
    //Initialize flash memory
    ESP_ERROR_CHECK(nvs_flash_init());

    //Initialize gpios to be used as interrupts
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    gpio_set_direction(4, GPIO_MODE_OUTPUT);

    //Initialize UART drivers
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);

    //Initialize the wifi driver 
    wifi_init_sta();

    //Start communicating with TCP server
    TCP_connect();

}

