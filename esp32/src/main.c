// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// I2C driver
#include "driver/i2c.h"

// Error library
#include "esp_err.h"



#include <stdio.h>
#include "esp_event_loop.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_system.h"

#include <string.h>

#include "lwip/netdb.h"
#include "lwip/sockets.h"

#define ADXL345_ADDR 0x53
#define AXIS_X_0 0x32
#define AXIS_X_1 0x33 
#define AXIS_Y_0 0x34
#define AXIS_Y_1 0x35
#define AXIS_Z_0 0x36
#define AXIS_Z_1 0x37
#define ACK_VAL    0x0
#define NACK_VAL   0x1


#define MAX_APs 20

#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WIFI_PASS"
#define SERVER_PORT "5000"

#define CONFIG_RESOURCE "/accelerometer"
#define CONFIG_WEBSITE "ip_raspberry_pi"

// Event group
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;


static char* getAuthModeName(wifi_auth_mode_t auth_mode) {
	
	char *names[] = {"OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK", "MAX"};
	return names[auth_mode];
}

// Empty event handler
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
		
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    
	case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    
	case SYSTEM_EVENT_STA_DISCONNECTED:
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    
	default:
        break;
    }
   
	return ESP_OK;
}

// Main task
void main_task(void *pvParameter)
{
	// wait for connection
	printf("Main task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	printf("connected!\n");
	
	// print the local IP address
	tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
	
	while(1) {
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

//read from ADXL345_ADDR
uint8_t read_register_from_ADXL345(uint8_t register_to_read)
{
    uint8_t result = 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, register_to_read, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &result, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return result;

}

//write to ADXL345_ADDR
void write_register_from_ADXL345(uint8_t register_to_write, uint8_t data)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, register_to_write, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

void send_post_request(uint16_t x, uint16_t y, uint16_t z)
{
	const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
	
	// address info struct and receive buffer
    struct addrinfo *res;
	char recv_buf[100];
	
	printf("%s\n", CONFIG_WEBSITE);

	// resolve the IP of the target website
	int result = getaddrinfo(CONFIG_WEBSITE, SERVER_PORT, &hints, &res);
	if((result != 0) || (res == NULL)) {
		printf("Unable to resolve IP for target website %s\n", CONFIG_WEBSITE);
		while(1) vTaskDelay(1000 / portTICK_RATE_MS);
	}
	printf("Target website's IP resolved\n");
	
	// create a new socket
	int s = socket(res->ai_family, res->ai_socktype, 0);
	if(s < 0) {
		printf("Unable to allocate a new socket\n");
		while(1) vTaskDelay(1000 / portTICK_RATE_MS);
	}
	printf("Socket allocated, id=%d\n", s);
	// connect to the specified server
	result = connect(s, res->ai_addr, res->ai_addrlen);
	if(result != 0) {
		printf("Unable to connect to the target website %i\n",result);
		close(s);
		while(1) vTaskDelay(1000 / portTICK_RATE_MS);
	}
	printf("Connected to the target website\n");
	
	char *REQUEST_POST = (char*)malloc(100 * sizeof(char));;

	sprintf(REQUEST_POST, "POST %s?x=%i&y=%i&z=%i HTTP/1.1\n Host: %s\n User-Agent: ESP32\n\n",CONFIG_RESOURCE,x,y,z,CONFIG_WEBSITE);



	// send the request
	result = write(s, REQUEST_POST, strlen(REQUEST_POST));
		if(result < 0) {
		printf("Unable to send the HTTP request\n");
		close(s);
		while(1) vTaskDelay(1000 / portTICK_RATE_MS);
	}
	printf("HTTP request sent\n");
	
	// print the response
	printf("HTTP response:\n");
	printf("--------------------------------------------------------------------------------\n");
	int r;
	do {
		bzero(recv_buf, sizeof(recv_buf));
		r = read(s, recv_buf, sizeof(recv_buf) - 1);
		for(int i = 0; i < r; i++) {
			putchar(recv_buf[i]);
		}
	} while(r > 0);	
	printf("--------------------------------------------------------------------------------\n");
	
	close(s);
	printf("Socket closed\n");
}

// loop task
void loop_task(void *pvParameter)
{
    uint8_t x_0,x_1,y_0,y_1,z_0,z_1;
    uint16_t x,y,z;
    write_register_from_ADXL345(0x2D,1<<3);
    write_register_from_ADXL345(0x31,0x0B);
    write_register_from_ADXL345(0x2C,0x09);

    while(1) { 
		vTaskDelay(1000 / portTICK_RATE_MS);	
        x_0 = read_register_from_ADXL345(AXIS_X_0);
        x_1 = read_register_from_ADXL345(AXIS_X_1);
        x = (x_1<<8) | x_0;
        y_0 = read_register_from_ADXL345(AXIS_Y_0);
        y_1 = read_register_from_ADXL345(AXIS_Y_1);
        y = (y_1<<8) | y_0;
        z_0 = read_register_from_ADXL345(AXIS_Z_0);
        z_1 = read_register_from_ADXL345(AXIS_Z_1);
        z = (z_1<<8) | z_0;
		send_post_request(x,y,z);
        //printf("---------------------------\r\n");
        //printf("-> X axis value is %i\r\n", x);
        //printf("-> Y axis value is %i\r\n", y);
        //printf("-> Z axis value is %i\r\n", z);
    }
}




// Main application
void app_main() {

    // initialize NVS
	ESP_ERROR_CHECK(nvs_flash_init());

    // create the event group to handle wifi events
	wifi_event_group = xEventGroupCreate();
	
	// initialize the tcp stack
	tcpip_adapter_init();

	// initialize the wifi event handler
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	
	// configure, initialize and start the wifi driver
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());
	

    // configure the wifi connection and start the interface
	wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
	printf("Connecting to %s\n", WIFI_SSID);
	
    // wait for connection
	printf("Main task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	printf("connected!\n");
	
	// print the local IP address
	tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
	// start the main task
    //xTaskCreate(&main_task, "main_task", 2048, NULL, 5, NULL);

    
	

///////////////////////////////////////////////////////////////
	printf("i2c scanner\r\n\r\n");

	// configure the i2c controller 0 in master mode, normal speed
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = 18;
	conf.scl_io_num = 19;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	printf("- i2c controller configured\r\n");
	
	// install the driver
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	printf("- i2c driver installed\r\n\r\n");
	
	printf("scanning the bus...\r\n\r\n");
	int devices_found = 0;
	
	for(int address = 1; address < 127; address++) {
	
		// create and execute the command link
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
		i2c_master_stop(cmd);
		if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
			printf("-> found device with address 0x%02x\r\n", address);
			devices_found++;
		}
		i2c_cmd_link_delete(cmd);
	}
	if(devices_found == 0) printf("\r\n-> no devices found\r\n");
	printf("\r\n...scan completed!\r\n");

	
	// start the loop task
	xTaskCreate(&loop_task, "loop_task", 2048, NULL, 5, NULL);
}
