#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"

#include "nvs_flash.h"

#include "lwip/netif.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_smartconfig.h"
#include "freertos/event_groups.h"
#include "esp_private/wifi.h"

#include "log.h"
#include "link_glue.h"
/**
20220131 chuanjin pang<pcj203@126.com> first relese for cn tiger year 

**/

int usblink_issue_ap_list(int nr,wifi_ap_record_t *ap_list_buffer);
uint16_t crc16_calc(unsigned char *puchMsg, unsigned int usDataLen ) ;


static const char *TAG = "xfz1986_wifi";

static EventGroupHandle_t wifi_event_group;
esp_netif_t *sta_netif;
static bool wifi_start = false;
bool s_wifi_is_connected = false;
bool reconnect = false;

const int CONNECTED_BIT = BIT0;
const int DISCONNECTED_BIT = BIT1;


esp_err_t wifi_cmd_sta_scan(const char* ssid)
{
    wifi_scan_config_t scan_config = { 0 };
    scan_config.ssid = (uint8_t *) ssid;

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    esp_wifi_scan_start(&scan_config, false);

    return ESP_OK;
}

esp_err_t wifi_cmd_sta_connect_ap(char * ssid, const char* pass)
{
    int bits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, 0, 1, 0);

    wifi_config_t wifi_config = { 0 };
    wifi_config.sta.pmf_cfg.capable = true;

    strlcpy((char*) wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (pass) {
        strlcpy((char*) wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    }
    reconnect = true;
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    esp_wifi_connect();

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, 0, 1, 50/portTICK_RATE_MS);

    return ESP_OK;
}



static void scan_done_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    uint16_t sta_number = 0;
    uint8_t i;
    wifi_ap_record_t *ap_list_buffer;
    size_t lenth = 0;

    esp_wifi_scan_get_ap_num(&sta_number);
    if (!sta_number) {
        ESP_LOGE(TAG, "No AP found");
        return;
    }

    ap_list_buffer = malloc(sta_number * sizeof(wifi_ap_record_t));
    if (ap_list_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to malloc buffer to print scan results");
        return;
    }

    if (esp_wifi_scan_get_ap_records(&sta_number,(wifi_ap_record_t *)ap_list_buffer) == ESP_OK) {        
        for(i=0; i<sta_number; i++) {
            ESP_LOGI(TAG, "%d[%s][rssi=%d]", i,ap_list_buffer[i].ssid, ap_list_buffer[i].rssi);
            
        }
    }
	usblink_issue_ap_list(sta_number,ap_list_buffer);
    free(ap_list_buffer);
}


void pkt_host2wifi(uint8_t * msg, int len )
{
	esp_err_t ret=0;

	if (s_wifi_is_connected) {

	   ret=esp_wifi_internal_tx(ESP_IF_WIFI_STA, msg,len);
	  }


}


esp_err_t pkt_wifi2host(void *buffer, uint16_t len, void *eb)
{
	esp_err_t ret=ERR_USE;	
	
		
if(link_send_msg(buffer,len)<0)
	goto out;

ret=ESP_OK;
out:
    esp_wifi_internal_free_rx_buffer(eb);
    return ret;
}



static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    char *wifi_event_buf = (char *)malloc(128);
    size_t lenth = 0;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
   
        esp_wifi_connect();		
        wifi_start = true;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_is_connected = false;
        esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);
#if 1
       
        if (reconnect) {
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG, "sta disconnect with no reconnect");
        }
#endif
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        xEventGroupSetBits(wifi_event_group, DISCONNECTED_BIT);
      
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
         {
            ESP_LOGI(TAG, "Wi-Fi STA connected");
            esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, pkt_wifi2host);
            s_wifi_is_connected = true;
            xEventGroupClearBits(wifi_event_group, DISCONNECTED_BIT);
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);       
	
            {
            	uint8_t mac[6]={0};
            	wifi_ap_record_t asso_ap_info;
				esp_err_t err= esp_wifi_sta_get_ap_info(&asso_ap_info);
				if(ESP_OK==err) {
					memcpy(mac,asso_ap_info.bssid,6);
					}
				else {
					ESP_LOGI(TAG, "Wi-Fi get ap info error %d\n",err );
				}
			
				
				usblink_issue_connect_done(mac);
			}
			
        }
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
    }  
    free(wifi_event_buf);
}


/* Initialize Wi-Fi as sta and set scan method */
void initialise_wifi(void)
{
	esp_err_t ret;
    esp_log_level_set("wifi", ESP_LOG_WARN);
    static bool initialized = false;

	ESP_LOGI(TAG, "wifi init...");	
    if (initialized) {
        return;
    }
    
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        WIFI_EVENT_SCAN_DONE,
                                                        &scan_done_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    initialized = true;
	ESP_LOGI(TAG,"wifi go...\n");
}




void app_main(void)
{
    ESP_LOGI(TAG, "xfz1986 usb wifi go...");
	  //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ret=nvs_flash_erase();    	
      ret = nvs_flash_init();	  
    }
	
    ESP_ERROR_CHECK(ret);	
	initialise_wifi();
    usb_link_init();

}
