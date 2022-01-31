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
#include "class/vendor/vendor_device.h"

#include "log.h"
#include "link_glue.h"
#include "usb_link.h"

/**
20220131 chuanjin pang<pcj203@126.com> first relese for cn tiger year 

**/

static const char *TAG = "xfz1986_wifi";

esp_err_t wifi_cmd_sta_scan(const char* ssid);

int esp_wpa_set_psk_ext(uint8_t *psk);
esp_err_t wifi_cmd_sta_connect_ap(char * ssid, const char* pass);
void pkt_host2wifi(uint8_t * msg, int len );

extern bool reconnect;


void push_cmd_resp( uint8_t * msg, int msg_len) 
{
cmd_resp64_t * cmd=	msg;


switch(cmd->cmd_op){

case OUT_BAND_CMD_GET_MAC:
{
	uint8_t mac[6];
	 esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
	 memcpy(cmd->buf,mac,6);
	 cmd->len=6;
	 usblink_issue_resp(msg,cmd);
}
break;

case OUT_BAND_CMD_SCAN:
	wifi_cmd_sta_scan(NULL);
	usblink_issue_resp(msg,cmd);
	break;
case OUT_BAND_CMD_CONNECT_AP:
{
	ssid_psk_msg_t * spsk=cmd->buf;
	esp_wpa_set_psk_ext(spsk->psk);

	spsk->ssid[cmd->len-32]='\0';
	reconnect=true;
	wifi_cmd_sta_connect_ap((char *)spsk->ssid,"12345678");
	usblink_issue_resp(msg,cmd);
	break;
}
case OUT_BAND_CMD_DISCONNECT_AP:
{
	esp_err_t err;
	reconnect=false;
	err=esp_wifi_disconnect();
	
	
	usblink_issue_resp(msg,cmd);
	break;
}

default:
break;

}


}


int usblink_issue_connect_done( uint8_t * mac){
	urb_msg_t * msg=(char *)malloc(sizeof(urb_msg_t));
	esp_err_t ret=0;
	if(!msg){
		return -1;
		}
		
		cmd_resp64_t * pcmd=msg->payload;
		pcmd->cmd_op=OUT_BAND_CMD_CONNECTED_DONE;
		pcmd->seq=5518;
		pcmd->len=sizeof(cmd_resp64_t);		
		memcpy(pcmd->buf,mac,6);
		msg->cmd_flg=USBLINK_PROTOCOL_HEADER_OPCODE;
		msg->len=pcmd->len;
	
		usb_link_send_outband_msg(msg);
	free(msg);
return 0;
}


#define X_MIN(x,y) (x)>(y)?(y):(x)
int usblink_issue_ap_list(int nr,wifi_ap_record_t *ap_list_buffer)
{
	urb_msg_t * msg=(char *)malloc(sizeof(urb_msg_t));
	esp_err_t ret=0;
	int tnr=0;
	if(!msg){
		return -1;
		}
		tnr=X_MIN(nr,18);

		cmd_resp64_t * pcmd=msg->payload;
		pcmd->cmd_op=OUT_BAND_CMD_AP_LIST;
		pcmd->seq=5511;
		pcmd->len=sizeof(cmd_resp64_t)-32;		
		pcmd->len+=sizeof(wifi_ap_record_t)*tnr;
		if(sizeof(wifi_ap_record_t)*tnr > USBLINK_MSG_MAX_SIZE ) {
			ESP_LOGI(TAG,"bug issue ap list may overflow\n");
			}
		memcpy(pcmd->buf,ap_list_buffer,sizeof(wifi_ap_record_t)*tnr);
		msg->cmd_flg=USBLINK_PROTOCOL_HEADER_OPCODE;
		msg->len=pcmd->len;
		
		usb_link_send_outband_msg(msg);
	free(msg);
return 0;
}


int link_send_msg( uint8_t * msg_data, int msg_len){
return usb_link_send_msg(msg_data,msg_len);
}



void link_host2wifi(urb_msg_t * msg )
{


	pkt_host2wifi(msg->payload, msg->len);

}


