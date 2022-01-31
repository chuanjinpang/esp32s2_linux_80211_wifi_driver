
#include <stdint.h>
#include "esp_log.h"
#include <stdlib.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "sdkconfig.h"

#include "soc/usb_periph.h"
#include "link_glue.h"
#include "log.h"
#include "usb_link.h"
/**
20220131 chuanjin pang<pcj203@126.com> first relese for cn tiger year 
**/


static const char *TAG = "xfz1986_wifi";
#if XFZ_DEBUG
#define TX_SPEED_LOG_THD (1)
#define RX_SPEED_LOG_THD (1)

#else
#define TX_SPEED_LOG_THD (16*32)
#define RX_SPEED_LOG_THD (16*32)

#endif

#define CONFIG_USB_VENDOR_RX_BUFSIZE 64
#define USBLINK_TO_HOST_QUEUE_SIZE 3

QueueHandle_t usblink_rx_queue= NULL;
QueueHandle_t usblink_tx_queue= NULL;
QueueHandle_t usblink_tx_cmd_queue= NULL;

int64_t esp_timer_get_time(void);
void usblink_process_rx_pkt(urb_msg_t * msg );
long usb_get_rx_speed(void);
void push_cmd_resp( uint8_t * msg, int msg_len) ;


void usbd_reset(uint8_t rhport);





long get_system_us(void)
{
    return esp_timer_get_time();
}



int get_msg_id(void){
static int msg_id=0;
return msg_id++;
}








uint16_t crc16_calc_multi(uint16_t crc_reg, unsigned char *puchMsg, unsigned int usDataLen ) 
{ 
uint16_t i,j,check; 
	for(i=0;i<usDataLen;i++) 
	{ 
	crc_reg = (crc_reg>>8) ^ puchMsg[i]; 
		for(j=0;j<8;j++) 
		{ 
			check = crc_reg & 0x0001; 
			crc_reg >>= 1; 
			if(check==0x0001){ 
				crc_reg ^= 0xA001; 
			} 
		} 
	} 
return crc_reg; 
}

uint16_t crc16_calc(unsigned char *puchMsg, unsigned int usDataLen ) 
{ 
return crc16_calc_multi(0xFFFF,puchMsg,usDataLen);
}




int pop_msg_data(usblink_msg_mgr_t *mgr,uint8_t * rx_buf, int len)
{
   if(0 == len) {
        return 0;
    }

	if(mgr->cur_cnt+len>USBLINK_MSG_MAX_SIZE)
	{
		return -1;
	}

   	memcpy(&mgr->msg.payload[mgr->cur_cnt],rx_buf,len);
	mgr->cur_cnt+=len;

    return 0;
}

void usblink_msg_mgr_reset(usblink_msg_mgr_t *mgr){
	mgr->cur_cnt=0;
	mgr->total=0;
	mgr->flag=0;
}

#define XFZ_MIN(x,y) (x)<(y)?(x):(y)
int decode_urb_msg(usblink_msg_mgr_t *mgr,uint8_t * rx_buf,int rx_len)
{
	int read_res=0;
	int remain=rx_len;
	uint16_t crc_ret=0;
	static int crc_total_cnt=0;
	static int rx_crc_cnt=0;
	
	crc_total_cnt++;
	
	while(remain) {
			
				read_res=XFZ_MIN(CONFIG_USB_VENDOR_RX_BUFSIZE,remain);
				remain-=read_res;
	
		if(read_res > 0) {
				usblink_start_crc_msg_t *  pstart = (usblink_start_crc_msg_t *)rx_buf;
			   if(pstart->flag_total_bytes & USBLINK_PROTOCOL_HEADER_START) {
					   mgr->flag = USBLINK_PROTOCOL_HEADER_START ;
					   if(pstart->flag_total_bytes & USBLINK_PROTOCOL_HEADER_OPCODE)
							mgr->flag |= USBLINK_PROTOCOL_HEADER_OPCODE ;
					   mgr->total = pstart->flag_total_bytes  & USBLINK_PROTOCOL_HEADER_LEN_MASK;
					   mgr->cur_cnt=0;
					   mgr->crc=pstart->crc;
						if(mgr->total>USBLINK_MSG_MAX_SIZE){
							mgr->total=0;
							return -2;
						}					
					  
				   }
			   //pop data
				if (pstart->flag_total_bytes & USBLINK_PROTOCOL_HEADER_END){
						if(0==mgr->cur_cnt)	{
							pop_msg_data(mgr,&rx_buf[sizeof(usblink_start_crc_msg_t)], read_res - sizeof(usblink_start_crc_msg_t));

							}
						else {
							pop_msg_data(mgr,&rx_buf[sizeof(usblink_start_msg_t)], read_res - sizeof(usblink_start_msg_t));
						}
					
						if(mgr->cur_cnt<mgr->total){
						   return -2;//drop the pkt
						   }
						else {//ok, we got a usblink msg, the issue it
						   mgr->msg.len= mgr->total;   
						   crc_ret=crc16_calc(mgr->msg.payload,mgr->msg.len);
						   if(crc_ret != mgr->crc){						
								rx_crc_cnt++;
								return -1;
							}else {
							
								return 0;
								}
												 
						}
			   }
			   else {		

			   		if(0==mgr->cur_cnt)	{
						pop_msg_data(mgr,&rx_buf[sizeof(usblink_start_crc_msg_t)], read_res - sizeof(usblink_start_crc_msg_t));

						}
					else {
						pop_msg_data(mgr,&rx_buf[sizeof(usblink_start_msg_t)], read_res - sizeof(usblink_start_msg_t));
					}
					
				   }
	
		   }
		rx_buf+=CONFIG_USB_VENDOR_RX_BUFSIZE;

		}
	return -3;


}




void tud_vendor_rx_cb(uint8_t itf,uint8_t * msg, int len)
{
	static usblink_msg_mgr_t m_mgr={0};
	usblink_msg_mgr_t *mgr=&m_mgr;
	int ret=0;


		mgr->msg.id=get_msg_id();
		  static int cnt=0;
		


		ret=decode_urb_msg(mgr,msg,len);
		if(ret>=0) {
		    if(mgr->flag & USBLINK_PROTOCOL_HEADER_OPCODE){
				push_cmd_resp(mgr->msg.payload,mgr->msg.len);
			}
			else {	

					link_host2wifi(&mgr->msg);
				
			}
		}
}









void usblink_process_rx_pkt(urb_msg_t * msg );


/* Get data from host */
void usblink_rx_task(void* pvParameters)
{
	urb_msg_t msg = {0};

	esp_err_t ret = ESP_OK;

	for (;;) {
		{
			vTaskDelay(10);
		}
		
	}
}



///////////////

void usblink_issue_resp( urb_msg_t * msg,cmd_resp64_t * cmd)
{
	esp_err_t ret=0;
	memcpy(msg->payload,cmd,sizeof(cmd_resp64_t));
	msg->cmd_flg=USBLINK_PROTOCOL_HEADER_OPCODE;
	msg->id=get_msg_id();
	msg->len=sizeof(cmd_resp64_t);
	if(pdTRUE!= xQueueSend(usblink_tx_cmd_queue, msg, 100 / portTICK_RATE_MS))
		{
		ESP_LOGI(TAG,"send msg %d to tx queue NG\n",msg->id);
		}

}



static int usblink_encode_urb_msg(uint16_t cmd_flg,uint8_t * urb_msg, const uint8_t * data, size_t data_len,int out_ep_max_size )
{

	int encoded_pos=0;
    const uint8_t * payload_in_bytes = data;
	usblink_start_msg_t *phd=NULL;
	int count=data_len;
    int len = 0;
	int size_to_copy=0;
	int buffer_avail_length= out_ep_max_size-sizeof(usblink_start_msg_t);
	uint16_t crc=0;
	crc=crc16_calc(data,data_len);
	count+=2;//resv crc bytes
    while(count) {
        // fill the buffer as much as possible...
        
        size_to_copy = count > buffer_avail_length ? buffer_avail_length : count;
		phd=(usblink_start_msg_t *)(urb_msg + encoded_pos);
		phd->flag_total_bytes=data_len;
		encoded_pos+=sizeof(usblink_start_msg_t);
		if(0 == len) {//mark start flag
			phd->flag_total_bytes|=USBLINK_PROTOCOL_HEADER_START|cmd_flg;
			memcpy(urb_msg + encoded_pos,&crc,2);//fill crc at header			
			size_to_copy-=2;
			encoded_pos+=2;
			len+=2;
			count-=2;
        	}
		if(size_to_copy>=count)//end case
		{	
			phd->flag_total_bytes|=USBLINK_PROTOCOL_HEADER_END|cmd_flg;			
    	}
		

        memcpy(urb_msg + encoded_pos, payload_in_bytes,size_to_copy);
        len += size_to_copy;
        payload_in_bytes += size_to_copy;
        encoded_pos += size_to_copy;
        count -= size_to_copy;

	

     }
	
	#if 1
	if(0==encoded_pos%out_ep_max_size){
		encoded_pos+=4;
	}
	#endif
	return encoded_pos;

}



void usb_error_recovery2(void)
{

	esp_restart();
		

}


#define USB_EP_OUT_SIZE 64

void usblink_process_tx_pkt(urb_msg_t * msg )
{

	static long t_a=0,t_b=0,t_diff=0;
	uint8_t urb_msg[USBLINK_MSG_MAX_SIZE];
	uint32_t wt_cnt=0;
	int max_retry=30;
	
	int urb_len=usblink_encode_urb_msg(msg->cmd_flg,urb_msg,msg->payload,msg->len,USB_EP_OUT_SIZE);


		memcpy(msg->payload,urb_msg,urb_len);
		msg->len=urb_len;
		wt_cnt=tud_vendor_submit_tx_urb(msg);
	

		//
		if(0==wt_cnt ){
			static int err_cnt=0;	
			usb_error_recovery2();
						
			return;
		} 

}


/* Send data to host */
void usblink_tx_task(void* pvParameters)
{

	urb_msg_t msg = {0};
	uint16_t pkts_waiting = 0;

	while (1) {
		{
			int i=0;
		
				if (uxQueueMessagesWaiting(usblink_tx_queue)) { 
					if (pdTRUE==xQueueReceive(usblink_tx_queue, &msg, 20 / portTICK_RATE_MS)){
						if (tud_ready()) {
							usblink_process_tx_pkt(&msg);
						}
					}
					else{
						ESP_LOGI(TAG,"tx queue recv timeout NG\n");						
						}
				} 
				//handle response
				if (uxQueueMessagesWaiting(usblink_tx_cmd_queue)) { 
					if (pdTRUE==xQueueReceive(usblink_tx_cmd_queue, &msg, 20 / portTICK_RATE_MS)){
						if (tud_ready()) {
							usblink_process_tx_pkt(&msg);
						}else {
							ESP_LOGI(TAG,"tusb no ready\n");
						}
					}
					else{
						ESP_LOGI(TAG,"tx cmd queue recv timeout NG\n");
						break;
						}
				} 
		}
		//
	}
}


int usb_link_send_msg( uint8_t * msg_data, int msg_len){
	static urb_msg_t  usb_msg;
	urb_msg_t *msg=&usb_msg;
	static int usb_go_flg=0;
	int ret=-1;	
		if (!tud_ready()) {		
			ret= -1;
			if(!tud_mounted() && usb_go_flg){
				esp_restart();
				}
			goto out;
		}
		
		usb_go_flg=1;
		if(msg_len > USBLINK_MSG_MAX_SIZE ) {
			ret =-2;
			goto out;
		}
		memcpy(msg->payload,msg_data,msg_len);
		msg->len=msg_len;
		msg->id=get_msg_id();
		if(pdTRUE!= xQueueSend(usblink_tx_queue, msg, 50 / portTICK_RATE_MS))
		{
			ESP_LOGI(TAG,"send wifi msg%d to tx queue NG\n",msg->id);
			ret =-3;
			goto out;
		}
		ret=0;

out:
return ret;

}


int usb_link_send_outband_msg( urb_msg_t * msg){
	msg->id=get_msg_id();

	if(pdTRUE!= xQueueSend(usblink_tx_cmd_queue, msg, 500 / portTICK_RATE_MS))
	{
		ESP_LOGI(TAG,"send scan ap list msg%d to tx queue NG\n",msg->id);
		return -1;
	}
return 0;
}



void usb_link_init(void)
{
    ESP_LOGI(TAG, "USB initialization");

    // Setting of descriptor. You can use descriptor_tinyusb and
    // descriptor_str_tinyusb as a reference
    tusb_desc_device_t my_descriptor = {
        .bLength = sizeof(my_descriptor),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200, // USB version. 0x0200 means version 2.0
        .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
        .bMaxPacketSize0 = CFG_TUD_ENDOINT0_SIZE,

        .idVendor = 0x303A,
        .idProduct = 0x1987,
        .bcdDevice = 0x0101, // Device FW version

        .iManufacturer = 0x01, // see string_descriptor[1] bellow
        .iProduct = 0x02,      // see string_descriptor[2] bellow
        .iSerialNumber = 0x03, // see string_descriptor[3] bellow

        .bNumConfigurations = 0x01
    };

    tusb_desc_strarray_device_t my_string_descriptor = {
        // array of pointer to string descriptors
        (char[]) {
            0x09, 0x04
        }, // 0: is supported language is English (0x0409)
        "xfz",                  // 1: Manufacturer
        "uwifi",   // 2: Product
        "012-2021",            // 3: Serials, should use chip ID
        "1986",            // 4: vendor
    };

    tinyusb_config_t tusb_cfg = {
        .descriptor = &my_descriptor,
        .string_descriptor = my_string_descriptor,
        .external_phy = false // In the most cases you need to use a `false` value
    };


    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	usblink_rx_queue = xQueueCreate(USBLINK_TO_HOST_QUEUE_SIZE, sizeof(urb_msg_t));
	usblink_tx_queue = xQueueCreate(USBLINK_TO_HOST_QUEUE_SIZE, sizeof(urb_msg_t));
	usblink_tx_cmd_queue = xQueueCreate(USBLINK_TO_HOST_QUEUE_SIZE, sizeof(urb_msg_t));
	assert(xTaskCreate(usblink_tx_task , "tx_task" , 1024*8 , NULL , 4 , NULL) == pdTRUE);
	assert(xTaskCreate(usblink_rx_task , "rx_task" , 1024*8 , NULL , 4 , NULL) == pdTRUE);

    ESP_LOGI(TAG, "USB initialization DONE");
}





