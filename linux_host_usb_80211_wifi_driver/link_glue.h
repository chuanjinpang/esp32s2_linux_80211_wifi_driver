
#ifndef __LINK_GLUE__H_
#define __LINK_GLUE__H_

#define OUT_BAND_CMD_GET_MAC 1
#define OUT_BAND_CMD_SCAN 2
#define OUT_BAND_CMD_AP_LIST 3
#define OUT_BAND_CMD_CONNECT_AP 4
#define OUT_BAND_CMD_DISCONNECT_AP 5
#define OUT_BAND_CMD_CONNECTED_DONE 6

#define LINK_MSG_MAX_SIZE  2048


typedef struct {
uint16_t cmd_op;
uint16_t seq;
uint16_t  len;
uint8_t  buf[53];
} __attribute__((packed)) cmd_resp64_t; //should less than 60 bytes, due to usb max is 64byte. overhead 2 byte. 62 less 64 means a short packet


#define WLAN_PMK_LEN			32

typedef struct {
uint8_t psk[WLAN_PMK_LEN];
uint8_t ssid[21];
} __attribute__((packed)) ssid_psk_msg_t;

int upper_layer_msg_handler(void * ctx, uint8_t * resp, size_t len);
void upper_layer_queue_wakeup(void * uobj);
void upper_layer_status (void * uobj);

void  push_skb_upper_layer(void * uobj, struct sk_buff *skb);


void link_tx_msg(void * bottom_obj,const uint8_t * msg_data, int msg_len);

int  link_issue_cmd_resp(void * bottom_obj, uint8_t * msg,int msg_len,uint8_t * resp,int resp_len);

void * upper_layer_init(void * bottom_obj);

void  upper_layer_deinit(void);

#endif
