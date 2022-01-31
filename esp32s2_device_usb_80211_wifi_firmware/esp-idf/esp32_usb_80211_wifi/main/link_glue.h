


#ifndef __LINK_GLUE__H_
#define __LINK_GLUE__H_


#include "class/vendor/vendor_device.h"


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
} __attribute__((packed)) cmd_resp64_t;

#define WLAN_PMK_LEN			32

typedef struct {
uint8_t psk[WLAN_PMK_LEN];
uint8_t ssid[21];
} __attribute__((packed)) ssid_psk_msg_t;

int link_send_msg( uint8_t * msg_data, int msg_len);



int usblink_issue_connect_done( uint8_t * mac);

void usb_link_init(void);



#endif

