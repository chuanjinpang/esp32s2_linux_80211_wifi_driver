
#ifndef __USB_LINK_H__
#define __USB_LINK_H__


#define USBLINK_PROTOCOL_HEADER_START  (0x1<<15)
#define USBLINK_PROTOCOL_HEADER_END    (0x1<<14)
#define USBLINK_PROTOCOL_HEADER_OPCODE (0x1<<13)
#define USBLINK_PROTOCOL_HEADER_LEN_MASK (0x0fff)  //max 4KB


#if 0
#define USBLINK_MSG_MAX_SIZE (1728) //27*64


typedef struct {
	int id;
	uint16_t cmd_flg;
	size_t len;
	uint8_t  payload[USBLINK_MSG_MAX_SIZE];
	
} urb_msg_t;
#endif



typedef struct {
    uint16_t flag_total_bytes;
} __attribute__((packed)) usblink_start_msg_t;

typedef struct {
    uint16_t flag_total_bytes;
	uint16_t crc;
} __attribute__((packed)) usblink_start_crc_msg_t;


typedef struct  {
    int flag;
    int total;    
    int cur_cnt; 
	uint16_t crc;
	urb_msg_t msg;
} usblink_msg_mgr_t;


void usb_link_init(void);
int usb_link_send_msg( uint8_t * msg_data, int msg_len);

int usb_link_send_outband_msg( urb_msg_t * msg);

void link_host2wifi(urb_msg_t * msg );
void usblink_issue_resp( urb_msg_t * msg,cmd_resp64_t * cmd);

#endif

