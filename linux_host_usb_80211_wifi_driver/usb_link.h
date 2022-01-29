/*
 *    xfz1986 Linux wifi usb layer Driver
 *
 *    Copyright (C) 2022-
 *    This file is licensed under the GPL. 
 *
 *    Author chaunjin
 *   -----------------------------------------------------------------
 *    USB layer Driver Implementations
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>


#ifndef __USB_LINK_H__
#define __USB_LINK_H__


#define USBLINK_PROTOCOL_HEADER_START  (0x1<<15)
#define USBLINK_PROTOCOL_HEADER_END    (0x1<<14)
#define USBLINK_PROTOCOL_HEADER_OPCODE (0x1<<13)
#define USBLINK_PROTOCOL_HEADER_LEN_MASK (0x0fff)  //max 4KB




typedef struct {
    uint16_t flag_total_bytes;
} __attribute__((packed)) usblink_start_msg_t;

typedef struct {
    uint16_t flag_total_bytes;
	uint16_t crc;
} __attribute__((packed)) usblink_start_crc_msg_t;


#define USBLINK_MSG_MAX_SIZE 2048


typedef struct {
	uint8_t  payload[USBLINK_MSG_MAX_SIZE];
	size_t len;
} usblink_msg_t;



#define USBLINK_RX_BUFFER_SIZE   1728
#define USBLINK_TX_BUFFER_SIZE   1728
#define USBLINK_RX_EP_SIZE 64


typedef struct  {
    int flag;
    int total;    
    int cur_cnt; 
	uint16_t crc;
	usblink_msg_t msg;
} usblink_msg_mgr_t;


struct usb_link_dev {
    struct mutex                       op_locker;
    int                               is_alive;

    // usb device info
    struct usb_device               *  udev;
    struct usb_interface            *  interface;
	void * upper_obj;

	usblink_msg_mgr_t msg_mgr;

    // status package related
    uint8_t                               resp_buffer[USBLINK_RX_BUFFER_SIZE];

	int resp_len;
    uint8_t                               rx_buffer[USBLINK_RX_BUFFER_SIZE]; // data buffer for the IN endpoint

    uint8_t                               tx_buffer[USBLINK_TX_BUFFER_SIZE]; // data buffer for the OUT endpoint

	int tx_len;
    uint8_t                               tx_cmd_buffer[USBLINK_TX_BUFFER_SIZE]; // data buffer for the OUT endpoint

	int tx_cmd_len;

	
    uint8_t                               in_rx_ep_addr;
	uint8_t                               out_tx_ep_addr;

	struct urb *tx_urb;	/* URB for sending data */
	struct urb *tx_cmd_urb;	/* URB for cmd, we use this for avoid race-cond bug */
	struct urb *rx_urb;	/* URB for receiving data */

    int                                urb_status_fail_count;
    size_t                             out_ep_tx_max_size;  
	struct work_struct ws_rx;
	struct work_struct ws_tx;
	int usb_rx_pipe_status;
	

};


int  usb_link_issue_cmd_resp(struct usb_link_dev * dev,const uint8_t * cmd, int msg_len,uint8_t * resp,int resp_len);

void usb_link_send_msg(struct usb_link_dev * dev,const uint8_t * msg_data, int msg_len);


#endif
