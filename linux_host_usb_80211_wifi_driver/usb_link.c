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
#include <linux/fb.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/delay.h>

#include "usb_link.h"
#include "link_glue.h"

#include "log.h"


#define USB_EP_OUT_SIZE 64


#define USBLINK_RX_RETRY_COUNT 3
#define USBLINK_MAX_TRANSFER_SIZE          (4096)


#define TX_DELAY_MIN 500
#define TX_DELAY_MAX 900

#define RX_DELAY_MIN 500
#define RX_DELAY_MAX 900


#define DL_ALIGN_UP(x, a) ALIGN(x, a)
#define DL_ALIGN_DOWN(x, a) ALIGN(x-(a-1), a)


#define XFZ1986_USB_VENDOR_ID   0x303a 
#define XFZ1986_USB_PRODUCT_ID  0x1987 //1986 used for usb display,so we use +1=1987

static const struct usb_device_id id_table[] = {
    {
        .idVendor    = XFZ1986_USB_VENDOR_ID,
        .idProduct   = XFZ1986_USB_PRODUCT_ID,
        .match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT,
    },
    { },
};



#include <linux/completion.h>

DECLARE_COMPLETION(cmd_completion);



static void _usblink_rx_handler(struct usb_link_dev * dev);


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






#define RAND_RANGE 3
static uint32_t x_random(void) {
//X(n+1) = (a * X(n) + c) % m
static uint32_t a=1103515245,c=12345,m=134217728, seed=31;
    seed = (a*seed + c)%m;
    return seed;
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




void push_skb( struct usb_link_dev *dev,uint8_t * msg, int len) 
{
u8 *rx_buf;
	int skb_len=0;
		struct sk_buff *skb = dev_alloc_skb(LINK_MSG_MAX_SIZE);
		if (!skb) {
			return;
		}
		skb_reserve(skb,2);
		rx_buf=skb_put(skb,0);
		if(len>LINK_MSG_MAX_SIZE){
			return;
			}
		memcpy(rx_buf,msg,len);
		skb_len=len;
		skb_put(skb,skb_len);
		push_skb_upper_layer(dev->upper_obj,skb);
}








void push_cmd_resp( struct usb_link_dev *dev,uint8_t * msg, int len) 
{
	cmd_resp64_t * pcmd=(cmd_resp64_t * )dev->resp_buffer;
	if(len>=USBLINK_RX_BUFFER_SIZE){
		return ;
	}
	memcpy(dev->resp_buffer,msg,len);
	dev->resp_len=len;
	
	if(upper_layer_msg_handler(dev->upper_obj, dev->resp_buffer,len))//upper layer hanle it
		return;
	switch(pcmd->cmd_op) {

		default:
			complete(&cmd_completion);
	}	

}




#define XFZ_MIN(x,y) (x)<(y)?(x):(y)


int decode_urb_msg(usblink_msg_mgr_t *mgr,uint8_t * rx_buf,int rx_len)
{

	int i=0;
	int read_res=0;
	int remain=rx_len;
	uint16_t crc_ret=0;
	static int crc_total_cnt=0;
	static int rx_crc_cnt=0;
	
	crc_total_cnt++;
	
	while(remain) {
			
				read_res=XFZ_MIN(USB_EP_OUT_SIZE,remain);
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
							LOGW("rx wrong len oversize:%d\n",mgr->total);
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
						   LOGW("usb rx error when end %d %d\n",mgr->cur_cnt,mgr->total);
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
		rx_buf+=USB_EP_OUT_SIZE;
	}
	return -3;


}
void push_urb_data(struct usb_link_dev *dev,uint8_t * buf,int len) {
usblink_msg_mgr_t *mgr=&dev->msg_mgr;
int ret=0;

	upper_layer_status (dev->upper_obj);

	ret=decode_urb_msg(mgr,buf,len);

	
	if(ret>=0) {
	    if(mgr->flag & USBLINK_PROTOCOL_HEADER_OPCODE){
			//so it maybe wrong data, it no integrity protect it.
			push_cmd_resp(dev,mgr->msg.payload,mgr->msg.len);
	    }
		else
			push_skb(dev,mgr->msg.payload,mgr->msg.len);
		

	}
}



static void  usb_rx_routine(struct work_struct *w) {
	struct usb_link_dev *dev= container_of(w, struct usb_link_dev, ws_rx);

	if(-EPIPE == dev->usb_rx_pipe_status) {
		LOGW("in %s  NG_cnt:%d clear halt\n",__FUNCTION__,dev->urb_status_fail_count);
		//usb_clear_halt(dev->udev, usb_rcvintpipe(dev->udev, dev->in_rx_ep_addr));
		dev->usb_rx_pipe_status=0;
		dev->urb_status_fail_count=0;
		msleep(50);
		}
	if(dev->usb_rx_pipe_status<0){
		LOGW("in %s rx urb error so wait\n",__FUNCTION__);
		msleep(20);
		}
#if 1	

		{		

		usleep_range(RX_DELAY_MIN,RX_DELAY_MAX);

		}
#endif

	_usblink_rx_handler(dev); 
}


static void usblink_rx_finished(struct urb *urb)
{
    struct usb_link_dev *dev = urb->context;

    if(!dev->is_alive) {
        return;
    }


    switch(urb->status) {
    case 0:
        // succeed    
		if(urb->actual_length>2)
			push_urb_data(dev,dev->rx_buffer,urb->actual_length);
     
        break;
    case -EPIPE:
		LOGW("%s EPIPE\n",__FUNCTION__);
		dev->usb_rx_pipe_status=-EPIPE;
    default:
        dev->urb_status_fail_count++;	
		LOGW("in %s error? %d actlen:%d\n",__FUNCTION__,urb->status,urb->actual_length);

    }


#if 1
    if(dev->urb_status_fail_count < USBLINK_RX_RETRY_COUNT) {

       dev->usb_rx_pipe_status=urb->status;
	   
       if (!schedule_work(&dev->ws_rx)) {
				   LOGE("%s shecd evt ws for rx NG\n",__FUNCTION__);
		}
    } else {
		dev->usb_rx_pipe_status=-EPIPE;
		if (!schedule_work(&dev->ws_rx)) {
				   LOGE("%s shecd evt ws EPIPE NG\n",__FUNCTION__);
		}


    }
	
#endif
}



static void _usblink_rx_handler(struct usb_link_dev * dev)
{
    unsigned int pipe;
    int status;
    struct usb_host_endpoint *ep;



    if(!dev->is_alive) {
        // the device is del
        return;
    }


	
#if 1
	

    pipe = usb_rcvbulkpipe(dev->udev, dev->in_rx_ep_addr);
    ep = usb_pipe_endpoint(dev->udev, pipe);


    if(!ep) return;	

    usb_fill_bulk_urb(dev->rx_urb, dev->udev, pipe, dev->rx_buffer, USBLINK_RX_BUFFER_SIZE,
                     usblink_rx_finished, dev );

    //submit it
    status = usb_submit_urb(dev->rx_urb, GFP_ATOMIC);
    if(status) {
        if(status == -EPIPE) {
			LOGE("in %s %d NG_cnt:%d clera halt\n",__FUNCTION__,status,dev->urb_status_fail_count);
            usb_clear_halt(dev->udev, pipe);
        }

        ++ dev->urb_status_fail_count;
    }
#else

    return ;
#endif
}




static int _usblink_encode_urb_msg(uint16_t cmd_flg,uint8_t * urb_msg, const uint8_t * data, size_t data_len,int out_ep_max_size )
{
	int encoded_pos=0;
    const uint8_t * payload_in_bytes = data;
	usblink_start_msg_t *phd=NULL;
    int len = 0;
	int count=data_len;
	int size_to_copy=0;
	int buffer_avail_length= out_ep_max_size-sizeof(usblink_start_msg_t);
	uint16_t crc=0;
	crc=crc16_calc(data,data_len);
	count+=2;//resv crc bytes, we can't padding crc to endof msg, it could ovf bug, so we add it at begin of encode msg. encode[crc16+msg]
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


    return encoded_pos;
}

static int usblink_encode_urb_msg(uint16_t cmd_flg,uint8_t * urb_msg, const uint8_t * data, size_t count,int out_ep_max_size ){
int urb_len=0;



urb_len=_usblink_encode_urb_msg(cmd_flg,urb_msg,data,count,out_ep_max_size);


	if(0==(urb_len%USB_EP_OUT_SIZE)){
		urb_len+=4;
	}

	 
	return urb_len;

}

typedef void ( *urb_tx_finished_cb )(struct urb *urb);

static void _on_urb_skb_tx_finished(struct urb *urb)
{
    struct usb_link_dev * dev= (struct usb_link_dev *)urb->context;
  
    if(urb->status) {

        LOGE("skb tx failed for urb %p, error code %x %d\n", urb, urb->status, urb->status);

    }
	upper_layer_queue_wakeup(dev->upper_obj);


}

static void _on_urb_cmd_tx_finished(struct urb *urb)
{
    struct usb_link_dev * dev= (struct usb_link_dev *)urb->context;
  
    if(urb->status) {

        LOGE("cmd tx failed for urb %p, error code %x %d\n", urb, urb->status, urb->status);

    }	


}


int issue_urb(struct usb_link_dev * dev, struct urb * turb,urb_tx_finished_cb cb, uint8_t * msg,size_t transfer_size )
{

	if(!dev->is_alive){
		LOGW("%s dev is no alive\n",__FUNCTION__);
		return -1;
		}


	if(transfer_size) {
		usb_fill_bulk_urb(turb, dev->udev, usb_sndbulkpipe(dev->udev, dev->out_tx_ep_addr), msg,
			  transfer_size, cb, dev);

	if(usb_submit_urb(turb, GFP_ATOMIC)) {
			// submit failure			
			LOGW("submit urb NG\n");
			return -1; 
			//abort
		}		
	}

	return 0;
}

static int tx_ws_cnt=0;
static void  usb_tx_routine(struct work_struct *w) {
	struct usb_link_dev *dev= container_of(w, struct usb_link_dev, ws_tx);


	if(dev->tx_len> 128){

		//wait a while for tx
		usleep_range(TX_DELAY_MIN,TX_DELAY_MAX);
		
	}

	

		if(issue_urb(dev,dev->tx_urb,_on_urb_skb_tx_finished,dev->tx_buffer,dev->tx_len)<0)
		{
			upper_layer_queue_wakeup(dev->upper_obj);//upper layer stop queue, we need wakeup it.
		}

	tx_ws_cnt--;

}



void usb_link_send_msg(struct usb_link_dev * dev,const uint8_t * msg_data, int msg_len)
{
		int urb_len=0;

	  urb_len=usblink_encode_urb_msg(0,dev->tx_buffer,msg_data,msg_len,USB_EP_OUT_SIZE);
      dev->tx_len=urb_len;
	  
       if (!schedule_work(&dev->ws_tx)) {
				   LOGE("%s shecd evt tx NG,tx_ws_cnt:%d\n",__FUNCTION__,tx_ws_cnt);
				   upper_layer_queue_wakeup(dev->upper_obj);
		}
	   else{
	   		tx_ws_cnt++;
		}

		

}




int  usb_link_issue_cmd_resp(struct usb_link_dev * dev,const uint8_t * cmd, int msg_len,uint8_t * resp,int resp_len)
{
	
	int i=0;
	uint32_t wt_cnt=0;
	int urb_len=0;


	urb_len=usblink_encode_urb_msg(USBLINK_PROTOCOL_HEADER_OPCODE,dev->tx_cmd_buffer,cmd,msg_len,USB_EP_OUT_SIZE);
	dev->tx_cmd_len=urb_len;



	reinit_completion(&cmd_completion);//need frist init completon before issue cmd,index no done before previous msg.
	

	issue_urb(dev,dev->tx_cmd_urb,_on_urb_cmd_tx_finished, dev->tx_cmd_buffer,urb_len);
	
	unsigned long time_left=wait_for_completion_timeout(&cmd_completion,3 * HZ);
	if(!time_left) {
		LOGE("wait for cmd resp timeout\n");
		return -1;
		}
	else {
		if(dev->resp_len > resp_len ){
			LOGE("got cmd resp too big %d %d\n",dev->resp_len,resp_len);
			return -2;
		}
		memcpy(resp,dev->resp_buffer,dev->resp_len);
		return 0;
		}
	
}





static int new_usb_link_device(struct usb_link_dev * dev)
{



    mutex_init(&dev->op_locker);  


    dev->tx_urb = usb_alloc_urb(0, GFP_KERNEL);
    if(! dev->tx_urb) {
        LOGE("can't allocate tx urb\n");
        goto urb_alloc_err;
    }

    dev->tx_cmd_urb = usb_alloc_urb(0, GFP_KERNEL);
    if(! dev->tx_cmd_urb) {
        LOGE("can't allocate tx_cmd_urb\n");
        goto urb_alloc_err;
    }


	dev->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(! dev->rx_urb) {
		LOGE("can't allocate rx urb\n");
		goto urb_alloc_err;
	}

    dev->is_alive = 1;

	// int tx & rx work struct
	INIT_WORK(&dev->ws_rx, usb_rx_routine);
	INIT_WORK(&dev->ws_tx, usb_tx_routine);
	dev->usb_rx_pipe_status=0;
    _usblink_rx_handler(dev);

	dev->upper_obj=upper_layer_init(dev);
    return 0;


urb_alloc_err:
	if(dev->tx_urb)
		usb_free_urb(dev->tx_urb);
	if(dev->tx_cmd_urb)
			usb_free_urb(dev->tx_cmd_urb);	
	if(dev->rx_urb )
		usb_free_urb(dev->rx_urb);
	

    return -ENOMEM;
}


static void _on_del_usb_device(struct usb_link_dev * dev)
{

    mutex_lock(&dev->op_locker);
    dev->is_alive = 0;
    mutex_unlock(&dev->op_locker);

 
    // kill all pending urbs
    usb_kill_urb(dev->rx_urb);
	usb_free_urb(dev->rx_urb);

	
	usb_kill_urb(dev->tx_urb);
    usb_free_urb(dev->tx_urb);

	usb_kill_urb(dev->tx_cmd_urb);
	usb_free_urb(dev->tx_cmd_urb);

	
    dev->tx_urb = NULL;
	dev->rx_urb = NULL;
	dev->tx_cmd_urb = NULL;

}



static int usb_link_probe(struct usb_interface *interface, const struct usb_device_id *id)
{

    struct usb_link_dev *dev = NULL;
    struct usb_host_interface *iface_desc;
    struct usb_endpoint_descriptor *endpoint;
    size_t buffer_size;
    int i;
    int retval = -ENOMEM;
   
    dev = kzalloc(sizeof(struct usb_link_dev), GFP_KERNEL);
    if(dev == NULL) {
        LOGE("out of memory\n");
        goto error;
    }

    dev->udev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;

  

    // check for endpoints
    iface_desc = interface->cur_altsetting;

    for(i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
        endpoint = &iface_desc->endpoint[i].desc;

        if(!dev->in_rx_ep_addr &&
           usb_endpoint_is_bulk_in(endpoint)) {
                   
            buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
            dev->in_rx_ep_addr = endpoint->bEndpointAddress;

        }

        if(!dev->out_tx_ep_addr &&
           usb_endpoint_is_bulk_out(endpoint) && endpoint->wMaxPacketSize) {
           
            dev->out_tx_ep_addr = endpoint->bEndpointAddress;
            dev->out_ep_tx_max_size = le16_to_cpu(endpoint->wMaxPacketSize);

        }
    }

    if(!(dev->in_rx_ep_addr && dev->out_tx_ep_addr)) {
        LOGE("we can't find the in:%x or out:%x ep\n",dev->in_rx_ep_addr,dev->out_tx_ep_addr);
        goto error;
    }


    // set ctx to itf
    usb_set_intfdata(interface, dev);

    // add the device  
    if(new_usb_link_device(dev)) {
        goto error;
    }


    return 0;

error:
    if(dev) {
        kfree(dev);
    }
    return retval;
}



static int usb_link_suspend(struct usb_interface *intf, pm_message_t message)
{
    struct usb_lcd *dev = usb_get_intfdata(intf);

    if(!dev)
        return 0;
    // not implemented yet
    return 0;
}


static int usb_link_resume(struct usb_interface *intf)
{
    // not implemented yet
    return 0;
}




static void usb_link_disconnect(struct usb_interface *interface)
{
    struct usb_link_dev *dev;

    dev = usb_get_intfdata(interface);
	dev->is_alive = 0;//frist no rx usr

	upper_layer_deinit();
    usb_set_intfdata(interface, NULL);
    _on_del_usb_device(dev);	
    kfree(dev);
}


static struct usb_driver xfz_usblink_driver = {
    .name       =  "xfz_usb_link",
    .probe      =  usb_link_probe,
    .disconnect =  usb_link_disconnect,
    .suspend    =  usb_link_suspend,
    .resume     =  usb_link_resume,
    .id_table   =  id_table,
    .supports_autosuspend = 0,
};

int __init register_usb_link(void)
{

    return usb_register(&xfz_usblink_driver);
}


void __exit unregister_usb_link(void)
{


    usb_deregister(&xfz_usblink_driver);
}



