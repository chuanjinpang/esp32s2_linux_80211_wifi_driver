
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <net/cfg80211.h> 

#include "xfz_cfg80211.h"
#include "link_glue.h"
#include "msg.h"
#include "log.h"
#include "usb_link.h"


void * xfz_cfg80211_init(void * bottom_obj);
void xfz_cfg80211_exit(void);


void upper_layer_status (void * uobj){

    struct xfz_cfg80211_adapter *adapter=uobj;

	if(!uobj)
			return;

	if (netif_queue_stopped((const struct net_device *) adapter->ndev) ) {
		
	}else {
		}

}


void  push_skb_upper_layer(void * uobj, struct sk_buff *skb){

	struct xfz_cfg80211_adapter *apt=uobj;


	if(!uobj)
		return;

	skb_queue_tail(&apt->rx_queue, skb);

	if(apt)
		queue_work(apt->if_rx_workqueue, &apt->if_rx_work);

}


/*

if we handler it, return true means we handle the msg, 
or the lower layer will handle it.
and the msg is in irq context, so should consider it for handle
*/
int upper_layer_msg_handler(void * ctx, uint8_t * resp, size_t len){
	cmd_resp64_t * pcmd=(cmd_resp64_t *)resp;
	struct xfz_cfg80211_adapter * apt = ctx;
int ret=0;
	switch(pcmd->cmd_op) {
		case OUT_BAND_CMD_AP_LIST:
		{
			msg_queue_put_msg(&apt->msg_mgr,OUT_BAND_CMD_AP_LIST,resp,len);
			
		  if (!schedule_work(&apt->ws_evt)) {
	        LOGE("%s shecd evt ws NG\n",__FUNCTION__);
	    	}

		  ret= 1;//handle it
		 break;
		  
		}
		case OUT_BAND_CMD_CONNECTED_DONE:
		{
			msg_queue_put_msg(&apt->msg_mgr,OUT_BAND_CMD_CONNECTED_DONE,resp,len);
			
		  if (!schedule_work(&apt->ws_evt)) {
	        LOGE("%s shecd evt ws NG\n",__FUNCTION__);
	    	}

		  ret= 1;//handle it
		 break;
		  
		}
		default:
		;
	}
return ret;//0 no handle

}



void link_tx_msg(void * bottom_obj,const uint8_t * msg_data, int msg_len){
	struct usb_link_dev * dev =bottom_obj;

	usb_link_send_msg(dev,msg_data,msg_len);

}




int  link_issue_cmd_resp(void * bottom_obj, uint8_t * msg,int msg_len,uint8_t * resp,int resp_len)
{

	struct usb_link_dev * dev=bottom_obj;

	return usb_link_issue_cmd_resp(dev,msg,msg_len,resp,resp_len);
		

}



void * upper_layer_init(void * bottom_obj){

	return xfz_cfg80211_init(bottom_obj);


}

void  upper_layer_deinit(void){

	 xfz_cfg80211_exit();


}

