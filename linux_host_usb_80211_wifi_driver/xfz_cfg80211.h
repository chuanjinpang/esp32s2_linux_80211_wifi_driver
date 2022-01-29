/*

a diy project refer espressif hosted project for usb 802.11 wifi
1.first write on 20211111 by chuanjin
 */


#ifndef __XFZ_CFG80211_H__
#define __XFZ_CFG80211_H__

#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>



#include "msg.h"

#define MAX_SSID_SIZE 128
struct xfz_cfg80211_adapter {
	//cfg 802.11
	struct net_device *ndev;
	struct net_device_stats stats;
	struct wiphy *wiphy;
	struct semaphore sem;
	struct cfg80211_scan_request *scan_request;
	struct work_struct ws_scan;
	char connecting_ssid[MAX_SSID_SIZE];
	int ssid_len;
	u8 psk[32];
	struct work_struct ws_connect;
	u16 disconnect_reason_code;
	struct work_struct ws_disconnect;

	struct sk_buff_head         rx_queue;

	struct workqueue_struct *if_rx_workqueue;
	struct work_struct       if_rx_work;

	/* Process TX work */
	struct work_struct ws_evt;
	msg_queue_t msg_mgr;

	void * bottom_obj;
};




struct xfz_cfg80211_skb_cb {
	struct xfz_cfg80211_adapter      *priv;
};


int xfz_cfg80211_wiphy_init(struct xfz_cfg80211_adapter * apt);

#endif
