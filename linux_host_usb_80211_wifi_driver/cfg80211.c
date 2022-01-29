
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <net/cfg80211.h> /* wiphy and probably everything that would required for FullMAC driver */


#include "xfz_cfg80211.h"
#include "link_glue.h"
#include "msg.h"
#include "log.h"

#define WIPHY_NAME "xwlan"


struct xfz_wiphy_priv_context {
    struct xfz_cfg80211_adapter * apt;
};


static struct xfz_wiphy_priv_context *
wiphy_get_xfz_context(struct wiphy *wiphy)
	{ return (struct xfz_wiphy_priv_context *) wiphy_priv(wiphy); }




typedef enum {
    WIFI_SECOND_CHAN_NONE = 0,  /**< the channel width is HT20 */
    WIFI_SECOND_CHAN_ABOVE,     /**< the channel width is HT40 and the secondary channel is above the primary channel */
    WIFI_SECOND_CHAN_BELOW,     /**< the channel width is HT40 and the secondary channel is below the primary channel */
} wifi_second_chan_t;

typedef enum {
    WIFI_AUTH_OPEN = 0,         /**< authenticate mode : open */
    WIFI_AUTH_WEP,              /**< authenticate mode : WEP */
    WIFI_AUTH_WPA_PSK,          /**< authenticate mode : WPA_PSK */
    WIFI_AUTH_WPA2_PSK,         /**< authenticate mode : WPA2_PSK */
    WIFI_AUTH_WPA_WPA2_PSK,     /**< authenticate mode : WPA_WPA2_PSK */
    WIFI_AUTH_WPA2_ENTERPRISE,  /**< authenticate mode : WPA2_ENTERPRISE */
    WIFI_AUTH_WPA3_PSK,         /**< authenticate mode : WPA3_PSK */
    WIFI_AUTH_WPA2_WPA3_PSK,    /**< authenticate mode : WPA2_WPA3_PSK */
    WIFI_AUTH_WAPI_PSK,         /**< authenticate mode : WAPI_PSK */
    WIFI_AUTH_MAX
} wifi_auth_mode_t;
	
typedef enum {
    WIFI_CIPHER_TYPE_NONE = 0,   /**< the cipher type is none */
    WIFI_CIPHER_TYPE_WEP40,      /**< the cipher type is WEP40 */
    WIFI_CIPHER_TYPE_WEP104,     /**< the cipher type is WEP104 */
    WIFI_CIPHER_TYPE_TKIP,       /**< the cipher type is TKIP */
    WIFI_CIPHER_TYPE_CCMP,       /**< the cipher type is CCMP */
    WIFI_CIPHER_TYPE_TKIP_CCMP,  /**< the cipher type is TKIP and CCMP */
    WIFI_CIPHER_TYPE_AES_CMAC128,/**< the cipher type is AES-CMAC-128 */
    WIFI_CIPHER_TYPE_SMS4,       /**< the cipher type is SMS4 */
    WIFI_CIPHER_TYPE_UNKNOWN,    /**< the cipher type is unknown */
} wifi_cipher_type_t;


/**
  * @brief WiFi antenna
  *
  */
typedef enum {
    WIFI_ANT_ANT0,          /**< WiFi antenna 0 */
    WIFI_ANT_ANT1,          /**< WiFi antenna 1 */
    WIFI_ANT_MAX,           /**< Invalid WiFi antenna */
} wifi_ant_t;


typedef enum {
    WIFI_COUNTRY_POLICY_AUTO,   /**< Country policy is auto, use the country info of AP to which the station is connected */
    WIFI_COUNTRY_POLICY_MANUAL, /**< Country policy is manual, always use the configured country info */
} wifi_country_policy_t;

/** @brief Structure describing WiFi country-based regional restrictions. */
typedef struct {
    char                  cc[3];   /**< country code string */
    uint8_t               schan;   /**< start channel */
    uint8_t               nchan;   /**< total channel number */
    int8_t                max_tx_power;   /**< This field is used for getting WiFi maximum transmitting power, call esp_wifi_set_max_tx_power to set the maximum transmitting power. */
    wifi_country_policy_t policy;  /**< country policy */
} wifi_country_t;


/** @brief Description of a WiFi AP */
typedef struct {
    uint8_t bssid[6];                     /**< MAC address of AP */
    uint8_t ssid[33];                     /**< SSID of AP */
    uint8_t primary;                      /**< channel of AP */
    wifi_second_chan_t second;            /**< secondary channel of AP */
    int8_t  rssi;                         /**< signal strength of AP */
    wifi_auth_mode_t authmode;            /**< authmode of AP */
    wifi_cipher_type_t pairwise_cipher;   /**< pairwise cipher of AP */
    wifi_cipher_type_t group_cipher;      /**< group cipher of AP */
    wifi_ant_t ant;                       /**< antenna used to receive beacon from AP */
    uint32_t phy_11b:1;                   /**< bit: 0 flag to identify if 11b mode is enabled or not */
    uint32_t phy_11g:1;                   /**< bit: 1 flag to identify if 11g mode is enabled or not */
    uint32_t phy_11n:1;                   /**< bit: 2 flag to identify if 11n mode is enabled or not */
    uint32_t phy_lr:1;                    /**< bit: 3 flag to identify if low rate is enabled or not */
    uint32_t wps:1;                       /**< bit: 4 flag to identify if WPS is supported or not */
    uint32_t ftm_responder:1;             /**< bit: 5 flag to identify if FTM is supported in responder mode */
    uint32_t ftm_initiator:1;             /**< bit: 6 flag to identify if FTM is supported in initiator mode */
    uint32_t reserved:25;                 /**< bit: 7..31 reserved */
    wifi_country_t country;               /**< country information of AP */
} wifi_ap_record_t;


#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]




int xfz_scan_done_evt(struct xfz_cfg80211_adapter * apt ,uint8_t * resp,bool aborted);


/* Just calls cfg80211_disconnected() that informs the kernel that disconnect is complete.
 * Overall disconnect may call cfg80211_connect_timeout() if disconnect interrupting connection routine, but for this demo I keep it simple.
 * This routine called through workqueue, when the kernel asks about disconnect through cfg80211_ops. */
static void xfz_disconnect_routine(struct work_struct *w) {

    struct xfz_cfg80211_adapter * apt = container_of(w, struct xfz_cfg80211_adapter, ws_disconnect);

	if(!apt->bottom_obj)
		return;//remove device case

    if(down_interruptible(&apt->sem)) {
        return;
    }
	
//issue cmd 	

	{

		cmd_resp64_t cmd;
		cmd.cmd_op=OUT_BAND_CMD_DISCONNECT_AP;
		cmd.len=0;		
								 
		if( link_issue_cmd_resp(apt->bottom_obj, (uint8_t * )&cmd,sizeof(cmd),(uint8_t *)&cmd,sizeof(cmd))>=0)
		{
			
		}
		else{
			LOGE("%s can't issue disconnect cmd\n",__FUNCTION__);		
			}
	}



    cfg80211_disconnected(apt->ndev, apt->disconnect_reason_code, NULL, 0, true, GFP_KERNEL);

    apt->disconnect_reason_code = 0;

	
    up(&apt->sem);

	xfz_scan_done_evt(apt, NULL,true);
}





static void inform_single_bss(struct xfz_cfg80211_adapter  * apt,wifi_ap_record_t *ap) {
    struct cfg80211_bss *bss = NULL;
	int ie_len=0;
    struct cfg80211_inform_bss data = {
            .chan = &apt->wiphy->bands[NL80211_BAND_2GHZ]->channels[0], /* the only channel for this demo */
            .scan_width = NL80211_BSS_CHAN_WIDTH_20,
            /* signal "type" not specified in this DEMO so its basically unused, it can be some kind of percentage from 0 to 100 or mBm value*/
            /* signal "type" may be specified before wiphy registration by setting wiphy->signal_type */
            .signal = ap->rssi*100,
    };
    //char bssid[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    char   rsn_ie[]={0x30,0x14,0x01,0x00,0x00,0x0F,0xAC,0x04,0x01,0x00,0x00,0x0F,0xAC,0x04,0x01,0x00,0x00,0x0F,0xAC,0x02,0x00,0x00};
    //char rsn_ie[]={0x30,0x14,0x01,0x00,0x00,0x0F,0xAC,0x04,0x01,0x00,0x00,0x0F,0xAC,0x04,0x01,0x00,0x00,0x0F,0xAC,0x02,0x00,0x00};
	char wpa_ie[]={0xDD,0x16,0x00,0x50,0xF2,0x01,0x01,0x00,0x00,0x50,0xF2,0x04,0x01,0x00,0x00,0x50,0xF2,0x04,0x01,0x00,0x00,0x50,0xF2,0x02};
    /* ie - array of tags that usually retrieved from beacon frame or probe responce. */
    char ie[256] = {0};
	ie[0]=WLAN_EID_SSID;
	ie[1]=strlen(ap->ssid);
	ie_len+=2;
	memcpy(ie + 2, ap->ssid, strlen(ap->ssid));
	ie_len+=strlen(ap->ssid);
	memcpy(ie + ie_len, rsn_ie, sizeof(rsn_ie));
	ie_len+= sizeof(rsn_ie);
	memcpy(ie + ie_len, wpa_ie, sizeof(wpa_ie));
	ie_len+= sizeof(wpa_ie);
   

    /* also it posible to use cfg80211_inform_bss() instead of cfg80211_inform_bss_data() */
    bss = cfg80211_inform_bss_data(apt->wiphy, &data, CFG80211_BSS_FTYPE_UNKNOWN, ap->bssid, 0, WLAN_CAPABILITY_ESS|WLAN_CAPABILITY_PRIVACY, 100,
                                   ie, ie_len, GFP_KERNEL);

    /* free, cfg80211_inform_bss_data() returning cfg80211_bss structure refcounter of which should be decremented if its not used. */
    cfg80211_put_bss(apt->wiphy, bss);
}

int xfz_scan_done_evt(struct xfz_cfg80211_adapter * apt ,uint8_t * resp,bool aborted){


	int i=0;

	    struct cfg80211_scan_info info = {
            /* if scan was aborted by user(calling cfg80211_ops->abort_scan) or by any driver/hardware issue - field should be set to "true"*/
            .aborted = aborted,
    };




	if (aborted)
		goto out;

	{
	
		cmd_resp64_t * pcmd=(cmd_resp64_t *)resp;
		int sta_number=pcmd->len/sizeof(wifi_ap_record_t);
		wifi_ap_record_t *ap_list_buffer=(wifi_ap_record_t *)pcmd->buf;

		for(i=0; i<sta_number; i++) {
		   inform_single_bss(apt,&ap_list_buffer[i]);
		}

	}
out:
    if(down_interruptible(&apt->sem)) {
        return -1;
    }
	if (apt->scan_request) { //atomic done it or panic
	
	/* finish scan */
	   cfg80211_scan_done(apt->scan_request, &info);
	   apt->scan_request = NULL;
	}
	up(&apt->sem);

}

/* "Scan" routine for DEMO. It just inform the kernel about "dummy" BSS and "finishs" scan.
 * When scan is done it should call cfg80211_scan_done() to inform the kernel that scan is finished.
 * This routine called through workqueue, when the kernel asks about scan through cfg80211_ops. */
static void xfz_scan_routine(struct work_struct *w) {
    struct xfz_cfg80211_adapter * apt = container_of(w, struct xfz_cfg80211_adapter, ws_scan);


    /* pretend some work, also u can't call cfg80211_scan_done right away after cfg80211_ops->scan(),
     * idk why, but netlink client would not get message with "scan done",
     * is it because of "scan_routine" and cfg80211_ops->scan() may run in concurrent and cfg80211_scan_done() called before cfg80211_ops->scan() returns? */
#ifdef NO_USB_SIMULATOR

	msleep(100);

	wifi_ap_record_t ap_list_buffer={
		.ssid="Gohighsec-2.4",
		.bssid={0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff},
		};
	
	inform_single_bss(apt,&ap_list_buffer);

#else

	
	{
		
			cmd_resp64_t cmd;
			cmd.cmd_op=OUT_BAND_CMD_SCAN;
			cmd.len=0;
			 
			if( link_issue_cmd_resp(apt->bottom_obj,(uint8_t *)&cmd,sizeof(cmd),(uint8_t *)&cmd,sizeof(cmd))>=0)
			{
				
				
			}
			else{
				LOGE("%s can't issue scan cmd\n",__FUNCTION__);
				xfz_scan_done_evt(apt,NULL,true);
				return;
				}
	}
#endif

    //wait 5s timeout scan list result
	{
	int i=0;
		for(i=0;i<10;i++) {
			msleep(500);
			if (NULL == apt->scan_request)
				break;
		}
		if (apt->scan_request) { //5s no scan done, we done it, for non-block user scan req
				xfz_scan_done_evt(apt,NULL,true);
		}
	}

}



/* callback that called by the kernel when user decided to scan.
 * This callback should initiate scan routine(through work_struct) and exit with 0 if everything ok.
 * Scan routine should be finished with cfg80211_scan_done() call. */
static int xfz_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request) {
     struct xfz_cfg80211_adapter  * apt = wiphy_get_xfz_context(wiphy)->apt;


    if(down_interruptible(&apt->sem)) {
        return -ERESTARTSYS;
    }

    if (apt->scan_request != NULL) {
        up(&apt->sem);
        return -EBUSY;
    }
    apt->scan_request = request;

    up(&apt->sem);

    if (!schedule_work(&apt->ws_scan)) {
        return -EBUSY;
    }

    return 0; /* OK */
}



/* callback that called by the kernel when there is need to "connect" to some network.
 * It inits connection routine through work_struct and exits with 0 if everything ok.
 * connect routine should be finished with cfg80211_connect_bss()/cfg80211_connect_result()/cfg80211_connect_done() or cfg80211_connect_timeout(). */
static int xfz_connect(struct wiphy *wiphy, struct net_device *dev,
                struct cfg80211_connect_params *sme) {
    struct xfz_cfg80211_adapter * apt = wiphy_get_xfz_context(wiphy)->apt;
    size_t ssid_len = sme->ssid_len > MAX_SSID_SIZE? MAX_SSID_SIZE : sme->ssid_len;

		

    if(down_interruptible(&apt->sem)) {
        return -ERESTARTSYS;
    }

	

    memcpy(apt->connecting_ssid, sme->ssid, ssid_len);
    apt->connecting_ssid[ssid_len] = 0;
	apt->ssid_len=sme->ssid_len;
	if(sme->crypto.psk){
		memcpy(apt->psk,sme->crypto.psk,WLAN_PMK_LEN);

		}

    up(&apt->sem);

    if (!schedule_work(&apt->ws_connect)) {
        return -EBUSY;
    }
    return 0;
}
/* callback that called by the kernel when there is need to "diconnect" from currently connected network.
 * It inits disconnect routine through work_struct and exits with 0 if everything ok.
 * disconnect routine should call cfg80211_disconnected() to inform the kernel that disconnection is complete. */
static int xfz_disconnect(struct wiphy *wiphy, struct net_device *dev,
                   u16 reason_code) {
    struct xfz_cfg80211_adapter * apt = wiphy_get_xfz_context(wiphy)->apt;

    if(down_interruptible(&apt->sem)) {
        return -ERESTARTSYS;
    }

    apt->disconnect_reason_code = reason_code;

    up(&apt->sem);

    if (!schedule_work(&apt->ws_disconnect)) {
        return -EBUSY;
    }
    return 0;
}

#define SSID_DUMMY "Gohighsec-2.4"
#define SSID_DUMMY_SIZE (sizeof("Gohighsec-2.4") - 1)





/* It just checks SSID of the ESS to connect and informs the kernel that connect is finished.
 * It should call cfg80211_connect_bss() when connect is finished or cfg80211_connect_timeout() when connect is failed.
 * This "demo" can connect only to ESS with SSID equal to SSID_DUMMY value.
 * This routine called through workqueue, when the kernel asks about connect through cfg80211_ops. */
static void xfz_connect_routine(struct work_struct *w) {
    struct xfz_cfg80211_adapter * apt = container_of(w, struct xfz_cfg80211_adapter, ws_connect);

    if(down_interruptible(&apt->sem)) {
        return;
    }
	{


		{
				
					cmd_resp64_t cmd;
					cmd.cmd_op=OUT_BAND_CMD_CONNECT_AP;
					cmd.seq=7841;
					cmd.len=0;
					ssid_psk_msg_t * spsk=(ssid_psk_msg_t * )cmd.buf;
					memcpy(spsk->psk,apt->psk,WLAN_PMK_LEN);
					memcpy(spsk->ssid,apt->connecting_ssid,apt->ssid_len);
					cmd.len=WLAN_PMK_LEN+apt->ssid_len;
					 
					if( link_issue_cmd_resp(apt->bottom_obj,(uint8_t *)&cmd,sizeof(cmd),(uint8_t *)&cmd,sizeof(cmd))>=0)
					{
						
					}
					else{
						LOGE("%s can't issue connect cmd\n",__FUNCTION__);
					
						}
			}

		

    }
    

    up(&apt->sem);
}



;


int xfz_connect_done_evt(struct xfz_cfg80211_adapter * apt,uint8_t * resp){

	cmd_resp64_t * pcmd=(cmd_resp64_t *)resp;



	
    if(down_interruptible(&apt->sem)) {
		LOGE("in %s can't down sem\n",__FUNCTION__);
        return;
    }

	/* finish connect */
	   char bssid[]={0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
	   /* also its possible to use cfg80211_connect_result() or cfg80211_connect_done() */
	   	memcpy(bssid, pcmd->buf,6);
	   cfg80211_connect_bss(apt->ndev, bssid, NULL, NULL, 0, NULL, 0, WLAN_STATUS_SUCCESS, GFP_KERNEL,
							NL80211_TIMEOUT_UNSPECIFIED);
	   apt->connecting_ssid[0] = 0;
	  

	up(&apt->sem);

}


static void  ws_evt_routine(struct work_struct *w) {
	struct xfz_cfg80211_adapter * apt = container_of(w, struct xfz_cfg80211_adapter, ws_evt);
    uint8_t msg[LINK_MSG_MAX_SIZE];
	int id=-1;
	if(msg_queue_get_msg(&apt->msg_mgr,&id,msg,LINK_MSG_MAX_SIZE)>0){
		switch(id){
			case OUT_BAND_CMD_AP_LIST:
				xfz_scan_done_evt(apt,msg,false);
				break;
			case OUT_BAND_CMD_CONNECTED_DONE:
				xfz_connect_done_evt(apt,msg);
				break;
			default:
			;

			}
	}
}

static int
xfz_cfg80211_dump_station(struct wiphy *wiphy, struct net_device *ndev,
			    int idx, u8 *mac, struct station_info *sinfo)
{

	return 0;
}




static s32
xfz_cfg80211_add_key(struct wiphy *wiphy, struct net_device *ndev,
		       u8 key_idx, bool pairwise, const u8 *mac_addr,
		       struct key_params *params)
{

	return 0;
}


static s32
xfz_cfg80211_del_key(struct wiphy *wiphy, struct net_device *ndev,
		       u8 key_idx, bool pairwise, const u8 *mac_addr)
{

	return 0;
}


static s32
xfz_cfg80211_config_default_key(struct wiphy *wiphy, struct net_device *ndev,
				  u8 key_idx, bool unicast, bool multicast)
{

	return 0;
}

static s32
xfz_cfg80211_get_key(struct wiphy *wiphy, struct net_device *ndev, u8 key_idx,
		       bool pairwise, const u8 *mac_addr, void *cookie,
		       void (*callback)(void *cookie,
					struct key_params *params))
{
	
	
		return 0;

}

int g_auth_stat=0;

static int
xfz_cfg80211_change_station(struct wiphy *wiphy, struct net_device *ndev,
			      const u8 *mac, struct station_parameters *params)
{

	if (params->sta_flags_set & BIT(NL80211_STA_FLAG_AUTHORIZED))
	{
		g_auth_stat=1;
		return 0;
	}
	g_auth_stat=0;
	return  0;

}


static s32
xfz_cfg80211_get_station(struct wiphy *wiphy, struct net_device *ndev,
			   const u8 *mac, struct station_info *sinfo)
{
	int rssi=-6000;

if(g_auth_stat) {
				rssi = le32_to_cpu(-40);
				sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL);
				sinfo->signal = rssi;
	
	}

	return 0;
}



/* Array of "supported" channels in 2ghz band. It's required for wiphy.
 * For demo - the only channel 6. */
static struct ieee80211_channel xfz_supported_channels_2ghz[] = {
        {
                .band = NL80211_BAND_2GHZ,
                .hw_value = 6,
                .center_freq = 2437,
        }
};


/* Array of supported rates. Its required to support at least those next rates for 2ghz band. */
static struct ieee80211_rate xfz_supported_rates_2ghz[] = {
        {
                .bitrate = 10,
                .hw_value = 0x1,
        },
        {
                .bitrate = 20,
                .hw_value = 0x2,
        },
        {
                .bitrate = 55,
                .hw_value = 0x4,
        },
        {
                .bitrate = 110,
                .hw_value = 0x8,
        }
};


/* Structure that describes supported band of 2ghz. */
static struct ieee80211_supported_band xfz_band_2ghz = {
        .ht_cap.cap = IEEE80211_HT_CAP_SGI_20, /* add other band capabilities if needed, like 40 width etc. */
        .ht_cap.ht_supported = false,

        .channels = xfz_supported_channels_2ghz,
        .n_channels = ARRAY_SIZE(xfz_supported_channels_2ghz),

        .bitrates = xfz_supported_rates_2ghz,
        .n_bitrates = ARRAY_SIZE(xfz_supported_rates_2ghz),
};



/* Structure of functions for FullMAC 80211 drivers.
 * Functions that implemented along with fields/flags in wiphy structure would represent drivers features.
 * This DEMO can only perform "scan" and "connect".
 * Some functions cant be implemented alone, for example: with "connect" there is should be function "disconnect". */
static struct cfg80211_ops xfz_cfg80211_ops = {
.scan = xfz_scan,
.connect = xfz_connect,
.disconnect = xfz_disconnect,
.add_key = xfz_cfg80211_add_key,
.del_key = xfz_cfg80211_del_key,
.change_station = xfz_cfg80211_change_station,
.get_station = xfz_cfg80211_get_station,
.dump_station = xfz_cfg80211_dump_station,
//.get_key = xfz_cfg80211_get_key,
.set_default_key = xfz_cfg80211_config_default_key,
};






int xfz_cfg80211_wiphy_init(struct xfz_cfg80211_adapter * apt)
{

	sema_init(&apt->sem, 1);
    INIT_WORK(&apt->ws_connect, xfz_connect_routine);
    apt->connecting_ssid[0] = 0;
    INIT_WORK(&apt->ws_disconnect, xfz_disconnect_routine);
    apt->disconnect_reason_code = 0;
    INIT_WORK(&apt->ws_scan, xfz_scan_routine);
    apt->scan_request = NULL;
	INIT_WORK(&apt->ws_evt, ws_evt_routine);	
	msg_queue_init(&apt->msg_mgr);
	apt->wiphy = wiphy_new_nm(&xfz_cfg80211_ops, sizeof(struct xfz_wiphy_priv_context), WIPHY_NAME);
    if (apt->wiphy == NULL) {
		LOGE("%s wiphy_new_nm NG\n", __FUNCTION__);
        goto error_wiphy;
    }

	 //save apt in wiphy private data.
     wiphy_get_xfz_context(apt->wiphy)->apt=apt;

	// set device object as wiphy "parent" 
	  /* set_wiphy_dev(ret->wiphy, dev); */
	
	  /* wiphy should determinate it type */
	  /* add other required types like	"BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_AP)" etc. */
	  apt->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	
	  /* wiphy should have at least 1 band. */
	  /* fill also NL80211_BAND_5GHZ if required, in this small example I use only 1 band with 1 "channel" */
	  apt->wiphy->bands[NL80211_BAND_2GHZ] = &xfz_band_2ghz;
	
	  /* scan - if ur device supports "scan" u need to define max_scan_ssids at least. */
	  apt->wiphy->max_scan_ssids = 18;
	  // set cipher suite or we will got INVALID error when nl call
	  	{

		static const u32 cipher_suites[] = {
				/* keep WEP first, it may be removed below */
				WLAN_CIPHER_SUITE_WEP40,
				WLAN_CIPHER_SUITE_WEP104,
				WLAN_CIPHER_SUITE_TKIP,
				WLAN_CIPHER_SUITE_CCMP,
				WLAN_CIPHER_SUITE_CCMP_256,
				WLAN_CIPHER_SUITE_GCMP,
				WLAN_CIPHER_SUITE_GCMP_256,
		
				/* keep last -- depends on hw flags! */
				WLAN_CIPHER_SUITE_AES_CMAC,
				WLAN_CIPHER_SUITE_BIP_CMAC_256,
				WLAN_CIPHER_SUITE_BIP_GMAC_128,
				WLAN_CIPHER_SUITE_BIP_GMAC_256,
			};
		apt->wiphy->cipher_suites= cipher_suites;
		apt->wiphy->n_cipher_suites = ARRAY_SIZE(cipher_suites);


	  	}
		//set RSSI mode or will no display
	  apt->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	  // register wiphy, if everything ok - there should be another wireless device in system.
	  //use command:	    $ iw list
	   
	  if (wiphy_register(apt->wiphy) < 0) {
	  		LOGE("%s wiphy_register NG\n", __FUNCTION__);
		  goto error_wiphy_register;
	  }
	  return 0;
	error_wiphy_register:
	wiphy_free(apt->wiphy);
	error_wiphy:
	return -1;

}

