/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <aos/network.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include "hal/wifi.h"
#include "common.h"
#include <k_api.h>
#include <aos/aos.h>
#include <aos/kernel.h>

#include "driverlib/rom_map.h"

#define FRAME_LENGTH                (1000)
#define BUFF_SIZE 1472
#define TI_PROPRIETARY_LENGTH 8

extern void hal_reboot(void);
extern void gpio_led_on(void);
extern void gpio_led_off(void);
extern void gpio_led_toggle(void);

static monitor_data_cb_t data_cb = NULL;
static monitor_data_cb_t mngt_data_cb = NULL;
static int raw_sock_fd = -1;
static int channel_num = 1;
static int stop_monitor_flag = 0;
unsigned char *recv_buffer;

// Status bits - These are used to set/reset the corresponding bits in 
// given variable
typedef enum{
    STATUS_BIT_NWP_INIT = 0, // If this bit is set: Network Processor is 
                             // powered up
                             
    STATUS_BIT_CONNECTION,   // If this bit is set: the device is connected to 
                             // the AP or client is connected to device (AP)
                             
    STATUS_BIT_IP_LEASED,    // If this bit is set: the device has leased IP to 
                             // any connected client

    STATUS_BIT_IP_ACQUIRED,   // If this bit is set: the device has acquired an IP
    
    STATUS_BIT_SMARTCONFIG_START, // If this bit is set: the SmartConfiguration 
                                  // process is started from SmartConfig app

    STATUS_BIT_P2P_DEV_FOUND,    // If this bit is set: the device (P2P mode) 
                                 // found any p2p-device in scan

    STATUS_BIT_P2P_REQ_RECEIVED, // If this bit is set: the device (P2P mode) 
                                 // found any p2p-negotiation request

    STATUS_BIT_CONNECTION_FAILED, // If this bit is set: the device(P2P mode)
                                  // connection to client(or reverse way) is failed

    STATUS_BIT_PING_DONE,         // If this bit is set: the device has completed
                                 // the ping operation

	STATUS_BIT_IPV6L_ACQUIRED,   // If this bit is set: the device has acquired an IPv6 address
	STATUS_BIT_IPV6G_ACQUIRED,   // If this bit is set: the device has acquired an IPv6 address
	STATUS_BIT_AUTHENTICATION_FAILED,
	STATUS_BIT_RESET_REQUIRED,
}e_StatusBits;

#define CLR_STATUS_BIT_ALL(status_variable)  (status_variable = 0)
#define SET_STATUS_BIT(status_variable, bit) status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit) status_variable &= ~(1<<(bit))
#define CLR_STATUS_BIT_ALL(status_variable)  (status_variable = 0)
#define GET_STATUS_BIT(status_variable, bit) (0 != (status_variable & (1<<(bit))))

#define IS_CONNECTED(status_variable)        GET_STATUS_BIT(status_variable,\
                                                        STATUS_BIT_CONNECTION)
#define IS_IP_ACQUIRED(status_variable)       GET_STATUS_BIT(status_variable,\
                                                         STATUS_BIT_IP_ACQUIRED)

typedef enum {
    HAL_WIFI_MODE_STATION,
    HAL_WIFI_MODE_SOFTAP
} hal_wifi_mode_t;

/* Control block definition */
typedef struct _cc3220_ControlBlock_t_
{
    uint32_t        slStatus;    //SimpleLink Status
    signed char     frameData[FRAME_LENGTH];
    SlSockAddrIn_t  ipV4Addr;
}cc3220_ControlBlock;

cc3220_ControlBlock cc3220_CB;

static hal_wifi_ip_stat_t _ip_stat;

//*****************************************************************************
// SimpleLink Callback Functions
//*****************************************************************************

void SimpleLinkNetAppRequestMemFreeEventHandler (uint8_t *buffer)
{
  // do nothing...
}

void SimpleLinkNetAppRequestEventHandler (SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse)
{
  // do nothing...
}

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    hal_wifi_module_t *m = hal_wifi_get_default_module();
    hal_wifi_ip_stat_t stat;

    switch(pWlanEvent->Id)
    {
        case SL_WLAN_EVENT_CONNECT:
        {

            SET_STATUS_BIT(cc3220_CB.slStatus, STATUS_BIT_CONNECTION);
        }
        break;

        case SL_WLAN_EVENT_DISCONNECT:
        {
            CLR_STATUS_BIT(cc3220_CB.slStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(cc3220_CB.slStatus, STATUS_BIT_IP_ACQUIRED);
        }
        break;

        default:
        {
            LOGD("wifi_port", "[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Id);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    switch (slFatalErrorEvent->Id)
    {
        case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
        {
            LOGD("wifi_port", "[ERROR] - FATAL ERROR: Abort NWP event detected: AbortType=%d, AbortData=0x%x\n\r",slFatalErrorEvent->Data.DeviceAssert.Code,slFatalErrorEvent->Data.DeviceAssert.Value);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
        {
            LOGD("wifi_port", "[ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
        {
            LOGD("wifi_port", "[ERROR] - FATAL ERROR: No Cmd Ack detected [cmd opcode = 0x%x] \n\r", slFatalErrorEvent->Data.NoCmdAck.Code);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
        {
            LOGD("wifi_port", "[ERROR] - FATAL ERROR: Sync loss detected n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
        {
            LOGD("wifi_port", "[ERROR] - FATAL ERROR: Async event timeout detected [event opcode =0x%x]  \n\r", slFatalErrorEvent->Data.CmdTimeout.Code);
        }
        break;

        default:
            LOGD("wifi_port", "[ERROR] - FATAL ERROR: Unspecified error detected \n\r");
        break;
    }

    hal_reboot();
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info 
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    hal_wifi_module_t *m = hal_wifi_get_default_module();
    hal_wifi_ip_stat_t stat;

    switch(pNetAppEvent->Id)
    {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        {
            SET_STATUS_BIT(cc3220_CB.slStatus, STATUS_BIT_IP_ACQUIRED);
            LOGD("wifi_port", "[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
            "Gateway=%d.%d.%d.%d\n\r", 
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,3),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,2),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,1),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,0),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,0));

	    sprintf(_ip_stat.ip, "%d.%d.%d.%d", SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,3),
		            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,2),
            		    SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,1),
            		    SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,0));

	    sprintf(_ip_stat.mask, "255.255.255.0");
	    sprintf(_ip_stat.gate, "%d.%d.%d.%d", SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,3),
            		    SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,2),
            		    SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,1),
            		    SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,0));
            if (m->ev_cb && m->ev_cb->ip_got) {
		LOG("Send ip got event");
                m->ev_cb->ip_got(m, &_ip_stat, NULL);
            }
	    gpio_led_on();
        }
        break;

        default:
        {
            LOGD("wifi_port", "[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Id);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pHttpEvent,
                                    SlNetAppHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info 
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    LOGD("wifi_port", "[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->Data.Error.Code,
               pDevEvent->Data.Error.Source);
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->SocketAsyncEvent.SockTxFailData.Status)
            {
                case SL_ERROR_BSD_ECLOSE:
                    LOGD("wifi_port", "[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\r",
                                    pSock->SocketAsyncEvent.SockTxFailData.Sd);
                    break;
                default: 
                    LOGD("wifi_port", "[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->SocketAsyncEvent.SockTxFailData.Sd, pSock->SocketAsyncEvent.SockTxFailData.Status);
                  break;
            }
            break;

        default:
            LOGD("wifi_port", "[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }

}

void SimpleLinkSocketTriggerEventHandler(SlSockTriggerEvent_t *pSlTriggerEvent)
{
	// Unused in this application
}
void convertIPStringtoInt(char *ipV4string, uint32_t *ipV4value){
    int a[4];
    char buffer[32];
    int i;
    char *token;

    strncpy(buffer, ipV4string, sizeof(buffer));
    token = strtok(buffer,".");
    for(i = 0;i<3;i++){
        
        a[i] = strtoul(token,0,10);
        token = strtok(NULL, ".");
    }
    *ipV4value = SL_IPV4_VAL(a[3],a[2],a[1],a[0]);
}

static int wifi_set_mode(hal_wifi_module_t *m, hal_wifi_mode_t mode)
{
    int ret = -1;
    return ret;
}

static int32_t StartConnection(char *ssid, char *password)
{
    SlWlanSecParams_t secParams = {0};
    int32_t retVal = 0;

    //Enable NWP incase it's off.
    start_nwp();

    //Stop wlan connection before start it.
    sl_WlanDisconnect();

    secParams.Key = (signed char*)password;
    secParams.KeyLen = strlen(password);
    secParams.Type = SL_WLAN_SEC_TYPE_WPA_WPA2;

    LOGD("wifi_port", "WlanConnect SSID=%s\n\r", ssid);
    retVal = sl_WlanConnect((signed char*)ssid,\
                             strlen(ssid), 0, &secParams, 0);

    return retVal;
}

static int wifi_connect_prepare(hal_wifi_module_t *m, char *ssid, char *password)
{
    if(stop_monitor_flag == 1) {
	LOG("Reset WiFi to connect router\r\n");
	hal_reboot();
    }
    
    StartConnection(ssid, password);
    
    return 0;
}

typedef enum {
    HAL_WIFI_SMART_CONFIG_START,
    HAL_WIFI_SMART_CONFIG_STOP,
    HAL_WIFI_CHECK_SUPPORT_SMART_CONFIG,
    HAL_WIFI_GET_CHANNEL,
    HAL_WIFI_SET_CHANNEL,
    HAL_WIFI_PROMISCUOUS_START,
    HAL_WIFI_PROMISCUOUS_STOP,
    HAL_WIFI_SET_AUTO_CONNECT,
    HAL_WIFI_GET_MAC_ADDRESS,
    HAL_WIFI_GET_AP_RSSI
} hal_wifi_getset_cmd_t;

static void app_sock_pkt_handle(void *arg)
{
    int cur_channel;
    SlSockNonblocking_t enableOption;
    int recv_ret;
    unsigned char  *fr_ctrl;
    hal_wifi_link_info_t info;
    static int last_channel;

    raw_sock_fd = sl_Socket(SL_AF_RF,SL_SOCK_RAW,channel_num);

    last_channel = channel_num;

    enableOption.NonBlockingEnabled = 1;
    sl_SetSockOpt(raw_sock_fd, SL_SOL_SOCKET, SL_SO_NONBLOCKING, (_u8 *)&enableOption, sizeof(enableOption));

    recv_buffer = aos_malloc(BUFF_SIZE);
    while(1) {
#if 1
	if(stop_monitor_flag == 1){
	    if(raw_sock_fd >= 0) {
	        aos_free(recv_buffer);
                sl_Close (raw_sock_fd);
	        raw_sock_fd = -1;
	    }
	    aos_msleep(2000);
            continue;
        }
#endif
#if 1
	if(raw_sock_fd >= 0 && channel_num != last_channel){
            sl_SetSockOpt(raw_sock_fd, SL_SOL_SOCKET, SL_SO_CHANGE_CHANNEL, &channel_num, sizeof(channel_num));
	    last_channel = channel_num;
	}
#endif
	recv_ret = sl_Recv (raw_sock_fd , recv_buffer , BUFF_SIZE , 0) ;
	aos_msleep(30);
	if(recv_ret <=0) {
	    continue;
	}
	
	gpio_led_toggle();	

	info.rssi = 0;
	if(stop_monitor_flag == 1){
	    continue;
        } else {
	    data_cb(recv_buffer + TI_PROPRIETARY_LENGTH, recv_ret - TI_PROPRIETARY_LENGTH, &info);
	}
    }
}

static int wifi_getset_ops(hal_wifi_module_t *m, hal_wifi_getset_cmd_t cmd, ...)
{
    LOG("%s cmd: %d\r\n", __func__, cmd);
    switch (cmd) {
    case HAL_WIFI_GET_CHANNEL: {
        return channel_num;
    }
    case HAL_WIFI_PROMISCUOUS_START: {
	stop_monitor_flag = 0;
	aos_task_new("SLRAWSOCK", app_sock_pkt_handle, 0, 2048);
        break;
    }
    case HAL_WIFI_PROMISCUOUS_STOP: {
	stop_monitor_flag = 1;
	gpio_led_off();
        break;
    }
    case HAL_WIFI_SET_AUTO_CONNECT: {
        int c;
        break;
    }
    case HAL_WIFI_GET_AP_RSSI: {
        return -1;
    }
    default:{
        return -1;
    }
    }

    return 0;
}

static int wifi_init(hal_wifi_module_t *m)
{
    int ret = 0;
    return 0;
};

static void wifi_get_mac_addr(hal_wifi_module_t *m, uint8_t *mac)
{
    _u16 macAddressLen = SL_MAC_ADDR_LEN;
    _u16 ConfigOpt = 0;

    sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET,&ConfigOpt,&macAddressLen,(_u8 *)mac);
};

static int wifi_start(hal_wifi_module_t *m, hal_wifi_init_type_t *init_para)
{
    wifi_connect_prepare(m, init_para->wifi_ssid, init_para->wifi_key);
    return 0;
}

static int wifi_start_adv(hal_wifi_module_t *m,
                          hal_wifi_init_type_adv_t *init_para_adv)
{
    SL_IPV4_VAL(1,2,3,4);
    LOG("WiFi HAL %s not implemeted yet!\r\n", __func__);
    return 0;
}

static int get_ip_stat(hal_wifi_module_t *m,
                       hal_wifi_ip_stat_t *out_net_para,
                       hal_wifi_type_t wifi_type)
{

    if(IS_IP_ACQUIRED(cc3220_CB.slStatus))
	return 1;
    else
    	return 0;
}

static int get_link_stat(hal_wifi_module_t *m,
                         hal_wifi_link_stat_t *out_stat)
{
    if(IS_CONNECTED(cc3220_CB.slStatus))
        return 1;
    else
        return 0;
}

static void wifi_scan(hal_wifi_module_t *m)
{
    //wifi_scan_helper(m, SCAN_NORMAL);
}

static void start_scan_adv(hal_wifi_module_t *m)
{
    //wifi_scan_helper(m, SCAN_ADV);
}

static int power_off(hal_wifi_module_t *m)
{
    LOG("WiFi HAL %s not implemeted yet!\r\n", __func__);
    MAP_PRCMHibernateCycleTrigger();
    return 0;
}

static int power_on(hal_wifi_module_t *m)
{
    LOG("WiFi HAL %s not implemeted yet!\r\n", __func__);
    return 0;
}

static int suspend(hal_wifi_module_t *m)
{
    return 0;
}

static int suspend_station(hal_wifi_module_t *m)
{
    return 0;
}

static int suspend_soft_ap(hal_wifi_module_t *m)
{
    return 0;
}

static int set_channel(hal_wifi_module_t *m, int ch)
{
    channel_num = ch;

    return 0;
}

static void start_monitor(hal_wifi_module_t *m)
{
    wifi_getset_ops(m, HAL_WIFI_PROMISCUOUS_START);
}

static void stop_monitor(hal_wifi_module_t *m)
{
    wifi_getset_ops(m, HAL_WIFI_PROMISCUOUS_STOP);
}

static void register_monitor_cb(hal_wifi_module_t *m, monitor_data_cb_t fn)
{
    data_cb = fn;
}

static void register_wlan_mgnt_monitor_cb(hal_wifi_module_t *m,
                                          monitor_data_cb_t fn)
{
    //mngt_data_cb = fn;
    // Workaround for zero config <TODO>
    //hal_wifi_register_monitor_cb(NULL, NULL);
    //hal_wifi_start_wifi_monitor(NULL);
}

static int wlan_send_80211_raw_frame(hal_wifi_module_t *m,
                                     uint8_t *buf, int len)
{
    return 0;
}

hal_wifi_module_t sim_aos_wifi_cc3220 = {
    .base.name           = "sim_aos_wifi_cc3220",
    .init                =  wifi_init,
    .get_mac_addr        =  wifi_get_mac_addr,
    .start               =  wifi_start,
    .start_adv           =  wifi_start_adv,
    .get_ip_stat         =  get_ip_stat,
    .get_link_stat       =  get_link_stat,
    .start_scan          =  wifi_scan,
    .start_scan_adv      =  start_scan_adv,
    .power_off           =  power_off,
    .power_on            =  power_on,
    .suspend             =  suspend,
    .suspend_station     =  suspend_station,
    .suspend_soft_ap     =  suspend_soft_ap,
    .set_channel         =  set_channel,
    .start_monitor       =  start_monitor,
    .stop_monitor        =  stop_monitor,
    .register_monitor_cb =  register_monitor_cb,
    .register_wlan_mgnt_monitor_cb = register_wlan_mgnt_monitor_cb,
    .wlan_send_80211_raw_frame = wlan_send_80211_raw_frame,
};
