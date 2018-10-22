#include <ti/devices/cc32xx/inc/hw_ints.h>
#include <ti/devices/cc32xx/inc/hw_types.h>
#include <ti/devices/cc32xx/inc/hw_memmap.h>

#include <ti/devices/cc32xx/driverlib/rom_map.h>
#include <ti/devices/cc32xx/driverlib/interrupt.h>
#include <ti/devices/cc32xx/driverlib/prcm.h>
#include <ti/devices/cc32xx/driverlib/timer.h>
#include <ti/devices/cc32xx/driverlib/utils.h>
#include <ti/devices/cc32xx/driverlib/pin.h>
#include <ti/devices/cc32xx/driverlib/uart.h>
#include <ti/devices/cc32xx/driverlib/udma.h>

#include <aos/aos.h>
#include <ti/drivers/net/wifi/simplelink.h>

extern int start_nwp(void);
extern int stop_nwp(int timeout);

extern void switchToStaMode(void);

#define SIMPLELINK_NWP_STARTED 0xFF
#define SIMPLELINK_NWP_STOPPED 0xEE

static int sl_nwp_started = SIMPLELINK_NWP_STOPPED;

typedef struct sBootInfo
{
  uint8_t ucActiveImg;
  uint32_t ulImgStatus;
  uint32_t ulStartWdtKey;
  uint32_t ulStartWdtTime;
}sBootInfo_t;

#define APPS_WDT_START_KEY          0xAE42DB15
#define HWREG(x) (*((volatile unsigned long *)(x)))
#define PRCM_WDT                  0x0000000B

int32_t Platform_CommitWdtConfig(int32_t TimeoutInSeconds);
void Platform_CommitWdtStop();

int32_t OtaCheckAndDoCommit()
{
    int32_t isPendingCommit;
    int32_t isPendingCommit_len;
    int32_t Status;

    csp_printf("OTA Checking...\r\n");
    isPendingCommit = OtaArchive_getPendingCommit();

    /* commit now because 1. the state is PENDING_COMMIT 2. there was successful wlan connection */
    if (isPendingCommit)
    {
	OtaArchive_commit();
        csp_printf("\r\n");
        csp_printf("OtaCheckDoCommit: OTA success, new image commited and currently run\n");
        csp_printf("\r\n");
    }

    Platform_CommitWdtStop();

    return 0;
}

int start_nwp(void)
{
    int ret = -1;
    
    if(sl_nwp_started == SIMPLELINK_NWP_STARTED) {
        return sl_nwp_started;
    }
   
    csp_printf("Turn on NWP -- 0410\r\n"); 
    ret = sl_Start(NULL, NULL, NULL);
    
    if(ret >= 0) {
	csp_printf("NWP has been turned on\r\n");
        sl_nwp_started = SIMPLELINK_NWP_STARTED;
	if(ret != ROLE_STA) {
	    csp_printf("Switch to STA mode...\r\n");
	    aos_msleep(1000);
	    switchToStaMode();
	}
    }

    Platform_CommitWdtConfig(50);
    OtaCheckAndDoCommit();
 
    return ret;
}

int stop_nwp(int timeout)
{
    int ret;

    ret = sl_Stop(timeout);

    sl_nwp_started = SIMPLELINK_NWP_STOPPED;

    return ret;
}

//*****************************************************************************
//
//! Check the device mode and switch to STATION(STA) mode
//! restart the NWP to activate STATION mode
//!
//! \param  iMode (device mode)
//!
//! \return None
//
//*****************************************************************************
void switchToStaMode(void)
{
    int32_t status = -1;

    status = sl_WlanSetMode(ROLE_STA);
    if (status < 0)
    {
	csp_printf("Switching to STA mode failed.\r\n");
	return;
    }
    sl_Stop(200);
    //
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    status = sl_Start(0, 0, 0);
    if (status < 0 || ROLE_STA != status)
    {
        csp_printf("Failed to start the device \n\r");
    }
}

//*****************************************************************************
//
//! \brief
//
//*****************************************************************************
void Platform_CommitWdtStop()
{
    PRCMPeripheralReset(PRCM_WDT);
}

//*****************************************************************************
//
//! \brief
//
//*****************************************************************************
int32_t Platform_CommitWdtConfig(int32_t TimeoutInSeconds)
{
    int32_t lFileHandle;
    uint32_t ulToken=0;
    sBootInfo_t sBootInfo;
    int32_t lRetVal;

    lFileHandle = sl_FsOpen((unsigned char *)"/sys/mcubootinfo.bin",
                                SL_FS_CREATE | SL_FS_OVERWRITE | SL_FS_CREATE_MAX_SIZE( sizeof(sBootInfo)) |
                                SL_FS_CREATE_SECURE | SL_FS_CREATE_PUBLIC_WRITE | SL_FS_CREATE_NOSIGNATURE,
                                (_u32 *)&ulToken);

    if(0 > lFileHandle )
    {
        //OTA_DBG_PRINT("OtaWatchDog: Error opening bootinfo file : %d\n\r",lFileHandle);
        return -1;
    }

    memset(&sBootInfo,0,sizeof(sBootInfo_t));
    sBootInfo.ulStartWdtTime = 40000000*TimeoutInSeconds; /* max 104 seconds */
    sBootInfo.ulStartWdtKey = APPS_WDT_START_KEY;
    lRetVal = sl_FsWrite(lFileHandle, 0, (uint8_t*)&sBootInfo, sizeof(sBootInfo_t));
    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if(0 != lRetVal)
    {
        //OTA_DBG_PRINT("OtaWatchDog: Failed to close the bootinfo file");
        return -1;
    }
    //Enable WDT - done by PRCMCC3200MCUInit
    //HWREG(0x4402E188) |= 0x00000020;
    return 0;
}
