#include "hal/ota.h"
#include "hal/soc/flash.h"

#include <ti/net/ota/ota.h>
#include <ti/net/ota/source/OtaArchive.h>

//#define DEBUG_OTA csp_printf

#define DEBUG_OTA

OtaArchive_t ota_archive;

int cc3220_ota_init(hal_ota_module_t *m, void *something)
{
    DEBUG_OTA("cc3220_ota_init\r\n");
    OtaArchive_init(&ota_archive);
    ota_archive.TotalBytesReceived = 0;
    ota_archive.State = ARCHIVE_STATE_PARSE_HDR;
    ota_archive.BundleCmdTable.NumFiles = 0;
    ota_archive.CurrTarObj.lFileHandle = -1;

    return 0;
}
int cc3220_ota_write(hal_ota_module_t *m, volatile uint32_t *off_set,
				 uint8_t *in_buf , uint32_t in_buf_len)
{
    static uint32_t total_len = 0;
    uint16_t processed_bytes;
    int16_t status;

    total_len += in_buf_len;
    DEBUG_OTA("cc3220_ota_write len:%d, total:%d\r\n", in_buf_len, total_len);
    
    return 0; 
}

int cc3220_ota_read(hal_ota_module_t *m,  volatile uint32_t *off_set,
				uint8_t *out_buf , uint32_t out_buf_len)
{
    DEBUG_OTA("cc3220_ota_read\r\n");
    return 0;
}

int cc3220_ota_set_boot(hal_ota_module_t *m, void *something)
{
    DEBUG_OTA("cc3220_ota_set_boot\r\n");
    stop_nwp(200);
    return 0;
}


struct hal_ota_module_s hal_cc3220_ota_module = {
    .init = cc3220_ota_init,
    .ota_write = cc3220_ota_write,
    .ota_read = cc3220_ota_read,
    .ota_set_boot = cc3220_ota_set_boot,
};


