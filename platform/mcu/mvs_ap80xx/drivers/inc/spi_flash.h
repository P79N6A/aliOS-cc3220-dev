/**
 *****************************************************************************
 * @file     spi_flash.h
 * @author   lujiangang
 * @version  V1.0.3
 * @date     30-May-2013
 * @brief    spi code flash module driver interface
 *****************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 MVSilicon </center></h2>
 */

/**
* @addtogroup ����
* @{
* @defgroup SPIFlash SPIFlash
* @{
*/

#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#define     POSEDGE_SEL          (0)       /**<sampling at clk posedge */
#define     NEGEDGE_SEL          (1)       /**<sampling at clk negedge */
#define     DELAY_CHAIN_SEL      (4)       /**<sampling at the n clks */

#define     FLASH_GD             (0xC8)    /**<MID */
#define     FLASH_MX             (0xC2)    /**<MID */
#define     FLASH_WINBOUND       (0xEF)    /**<MID */
#define     FLASH_PCT            (0xBF)    /**<MID */
#define     FLASH_EON            (0x1C)    /**<MID */
#define     FLASH_BG             (0xE0)    /**<MID */
#define     FLASH_ESMT           (0x8C)    /**<MID */

#define     FLASH_HALF_PROTECT              (1)     /**<��Flash 0��ַ����1�����*/
#define     FLASH_THREE_QUARTER_PROTECT     (2)     /**<��Flash 0��ַ����3/4����*/
#define     FLASH_SEVEN_EIGHTH_PROTECT      (3)     /**<��Flash 0��ַ����7/8����*/
#define     FLASH_ALL_PROTECT               (4)     /**<ȫ����*/


/**
 * @brief Flash�����������Ͷ���
 */
typedef enum
{
    SPI_FLASH_CONSTDAT_BTAD = 0x00,
    SPI_FLASH_CONSTDAT_BTNM,
    SPI_FLASH_CONSTDAT_BTCN,
    SPI_FLASH_CONSTDAT_BTDC,
    SPI_FLASH_CONSTDAT_BTCM,

    SPI_FLASH_CONSTDAT_POMU = 0x10,
    SPI_FLASH_CONSTDAT_PFMU,

    SPI_FLASH_CONSTDAT_STSD = 0x20,
    SPI_FLASH_CONSTDAT_STPU,
    SPI_FLASH_CONSTDAT_STBT,
    SPI_FLASH_CONSTDAT_STLN,
    SPI_FLASH_CONSTDAT_STFM,
    SPI_FLASH_CONSTDAT_STUD,
} SPI_FLASH_CONSTDTA_TYPE;


/**
 * @brief �û������������Ͷ���
 */
typedef enum
{
    SPI_FLASH_USRDTA_BTKEY = 0x01,

} SPI_FLASH_USRDTA_TYPE;



/**
 * @brief Flash���������붨��
 */
typedef enum _SPI_FLASH_ERR_CODE
{
    TIME_OUT_ERR = -255,             /**<function execute time out*/
    UNKNOWN_MID_ERR,                 /**<MID can't indentify*/
    UNKNOWN_DID_ERR,                 /**<identify MID but can't identify DID*/
    FIFO_ALWAYS_EMPTY_ERR,           /**<during reading, time out but fifo is still empty*/
    FIFO_ALWAYS_FULL_ERR,            /**<during writing, time out but fifo is still full*/
    DATALEN_LESS_THEN_ZERO_ERR,      /**<read data but data len is < 0*/
    CODE_AREA_PROTECT_NOT_SUPPORT,   /**<indicate this area which choosed before not support*/
    UNLOCK_FLASH_ERR,                /**<unlock flash err*/
    FLASH_NONE_ERR = 0,
} SPI_FLASH_ERR_CODE;

/**
* @brief SPI Flash IOCTL command
*/
typedef enum _SPI_FLASH_IOCTL_CMD
{
	IOCTL_FLASH_PROTECT = 1,
	IOCTL_FLASH_UNPROTECT,
	IOCTL_STATUS_REGISTER,
	IOCTL_SET_CLK_HPM,
	IOCTL_DEEP_POWER_DOWN,
	IOCTL_RELEASE_DEEP_POWER_DOWN
}SPI_FLASH_IOCTL_CMD;

/**
* @brief SPI Flash ������Χ����
*/
typedef enum _SPI_FLASH_LOCK_RANGE
{
	FLASH_LOCK_RANGE_HALF = 1,
	FLASH_LOCK_RANGE_THREE_QUARTERS,
	FLASH_LOCK_RANGE_SEVENTH_EIGHT,
	FLASH_LOCK_RANGE_ALL
}SPI_FLASH_LOCK_RANGE;

#pragma pack(1)

/**
 * @brief Flash ��Ϣ�ṹ�嶨��
 * each byte definition like:    A B CMP SEC/BP4 TB/BP3 BP2 BP1 BP0 <A:area protection support flag,B:CMP bit exist flag>
 */
typedef struct _SPI_FLASH_INFO
{
	__IO uint8_t      Mid;             /**<����ID*/
	__IO uint16_t      Did;             /**<�豸ID*/
	__IO uint8_t      SqiModeFlag;     /**<bit0:0:spiģʽ, 1:sqiģʽ. r/w*/
	__IO uint8_t      SqiWriteFlag;    /**<bit1:1 ֧��4bit���, 0 ��֧��. �ó�Աֻ��*/
	__IO uint32_t     Capacity;        /**<flash����*/
	__IO uint8_t      NoneAreaProtect;
	__IO uint8_t      HalfAreaProtect;
	__IO uint8_t      ThreeQuarterAreaProtect;
	__IO uint8_t      SevenEighthsAreaProtect;
	__IO uint8_t      AllAreaProtect;

} SPI_FLASH_INFO;

#pragma pack()

//-------------------------------------------------------------------------------------------------------------------
// For code flash read and write ONLY usage
// Flash API���������ǡ�ԭ�ӡ�����

/**
* @brief SPI Flash ʱ��Ƶ������
*/
#define FLASHCLK_60MHZ_SEL (0x0)		/**<Flashʱ��ʹ��DPLL 60MHz*/
#define FLASHCLK_SYSCLK_SEL (0x1)		/**<Flashʱ��ʹ��ϵͳʱ��*/
#define FLASHCLK_80MHZ_SEL (0x2)		/**<Flashʱ��ʹ��DPLL 80MHz*/
#define FLASHCLK_48MHZ_SEL (0x3)		/**<Flashʱ��ʹ��DPLL 48MHz*/


extern int32_t SpiFlashConstGetInfo(uint32_t IdxId, uint32_t* DatSz, uint32_t* Offset);
extern int32_t SpiFlashConstDataRead(uint8_t* Buf, uint32_t BufLen, uint32_t Offset);

/**
 * @brief  ����Flash��ʱ��Ƶ��
 * @param  FshcClkSel	�����õ�Flashʱ��Ƶ��Ϊ60MHz��ϵͳʱ�ӣ�80MHz, 48MHz���������궨��
 * @param  EnHpm	 	1:����Hpm; 0:�ر�Hpm�������GD FLASH ���Կ���Hpm�������ͺŽ���ر�Hpm��		 			
 * @return 0���ɹ�, ��0��ʧ��
 * @Note   �ú����ĵ�����Ҫ����ClkPorRcToDpll()֮��
 */
int32_t SpiFlashClkSet(uint8_t FshcClkSel, bool EnHpm);

/**
 * @brief  ��Flash��ʱ���л���ϵͳʱ��
 * @param  ��.
 * @return �ɹ�����0, ���󷵻ط�0ֵ.
 */
int32_t SpiFlashClktoSysClk(void);

/**
 * @brief  �ָ�ԭ����Flashʱ������
 * @param  ��.
 * @return �ɹ�����0, ���󷵻ط�0ֵ.
 */
int32_t SpiFlashClkRestore(void);

/**
 * @brief  ��ʼ������ȡ�����Ϣ����Flash ID��������
 * @param  ��
 * @return ��
 * @note   �����ȵ��øú���֮����ܵ���Flash ��/д/IO��������
 **/
void SpiFlashInfoInit(void);

/**
 * @brief  ��ȡFlash��Ϣ
 * @param  ������Ϣbuffer
 * @return �ɹ�����0�����򷵻ظ���
 * @note   �ú���������SpiFlashInfoInit֮��ſ��Ե���
 **/
int32_t SpiFlashGetInfo(SPI_FLASH_INFO * FlashInfo);

/**
 * @brief	Flash������
 * @param	Offset	FlashƬ����Ч��һƫ�Ƶ�ַ
 * @param	Buf		��Ҫ����flash�����ݴ���ڴ��׵�ַ
 * @param	BufLen	��Ҫ����flash�����ݵĳ���
 * @return  �ɹ�����0�����򷵻ظ���
**/
int32_t SpiFlashRead(uint32_t Offset, uint8_t* Buf, uint32_t BufLen);

/**
 * @brief	Flashд����
 * @param	Offset	FlashƬ����Ч��һƫ�Ƶ�ַ
 * @param	Buf		��Ҫд��flash�����ݴ���ڴ��׵�ַ
 * @param	BufLen	��Ҫд��flash�����ݵĳ���
 * @return  �ɹ�����0�����򷵻ظ���
**/
int32_t SpiFlashWrite(uint32_t Offset, uint8_t* Buf, uint32_t BufLen);


/**
 * @brief	Flash��������
 * @param	Offset	flashƬ��ƫ�Ƶ�ַ��4KB����
 * @param	Size	flash�����ֽ������粻��4KB���Զ�4KB����
 * @return  �ɹ�����0�����򷵻ظ���
**/
int32_t SpiFlashErase(uint32_t Offset, uint32_t Size);

/**
 * @brief	Flash IO ����
 * @param	Cmd	 	IO control ����
 * @param	�ɱ����,arg1,arg2... ����Cmd������
 *			IOCTL_FLASH_PROTECT: 
 *				arg1: uint8_t ProtectRange
 *					1: Half code area protect;
 *					2: Three Quarter Area Protect;
 *					3: Seven Eighths Area Protect;
 *					4: All Area Protect
 *					
 *			IOCTL_FLASH_UNPROTECT:
 *				arg1: uint8_t* Buf, Bufָ���������Ϊ"\x35\xBA\x69"
 *				arg2: int32_t BufLen �̶�Ϊ 3
 *				
 *			IOCTL_FLASH_RDID:	
 *				arg1: uint16_t * did
 *				arg2: uint8_t * mid
 *				
 *			IOCTL_FLASH_RDSTATUS:
 *				arg1: uint32_t *status
 *				
 *			IOCTL_FLASH_CRMRESET:
 *			
 *			IOCTL_FLASH_IO_MODE:
 *				arg1: SPIFLASH_IO_MODE io_mode
 *				
 *			IOCTL_FLASH_DEEPPOWERDOWN:
 *			
 *			IOCTL_FLASH_EXIT_DEEEPPOWERDOWN:
 *			
 *			IOCTL_FLASH_SETHPM:
 *				arg1: bool hpmEn
 *			
 * @return  �ɹ�����0�����򷵻ظ���
**/
int32_t SpiFlashIOCtl(uint32_t Cmd, ...);




#ifdef  __cplusplus
}
#endif//__cplusplus

#endif  //__SPI_FLASH_H__

/**
 * @}
 * @}
 */
