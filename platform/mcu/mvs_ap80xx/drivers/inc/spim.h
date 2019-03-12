/**
  *****************************************************************************
  * @file:			spim.h
  * @author			Ingrid Chen
  * @maintainer 	Lilu
  * @version		V1.0.0
  * @data			18-June-2013
  * @Brief			SPI Master driver header file.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MVSILICON SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 MVSilicon </center></h2>
  */

/**
* @addtogroup ����
* @{
* @defgroup SPIM SPIM
* @{
*/

#ifndef __SPIM_H__
#define __SPIM_H__

#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#define	SPIM_CLK_DIV_24M	0x0			/**< SPI master clock Div 24MHz*/
#define	SPIM_CLK_DIV_12M	0x1			/**< SPI master clock Div 12MHz*/
#define	SPIM_CLK_DIV_6M		0x2			/**< SPI master clock Div 6MHz*/
#define	SPIM_CLK_DIV_3M		0x3			/**< SPI master clock Div 3MHz*/
#define	SPIM_CLK_DIV_1M5	0x4			/**< SPI master clock Div 1.5MHz*/
#define	SPIM_CLK_DIV_750K	0x5			/**< SPI master clock Div 750KHz*/
#define	SPIM_CLK_DIV_325K	0x6			/**< SPI master clock Div 325KHz*/

#define	NONE_FLASH  0
#define	FLASH_GD	1
#define	FLASH_SST	2

/**
 * ������
 */
typedef enum _SPI_MASTER_ERR_CODE
{
    ERR_SPIM_TIME_OUT = -255,			/**<function execute time out*/
    ERR_SPIM_DATALEN_OUT_OF_RANGE,		/**<data len is out of range < 0*/
    SPIM_NONE_ERR = 0,
} SPI_MASTER_ERR_CODE;


/**
 * @brief	SPIM��ʼ��
 * @param	Mode
 *			0 - CPOL = 0 & CPHA = 0, 1 - CPOL = 0 & CPHA = 1,
 *			2 - CPOL = 1 & CPHA = 0, 3 - CPOL = 1 & CPHA = 1,
 * @param	ClkDiv 	��Чֵ:(0 ~ 11)
 * 			��Ƶϵ����ӦSPIM��Ƶ�ʣ�0 - 24M��1 - 12M��2 - 6M��3 - 3M��4 - 1.5M....
 * @return	��
 * @note@	��ϵͳƵ�ʽ��з�Ƶ����Ƶ��Ӱ��SPIM���Ƶ��
 */
void SpiMasterInit(uint8_t Mode, uint8_t ClkDiv);

/**
 * @brief	SPIM����1�ֽ�����
 * @param	val  ��Ҫ���͵�1�ֽ�����ֵ
 * @return	��
 */
void SpiMasterSendByte(uint8_t val);

/**
 * @brief	SPIM����1�ֽ�����
 * @param	��
 * @return	���յ���1�ֽ�����
 */
uint8_t SpiMasterRecvByte(void);

/**
 * @brief	SPIM��������
 * @param	SendBuf	��������Buf���׵�ַ
 * @param	Length 	�������ݵĳ��ȣ��ڲ�ʹ��DMA�������ݣ���󳤶�65535�ֽڣ�
 * @return	������
 * @note@
 */
SPI_MASTER_ERR_CODE SpiMasterSendData(uint8_t* SendBuf, uint32_t Length);

/**
 * @brief	SPIM��������
 * @param	RecvBuf	��������Buf���׵�ַ
 * @param	Length	�������ݵĳ��ȣ��ڲ�ʹ��DMA�������ݣ���󳤶�65535�ֽڣ�
 * @return	������
 * @note@
 */
SPI_MASTER_ERR_CODE SpiMasterRecvData(uint8_t* RecvBuf, uint32_t Length);

/**
 * @brief	SPIM��ʼ���͡���������
 *				SpiMasterStartData(...), SpiMasterGetDmaDone(), SpiMasterIntClr()�����������ʹ�ù���SPIM�շ����ݵķ������ӿ�
 * @param	Buf			����Buf���׵�ַ
 * @param	Length		���ݵĳ��ȣ��ڲ�ʹ��DMA�������ݣ���󳤶�65535�ֽڣ�
 * @param	Direction	���䷽�� 0:����   1:����
 * @return	��
 * @note@
 */
void SpiMasterStartData(uint8_t* Buf, uint32_t Length, uint8_t Dir);
#define SpiMasterSendBytes(Buf, Length)   SpiMasterStartData(Buf, Length, 0)
#define SpiMasterRecvBytes(Buf, Length)   SpiMasterStartData(Buf, Length, 1)

/**
 * @brief	��ȡSPIM�շ����ݵ����״̬
 *				SpiMasterStartData(...), SpiMasterGetDmaDone(), SpiMasterIntClr()�����������ʹ�ù���SPIM�շ����ݵķ������ӿ� 
 * @param	��
 * @return	�������ݴ����Ƿ����
 * @note@
 */
bool SpiMasterGetDmaDone(void);

/**
 * @brief	���SPIM��ɱ�־λ
 *				SpiMasterStartData(...), SpiMasterGetDmaDone(), SpiMasterIntClr()�����������ʹ�ù���SPIM�շ����ݵķ������ӿ� 
 * @param	��
 * @return	��
 * @note@
 */
void SpiMasterIntClr(void);

#ifdef __cplusplus
}
#endif//__cplusplus

#endif

/**
 * @}
 * @}
 */
