/**
  *****************************************************************************
  * @file:			spis.h
  * @author			Ingrid Chen
  * @maintainer		lilu
  * @version		V1.0.0
  * @data			18-June-2013
  * @Brief			SPI Slave driver header file.
  * @note			For sdio and spi can't access memory simultaneously
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
* @defgroup SPIS SPIS
* @{
*/

#ifndef __SPIS_H__
#define __SPIS_H__

#ifdef __cplusplus
extern "C" {
#endif//__cplusplus


/**
 * @brief	SPIS��ʼ��
 * @param	Mode
 *			0 - CPOL = 0 & CPHA = 0, 1 - CPOL = 0 & CPHA = 1,
 *			2 - CPOL = 1 & CPHA = 0, 3 - CPOL = 1 & CPHA = 1
 * @param	StartAddr	SPIS��������ʼ��ַ����PMEM��ʼ����Ե�ַ
 * @param	RXDepth		���ջ�������ȣ������ȿɵ�32767�ֽڣ�
 * @param	TXDepth		���ͻ�������ȣ������ȿɵ�32767�ֽڣ���StartAddr+RXDepth��Ϊ���ͻ������׵�ַ��
 * @return	��
 * @note@ 	���ڹ���PMEM����spis��uart����ͬʱ����
 */
bool SpiSlaveInit(uint8_t Mode, uint32_t StartAddr, uint32_t RxDepth, uint32_t TxDepth);

/**
 * @brief	SPIS�رգ���ҪΪ�ͷ�PMEM����
 * @param	��
 * @return	��
 * @note@
 */
void SpiSlaveClose(void);

/**
 * @brief	SPIS��������
 * @param	Buf 		��������buf���׵�ַ
 * @param	BufLen 		�������ݵĳ��ȣ��ֽڵ�λ��
 * @param	TimeOut		��ʱʱ��
 * @return	ʵ�ʷ��͵����ݳ��ȣ��ֽڵ�λ��
 */
uint32_t SpiSlaveSend(uint8_t* Buf, uint32_t BufLen, uint32_t TimeOut);

/**
 * @brief	SPIS��������
 * @param	Buf 		��������buf���׵�ַ
 * @param	BufLen 		�������ݳ��ȣ��ֽڵ�λ��
 * @param	TimeOut 	��ʱʱ��
 * @return	ʵ�ʽ��յ����ݳ���
 */
uint32_t SpiSlaveReceive(uint8_t* Buf, uint32_t BufLen, uint32_t TimeOut);

/**
 * @brief	SPIS����жϱ�־
 * @param	��
 * @return	��
 * @note@
 */
void SpiSlaveIntClr(void);

/**
 * @brief  ��ȡ���ջ�������ʣ�����ݳ��ȣ��ֽڵ�λ��
 * @param  ��
 * @return ���ؽ��ջ�����ʣ�����ݳ��ȣ��ֽڵ�λ��
 */
uint32_t SpiSlaveRxDataLenGet(void);

/**
 * @brief  ��ս��ջ�����
 * @param  ��
 * @return ��
 */
void SpisClrRxBuff(void);
#ifdef  __cplusplus
}
#endif//__cplusplus

#endif

/**
 * @}
 * @}
 */
