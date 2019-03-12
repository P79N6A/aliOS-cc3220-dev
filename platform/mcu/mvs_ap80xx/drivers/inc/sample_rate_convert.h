/**
 *****************************************************************************
 * @file     sample_rate_convert.h
 * @author   orson
 * @version  V1.0.0
 * @date     06-May-2014
 * @maintainer: YunWang
 * @brief    declare i2s module driver interface
 *****************************************************************************
 * @attention
 * <h2><center>&copy; COPYRIGHT 2013 MVSilicon </center></h2>
 */

/**
* @addtogroup ����
* @{
* @defgroup SampleRateConvert SampleRateConvert
* @{
*/

#ifndef __SAMPLE_RATE_CONVERT_H__
#define __SAMPLE_RATE_CONVERT_H__

#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#include "type.h"
    
    
/***************************************************************
            �������������������в�����ת44.1KHZʹ��
***************************************************************/ 
/**
 * @brief  ������ת��ģ���ʼ��
 * @param  SrcIndex       �ṩ1��������ת��ʵ�壬���Ϊ0
 * @param  InSampleRate   ������ת��ǰ�Ĳ����ʣ�9�ֲ�����
 * @param  OutSampleRate  ������ת����Ĳ����ʣ�Ŀǰֻ����44100
 * @param  ChannelNum     ����������1 or 2
 * @return TRUE or FALSE
 */   
bool SampleRateConvertInit(uint8_t SrcIndex, uint16_t InSampleRate, uint16_t OutSampleRate, uint8_t ChannelNum);   

/**
 * @brief  ������ת��ģ���ʼ��
 * @param  SrcIndex       �ṩ1��������ת��ʵ�壬���Ϊ0
 * @param  InBuf          ������ת��ǰ��PCM����
 * @param  OutBuf		  ������ת�����PCM����
 * @param  SampleCnt      ������ת��ǰ�Ĳ�������
 * @return ת�����OutBuf����������Ч��������
 */
uint16_t SampleRateConvert(uint8_t SrcIndex, int16_t* InBuf, int16_t* OutBuf, uint16_t SampleCnt);


/***************************************************************
            ���º����������в�������4����
***************************************************************/ 
/**
 * @brief  ��������4����ʼ��
 * @param  W_Addr     ����������ʱ��Ҫ�õ���FIR�˲��������ַ
 * @param  W_Sample   FIR�˲��������С����λΪ��Sample
 * @param  Channel    ͨ���ţ�1 -- �������� 2 -- ˫������Ŀǰ��֧�ֵ�����
 * @return ��
 */
void SampleRateUpSampleby4Init(int16_t *W_Addr, int32_t W_Sample, int16_t Channel);

/**
 * @brief  ��������4��Ŀǰֻ֧�ֵ�����
 * @param  BufIn    ����������ʱ��Ҫ�õ���FIR�˲��������ַ
 * @param  BufOut   FIR�˲��������С����λΪ��Sample
 * @param  n 		����Ĳ����㣬ע��n * 4 + 30������W_Sample
 * @return >=0����Ĳ�������, -1 �����������
 */
int32_t SampleRateUpSampleby4(int16_t *BufIn, int16_t *BufOut, int32_t n);
    
#ifdef  __cplusplus
}
#endif//__cplusplus

#endif   //

/**
 * @}
 * @}
 */
