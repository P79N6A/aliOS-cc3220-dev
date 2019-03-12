/******************************************************************************
 * @file    mixer.h
 * @author  Orson
 * @version V1.0.0
 * @date    29-April-2014
 * @brief   audio mixer
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 MVSilicon </center></h2>
 */

/**
* @addtogroup ����
* @{
* @defgroup Mixer Mixer
* @{
*/

#ifndef __MIXER_H__
#define __MIXER_H__

#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#define MIXER_FORMAT_MONO			1	/**��������Դ��ÿ��������2���ֽڡ����������룬˫�������*/
#define MIXER_FORMAT_STERO			2	/**˫������Դ��ÿ��������4���ֽڡ�˫�������룬˫�������*/
#define MIXER_FORMAT_MONO_1	        3	/**��������Դ��ÿ��������2���ֽڡ����������룬�������������DAC���������*/
#define MIXER_FORMAT_MONO_2     	4	/**��������Դ��ÿ��������2���ֽڡ����������룬�������������DAC���������*/
#define MIXER_FORMAT_STERO_1	    5	/**˫������Դ��ÿ��������4���ֽڡ�˫�������룬�������������ȡ����Դ������������DAC���������*/
#define MIXER_FORMAT_STERO_2	    6	/**˫������Դ��ÿ��������4���ֽڡ�˫�������룬�������������ȡ����Դ������������DAC���������*/
#define MIXER_FORMAT_STERO_3	    7	/**˫������Դ��ÿ��������4���ֽڡ�˫�������룬���������������Դ������������ӳ���2����DAC���������*/
#define MIXER_FORMAT_STERO_4	    8	/**˫������Դ��ÿ��������4���ֽڡ�˫�������룬���������������Դ������������ӳ���2����DAC���������*/


/**
 * @brief  Mixerģ���ʼ��
 * @param  PcmFifoAddr PcmFifo��ʼ��ַ
 * @param  PcmFifoSize PcmFifo����
 * @return ��
 * @note PcmFifo������PMEM��
 */
void MixerInit(void* PcmFifoAddr, uint16_t PcmFifoSize);

/**
 * @brief  ����Mixerģ���ڲ��Ƿ���������ת����ֻ������ͨ��0��Ч
 * @param  SrcEnable �Ƿ�ת����ʹ���ͺ�
 * @arg        SrcEnable = FALSE�����������PCM������������ת��
 * @arg        SrcEnable = TRUE�� Mixerģ���ڲ��������PCM���ݲ�����ͳһת��Ϊ44.1KHZ
 * @return ��
 */
void MixerSrcEnable(bool SrcEnable);

/**
 * @brief  Mixerģ��ʹ��ĳ������ͨ��
 * @param  SourceID mixerĳ������ͨ��
 * @return ��
 */
void MixerEnable(uint8_t SourceID);

/**
 * @brief  Mixerģ���ֹĳ������ͨ��
 * @param  SourceID mixerĳ������ͨ��
 * @return ��
 */
void MixerDisable(uint8_t SourceID);

/**
 * @brief  Mixerģ�鶯̬�ı�ĳ��ͨ��������
 * @param  SourceID mixerĳ������ͨ����0--3��
 * @param  LeftVolume 0--8191, 4095: 0dB, 8191: +6dB
 * @param  RightVolume 0--8191, 4095: 0dB, 8191: +6dB
 * @return ��
 */
void MixerConfigVolume(uint8_t SourceID, uint16_t LeftVolume, uint16_t RightVolume);

/**
 * @brief  Mixerģ��MUTEĳ������ͨ��
 * @param  SourceID mixerĳ������ͨ��
 * @return ��
 */
void MixerMute(uint8_t SourceID);

/**
 * @brief  Mixerģ��UNMUTEĳ������ͨ��
 * @param  SourceID mixerĳ������ͨ��
 * @return ��
 */
void MixerUnmute(uint8_t SourceID);

/**
 * @brief  Mixerģ�鶯̬�ı�ĳ��ͨ���Ĳ��������ʽ
 * @param  SourceID  mixerĳ������ͨ��(0--3)
 * @param  SampleRate �����ʣ�����Դ0֧��9�ֲ����ʣ���������Դֻ֧��44.1KHZ������
 * @param  PcmFormat  1--��������2--˫����
 * @return ��
 */
void MixerConfigFormat(uint8_t SourceID, uint16_t SampleRate, uint8_t PcmFormat);

/**
 * @brief  Mixerģ����ĳͨ���Ƿ���������µ�����
 * @param  SourceID mixerĳ������ͨ��
 * @return �������������ݷ���TRUE�����򷵻�FALSE
 */
bool MixerIsDone(uint8_t SourceID);

/**
 * @brief  Mixerģ�������µ�����
 * @param  SourceID mixerĳ������ͨ����0--3��
 * @param  PcmBuf  PCM�������׵�ַ
 * @param  SampleCnt  ��������
 * @return ��
 */
void MixerSetData(uint8_t SourceID, void* PcmBuf, uint16_t SampleCnt);

/**
 * @brief  Mixerģ���ȡĳ����Դ��Mixer��ʣ��Ĳ�������������Ϊ�ڸ�����Դ�������µĵ���
 * @param  SourceID mixerĳ������ͨ��
 * @return ������Դ��mixer�ڵ�ʣ���������
 */
uint16_t MixerGetRemainSamples(uint8_t SourceID);

/**
 * @brief  Mixerģ������fadein/fadeout�ٶ�
 * @param  SourceID mixerĳ������ͨ��
 * @param  FadeinTime  0--2000��������0���䵽4095��ʱ�䣬��λ: ms
 * @param  FadeoutTime 0--2000��������4095���䵽0��ʱ�䣬��λ: ms
 * @return ��
 */
void MixerSetFadeSpeed(uint8_t SourceID, uint16_t FadeinTime, uint16_t FadeoutTime);

/**
 * @brief  Mixerģ����������ʹ��/����
 * @param  en TRUE/FALSE
 * @arg 	TRUE - MixerConfigVolume���Կ�������
 * @arg 	FALSE - MixerConfigVolume���ܿ���������Mixer����Ƶ���ݲ����κ�����/˥��
 * @return ��
 */
void MixerVolEn(bool en);

#ifdef  __cplusplus
}
#endif//__cplusplus

#endif

/**
 * @}
 * @}
 */
