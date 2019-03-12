/**
 *******************************************************************************
 * @file    time_api.h
 * @author  lujiangang
 * @version V1.0.1
 * @date    27-May-2013
 * @brief   timer API header file
 * @note ���Timer��һ���;��ȵĶ�ʱ������ʱ�������С��MIN_TIMER_PERIOD
 *******************************************************************************
 * @attention
 *
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
* @defgroup TimerAPI TimerAPI
* @{
*/
 
#ifndef __TIMER_API_H__
#define __TIMER_API_H__

#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#define APP_TIMER_API 	/**<��OS Timer �ķ�װ */


#ifdef APP_TIMER_API

unsigned int xTaskGetTickCount(void);

#define MIN_TIMER_PERIOD 50				/**<��С��ʱ���ms */ 

#define MAX_SYS_TICKS 0xFFFFFFFF 	/**<ϵͳTicks���ֵ�����ڼ�ʱ��ת */

typedef void (*TIMER_CALLBACK)(void* Param);

#pragma pack(1)

typedef struct _SW_TIMER_
{
	uint32_t TimerPeriod;  /**< ms,ע�⣬��С��ʱ�����Ӧ��ģʽ��ѭ����������������ʱ���йأ����鲻Ҫ������ѭ�����������ʱ��*/
	uint32_t LastTickets;  /**< ����timeout�ж�*/
	uint32_t InitTicks;    /**<Timer��һ�ο���ʱ��ticks*/
	uint32_t PausedTimes;  /**<ms������ͣ��ʱ��*/
	TIMER_CALLBACK Callback;	
	uint8_t  IsRunning;				
	uint8_t  TicksResetCount; /**<��¼SysTicks��ת����*/
} SW_TIMER;

#pragma pack()

/**
 * @brief  ��õ�ǰTimerʵ������ʱ�䣬��λms
 * @param  *TimerHandle	Timer���
 * @return Timer����ʱ��
 */
uint32_t GetPassTime(SW_TIMER* TimerHandle);

/**
 * @brief  Ӧ��ģʽ��ѭ���У����øú�������ѯTimer����ʱ����CallBack��
 * @param  *TimerHandle Timer���
 * @return None
 */
void CheckTimer(SW_TIMER* TimerHandle);

/**
 * @brief  ��ʼ��Timer
 * @param  *TimerHandle	Timer���
 * @param TimerPeriod  	��ʱ���ms
 * @param CallbackFunc	 ��ʱ�ص�����
 * @return �ɹ� TRUE��ʧ��FALSE
 */
bool InitTimer(SW_TIMER* TimerHandle,
	               uint32_t TimerPeriod,
	               TIMER_CALLBACK CallbackFunc);

/**
 * @brief  ����Timer
 * @param  *TimerHandle Timer���
 * @return �ɹ�TRUE��ʧ��FALSE
 */
bool StartTimer(SW_TIMER* TimerHandle);

/**
 * @brief  ֹͣTimer
 * @param  *TimerHandle	Timer���
 * @return �ɹ�TRUE��ʧ��FALSE
 */
bool StopTimer(SW_TIMER* TimerHandle);


/**
 * @brief  ����Timer
 * @param  *TimerHandle:Timer���
 * @return �ɹ�TRUE��ʧ��FALSE
 */
bool DeinitTimer(SW_TIMER* TimerHandle);

/**
 * @brief  ����Timer�ļ�ʱ״̬
 * @param  *TimerHandle	Timer���
 * @return �ɹ�TRUE��ʧ��FALSE
 * @note �����ǰ��ֵ����ʵ������ʱ�䣬Timer����ʱ����
 */
bool ResetTimer(SW_TIMER* TimerHandle);

/**
 * @brief  ����Timer��ʱ���
 * @param  *TimerHandle	Timer���
 * @param  TimerPeriod	��ʱ���
 * @return �ɹ�TRUE��ʧ��FALSE
 */
bool ChangeTimerPeriod(SW_TIMER* TimerHandle, uint32_t TimerPeriod);

#endif/*SYS_TIMER_API*/

#ifdef __cplusplus
}
#endif//__cplusplus

#endif/*TIMER_API_H*/

/**
 * @}
 * @}
 */
