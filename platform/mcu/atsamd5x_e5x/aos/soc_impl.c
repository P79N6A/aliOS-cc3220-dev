/**
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

#include <k_api.h>
#include <assert.h>
#include <stdio.h>
#include <sys/time.h>

#if (RHINO_CONFIG_HW_COUNT > 0)
void soc_hw_timer_init(void)
{
}

hr_timer_t soc_hr_hw_cnt_get(void)
{
    return 0;
}

lr_timer_t soc_lr_hw_cnt_get(void)
{
    return 0;
}
#endif /* RHINO_CONFIG_HW_COUNT */

#if (RHINO_CONFIG_INTRPT_STACK_OVF_CHECK > 0)
void soc_intrpt_stack_ovf_check(void)
{
}
#endif

#if (RHINO_CONFIG_MM_TLF > 0)

#if defined (__CC_ARM) /* Keil / armcc */
#if 1
#define HEAP_BUFFER_SIZE 1024*30
uint8_t g_heap_buf[HEAP_BUFFER_SIZE];
k_mm_region_t g_mm_region[1];
int           g_region_num = 1;
void aos_heap_set()
{
    g_mm_region[0].start = g_heap_buf;
    g_mm_region[0].len   = HEAP_BUFFER_SIZE;
}
#else
extern unsigned int Image$$RW_IRAM1$$ZI$$Limit;
extern size_t g_iram1_start;
extern size_t g_iram1_total_size;
k_mm_region_t g_mm_region[1];
int           g_region_num = 1;
void aos_heap_set()
{
    g_mm_region[0].start = (uint8_t*)&Image$$RW_IRAM1$$ZI$$Limit;
    g_mm_region[0].len   =
        (g_iram1_start + g_iram1_total_size - (size_t)&Image$$RW_IRAM1$$ZI$$Limit);
    printf("g_mm_region[0].start is 0x%x, g_mm_region[0].len is 0x%x \r\n", (size_t)g_mm_region[0].start, g_mm_region[0].len);
}
#endif
#elif defined (__ICCARM__)/* IAR */
#define HEAP_BUFFER_SIZE 1024*64
int           g_region_num = 1;
uint8_t g_heap_buf[HEAP_BUFFER_SIZE];
k_mm_region_t g_mm_region[] = {{g_heap_buf, HEAP_BUFFER_SIZE}};
void aos_heap_set()
{
    g_mm_region[0].start = g_heap_buf;
    g_mm_region[0].len   = HEAP_BUFFER_SIZE;
}

#else /* GCC */
extern void         *_estack;
extern void         *__bss_end__;
/* __bss_end__ and _estack is set by linkscript(*.ld)
   heap and stack begins from __bss_end__ to _estack */
k_mm_region_t g_mm_region[1];
int           g_region_num = 1;
void aos_heap_set()
{
    g_mm_region[0].start = (uint8_t*)&__bss_end__;
    g_mm_region[0].len   =
        ((uint8_t*)&_estack - (uint8_t*)&__bss_end__) - RHINO_CONFIG_SYSTEM_STACK_SIZE;
}
#endif
#endif

void soc_err_proc(kstat_t err)
{
    (void)err;

<<<<<<< HEAD
    #if (RHINO_CONFIG_TASK_STACK_CUR_CHECK > 0)
    soc_print_stack();
    #endif
=======
>>>>>>> AliOSOfficial/master
    assert(0);
}

krhino_err_proc_t g_err_proc = soc_err_proc;

