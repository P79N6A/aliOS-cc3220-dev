/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <k_api.h>
#include <assert.h>

#if (RHINO_CONFIG_HW_COUNT > 0)
void soc_hw_timer_init(void)
{
}

hr_timer_t soc_hr_hw_cnt_get(void)
{
    return 0;
    //return *(volatile uint64_t *)0xc0000120;
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

extern void         *heap_start;
extern void         *heap_end;
extern void         *heap_len;
extern void         *__IramLeft;
extern void         *__IramLeftLen;
extern void         *__SmemLeft;
extern void         *__SmemLeftLen;

k_mm_region_t g_mm_region[] =
    {{(uint8_t*)&heap_start,(size_t)&heap_len},{(uint8_t*)&__SmemLeft,(size_t)&__SmemLeftLen},{(uint8_t*)&__IramLeft,(size_t)&__IramLeftLen}};
int           g_region_num  = sizeof(g_mm_region)/sizeof(k_mm_region_t);

size_t soc_get_cur_sp()
{
    size_t sp = 0;
    asm volatile(
        "mov %0,sp\n"
        :"=r"(sp));
    return sp;
}

static void soc_print_stack()
{

    uint32_t offset = 0;
    kstat_t  rst    = RHINO_SUCCESS;
    void    *cur, *end;
    int      i=0;
    int     *p;

    end   = krhino_cur_task_get()->task_stack_base + krhino_cur_task_get()->stack_size;
    cur = soc_get_cur_sp();
    p = (int*)cur;
    while(p < (int*)end) {
        if(i%4==0) {
            printf("\r\n%08x:",(uint32_t)p);
        }
        printf("%08x ", *p);
        i++;
        p++;
    }
    printf("\r\n");
    return;
}
void soc_err_proc(kstat_t err)
{
    (void)err;
    soc_print_stack();
    assert(0);
}

krhino_err_proc_t g_err_proc = soc_err_proc;

