#include <board.h>
#include "vic.h"

/*vic handle*/

void xm_ack_irq(unsigned int irq)
{
	//writel(1<<irq, (INTC_INTENCLEAR));
}

void xm_mask_irq(unsigned int irq)
{
	writel(1<<irq, INTC_INTENCLEAR);
}

void xm_unmask_irq(unsigned int irq)
{
	uint32_t value = readl(INTC_INTENABLE);
	writel(value | ( 1 << irq), INTC_INTENABLE);
}

void  xm510_init_irq(void)
{
//ȫ��ʹ����ͨ�жϣ���ʹ�����жϣ��Ժ���Ը�Ϊʹ��fiq ��vector�ж�
	writel(~0, INTC_INTENCLEAR);
	writel(0, INTC_INTSELECT);
	writel(~0, INTC_SOFTINTCLEAR);
}

