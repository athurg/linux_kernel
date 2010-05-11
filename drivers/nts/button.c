/*
 * Driver for buttons on GPIO lines capable of generating interrupts.
 *
 *	kernel/drivers/nts/button.c
 *
 *
 * Author: AT <athurg.feng@nts-intl.com>
 *
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>

typedef struct{
	unsigned int irq;
	const char * devname;
}IRQ_ITERM;

IRQ_ITERM button_irqs[]={
	{IRQ_GPIO_00,	"GPIO 00,button 0"},
	{IRQ_GPIO_01,	"GPIO 01,button 1"},
	{IRQ_GPI_03,	"GPI  03,button 2"},
	{IRQ_GPI_07,	"GPI  04 button 3"},
};

irqreturn_t button_irs(int irq, void * dev_id)
{
	printk("Button pressed!\n");
	return IRQ_HANDLED;
}

static int __init button_probe(void)
{
	unsigned int ret,i;
	printk("Button Probe...\n");

	for(i=0; i<sizeof(button_irqs); i++){
		set_irq_type(button_irqs[i].irq, IRQ_TYPE_EDGE_FALLING);
		ret = request_irq(button_irqs[i].irq, &button_irs, IRQ_DISABLED, button_irqs[i].devname, NULL);
		if(ret <0){
			printk("request irq %d Failed",button_irqs[i].irq);
		}
	}

	return 0;
}

static void __exit button_remove(void)
{
	void * dev_id=NULL;
	printk("Button Remove...\n");
	free_irq(IRQ_GPIO_00, dev_id);
}

module_init(button_probe);
module_exit(button_remove);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Athurg <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("Button IRQ test");
MODULE_ALIAS("platform:button");
