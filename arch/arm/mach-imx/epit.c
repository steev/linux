/*
 *  linux/arch/arm/mach-imx/epit.c
 *
 *  Copyright (C) 2010 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/sched_clock.h>

#include "common.h"
#include "hardware.h"

#define EPITCR		0x00
#define EPITSR		0x04
#define EPITLR		0x08
#define EPITCMPR	0x0c
#define EPITCNR		0x10

#define EPITCR_EN			(1 << 0)
#define EPITCR_ENMOD			(1 << 1)
#define EPITCR_OCIEN			(1 << 2)
#define EPITCR_RLD			(1 << 3)
#define EPITCR_PRESC(x)			(((x) & 0xfff) << 4)
#define EPITCR_SWR			(1 << 16)
#define EPITCR_IOVW			(1 << 17)
#define EPITCR_DBGEN			(1 << 18)
#define EPITCR_WAITEN			(1 << 19)
#define EPITCR_RES			(1 << 20)
#define EPITCR_STOPEN			(1 << 21)
#define EPITCR_OM_DISCON		(0 << 22)
#define EPITCR_OM_TOGGLE		(1 << 22)
#define EPITCR_OM_CLEAR			(2 << 22)
#define EPITCR_OM_SET			(3 << 22)
#define EPITCR_CLKSRC_OFF		(0 << 24)
#define EPITCR_CLKSRC_PERIPHERAL	(1 << 24)
#define EPITCR_CLKSRC_REF_HIGH		(1 << 24)
#define EPITCR_CLKSRC_REF_LOW		(3 << 24)

#define EPITSR_OCIF			(1 << 0)

static struct clock_event_device clockevent_epit;
static enum clock_event_mode clockevent_mode = CLOCK_EVT_MODE_UNUSED;

static void __iomem *epit_base;
static u32 sched_clock_reg;

#define epit_read(reg)		__raw_readl(epit_base + (reg))
#define epit_write(val,reg)	__raw_writel(val, epit_base + (reg))

static u32 notrace epit_read_sched_clock(void)
{
	return sched_clock_reg ? ~epit_read(sched_clock_reg) : 0;
}

static int __init epit_clocksource_init(struct clk *timer_clk, bool use_sched_clock)
{
	unsigned int rate = clk_get_rate(timer_clk);

	if (use_sched_clock) {
		sched_clock_reg = EPITCNR;
		setup_sched_clock(epit_read_sched_clock, 32, rate);
	}

	return clocksource_mmio_init(epit_base + EPITCNR, "epit", rate, 200, 32,
			clocksource_mmio_readl_down);
}

/* clock event */
static int epit_set_next_event(unsigned long evt, struct clock_event_device *unused)
{
	unsigned long tcmp;

	tcmp = epit_read(EPITCNR);

	epit_write(tcmp - evt, EPITCMPR);

	return 0;
}

static void epit_set_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	unsigned long flags;
	u32 val;

	/*
	 * The timer interrupt generation is disabled at least
	 * for enough time to call epit_set_next_event()
	 */
	local_irq_save(flags);

	/* Disable interrupt in EPIT module */
	val = epit_read(EPITCR);
	val &= ~EPITCR_OCIEN;
	epit_write(val, EPITCR);

	if (mode != clockevent_mode) {
		/* Set event time into far-far future */

		/* Clear pending interrupt */
		epit_write(EPITSR_OCIF, EPITSR);
	}

	/* Remember timer mode */
	clockevent_mode = mode;
	local_irq_restore(flags);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		pr_err("%s: CLK_EVT_MODE_PERIODIC is not supported for i.MX EPIT\n", __func__);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/*
		* Do not put overhead of interrupt enable/disable into
		* epit_set_next_event(), the core has about 4 minutes
		* to call epit_set_next_event() or shutdown clock after
		* mode switching
		*/
		local_irq_save(flags);
		val = epit_read(EPITCR);
		val |= EPITCR_OCIEN;
		epit_write(val, EPITCR);
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_RESUME:
		/* Left event sources disabled, no more interrupts appear */
		break;
	}
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t epit_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_epit;

	epit_write(EPITSR_OCIF, EPITSR);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction epit_timer_irq = {
	.name		= "i.MX EPIT Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= epit_timer_interrupt,
};

static struct clock_event_device clockevent_epit = {
	.name		= "epit",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= epit_set_mode,
	.set_next_event	= epit_set_next_event,
	.rating		= 200,
};

static int __init epit_clockevent_init(struct clk *timer_clk)
{
	unsigned int rate = clk_get_rate(timer_clk);

	clockevent_epit.cpumask = cpumask_of(0);

	clockevents_config_and_register(&clockevent_epit, rate, 0x800, 0xfffffffe);

	return 0;
}

static const struct of_device_id epit_of_match[] __initconst = {
	{ .compatible = "fsl,imx6q-epit", },
	{ .compatible = "fsl,imx51-epit", },
	{ .compatible = "fsl,imx35-epit", },
	{},
};

void __init imx_epit_register(void)
{
	struct device_node *np;
	void __iomem *base;
	int irq;
	struct clk *clk_ipg, *clk_per;
	bool use_sched_clock = false;

	np = of_find_matching_node(NULL, epit_of_match);

	base = of_iomap(np, 0);
	WARN_ON(!base);

	irq = irq_of_parse_and_map(np, 0);

	clk_ipg = of_clk_get_by_name(np, "ipg");
	if (IS_ERR(clk_ipg)) {
		pr_err("%s: unable to get ipg clock\n", __func__);
		return;
	}

	clk_per = of_clk_get_by_name(np, "per");
	if (IS_ERR(clk_per)) {
		pr_err("%s: unable to get per clock\n", __func__);
		return;
	}

	clk_prepare_enable(clk_ipg);
	clk_prepare_enable(clk_per);

	pr_info("%s: base 0x%08x, ipg rate %ld, per rate %ld\n", __func__,
			base, clk_get_rate(clk_ipg), clk_get_rate(clk_per));

	epit_base = base;

	/*
	 * Initialise to a known state (all timers off, and timing reset)
	 */
	epit_write(0x0, EPITCR);

#define EPITCR_FLAGS 	EPITCR_EN | EPITCR_CLKSRC_REF_HIGH | EPITCR_WAITEN
	epit_write(0xffffffff, EPITLR);
	epit_write(EPITCR_FLAGS, EPITCR);

	if (of_find_property(np, "linux,scheduler-clock", NULL))
		use_sched_clock = true;

	/* init and register the timer to the framework */
	epit_clocksource_init(clk_ipg, use_sched_clock);
	epit_clockevent_init(clk_ipg);

	/* Make irqs happen */
	setup_irq(irq, &epit_timer_irq);
}
