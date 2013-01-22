/*
 *  linux/arch/arm/mach-imx/time.c
 *
 *  Copyright (C) 2000-2001 Deep Blue Solutions
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright (C) 2006-2007 Pavel Pisa (ppisa@pikron.com)
 *  Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 *  Copyright (C) 2013 Genesi USA, Inc.
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
#include <asm/delay.h>

#include "common.h"
#include "hardware.h"

#define REG_GPTCR		0x00
#define GPTCR_EN		(1 << 0) /* Enable module */
#define GPTCR_WAITEN		(1 << 3) /* Wait enable mode */
#define GPTCR_CLKSRC_IPG	(1 << 6)
#define GPTCR_CLKSRC_IPG_HIGH	(2 << 6)
#define GPTCR_CLKSRC_CLK32K	(4 << 6)
#define GPTCR_FRR		(1 << 9)

#define REG_GPTPR		0x04

#define REG_GPTSR		0x08
#define GPTSR_OF1		(1 << 0)

#define REG_GPTIR		0x0c
#define GPTIR_OF1IE		(1 << 0)

#define REG_GPTOCR1		0x10
#define REG_GPTCNT		0x24


static void __iomem *imx_gpt_base;
static u32 sched_clock_reg;

#define imx_gpt_read(reg)	__raw_readl(imx_gpt_base + (reg))
#define imx_gpt_write(val,reg)	__raw_writel(val, imx_gpt_base + (reg))

static struct clock_event_device imx_gpt_clockevent;
static enum clock_event_mode imx_gpt_clockevent_mode = CLOCK_EVT_MODE_UNUSED;

static u32 notrace imx_gpt_read_sched_clock(void)
{
	return sched_clock_reg ? imx_gpt_read(sched_clock_reg) : 0;
}

static unsigned long imx_gpt_read_current_timer(void)
{
	return imx_gpt_read(REG_GPTCNT);
}

static int __init imx_gpt_clocksource_init(struct clk *timer_clk, bool use_sched_clock)
{
	unsigned int rate = clk_get_rate(timer_clk);

	if (use_sched_clock) {
		sched_clock_reg = REG_GPTCNT;
		setup_sched_clock(imx_gpt_read_sched_clock, 32, rate);
	}

	return clocksource_mmio_init(imx_gpt_base + REG_GPTCNT, "gpt", rate, 200, 32,
			clocksource_mmio_readl_up);
}

/* clock event */
static int imx_gpt_set_next_event(unsigned long evt,
			      struct clock_event_device *unused)
{
	unsigned long tcmp;
	int ret;

	tcmp = imx_gpt_read(REG_GPTCNT) + evt;
	imx_gpt_write(tcmp, REG_GPTOCR1);

	ret = ((tcmp - imx_gpt_read(REG_GPTCNT)) < 0) ? -ETIME : 0;

	return ret;
}

#ifdef DEBUG
static const char *clock_event_mode_label[] = {
	[CLOCK_EVT_MODE_PERIODIC] = "CLOCK_EVT_MODE_PERIODIC",
	[CLOCK_EVT_MODE_ONESHOT]  = "CLOCK_EVT_MODE_ONESHOT",
	[CLOCK_EVT_MODE_SHUTDOWN] = "CLOCK_EVT_MODE_SHUTDOWN",
	[CLOCK_EVT_MODE_UNUSED]   = "CLOCK_EVT_MODE_UNUSED",
	[CLOCK_EVT_MODE_RESUME]   = "CLOCK_EVT_MODE_RESUME",
};
#endif /* DEBUG */

static void imx_gpt_set_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	unsigned long flags;

	/*
	 * The timer interrupt generation is disabled at least
	 * for enough time to call imx_gpt_set_next_event()
	 */
	local_irq_save(flags);

	/* Disable interrupt in GPT module */
	imx_gpt_write(0, REG_GPTIR);

	if (mode != imx_gpt_clockevent_mode) {
		/* Set event time into far-far future */
		u32 tcn = imx_gpt_read(REG_GPTCNT) - 3;
		imx_gpt_write(tcn, REG_GPTOCR1);

		/* Clear pending interrupt */
		imx_gpt_write(GPTSR_OF1, REG_GPTSR);
	}

/* wrapped in DEBUG because clock_event_mode_label is DEBUG-only */
#ifdef DEBUG
	pr_dbg("%s: changing mode from %s to %s\n", __func__,
		clock_event_mode_label[clockevent_mode],
		clock_event_mode_label[mode]);
#endif /* DEBUG */

	/* Remember timer mode */
	imx_gpt_clockevent_mode = mode;
	local_irq_restore(flags);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		pr_err("%s: CLK_EVT_MODE_PERIODIC is not supported for i.MX GPT\n", __func__);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/*
		 * Do not put overhead of irq_enable/disable into
		 * imx_gpt_set_next_event(), the core has about 4 minutes
		 * to call imx_gpt_set_next_event() or shutdown clock after
		 * mode switching
		 */
		local_irq_save(flags);
		imx_gpt_write(GPTIR_OF1IE, REG_GPTIR);
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
static irqreturn_t imx_gpt_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &imx_gpt_clockevent;

	imx_gpt_write(GPTSR_OF1, REG_GPTSR);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction imx_gpt_irq = {
	.name		= "i.MX Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= imx_gpt_timer_interrupt,
};

static struct clock_event_device imx_gpt_clockevent = {
	.name		= "imx_gpt",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= imx_gpt_set_mode,
	.set_next_event	= imx_gpt_set_next_event,
	.rating		= 200,
};

static struct delay_timer imx_gpt_delay_timer = {
	.read_current_timer = imx_gpt_read_current_timer,
};

static int __init imx_gpt_clockevent_init(struct clk *timer_clk)
{
	unsigned int rate = clk_get_rate(timer_clk);

	imx_gpt_clockevent.cpumask = cpumask_of(0);

	clockevents_config_and_register(&imx_gpt_clockevent, rate, 0xff, 0xfffffffe);

	return 0;
}

static const struct of_device_id imx_gpt_of_match[] __initconst = {
	{ .compatible = "fsl,imx6q-gpt", },
	{ .compatible = "fsl,imx51-gpt", },
	{ .compatible = "fsl,imx35-gpt", },
	{},
};

void __init imx_gpt_register(void)
{
	struct device_node *np;
	void __iomem *base;
	int irq;
	struct clk *clk_ipg, *clk_per;
	bool use_sched_clock = false;

	np = of_find_matching_node(NULL, imx_gpt_of_match);

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

	imx_gpt_base = base;

	/*
	 * Initialise to a known state (all timers off, and timing reset)
	 */

	imx_gpt_write(0, REG_GPTCR);
	imx_gpt_write(0, REG_GPTPR); /* see datasheet note */

#define GPTCR_FLAGS	GPTCR_CLKSRC_IPG_HIGH | GPTCR_FRR | GPTCR_WAITEN | GPTCR_EN
	imx_gpt_write(GPTCR_FLAGS, REG_GPTCR);

	if (of_find_property(np, "linux,delay-timer", NULL)) {
		imx_gpt_delay_timer.freq = clk_get_rate(clk_per);
		register_current_timer_delay(&imx_gpt_delay_timer);
	}

	if (of_find_property(np, "linux,scheduler-clock", NULL))
		use_sched_clock = true;

	/* init and register the timer to the framework */
	imx_gpt_clocksource_init(clk_per, use_sched_clock);
	imx_gpt_clockevent_init(clk_per);

	/* Make irqs happen */
	setup_irq(irq, &imx_gpt_irq);
}
