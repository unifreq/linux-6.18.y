// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic SD/eMMC driver for the GX/S905 family SoCs
 *
 * Copyright (c) 2016 BayLibre, SAS.
 * Author: Kevin Hilman <khilman@baylibre.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/bitfield.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define DRIVER_NAME "meson-gx-mmc"

#define SD_EMMC_CLOCK 0x0
#define   CLK_DIV_MASK GENMASK(5, 0)
#define   CLK_SRC_MASK GENMASK(7, 6)
#define   CLK_CORE_PHASE_MASK GENMASK(9, 8)
#define   CLK_TX_PHASE_MASK GENMASK(11, 10)
#define   CLK_RX_PHASE_MASK GENMASK(13, 12)
#define   CLK_PHASE_0 0
#define   CLK_PHASE_180 2
#define   CLK_V2_TX_DELAY_MASK GENMASK(19, 16)
#define   CLK_V2_RX_DELAY_MASK GENMASK(23, 20)
#define   CLK_V2_ALWAYS_ON BIT(24)
#define   CLK_V2_IRQ_SDIO_SLEEP BIT(25)

#define   CLK_V3_TX_DELAY_MASK GENMASK(21, 16)
#define   CLK_V3_RX_DELAY_MASK GENMASK(27, 22)
#define   CLK_V3_ALWAYS_ON BIT(28)
#define   CLK_V3_IRQ_SDIO_SLEEP BIT(29)

#define   CLK_TX_DELAY_MASK(h)		(h->data->tx_delay_mask)
#define   CLK_RX_DELAY_MASK(h)		(h->data->rx_delay_mask)
#define   CLK_ALWAYS_ON(h)		(h->data->always_on)
#define   CLK_IRQ_SDIO_SLEEP(h)		(h->data->irq_sdio_sleep)

#define SD_EMMC_DELAY 0x4
#define SD_EMMC_ADJUST 0x8
#define   ADJUST_ADJ_DELAY_MASK GENMASK(21, 16)
#define   ADJUST_DS_EN BIT(15)
#define   ADJUST_ADJ_EN BIT(13)

#define SD_EMMC_DELAY1 0x4
#define SD_EMMC_DELAY2 0x8
#define SD_EMMC_V3_ADJUST 0xc

#define SD_EMMC_CALOUT 0x10
#define SD_EMMC_START 0x40
#define   START_DESC_INIT BIT(0)
#define   START_DESC_BUSY BIT(1)
#define   START_DESC_ADDR_MASK GENMASK(31, 2)

#define SD_EMMC_CFG 0x44
#define   CFG_BUS_WIDTH_MASK GENMASK(1, 0)
#define   CFG_BUS_WIDTH_1 0x0
#define   CFG_BUS_WIDTH_4 0x1
#define   CFG_BUS_WIDTH_8 0x2
#define   CFG_DDR BIT(2)
#define   CFG_BLK_LEN_MASK GENMASK(7, 4)
#define   CFG_RESP_TIMEOUT_MASK GENMASK(11, 8)
#define   CFG_RC_CC_MASK GENMASK(15, 12)
#define   CFG_STOP_CLOCK BIT(22)
#define   CFG_CLK_ALWAYS_ON BIT(18)
#define   CFG_CHK_DS BIT(20)
#define   CFG_AUTO_CLK BIT(23)
#define   CFG_ERR_ABORT BIT(27)

#define SD_EMMC_STATUS 0x48
#define   STATUS_BUSY BIT(31)
#define   STATUS_DESC_BUSY BIT(30)
#define   STATUS_DATI GENMASK(23, 16)

#define SD_EMMC_IRQ_EN 0x4c
#define   IRQ_RXD_ERR_MASK GENMASK(7, 0)
#define   IRQ_TXD_ERR BIT(8)
#define   IRQ_DESC_ERR BIT(9)
#define   IRQ_RESP_ERR BIT(10)
#define   IRQ_CRC_ERR \
	(IRQ_RXD_ERR_MASK | IRQ_TXD_ERR | IRQ_DESC_ERR | IRQ_RESP_ERR)
#define   IRQ_RESP_TIMEOUT BIT(11)
#define   IRQ_DESC_TIMEOUT BIT(12)
#define   IRQ_TIMEOUTS \
	(IRQ_RESP_TIMEOUT | IRQ_DESC_TIMEOUT)
#define   IRQ_END_OF_CHAIN BIT(13)
#define   IRQ_RESP_STATUS BIT(14)
#define   IRQ_SDIO BIT(15)
#define   IRQ_EN_MASK \
	(IRQ_CRC_ERR | IRQ_TIMEOUTS | IRQ_END_OF_CHAIN)

#define SD_EMMC_CMD_CFG 0x50
#define SD_EMMC_CMD_ARG 0x54
#define SD_EMMC_CMD_DAT 0x58
#define SD_EMMC_CMD_RSP 0x5c
#define SD_EMMC_CMD_RSP1 0x60
#define SD_EMMC_CMD_RSP2 0x64
#define SD_EMMC_CMD_RSP3 0x68

#define SD_EMMC_RXD 0x94
#define SD_EMMC_TXD 0x94
#define SD_EMMC_LAST_REG SD_EMMC_TXD

#define SD_EMMC_SRAM_DATA_BUF_LEN 1536
#define SD_EMMC_SRAM_DATA_BUF_OFF 0x200

#define SD_EMMC_CFG_BLK_SIZE 512 /* internal buffer max: 512 bytes */
#define SD_EMMC_CFG_RESP_TIMEOUT 256 /* in clock cycles */
#define SD_EMMC_CMD_TIMEOUT 1024 /* in ms */
#define SD_EMMC_CMD_TIMEOUT_DATA 4096 /* in ms */
#define SD_EMMC_CMD_TIMEOUT_MAX 32768 /* in ms, 2^15: limit of CMD_CFG_TIMEOUT_MASK */
#define SD_EMMC_CFG_CMD_GAP 16 /* in clock cycles */
#define SD_EMMC_DESC_BUF_LEN PAGE_SIZE

#define SD_EMMC_PRE_REQ_DONE BIT(0)
#define SD_EMMC_DESC_CHAIN_MODE BIT(1)

#define MUX_CLK_NUM_PARENTS 2

#define MESON_MMC_MAX_SLOTS 2
#define MESON_MMC_MAX_SLOT_REGS 3
/* longest request one slot may keep the shared controller busy with */
#define MESON_MMC_SLOT_MAX_REQ_SIZE SZ_64K

/*
 * Sharing of the controller between slots. A slot with an SDIO card
 * moves many small transfers and suffers most from waiting, so it keeps
 * the controller for a time slice while other slots wait. Within the
 * slice the controller stays reserved for it for a short time after each
 * request, until its next request. Memory card slots get one request
 * per turn, and their requests are limited to what the bus moves in a
 * given time at the maximum clock of the slot.
 */
static unsigned int sdio_slice_us = 8000;
module_param(sdio_slice_us, uint, 0644);
MODULE_PARM_DESC(sdio_slice_us,
		 "Multi-slot: time an SDIO slot keeps the controller while other slots wait, in us");

static unsigned int slot_hold_us = 200;
module_param(slot_hold_us, uint, 0644);
MODULE_PARM_DESC(slot_hold_us,
		 "Multi-slot: time the controller stays reserved for a slot within its slice after a request, in us");

static unsigned int slot_req_us = 1000;
module_param(slot_req_us, uint, 0444);
MODULE_PARM_DESC(slot_req_us,
		 "Multi-slot: longest transfer time of one memory card request at the maximum clock, in us (0: no limit)");

struct meson_mmc_data {
	unsigned int tx_delay_mask;
	unsigned int rx_delay_mask;
	unsigned int always_on;
	unsigned int adjust;
	unsigned int irq_sdio_sleep;
	bool has_pipeline_clk;
	/* card timing registers, saved and restored on a slot switch */
	const unsigned int *slot_regs;
	unsigned int num_slot_regs;
};

struct sd_emmc_desc {
	u32 cmd_cfg;
	u32 cmd_arg;
	u32 cmd_data;
	u32 cmd_resp;
};

struct meson_host;

/* Per card slot state, the private data of each mmc_host */
struct meson_slot {
	struct mmc_host *mmc;
	struct meson_host *host;
	unsigned int id;

	unsigned long req_rate;
	bool ddr;

	/*
	 * Controller state of the slot while another slot owns the bus:
	 * module clock rate (0 if never set), clock gating, the slot owned
	 * CFG fields and the timing registers listed in meson_mmc_data.
	 */
	unsigned long clk_rate;
	bool clk_gated;
	u32 cfg;
	u32 regs[MESON_MMC_MAX_SLOT_REGS];

	/* SDIO interrupt enabled by the core, protected by host->lock */
	bool sdio_irq_en;

	/* waits for the controller, protected by host->bus_lock */
	bool bus_waiting;

	/*
	 * Multi-slot GPIO card-detect without an IRQ: poll the GPIO only and
	 * call mmc_detect_change() on edges. Avoids MMC_CAP_NEEDS_POLL, which
	 * would run bus_ops->alive() (CMD13) every second and claim the shared
	 * bus just to see that the card is still there.
	 */
	struct delayed_work cd_gpio_work;
	int cd_gpio_last;
	bool cd_gpio_poll;

	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_clk_gate;
	struct pinctrl_state *pins_sdio_transfer;
	struct pinctrl_state *pins_sdio_transfer_clk_gate;
};

struct meson_host {
	struct	device		*dev;
	const struct meson_mmc_data *data;
	struct	mmc_host	*mmc;
	struct	mmc_command	*cmd;

	/* slot the controller is currently set up for */
	struct meson_slot *cur_slot;

	/*
	 * A controller drives one card slot, or several slots described by
	 * "mmc-slot" child nodes that take turns on the controller. A slot
	 * owns the bus while the controller works for it: from the start of
	 * a request until its completion, which may be in hard IRQ context,
	 * and around the other host operations that touch the controller.
	 * The bus is only used with multiple slots.
	 */
	struct meson_slot *slots[MESON_MMC_MAX_SLOTS];
	unsigned int num_slots;
	bool multi_slot;
	/* protects the bus fields below and slot->bus_waiting */
	spinlock_t bus_lock;
	wait_queue_head_t bus_wq;
	/* slot owning the bus, or NULL */
	struct meson_slot *bus_owner;
	/* last slot that got the bus, its time slice and its reservation */
	struct meson_slot *bus_last;
	ktime_t bus_slice_end;
	ktime_t bus_hold_end;
	/* wakes up the waiters when the reservation ends */
	struct hrtimer bus_timer;
	/* slot running execute_tuning() with the bus held, or NULL */
	struct meson_slot *tuning_slot;
	/* the current request took the bus and releases it when done */
	bool req_owns_bus;
	/* some slot uses the SDIO interrupt */
	bool sdio_irq;

	void __iomem *regs;
	struct clk *mux_clk;
	struct clk *mmc_clk;

	bool dram_access_quirk;

	struct pinctrl *pinctrl;

	unsigned int bounce_buf_size;
	void *bounce_buf;
	void __iomem *bounce_iomem_buf;
	dma_addr_t bounce_dma_addr;
	struct sd_emmc_desc *descs;
	dma_addr_t descs_dma_addr;

	int irq;

	bool needs_pre_post_req;

	spinlock_t lock;
};

static inline struct meson_host *meson_mmc_to_host(struct mmc_host *mmc)
{
	struct meson_slot *slot = mmc_priv(mmc);

	return slot->host;
}

#define CMD_CFG_LENGTH_MASK GENMASK(8, 0)
#define CMD_CFG_BLOCK_MODE BIT(9)
#define CMD_CFG_R1B BIT(10)
#define CMD_CFG_END_OF_CHAIN BIT(11)
#define CMD_CFG_TIMEOUT_MASK GENMASK(15, 12)
#define CMD_CFG_NO_RESP BIT(16)
#define CMD_CFG_NO_CMD BIT(17)
#define CMD_CFG_DATA_IO BIT(18)
#define CMD_CFG_DATA_WR BIT(19)
#define CMD_CFG_RESP_NOCRC BIT(20)
#define CMD_CFG_RESP_128 BIT(21)
#define CMD_CFG_RESP_NUM BIT(22)
#define CMD_CFG_DATA_NUM BIT(23)
#define CMD_CFG_CMD_INDEX_MASK GENMASK(29, 24)
#define CMD_CFG_ERROR BIT(30)
#define CMD_CFG_OWNER BIT(31)

#define CMD_DATA_MASK GENMASK(31, 2)
#define CMD_DATA_BIG_ENDIAN BIT(1)
#define CMD_DATA_SRAM BIT(0)
#define CMD_RESP_MASK GENMASK(31, 1)
#define CMD_RESP_SRAM BIT(0)

static unsigned int meson_mmc_get_timeout_msecs(struct mmc_data *data)
{
	unsigned int timeout = data->timeout_ns / NSEC_PER_MSEC;

	if (!timeout)
		return SD_EMMC_CMD_TIMEOUT_DATA;

	timeout = roundup_pow_of_two(timeout);

	return min_t(unsigned int, timeout, SD_EMMC_CMD_TIMEOUT_MAX);
}

/*
 * For a command without data the controller waits for the response and, for
 * R1b, for the busy signal to be released, within the same timeout. The core
 * tells us how long the card may legitimately stay busy in cmd->busy_timeout;
 * honour it (bounded by the field), otherwise fall back to the default.
 */
static unsigned int meson_mmc_get_cmd_timeout_msecs(struct mmc_command *cmd)
{
	unsigned int timeout = cmd->busy_timeout;

	if (!timeout)
		return SD_EMMC_CMD_TIMEOUT;

	timeout = roundup_pow_of_two(timeout);

	return clamp_t(unsigned int, timeout, SD_EMMC_CMD_TIMEOUT,
		       SD_EMMC_CMD_TIMEOUT_MAX);
}

static struct mmc_command *meson_mmc_get_next_command(struct mmc_command *cmd)
{
	if (cmd->opcode == MMC_SET_BLOCK_COUNT && !cmd->error)
		return cmd->mrq->cmd;
	else if (mmc_op_multi(cmd->opcode) &&
		 (!cmd->mrq->sbc || cmd->error || cmd->data->error))
		return cmd->mrq->stop;
	else
		return NULL;
}

static void meson_mmc_get_transfer_mode(struct mmc_host *mmc,
					struct mmc_request *mrq)
{
	struct meson_host *host = meson_mmc_to_host(mmc);
	struct mmc_data *data = mrq->data;
	struct scatterlist *sg;
	int i;

	/*
	 * When Controller DMA cannot directly access DDR memory, disable
	 * support for Chain Mode to directly use the internal SRAM using
	 * the bounce buffer mode.
	 */
	if (host->dram_access_quirk)
		return;

	/* SD_IO_RW_EXTENDED (CMD53) can also use block mode under the hood */
	if (data->blocks > 1 || mrq->cmd->opcode == SD_IO_RW_EXTENDED) {
		/*
		 * In block mode DMA descriptor format, "length" field indicates
		 * number of blocks and there is no way to pass DMA size that
		 * is not multiple of SDIO block size, making it impossible to
		 * tie more than one memory buffer with single SDIO block.
		 * Block mode sg buffer size should be aligned with SDIO block
		 * size, otherwise chain mode could not be used.
		 */
		for_each_sg(data->sg, sg, data->sg_len, i) {
			if (sg->length % data->blksz) {
				dev_warn_once(mmc_dev(mmc),
					      "unaligned sg len %u blksize %u, disabling descriptor DMA for transfer\n",
					      sg->length, data->blksz);
				return;
			}
		}
	}

	for_each_sg(data->sg, sg, data->sg_len, i) {
		/* check for 8 byte alignment */
		if (sg->offset % 8) {
			dev_warn_once(mmc_dev(mmc),
				      "unaligned sg offset %u, disabling descriptor DMA for transfer\n",
				      sg->offset);
			return;
		}
	}

	data->host_cookie |= SD_EMMC_DESC_CHAIN_MODE;
}

static inline bool meson_mmc_desc_chain_mode(const struct mmc_data *data)
{
	return data->host_cookie & SD_EMMC_DESC_CHAIN_MODE;
}

static inline bool meson_mmc_bounce_buf_read(const struct mmc_data *data)
{
	return data && data->flags & MMC_DATA_READ &&
	       !meson_mmc_desc_chain_mode(data);
}

static void meson_mmc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct meson_host *host = meson_mmc_to_host(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	meson_mmc_get_transfer_mode(mmc, mrq);
	data->host_cookie |= SD_EMMC_PRE_REQ_DONE;

	if (!meson_mmc_desc_chain_mode(data))
		return;

	data->sg_count = dma_map_sg(host->dev, data->sg, data->sg_len,
				    mmc_get_dma_dir(data));
	if (!data->sg_count)
		dev_err(mmc_dev(mmc), "dma_map_sg failed");
}

static void meson_mmc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
			       int err)
{
	struct meson_host *host = meson_mmc_to_host(mmc);
	struct mmc_data *data = mrq->data;

	if (data && meson_mmc_desc_chain_mode(data) && data->sg_count)
		dma_unmap_sg(host->dev, data->sg, data->sg_len,
			     mmc_get_dma_dir(data));
}

/*
 * Gating the clock on this controller is tricky.  It seems the mmc clock
 * is also used by the controller.  It may crash during some operation if the
 * clock is stopped.  The safest thing to do, whenever possible, is to keep
 * clock running at stop it at the pad using the pinmux.
 */
static void meson_mmc_clk_gate(struct meson_slot *slot)
{
	struct meson_host *host = slot->host;
	u32 cfg;

	slot->clk_gated = true;

	if (slot->pins_clk_gate) {
		pinctrl_select_state(host->pinctrl, slot->pins_clk_gate);
	} else {
		/*
		 * If the pinmux is not provided - default to the classic and
		 * unsafe method
		 */
		cfg = readl(host->regs + SD_EMMC_CFG);
		cfg |= CFG_STOP_CLOCK;
		writel(cfg, host->regs + SD_EMMC_CFG);
	}
}

static void meson_mmc_clk_ungate(struct meson_slot *slot)
{
	struct meson_host *host = slot->host;
	u32 cfg;

	slot->clk_gated = false;

	if (slot->pins_clk_gate) {
		if (slot->pins_active)
			pinctrl_select_state(host->pinctrl, slot->pins_active);
		else
			pinctrl_select_default_state(host->dev);
	}

	/* Make sure the clock is not stopped in the controller */
	cfg = readl(host->regs + SD_EMMC_CFG);
	cfg &= ~CFG_STOP_CLOCK;
	writel(cfg, host->regs + SD_EMMC_CFG);
}

static int meson_mmc_clk_set(struct meson_slot *slot, unsigned long rate,
			     bool ddr)
{
	struct meson_host *host = slot->host;
	struct mmc_host *mmc = slot->mmc;
	int ret;
	u32 cfg;

	/* Same request - bail-out */
	if (slot->ddr == ddr && slot->req_rate == rate)
		return 0;

	/* stop clock */
	meson_mmc_clk_gate(slot);
	slot->req_rate = 0;
	mmc->actual_clock = 0;

	/* return with clock being stopped */
	if (!rate)
		return 0;

	/* Stop the clock during rate change to avoid glitches */
	cfg = readl(host->regs + SD_EMMC_CFG);
	cfg |= CFG_STOP_CLOCK;
	writel(cfg, host->regs + SD_EMMC_CFG);

	if (ddr) {
		/* DDR modes require higher module clock */
		rate <<= 1;
		cfg |= CFG_DDR;
	} else {
		cfg &= ~CFG_DDR;
	}
	writel(cfg, host->regs + SD_EMMC_CFG);
	slot->ddr = ddr;

	ret = clk_set_rate(host->mmc_clk, rate);
	if (ret) {
		dev_err(host->dev, "Unable to set cfg_div_clk to %lu. ret=%d\n",
			rate, ret);
		return ret;
	}

	slot->clk_rate = rate;
	slot->req_rate = rate;
	mmc->actual_clock = clk_get_rate(host->mmc_clk);

	/* We should report the real output frequency of the controller */
	if (ddr) {
		slot->req_rate >>= 1;
		mmc->actual_clock >>= 1;
	}

	dev_dbg(host->dev, "clk rate: %u Hz\n", mmc->actual_clock);
	if (rate != mmc->actual_clock)
		dev_dbg(host->dev, "requested rate was %lu\n", rate);

	/* (re)start clock */
	meson_mmc_clk_ungate(slot);

	return 0;
}

/*
 * The SD/eMMC IP block has an internal mux and divider used for
 * generating the MMC clock.  Use the clock framework to create and
 * manage these clocks.
 */
static int meson_mmc_clk_init(struct meson_host *host)
{
	struct clk_init_data init = {};
	struct clk_mux *mux;
	struct clk_divider *div;
	char clk_name[32];
	unsigned long f_min;
	int i, ret = 0;
	const char *mux_parent_names[MUX_CLK_NUM_PARENTS];
	const char *clk_parent[1];
	u32 clk_reg;

	/* init SD_EMMC_CLOCK to sane defaults w/min clock rate */
	clk_reg = CLK_ALWAYS_ON(host);
	clk_reg |= CLK_DIV_MASK;
	clk_reg |= FIELD_PREP(CLK_CORE_PHASE_MASK, CLK_PHASE_180);
	clk_reg |= FIELD_PREP(CLK_TX_PHASE_MASK, CLK_PHASE_0);
	clk_reg |= FIELD_PREP(CLK_RX_PHASE_MASK, CLK_PHASE_0);
	if (host->sdio_irq)
		clk_reg |= CLK_IRQ_SDIO_SLEEP(host);
	writel(clk_reg, host->regs + SD_EMMC_CLOCK);

	/* get the mux parents */
	for (i = 0; i < MUX_CLK_NUM_PARENTS; i++) {
		struct clk *clk;
		char name[16];

		snprintf(name, sizeof(name), "clkin%d", i);
		clk = devm_clk_get(host->dev, name);
		if (IS_ERR(clk))
			return dev_err_probe(host->dev, PTR_ERR(clk),
					     "Missing clock %s\n", name);

		mux_parent_names[i] = __clk_get_name(clk);
	}

	/* create the mux */
	mux = devm_kzalloc(host->dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	snprintf(clk_name, sizeof(clk_name), "%s#mux", dev_name(host->dev));
	init.name = clk_name;
	init.ops = &clk_mux_ops;
	init.flags = 0;
	init.parent_names = mux_parent_names;
	init.num_parents = MUX_CLK_NUM_PARENTS;

	mux->reg = host->regs + SD_EMMC_CLOCK;
	mux->shift = __ffs(CLK_SRC_MASK);
	mux->mask = CLK_SRC_MASK >> mux->shift;
	mux->hw.init = &init;

	host->mux_clk = devm_clk_register(host->dev, &mux->hw);
	if (WARN_ON(IS_ERR(host->mux_clk)))
		return PTR_ERR(host->mux_clk);

	/* create the divider */
	div = devm_kzalloc(host->dev, sizeof(*div), GFP_KERNEL);
	if (!div)
		return -ENOMEM;

	snprintf(clk_name, sizeof(clk_name), "%s#div", dev_name(host->dev));
	init.name = clk_name;
	init.ops = &clk_divider_ops;
	init.flags = CLK_SET_RATE_PARENT;
	clk_parent[0] = __clk_get_name(host->mux_clk);
	init.parent_names = clk_parent;
	init.num_parents = 1;

	div->reg = host->regs + SD_EMMC_CLOCK;
	div->shift = __ffs(CLK_DIV_MASK);
	div->width = __builtin_popcountl(CLK_DIV_MASK);
	div->hw.init = &init;
	div->flags = CLK_DIVIDER_ONE_BASED;

	host->mmc_clk = devm_clk_register(host->dev, &div->hw);
	if (WARN_ON(IS_ERR(host->mmc_clk)))
		return PTR_ERR(host->mmc_clk);

	/* init SD_EMMC_CLOCK to sane defaults w/min clock rate */
	f_min = clk_round_rate(host->mmc_clk, 400000);
	for (i = 0; i < host->num_slots; i++)
		host->slots[i]->mmc->f_min = f_min;
	ret = clk_set_rate(host->mmc_clk, f_min);
	if (ret)
		return ret;

	return clk_prepare_enable(host->mmc_clk);
}

/* CFG fields that belong to the card in a slot */
#define CFG_SLOT_MASK \
	(CFG_BUS_WIDTH_MASK | CFG_DDR | CFG_BLK_LEN_MASK | CFG_STOP_CLOCK)

static void meson_mmc_save_slot(struct meson_host *host,
				struct meson_slot *slot)
{
	unsigned int i;

	slot->cfg = readl(host->regs + SD_EMMC_CFG) & CFG_SLOT_MASK;
	for (i = 0; i < host->data->num_slot_regs; i++)
		slot->regs[i] = readl(host->regs + host->data->slot_regs[i]);
}

/*
 * Set the controller up for another slot: pads, clock rate, bus width and
 * timing. The SDIO interrupt of the old slot is masked, and an interrupt
 * enabled for the new slot is reported once since it may have been raised
 * while its pads were disconnected. Called with the bus held.
 */
static void meson_mmc_switch_slot(struct meson_host *host,
				  struct meson_slot *slot)
{
	struct meson_slot *old = host->cur_slot;
	struct pinctrl_state *pins;
	unsigned long flags;
	bool sdio_irq_pending;
	unsigned int i;
	u32 cfg;
	int ret;

	if (old == slot)
		return;

	spin_lock_irqsave(&host->lock, flags);
	host->cur_slot = NULL;
	writel(IRQ_EN_MASK, host->regs + SD_EMMC_IRQ_EN);
	spin_unlock_irqrestore(&host->lock, flags);

	if (old)
		meson_mmc_save_slot(host, old);

	/* Keep the clock stopped while the pads and the divider change */
	cfg = readl(host->regs + SD_EMMC_CFG);
	writel(cfg | CFG_STOP_CLOCK, host->regs + SD_EMMC_CFG);

	pins = slot->clk_gated && slot->pins_clk_gate ? slot->pins_clk_gate :
							 slot->pins_active;
	ret = pinctrl_select_state(host->pinctrl, pins);
	if (ret)
		dev_err(host->dev, "slot %u: pinctrl failed: %d\n", slot->id,
			ret);

	if (slot->clk_rate) {
		ret = clk_set_rate(host->mmc_clk, slot->clk_rate);
		if (ret)
			dev_err(host->dev, "slot %u: clock rate failed: %d\n",
				slot->id, ret);
	}

	for (i = 0; i < host->data->num_slot_regs; i++)
		writel(slot->regs[i], host->regs + host->data->slot_regs[i]);

	cfg &= ~CFG_SLOT_MASK;
	cfg |= slot->cfg;
	writel(cfg, host->regs + SD_EMMC_CFG);

	spin_lock_irqsave(&host->lock, flags);
	writel(IRQ_SDIO, host->regs + SD_EMMC_STATUS);
	host->cur_slot = slot;
	sdio_irq_pending = slot->sdio_irq_en;
	slot->sdio_irq_en = false;
	spin_unlock_irqrestore(&host->lock, flags);

	/* The core enables the interrupt again through ->ack_sdio_irq() */
	if (sdio_irq_pending)
		sdio_signal_irq(slot->mmc);
}

static bool meson_mmc_bus_others_wait(struct meson_host *host,
				      struct meson_slot *slot)
{
	unsigned int i;

	for (i = 0; i < host->num_slots; i++)
		if (host->slots[i] != slot && host->slots[i]->bus_waiting)
			return true;

	return false;
}

/* Called with host->bus_lock held */
static bool meson_mmc_bus_can_take(struct meson_host *host,
				   struct meson_slot *slot)
{
	if (host->bus_owner)
		return false;

	/* The last slot keeps the bus while it is reserved for it */
	if (ktime_before(ktime_get(), host->bus_hold_end))
		return host->bus_last == slot;

	/* Then the other slots go first */
	return host->bus_last != slot ||
	       !meson_mmc_bus_others_wait(host, slot);
}

static enum hrtimer_restart meson_mmc_bus_timer(struct hrtimer *timer)
{
	struct meson_host *host = container_of(timer, struct meson_host,
					       bus_timer);

	wake_up_all(&host->bus_wq);

	return HRTIMER_NORESTART;
}

static unsigned int meson_mmc_slot_slice_us(struct meson_slot *slot)
{
	struct mmc_card *card = slot->mmc->card;

	return card && mmc_card_sdio(card) ? READ_ONCE(sdio_slice_us) : 0;
}

static void meson_mmc_bus_get(struct meson_slot *slot)
{
	struct meson_host *host = slot->host;
	ktime_t now;

	if (!host->multi_slot)
		return;

	spin_lock_irq(&host->bus_lock);
	slot->bus_waiting = true;
	if (host->bus_last != slot &&
	    ktime_before(ktime_get(), host->bus_hold_end))
		hrtimer_start(&host->bus_timer, host->bus_hold_end,
			      HRTIMER_MODE_ABS);
	wait_event_lock_irq(host->bus_wq, meson_mmc_bus_can_take(host, slot),
			    host->bus_lock);
	slot->bus_waiting = false;
	host->bus_owner = slot;
	now = ktime_get();
	if (host->bus_last != slot ||
	    !ktime_before(now, host->bus_slice_end)) {
		host->bus_last = slot;
		host->bus_slice_end = ktime_add_us(now,
						   meson_mmc_slot_slice_us(slot));
	}
	host->bus_hold_end = 0;
	spin_unlock_irq(&host->bus_lock);

	meson_mmc_switch_slot(host, slot);
}

/* May be called from hard IRQ context */
static void meson_mmc_bus_put(struct meson_slot *slot)
{
	struct meson_host *host = slot->host;
	unsigned long flags;
	ktime_t now;

	if (!host->multi_slot)
		return;

	spin_lock_irqsave(&host->bus_lock, flags);
	host->bus_owner = NULL;
	now = ktime_get();
	if (ktime_before(now, host->bus_slice_end)) {
		/* Keep the bus for the next request of the slot */
		host->bus_hold_end = ktime_add_us(now, READ_ONCE(slot_hold_us));
		if (ktime_after(host->bus_hold_end, host->bus_slice_end))
			host->bus_hold_end = host->bus_slice_end;
		if (meson_mmc_bus_others_wait(host, slot))
			hrtimer_start(&host->bus_timer, host->bus_hold_end,
				      HRTIMER_MODE_ABS);
	}
	spin_unlock_irqrestore(&host->bus_lock, flags);

	wake_up_all(&host->bus_wq);
}

static void meson_mmc_disable_resampling(struct meson_host *host)
{
	unsigned int val = readl(host->regs + host->data->adjust);

	val &= ~ADJUST_ADJ_EN;
	writel(val, host->regs + host->data->adjust);
}

static void meson_mmc_reset_resampling(struct meson_host *host)
{
	unsigned int val;

	meson_mmc_disable_resampling(host);

	val = readl(host->regs + host->data->adjust);
	val &= ~ADJUST_ADJ_DELAY_MASK;
	writel(val, host->regs + host->data->adjust);
}

static int __meson_mmc_resampling_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct meson_slot *slot = mmc_priv(mmc);
	struct meson_host *host = slot->host;
	unsigned int val, dly, max_dly, i;
	int ret;

	if (slot->pins_sdio_transfer &&
	    slot->pins_active != slot->pins_sdio_transfer) {
		ret = pinctrl_select_state(host->pinctrl,
					   slot->pins_sdio_transfer);
		if (ret)
			return ret;

		slot->pins_active = slot->pins_sdio_transfer;
		slot->pins_clk_gate = slot->pins_sdio_transfer_clk_gate;

		val = readl(host->regs + host->data->adjust);
		val &= ~ADJUST_ADJ_DELAY_MASK;
		val |= ADJUST_ADJ_EN;
		val |= FIELD_PREP(ADJUST_ADJ_DELAY_MASK, 3);
		writel(val, host->regs + host->data->adjust);
		dev_dbg(host->dev,
			      "switched to SDIO transfer routing with vendor delay 3\n");
		return 0;
	}

	/* Resampling is done using the source clock */
	max_dly = DIV_ROUND_UP(clk_get_rate(host->mux_clk),
			       clk_get_rate(host->mmc_clk));

	val = readl(host->regs + host->data->adjust);
	val |= ADJUST_ADJ_EN;
	writel(val, host->regs + host->data->adjust);

	if (mmc_doing_retune(mmc))
		dly = FIELD_GET(ADJUST_ADJ_DELAY_MASK, val) + 1;
	else
		dly = 0;

	for (i = 0; i < max_dly; i++) {
		val &= ~ADJUST_ADJ_DELAY_MASK;
		val |= FIELD_PREP(ADJUST_ADJ_DELAY_MASK, (dly + i) % max_dly);
		writel(val, host->regs + host->data->adjust);

		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret) {
			dev_dbg(mmc_dev(mmc), "resampling delay: %u\n",
				(dly + i) % max_dly);
			return 0;
		}
	}

	meson_mmc_reset_resampling(host);
	return -EIO;
}

static int meson_mmc_resampling_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct meson_slot *slot = mmc_priv(mmc);
	struct meson_host *host = slot->host;
	int ret;

	/* Hold the bus across all the tuning commands */
	meson_mmc_bus_get(slot);
	WRITE_ONCE(host->tuning_slot, slot);
	ret = __meson_mmc_resampling_tuning(mmc, opcode);
	WRITE_ONCE(host->tuning_slot, NULL);
	meson_mmc_bus_put(slot);

	return ret;
}

static int meson_mmc_prepare_ios_clock(struct meson_slot *slot,
				       struct mmc_ios *ios)
{
	bool ddr;

	switch (ios->timing) {
	case MMC_TIMING_MMC_DDR52:
	case MMC_TIMING_UHS_DDR50:
		ddr = true;
		break;

	default:
		ddr = false;
		break;
	}

	return meson_mmc_clk_set(slot, ios->clock, ddr);
}

static void meson_mmc_check_resampling(struct meson_host *host,
				       struct mmc_ios *ios)
{
	switch (ios->timing) {
	case MMC_TIMING_LEGACY:
	case MMC_TIMING_MMC_HS:
	case MMC_TIMING_SD_HS:
	case MMC_TIMING_MMC_DDR52:
		meson_mmc_disable_resampling(host);
		break;
	}
}

static void meson_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_slot *slot = mmc_priv(mmc);
	struct meson_host *host = slot->host;
	u32 bus_width, val;
	int err;

	meson_mmc_bus_get(slot);

	/*
	 * GPIO regulator, only controls switching between 1v8 and
	 * 3v3, doesn't support MMC_POWER_OFF, MMC_POWER_ON.
	 */
	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		mmc_regulator_disable_vqmmc(mmc);
		msleep(50);

		break;

	case MMC_POWER_UP:
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);

		break;

	case MMC_POWER_ON:
		mmc_regulator_enable_vqmmc(mmc);

		break;
	}

	/* Bus width */
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		bus_width = CFG_BUS_WIDTH_1;
		break;
	case MMC_BUS_WIDTH_4:
		bus_width = CFG_BUS_WIDTH_4;
		break;
	case MMC_BUS_WIDTH_8:
		bus_width = CFG_BUS_WIDTH_8;
		break;
	default:
		dev_err(host->dev, "Invalid ios->bus_width: %u.  Setting to 4.\n",
			ios->bus_width);
		bus_width = CFG_BUS_WIDTH_4;
	}

	val = readl(host->regs + SD_EMMC_CFG);
	val &= ~CFG_BUS_WIDTH_MASK;
	val |= FIELD_PREP(CFG_BUS_WIDTH_MASK, bus_width);
	writel(val, host->regs + SD_EMMC_CFG);

	meson_mmc_check_resampling(host, ios);
	err = meson_mmc_prepare_ios_clock(slot, ios);
	if (err)
		dev_err(host->dev, "Failed to set clock: %d\n,", err);

	dev_dbg(host->dev, "SD_EMMC_CFG:  0x%08x\n", val);

	meson_mmc_bus_put(slot);
}

static void meson_mmc_request_done(struct mmc_host *mmc,
				   struct mmc_request *mrq)
{
	struct meson_slot *slot = mmc_priv(mmc);
	struct meson_host *host = slot->host;

	host->cmd = NULL;
	if (host->needs_pre_post_req)
		meson_mmc_post_req(mmc, mrq, 0);
	if (host->req_owns_bus)
		meson_mmc_bus_put(slot);
	mmc_request_done(mmc, mrq);
}

static void meson_mmc_set_blksz(struct mmc_host *mmc, unsigned int blksz)
{
	struct meson_host *host = meson_mmc_to_host(mmc);
	u32 cfg, blksz_old;

	cfg = readl(host->regs + SD_EMMC_CFG);
	blksz_old = FIELD_GET(CFG_BLK_LEN_MASK, cfg);

	if (!is_power_of_2(blksz))
		dev_err(host->dev, "blksz %u is not a power of 2\n", blksz);

	blksz = ilog2(blksz);

	/* check if block-size matches, if not update */
	if (blksz == blksz_old)
		return;

	dev_dbg(host->dev, "%s: update blk_len %d -> %d\n", __func__,
		blksz_old, blksz);

	cfg &= ~CFG_BLK_LEN_MASK;
	cfg |= FIELD_PREP(CFG_BLK_LEN_MASK, blksz);
	writel(cfg, host->regs + SD_EMMC_CFG);
}

static void meson_mmc_set_response_bits(struct mmc_command *cmd, u32 *cmd_cfg)
{
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			*cmd_cfg |= CMD_CFG_RESP_128;
		*cmd_cfg |= CMD_CFG_RESP_NUM;

		if (!(cmd->flags & MMC_RSP_CRC))
			*cmd_cfg |= CMD_CFG_RESP_NOCRC;

		if (cmd->flags & MMC_RSP_BUSY)
			*cmd_cfg |= CMD_CFG_R1B;
	} else {
		*cmd_cfg |= CMD_CFG_NO_RESP;
	}
}

static void meson_mmc_desc_chain_transfer(struct mmc_host *mmc, u32 cmd_cfg)
{
	struct meson_host *host = meson_mmc_to_host(mmc);
	struct sd_emmc_desc *desc = host->descs;
	struct mmc_data *data = host->cmd->data;
	struct scatterlist *sg;
	u32 start;
	int i;

	if (data->flags & MMC_DATA_WRITE)
		cmd_cfg |= CMD_CFG_DATA_WR;

	if (data->blocks > 1) {
		cmd_cfg |= CMD_CFG_BLOCK_MODE;
		meson_mmc_set_blksz(mmc, data->blksz);
	}

	for_each_sg(data->sg, sg, data->sg_count, i) {
		unsigned int len = sg_dma_len(sg);

		if (data->blocks > 1)
			len /= data->blksz;

		desc[i].cmd_cfg = cmd_cfg;
		desc[i].cmd_cfg |= FIELD_PREP(CMD_CFG_LENGTH_MASK, len);
		if (i > 0)
			desc[i].cmd_cfg |= CMD_CFG_NO_CMD;
		desc[i].cmd_arg = host->cmd->arg;
		desc[i].cmd_resp = 0;
		desc[i].cmd_data = sg_dma_address(sg);
	}
	desc[data->sg_count - 1].cmd_cfg |= CMD_CFG_END_OF_CHAIN;

	dma_wmb(); /* ensure descriptor is written before kicked */
	start = host->descs_dma_addr | START_DESC_BUSY;
	writel(start, host->regs + SD_EMMC_START);
}

/* Copy arbitrary SG data through the word-access-only controller SRAM. */
static void meson_mmc_copy_buffer(struct meson_host *host, struct mmc_data *data,
				  size_t buflen, bool to_buffer)
{
	size_t io_len = ALIGN(buflen, sizeof(u32));
	u32 *staging = host->bounce_buf;
	size_t copied;

	if (WARN_ON_ONCE(buflen > host->bounce_buf_size))
		return;

	if (to_buffer) {
		copied = sg_copy_to_buffer(data->sg, data->sg_len,
					   staging, buflen);
		if (WARN_ON_ONCE(copied != buflen))
			return;

		if (io_len > buflen)
			memset((u8 *)staging + buflen, 0, io_len - buflen);

		__iowrite32_copy(host->bounce_iomem_buf, staging,
				 io_len / sizeof(u32));
		return;
	}

	__ioread32_copy(staging, host->bounce_iomem_buf,
			  io_len / sizeof(u32));

	copied = sg_copy_from_buffer(data->sg, data->sg_len,
				     staging, buflen);
	WARN_ON_ONCE(copied != buflen);
}

static void meson_mmc_start_cmd(struct mmc_host *mmc, struct mmc_command *cmd)
{
	struct meson_host *host = meson_mmc_to_host(mmc);
	struct mmc_data *data = cmd->data;
	u32 cmd_cfg = 0, cmd_data = 0;
	unsigned int xfer_bytes = 0;

	/* Setup descriptors */
	dma_rmb();

	host->cmd = cmd;

	cmd_cfg |= FIELD_PREP(CMD_CFG_CMD_INDEX_MASK, cmd->opcode);
	cmd_cfg |= CMD_CFG_OWNER;  /* owned by CPU */

	meson_mmc_set_response_bits(cmd, &cmd_cfg);

	/*
	 * A hardware busy wait keeps the controller until the card releases
	 * DAT0, which has no useful bound for writes and erases. Without
	 * MMC_CAP_WAIT_WHILE_BUSY the core polls for busy itself, so drop the
	 * busy wait and let the other slots in between the polls.
	 */
	if (host->multi_slot)
		cmd_cfg &= ~CMD_CFG_R1B;

	/* data? */
	if (data) {
		data->bytes_xfered = 0;
		cmd_cfg |= CMD_CFG_DATA_IO;
		cmd_cfg |= FIELD_PREP(CMD_CFG_TIMEOUT_MASK,
				      ilog2(meson_mmc_get_timeout_msecs(data)));

		if (meson_mmc_desc_chain_mode(data)) {
			meson_mmc_desc_chain_transfer(mmc, cmd_cfg);
			return;
		}

		if (data->blocks > 1) {
			cmd_cfg |= CMD_CFG_BLOCK_MODE;
			cmd_cfg |= FIELD_PREP(CMD_CFG_LENGTH_MASK,
					      data->blocks);
			meson_mmc_set_blksz(mmc, data->blksz);
		} else {
			cmd_cfg |= FIELD_PREP(CMD_CFG_LENGTH_MASK, data->blksz);
		}

		xfer_bytes = data->blksz * data->blocks;
		if (data->flags & MMC_DATA_WRITE) {
			cmd_cfg |= CMD_CFG_DATA_WR;
			WARN_ON(xfer_bytes > host->bounce_buf_size);
			if (host->dram_access_quirk)
				meson_mmc_copy_buffer(host, data, xfer_bytes, true);
			else
				sg_copy_to_buffer(data->sg, data->sg_len,
						  host->bounce_buf, xfer_bytes);
			dma_wmb();
		}

		cmd_data = host->bounce_dma_addr & CMD_DATA_MASK;
	} else {
		cmd_cfg |= FIELD_PREP(CMD_CFG_TIMEOUT_MASK,
				      ilog2(meson_mmc_get_cmd_timeout_msecs(cmd)));
	}

	/* Last descriptor */
	cmd_cfg |= CMD_CFG_END_OF_CHAIN;
	writel(cmd_cfg, host->regs + SD_EMMC_CMD_CFG);
	writel(cmd_data, host->regs + SD_EMMC_CMD_DAT);
	writel(0, host->regs + SD_EMMC_CMD_RSP);
	wmb(); /* ensure descriptor is written before kicked */
	writel(cmd->arg, host->regs + SD_EMMC_CMD_ARG);
}

static void meson_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct meson_slot *slot = mmc_priv(mmc);
	struct meson_host *host = slot->host;
	/* During tuning the bus is already held for this slot */
	bool get_bus = READ_ONCE(host->tuning_slot) != slot;

	if (get_bus)
		meson_mmc_bus_get(slot);
	host->req_owns_bus = get_bus;

	host->needs_pre_post_req = mrq->data &&
			!(mrq->data->host_cookie & SD_EMMC_PRE_REQ_DONE);

	if (host->needs_pre_post_req) {
		meson_mmc_get_transfer_mode(mmc, mrq);
		if (!meson_mmc_desc_chain_mode(mrq->data))
			host->needs_pre_post_req = false;
	}

	if (host->needs_pre_post_req)
		meson_mmc_pre_req(mmc, mrq);

	/* Stop execution */
	writel(0, host->regs + SD_EMMC_START);

	meson_mmc_start_cmd(mmc, mrq->sbc ?: mrq->cmd);
}

static void meson_mmc_read_resp(struct mmc_host *mmc, struct mmc_command *cmd)
{
	struct meson_host *host = meson_mmc_to_host(mmc);

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[0] = readl(host->regs + SD_EMMC_CMD_RSP3);
		cmd->resp[1] = readl(host->regs + SD_EMMC_CMD_RSP2);
		cmd->resp[2] = readl(host->regs + SD_EMMC_CMD_RSP1);
		cmd->resp[3] = readl(host->regs + SD_EMMC_CMD_RSP);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = readl(host->regs + SD_EMMC_CMD_RSP);
	}
}

/* Called with host->lock held */
static void __meson_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct meson_slot *slot = mmc_priv(mmc);
	struct meson_host *host = slot->host;
	u32 reg_irqen = IRQ_EN_MASK;

	slot->sdio_irq_en = enable;

	/* Applied by meson_mmc_switch_slot() once the slot owns the bus */
	if (host->cur_slot != slot)
		return;

	if (enable)
		reg_irqen |= IRQ_SDIO;
	writel(reg_irqen, host->regs + SD_EMMC_IRQ_EN);
}

static irqreturn_t meson_mmc_irq(int irq, void *dev_id)
{
	struct meson_host *host = dev_id;
	struct meson_slot *slot;
	struct mmc_command *cmd;
	struct mmc_host *mmc;
	u32 status, raw_status, irq_mask = IRQ_EN_MASK;
	irqreturn_t ret = IRQ_NONE;

	if (host->sdio_irq)
		irq_mask |= IRQ_SDIO;
	raw_status = readl(host->regs + SD_EMMC_STATUS);
	status = raw_status & irq_mask;

	if (!status) {
		dev_dbg(host->dev,
			"Unexpected IRQ! irq_en 0x%08x - status 0x%08x\n",
			 irq_mask, raw_status);
		return IRQ_NONE;
	}

	/* ack all raised interrupts */
	writel(status, host->regs + SD_EMMC_STATUS);

	cmd = host->cmd;

	if (status & IRQ_SDIO) {
		spin_lock(&host->lock);
		/* No slot while switching: the new slot is told afterwards */
		slot = host->cur_slot;
		if (slot && slot->mmc->caps & MMC_CAP_SDIO_IRQ) {
			__meson_mmc_enable_sdio_irq(slot->mmc, 0);
			sdio_signal_irq(slot->mmc);
		}
		spin_unlock(&host->lock);
		status &= ~IRQ_SDIO;
		if (!status)
			return IRQ_HANDLED;
	}

	if (WARN_ON(!cmd))
		return IRQ_NONE;

	/* A request holds the bus, so its slot stays current */
	mmc = host->cur_slot->mmc;

	cmd->error = 0;
	if (status & IRQ_CRC_ERR) {
		dev_dbg(host->dev, "CRC Error - status 0x%08x\n", status);
		cmd->error = -EILSEQ;
		ret = IRQ_WAKE_THREAD;
		goto out;
	}

	if (status & IRQ_TIMEOUTS) {
		dev_dbg(host->dev, "Timeout - status 0x%08x\n", status);
		cmd->error = -ETIMEDOUT;
		ret = IRQ_WAKE_THREAD;
		goto out;
	}

	meson_mmc_read_resp(mmc, cmd);

	if (status & (IRQ_END_OF_CHAIN | IRQ_RESP_STATUS)) {
		struct mmc_data *data = cmd->data;

		if (data && !cmd->error)
			data->bytes_xfered = data->blksz * data->blocks;

		/*
		 * Waking the IRQ thread costs a scheduling round trip per
		 * SDIO transaction, which a busy RT task such as the schedutil
		 * worker can stretch to a millisecond. Successful terminal
		 * CMD52/CMD53 requests need no sleeping work when there is no
		 * bounce read: DMA unmapping and mmc_request_done() are
		 * IRQ-safe. Do not touch host/cmd after completing the request.
		 * Errors, bounce copies, command chains and hosts using the
		 * SRAM bounce buffer stay in the thread.
		 */
		if (!host->dram_access_quirk &&
		    (cmd->opcode == SD_IO_RW_DIRECT ||
		     cmd->opcode == SD_IO_RW_EXTENDED) &&
		    (!data || !data->error) &&
		    !meson_mmc_bounce_buf_read(data) &&
		    !meson_mmc_get_next_command(cmd)) {
			meson_mmc_request_done(mmc, cmd->mrq);
			return IRQ_HANDLED;
		}

		return IRQ_WAKE_THREAD;
	}

out:
	if (cmd->error) {
		/* Stop desc in case of errors */
		u32 start = readl(host->regs + SD_EMMC_START);

		start &= ~START_DESC_BUSY;
		writel(start, host->regs + SD_EMMC_START);
	}

	return ret;
}

static int meson_mmc_wait_desc_stop(struct meson_host *host)
{
	u32 status;

	/*
	 * It may sometimes take a while for it to actually halt. Here, we
	 * are giving it 5ms to comply
	 *
	 * If we don't confirm the descriptor is stopped, it might raise new
	 * IRQs after we have called mmc_request_done() which is bad.
	 */

	return readl_poll_timeout(host->regs + SD_EMMC_STATUS, status,
				  !(status & (STATUS_BUSY | STATUS_DESC_BUSY)),
				  100, 5000);
}

static irqreturn_t meson_mmc_irq_thread(int irq, void *dev_id)
{
	struct meson_host *host = dev_id;
	struct mmc_command *next_cmd, *cmd = host->cmd;
	struct mmc_data *data;
	unsigned int xfer_bytes;
	struct mmc_host *mmc;

	if (WARN_ON(!cmd))
		return IRQ_NONE;

	mmc = host->cur_slot->mmc;

	if (cmd->error) {
		meson_mmc_wait_desc_stop(host);
		meson_mmc_request_done(mmc, cmd->mrq);

		return IRQ_HANDLED;
	}

	data = cmd->data;
	if (meson_mmc_bounce_buf_read(data)) {
		xfer_bytes = data->blksz * data->blocks;
		WARN_ON(xfer_bytes > host->bounce_buf_size);
		if (host->dram_access_quirk)
			meson_mmc_copy_buffer(host, data, xfer_bytes, false);
		else
			sg_copy_from_buffer(data->sg, data->sg_len,
					    host->bounce_buf, xfer_bytes);
	}

	next_cmd = meson_mmc_get_next_command(cmd);
	if (next_cmd)
		meson_mmc_start_cmd(mmc, next_cmd);
	else
		meson_mmc_request_done(mmc, cmd->mrq);

	return IRQ_HANDLED;
}

static void meson_mmc_cfg_init(struct meson_host *host)
{
	u32 cfg = 0;

	cfg |= FIELD_PREP(CFG_RESP_TIMEOUT_MASK,
			  ilog2(SD_EMMC_CFG_RESP_TIMEOUT));
	cfg |= FIELD_PREP(CFG_RC_CC_MASK, ilog2(SD_EMMC_CFG_CMD_GAP));
	cfg |= FIELD_PREP(CFG_BLK_LEN_MASK, ilog2(SD_EMMC_CFG_BLK_SIZE));

	/* abort chain on R/W errors */
	cfg |= CFG_ERR_ABORT;

	writel(cfg, host->regs + SD_EMMC_CFG);
}

static int meson_mmc_card_busy(struct mmc_host *mmc)
{
	struct meson_slot *slot = mmc_priv(mmc);
	struct meson_host *host = slot->host;
	u32 regval;

	meson_mmc_bus_get(slot);
	regval = readl(host->regs + SD_EMMC_STATUS);
	meson_mmc_bus_put(slot);

	/* We are only interrested in lines 0 to 3, so mask the other ones */
	return !(FIELD_GET(STATUS_DATI, regval) & 0xf);
}

static int __meson_mmc_voltage_switch(struct mmc_host *mmc,
				      struct mmc_ios *ios)
{
	int ret;

	/* vqmmc regulator is available */
	if (!IS_ERR(mmc->supply.vqmmc)) {
		/*
		 * The usual amlogic setup uses a GPIO to switch from one
		 * regulator to the other. While the voltage ramp up is
		 * pretty fast, care must be taken when switching from 3.3v
		 * to 1.8v. Please make sure the regulator framework is aware
		 * of your own regulator constraints
		 */
		ret = mmc_regulator_set_vqmmc(mmc, ios);
		return ret < 0 ? ret : 0;
	}

	/* no vqmmc regulator, assume fixed regulator at 3/3.3V */
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		return 0;

	return -EINVAL;
}

static int meson_mmc_voltage_switch(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_slot *slot = mmc_priv(mmc);
	int ret;

	meson_mmc_bus_get(slot);
	ret = __meson_mmc_voltage_switch(mmc, ios);
	meson_mmc_bus_put(slot);

	return ret;
}

static void meson_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct meson_host *host = meson_mmc_to_host(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	__meson_mmc_enable_sdio_irq(mmc, enable);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void meson_mmc_ack_sdio_irq(struct mmc_host *mmc)
{
	meson_mmc_enable_sdio_irq(mmc, 1);
}

static const struct mmc_host_ops meson_mmc_ops = {
	.request	= meson_mmc_request,
	.set_ios	= meson_mmc_set_ios,
	.get_cd         = mmc_gpio_get_cd,
	.pre_req	= meson_mmc_pre_req,
	.post_req	= meson_mmc_post_req,
	.execute_tuning = meson_mmc_resampling_tuning,
	.card_busy	= meson_mmc_card_busy,
	.start_signal_voltage_switch = meson_mmc_voltage_switch,
	.enable_sdio_irq = meson_mmc_enable_sdio_irq,
	.ack_sdio_irq	= meson_mmc_ack_sdio_irq,
};

static int meson_mmc_slot_init(struct meson_host *host, struct mmc_host *mmc,
			       unsigned int id)
{
	struct meson_slot *slot = mmc_priv(mmc);
	int ret;

	slot->mmc = mmc;
	slot->host = host;
	slot->id = id;

	/* Get regulators and the supported OCR mask */
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		return ret;

	ret = mmc_of_parse(mmc);
	if (ret)
		return dev_err_probe(mmc_dev(mmc), ret, "error parsing DT\n");

	mmc->caps |= MMC_CAP_CMD23;

	/*
	 * The controller waits for R1b busy in hardware, but only up to the
	 * CMD_CFG timeout field. Tell the core the limit so that longer erase
	 * and cache-flush waits are done with R1 + CMD13 polling instead of
	 * timing out here.
	 */
	mmc->max_busy_timeout = SD_EMMC_CMD_TIMEOUT_MAX;

	if (mmc->caps & MMC_CAP_SDIO_IRQ) {
		mmc->caps2 |= MMC_CAP2_SDIO_IRQ_NOTHREAD;
		host->sdio_irq = true;
	}

	host->slots[host->num_slots++] = slot;

	return 0;
}

static void meson_mmc_slot_free(void *data)
{
	mmc_free_host(data);
}

static void meson_mmc_slot_destroy(void *data)
{
	struct platform_device *slot_pdev = data;

	of_platform_device_destroy(&slot_pdev->dev, NULL);
}

static int meson_mmc_add_slot(struct meson_host *host, struct device_node *np)
{
	struct platform_device *slot_pdev;
	struct mmc_host *mmc;
	unsigned int i;
	u32 id;
	int ret;

	if (host->num_slots == MESON_MMC_MAX_SLOTS)
		return dev_err_probe(host->dev, -EINVAL, "too many slots\n");

	ret = of_property_read_u32(np, "reg", &id);
	if (ret || id >= MESON_MMC_MAX_SLOTS)
		return dev_err_probe(host->dev, -EINVAL,
				     "%pOF: invalid slot number\n", np);

	for (i = 0; i < host->num_slots; i++)
		if (host->slots[i]->id == id)
			return dev_err_probe(host->dev, -EINVAL,
					     "%pOF: duplicate slot\n", np);

	slot_pdev = of_platform_device_create(np, NULL, host->dev);
	if (!slot_pdev)
		return dev_err_probe(host->dev, -ENODEV,
				     "%pOF: failed to create slot device\n",
				     np);

	ret = devm_add_action_or_reset(host->dev, meson_mmc_slot_destroy,
				       slot_pdev);
	if (ret)
		return ret;

	mmc = mmc_alloc_host(sizeof(struct meson_slot), &slot_pdev->dev);
	if (!mmc)
		return -ENOMEM;

	ret = devm_add_action_or_reset(host->dev, meson_mmc_slot_free, mmc);
	if (ret)
		return ret;

	return meson_mmc_slot_init(host, mmc, id);
}

/*
 * Without "mmc-slot" children the controller node describes the only slot,
 * and its mmc_host belongs to the controller device. Otherwise each
 * enabled child gets a platform device and an mmc_host of its own.
 */
static void meson_mmc_bus_timer_cancel(void *data)
{
	struct meson_host *host = data;

	hrtimer_cancel(&host->bus_timer);
}

static int meson_mmc_alloc_slots(struct meson_host *host)
{
	struct device_node *np = host->dev->of_node;
	struct mmc_host *mmc;
	bool has_slots = false;
	int ret;

	for_each_child_of_node_scoped(np, child) {
		if (!of_device_is_compatible(child, "mmc-slot"))
			continue;

		has_slots = true;
		if (!of_device_is_available(child))
			continue;

		ret = meson_mmc_add_slot(host, child);
		if (ret)
			return ret;
	}

	if (has_slots) {
		if (!host->num_slots)
			return dev_err_probe(host->dev, -ENODEV,
					     "no enabled slot\n");

		host->multi_slot = true;
		host->mmc = host->slots[0]->mmc;
		spin_lock_init(&host->bus_lock);
		init_waitqueue_head(&host->bus_wq);
		hrtimer_setup(&host->bus_timer, meson_mmc_bus_timer,
			      CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
		return devm_add_action_or_reset(host->dev,
						meson_mmc_bus_timer_cancel,
						host);
	}

	mmc = devm_mmc_alloc_host(host->dev, sizeof(struct meson_slot));
	if (!mmc)
		return -ENOMEM;

	host->mmc = mmc;
	host->cur_slot = mmc_priv(mmc);

	return meson_mmc_slot_init(host, mmc, 0);
}

static int meson_mmc_get_pins(struct meson_host *host)
{
	struct meson_slot *slot = host->slots[0];
	unsigned int i;
	char name[24];

	host->pinctrl = devm_pinctrl_get(host->dev);
	if (IS_ERR(host->pinctrl))
		return PTR_ERR(host->pinctrl);

	if (host->multi_slot) {
		for (i = 0; i < host->num_slots; i++) {
			slot = host->slots[i];

			snprintf(name, sizeof(name), "slot%u", slot->id);
			slot->pins_active = pinctrl_lookup_state(host->pinctrl,
								 name);
			if (IS_ERR(slot->pins_active))
				return dev_err_probe(host->dev,
						     PTR_ERR(slot->pins_active),
						     "missing pinctrl state %s\n",
						     name);

			snprintf(name, sizeof(name), "slot%u-clk-gate",
				 slot->id);
			slot->pins_clk_gate = pinctrl_lookup_state(host->pinctrl,
								   name);
			if (IS_ERR(slot->pins_clk_gate)) {
				dev_warn(host->dev,
					 "can't get %s pinctrl, using clk_stop bit\n",
					 name);
				slot->pins_clk_gate = NULL;
			}
		}

		return 0;
	}

	slot->pins_clk_gate = pinctrl_lookup_state(host->pinctrl,
						   "clk-gate");
	if (IS_ERR(slot->pins_clk_gate)) {
		dev_warn(host->dev,
			 "can't get clk-gate pinctrl, using clk_stop bit\n");
		slot->pins_clk_gate = NULL;
	}

	slot->pins_active = pinctrl_lookup_state(host->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(slot->pins_active))
		slot->pins_active = NULL;

	slot->pins_sdio_transfer = pinctrl_lookup_state(host->pinctrl,
							"sdio-transfer");
	slot->pins_sdio_transfer_clk_gate =
		pinctrl_lookup_state(host->pinctrl,
				     "sdio-transfer-clk-gate");
	if (IS_ERR(slot->pins_sdio_transfer) ||
	    IS_ERR(slot->pins_sdio_transfer_clk_gate)) {
		slot->pins_sdio_transfer = NULL;
		slot->pins_sdio_transfer_clk_gate = NULL;
	}

	return 0;
}

/*
 * Request size limit of a slot in multi-slot mode: what the bus moves in
 * slot_req_us at the maximum clock and bus width of the slot, as a power
 * of two between one block and MESON_MMC_SLOT_MAX_REQ_SIZE. Non-removable
 * slots with SDIO interrupts are only bound by MESON_MMC_SLOT_MAX_REQ_SIZE:
 * their requests are packets, and they own the bus for time slices.
 */
static unsigned int meson_mmc_slot_max_req(struct mmc_host *mmc)
{
	unsigned int width = 1;
	u64 bytes;

	if (!slot_req_us || !mmc->f_max ||
	    (mmc->caps & MMC_CAP_NONREMOVABLE &&
	     mmc->caps & MMC_CAP_SDIO_IRQ))
		return MESON_MMC_SLOT_MAX_REQ_SIZE;

	if (mmc->caps & MMC_CAP_8_BIT_DATA)
		width = 8;
	else if (mmc->caps & MMC_CAP_4_BIT_DATA)
		width = 4;

	bytes = div_u64((u64)mmc->f_max / 8 * width * slot_req_us,
			USEC_PER_SEC);
	bytes = clamp_t(u64, bytes, 512, MESON_MMC_SLOT_MAX_REQ_SIZE);

	return rounddown_pow_of_two((unsigned long)bytes);
}

static void meson_mmc_init_slot(struct meson_host *host,
				struct meson_slot *slot)
{
	struct mmc_host *mmc = slot->mmc;

	if (host->dram_access_quirk) {
		/* Limit segments to 1 due to low available sram memory */
		mmc->max_segs = 1;
		/* Limit to the available sram memory */
		mmc->max_blk_count = SD_EMMC_SRAM_DATA_BUF_LEN /
				     mmc->max_blk_size;
	} else {
		mmc->max_blk_count = CMD_CFG_LENGTH_MASK;
		mmc->max_segs = SD_EMMC_DESC_BUF_LEN /
				sizeof(struct sd_emmc_desc);
	}

	/*
	 * The other slots wait for the controller while a request runs, so
	 * bound its size, and with it the time a slot holds the bus.
	 */
	if (host->multi_slot)
		mmc->max_blk_count = min_t(unsigned int, mmc->max_blk_count,
					   meson_mmc_slot_max_req(mmc) /
					   mmc->max_blk_size);
	mmc->max_req_size = mmc->max_blk_count * mmc->max_blk_size;
	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * At the moment, we don't know how to reliably enable HS400.
	 * From the different datasheets, it is not even clear if this mode
	 * is officially supported by any of the SoCs
	 */
	mmc->caps2 &= ~MMC_CAP2_HS400;

	mmc->ops = &meson_mmc_ops;

	/* Every slot starts from the controller defaults */
	meson_mmc_save_slot(host, slot);
}

/*
 * On a multi-slot controller the platform CD IRQ (optional IRQ 1) belongs to
 * the controller as a whole and cannot be bound to one slot's mmc_host, so
 * meson_mmc_probe() does not call mmc_gpio_set_cd_irq() for multi-slot.
 * mmc_gpiod_request_cd_irq() then tries gpiod_to_irq() on the slot's cd-gpios
 * line. On Meson G12A that mapping is unavailable for the SD slot CD GPIO
 * (GPIOC_6 on the usual u200-derived boards): the periphs GPIO irqchip does
 * not expose a usable IRQ for that line, request fails, and the core sets
 * MMC_CAP_NEEDS_POLL.
 *
 * NEEDS_POLL rescan calls bus_ops->detect() -> alive() (CMD13) every second
 * while a card is present. Each alive() takes the shared bus and switches
 * pinmux away from the SDIO slot, which wrecks Wi-Fi throughput when the SD
 * card is only sitting idle in the slot. With a working GPIO CD, presence is
 * already known without touching the bus: clear NEEDS_POLL and poll the GPIO
 * alone; only a level change schedules mmc_detect_change(). Real SD I/O still
 * goes through meson_mmc_bus_get() and the time arbiter as before.
 */
static void meson_mmc_cd_gpio_work(struct work_struct *work)
{
	struct meson_slot *slot = container_of(work, struct meson_slot,
					       cd_gpio_work.work);
	struct mmc_host *mmc = slot->mmc;
	int cd;

	if (mmc->rescan_disable)
		return;

	cd = mmc_gpio_get_cd(mmc);
	if (cd >= 0 && cd != slot->cd_gpio_last) {
		slot->cd_gpio_last = cd;
		mmc_detect_change(mmc, msecs_to_jiffies(200));
	} else if (cd > 0 && !mmc->card) {
		/*
		 * CD says present but the card never attached (e.g. the
		 * first probe raced the other slot). Retry init without
		 * going through the NEEDS_POLL alive()/CMD13 path.
		 */
		mmc_detect_change(mmc, 0);
	}

	schedule_delayed_work(&slot->cd_gpio_work, HZ);
}

static void meson_mmc_cd_gpio_poll_stop(void *data)
{
	struct meson_slot *slot = data;

	cancel_delayed_work_sync(&slot->cd_gpio_work);
}

static int meson_mmc_cd_gpio_poll_start(struct meson_slot *slot)
{
	struct mmc_host *mmc = slot->mmc;
	struct meson_host *host = slot->host;
	int cd, ret;

	if (!host->multi_slot)
		return 0;

	/* Non-removable slots (e.g. SDIO Wi-Fi) never need CD polling. */
	if (mmc->caps & MMC_CAP_NONREMOVABLE)
		return 0;

	/* No GPIO CD: leave the core's NEEDS_POLL / broken-cd behaviour. */
	cd = mmc_gpio_get_cd(mmc);
	if (cd < 0)
		return 0;

	/* IRQ CD already works: nothing to do. */
	if (mmc->slot.cd_irq >= 0)
		return 0;

	/*
	 * GPIO CD without IRQ: stop CMD13 polling on the shared bus and
	 * watch the GPIO instead.
	 */
	mmc->caps &= ~MMC_CAP_NEEDS_POLL;
	/*
	 * mmc_start_host() already queued an initial detect. Flush it so the
	 * card can attach; with NEEDS_POLL cleared it will not reschedule.
	 * cancel_delayed_work_sync() would drop that first probe.
	 */
	flush_delayed_work(&mmc->detect);

	slot->cd_gpio_last = mmc_gpio_get_cd(mmc);
	if (slot->cd_gpio_last < 0)
		slot->cd_gpio_poll = true;
	INIT_DELAYED_WORK(&slot->cd_gpio_work, meson_mmc_cd_gpio_work);
	ret = devm_add_action_or_reset(host->dev, meson_mmc_cd_gpio_poll_stop,
				       slot);
	if (ret)
		return ret;

	schedule_delayed_work(&slot->cd_gpio_work, HZ);
	dev_info(mmc_dev(mmc),
		 "multi-slot: GPIO CD poll without bus claims (no CD IRQ)\n");
	return 0;
}

static int meson_mmc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct meson_host *host;
	struct mmc_host *mmc;
	struct clk *core_clk;
	unsigned int i;
	int cd_irq = -ENXIO;
	int ret;

	host = devm_kzalloc(&pdev->dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	host->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, host);
	spin_lock_init(&host->lock);

	host->data = of_device_get_match_data(&pdev->dev);
	if (!host->data)
		return -EINVAL;

	/* The G12A SDIO Controller needs an SRAM bounce buffer */
	host->dram_access_quirk =
		device_property_read_bool(&pdev->dev, "amlogic,dram-access-quirk");

	ret = meson_mmc_alloc_slots(host);
	if (ret)
		return ret;

	ret = device_reset_optional(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "device reset failed\n");

	host->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(host->regs))
		return PTR_ERR(host->regs);

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0)
		return host->irq;

	/*
	 * The optional platform CD IRQ is one line for the whole controller.
	 * With several slots it cannot be attributed to one mmc_host; GPIO CD
	 * is handled per slot after mmc_add_host() (see
	 * meson_mmc_cd_gpio_poll_start()).
	 */
	if (!host->multi_slot) {
		ret = platform_get_irq_optional(pdev, 1);
		if (ret < 0 && ret != -ENXIO)
			return ret;
		if (ret > 0)
			cd_irq = ret;
		mmc_gpio_set_cd_irq(host->mmc, cd_irq);
	}

	ret = meson_mmc_get_pins(host);
	if (ret)
		return ret;

	core_clk = devm_clk_get_enabled(&pdev->dev, "core");
	if (IS_ERR(core_clk))
		return PTR_ERR(core_clk);

	if (host->data->has_pipeline_clk) {
		struct clk *pipe_clk;

		pipe_clk = devm_clk_get_enabled(&pdev->dev, "pipeline");
		if (IS_ERR(pipe_clk))
			return dev_err_probe(&pdev->dev, PTR_ERR(pipe_clk),
					     "missing pipeline clock\n");
	}

	ret = meson_mmc_clk_init(host);
	if (ret)
		return ret;

	/* set config to sane default */
	meson_mmc_cfg_init(host);

	/* Stop execution */
	writel(0, host->regs + SD_EMMC_START);

	/* clear, ack and enable interrupts */
	writel(0, host->regs + SD_EMMC_IRQ_EN);
	writel(IRQ_EN_MASK, host->regs + SD_EMMC_STATUS);
	writel(IRQ_EN_MASK, host->regs + SD_EMMC_IRQ_EN);

	ret = request_threaded_irq(host->irq, meson_mmc_irq,
				   meson_mmc_irq_thread, IRQF_ONESHOT,
				   dev_name(&pdev->dev), host);
	if (ret)
		goto err_init_clk;

	for (i = 0; i < host->num_slots; i++)
		meson_mmc_init_slot(host, host->slots[i]);

	/* All slots share the buffers and have the same limits */
	mmc = host->mmc;

	if (host->dram_access_quirk) {
		/*
		 * The MMC Controller embeds 1,5KiB of internal SRAM
		 * that can be used to be used as bounce buffer.
		 * In the case of the G12A SDIO controller, use these
		 * instead of the DDR memory.  Keep a normal-memory staging
		 * buffer so arbitrary SG layouts never cause unaligned MMIO.
		 */
		host->bounce_buf_size = SD_EMMC_SRAM_DATA_BUF_LEN;
		host->bounce_buf = devm_kzalloc(host->dev,
						 host->bounce_buf_size, GFP_KERNEL);
		if (!host->bounce_buf) {
			ret = -ENOMEM;
			goto err_free_irq;
		}
		host->bounce_iomem_buf = host->regs + SD_EMMC_SRAM_DATA_BUF_OFF;
		host->bounce_dma_addr = res->start + SD_EMMC_SRAM_DATA_BUF_OFF;
	} else {
		/* data bounce buffer */
		host->bounce_buf_size = mmc->max_req_size;
		host->bounce_buf =
			dmam_alloc_coherent(host->dev, host->bounce_buf_size,
					    &host->bounce_dma_addr, GFP_KERNEL);
		if (host->bounce_buf == NULL) {
			dev_err(host->dev, "Unable to map allocate DMA bounce buffer.\n");
			ret = -ENOMEM;
			goto err_free_irq;
		}
	}

	host->descs = dmam_alloc_coherent(host->dev, SD_EMMC_DESC_BUF_LEN,
					  &host->descs_dma_addr, GFP_KERNEL);
	if (!host->descs) {
		dev_err(host->dev, "Allocating descriptor DMA buffer failed\n");
		ret = -ENOMEM;
		goto err_free_irq;
	}

	for (i = 0; i < host->num_slots; i++) {
		ret = mmc_add_host(host->slots[i]->mmc);
		if (ret)
			goto err_remove_hosts;
	}

	for (i = 0; i < host->num_slots; i++) {
		ret = meson_mmc_cd_gpio_poll_start(host->slots[i]);
		if (ret)
			goto err_remove_hosts;
	}

	return 0;

err_remove_hosts:
	while (i--)
		mmc_remove_host(host->slots[i]->mmc);
err_free_irq:
	free_irq(host->irq, host);
err_init_clk:
	clk_disable_unprepare(host->mmc_clk);
	return ret;
}

static void meson_mmc_remove(struct platform_device *pdev)
{
	struct meson_host *host = dev_get_drvdata(&pdev->dev);
	unsigned int i;

	for (i = host->num_slots; i--; )
		mmc_remove_host(host->slots[i]->mmc);

	/* disable interrupts */
	writel(0, host->regs + SD_EMMC_IRQ_EN);
	free_irq(host->irq, host);

	clk_disable_unprepare(host->mmc_clk);
}

static const unsigned int meson_gx_slot_regs[] = {
	SD_EMMC_DELAY, SD_EMMC_ADJUST,
};

static const unsigned int meson_axg_slot_regs[] = {
	SD_EMMC_DELAY1, SD_EMMC_DELAY2, SD_EMMC_V3_ADJUST,
};

static const struct meson_mmc_data meson_gx_data = {
	.tx_delay_mask	= CLK_V2_TX_DELAY_MASK,
	.rx_delay_mask	= CLK_V2_RX_DELAY_MASK,
	.always_on	= CLK_V2_ALWAYS_ON,
	.adjust		= SD_EMMC_ADJUST,
	.irq_sdio_sleep	= CLK_V2_IRQ_SDIO_SLEEP,
	.slot_regs	= meson_gx_slot_regs,
	.num_slot_regs	= ARRAY_SIZE(meson_gx_slot_regs),
};

static const struct meson_mmc_data meson_axg_data = {
	.tx_delay_mask	= CLK_V3_TX_DELAY_MASK,
	.rx_delay_mask	= CLK_V3_RX_DELAY_MASK,
	.always_on	= CLK_V3_ALWAYS_ON,
	.adjust		= SD_EMMC_V3_ADJUST,
	.irq_sdio_sleep	= CLK_V3_IRQ_SDIO_SLEEP,
	.slot_regs	= meson_axg_slot_regs,
	.num_slot_regs	= ARRAY_SIZE(meson_axg_slot_regs),
};

static const struct meson_mmc_data meson_t7_data = {
	.tx_delay_mask	= CLK_V3_TX_DELAY_MASK,
	.rx_delay_mask	= CLK_V3_RX_DELAY_MASK,
	.always_on	= CLK_V3_ALWAYS_ON,
	.adjust		= SD_EMMC_V3_ADJUST,
	.irq_sdio_sleep	= CLK_V3_IRQ_SDIO_SLEEP,
	.has_pipeline_clk = true,
};

static const struct of_device_id meson_mmc_of_match[] = {
	{ .compatible = "amlogic,meson-gx-mmc",		.data = &meson_gx_data },
	{ .compatible = "amlogic,meson-gxbb-mmc", 	.data = &meson_gx_data },
	{ .compatible = "amlogic,meson-gxl-mmc",	.data = &meson_gx_data },
	{ .compatible = "amlogic,meson-gxm-mmc",	.data = &meson_gx_data },
	{ .compatible = "amlogic,meson-axg-mmc",	.data = &meson_axg_data },
	{ .compatible = "amlogic,t7-mmc",		.data = &meson_t7_data },
	{}
};
MODULE_DEVICE_TABLE(of, meson_mmc_of_match);

static struct platform_driver meson_mmc_driver = {
	.probe		= meson_mmc_probe,
	.remove		= meson_mmc_remove,
	.driver		= {
		.name = DRIVER_NAME,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = meson_mmc_of_match,
	},
};

module_platform_driver(meson_mmc_driver);

MODULE_DESCRIPTION("Amlogic S905*/GX*/AXG SD/eMMC driver");
MODULE_AUTHOR("Kevin Hilman <khilman@baylibre.com>");
MODULE_LICENSE("GPL v2");
