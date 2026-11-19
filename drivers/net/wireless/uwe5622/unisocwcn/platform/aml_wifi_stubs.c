// SPDX-License-Identifier: GPL-2.0
/*
 * aml_wifi_stubs.c - Amlogic BSP symbols for mainline Meson kernels
 * Timing: KryptonLee/uwe5621ds-aml (0 / 100ms / 1)
 * Scan: CE uwe5631-aml — reset host without unregistering card
 * Tips: github.com/devmfc/debian-on-amlogic/discussions/259
 */
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/device.h>
#include <linux/version.h>

#define STUB_TAG "aml_wifi_stub: "

#ifndef GPIOD_FLAGS_BIT_NONEXCLUSIVE
#define GPIOD_FLAGS_BIT_NONEXCLUSIVE 0
#endif
#ifndef GPIO_IRQ_HIGH
#define GPIO_IRQ_HIGH 1
#endif
#ifndef GPIO_IRQ_LOW
#define GPIO_IRQ_LOW 0
#endif

static DEFINE_MUTEX(wifi_en_lock);
static struct gpio_desc *wifi_en_gpio;

static int aml_bind_wifi_en_gpio(void)
{
	struct device_node *np;
	struct gpio_desc *desc;

	if (wifi_en_gpio)
		return 0;

	np = of_find_node_by_name(NULL, "sdio-pwrseq");
	if (!np) {
		pr_err(STUB_TAG "sdio-pwrseq node not found\n");
		return -ENODEV;
	}

	desc = fwnode_gpiod_get_index(of_fwnode_handle(np), "reset", 0,
				      GPIOD_ASIS | GPIOD_FLAGS_BIT_NONEXCLUSIVE,
				      "wifi-en");
	of_node_put(np);
	if (IS_ERR(desc)) {
		pr_err(STUB_TAG "reset-gpios get err %ld\n", PTR_ERR(desc));
		return PTR_ERR(desc);
	}

	wifi_en_gpio = desc;
	pr_info(STUB_TAG "bound wifi_en to sdio-pwrseq reset-gpios\n");
	return 0;
}

void extern_wifi_set_enable(int is_on)
{
	int ret;

	mutex_lock(&wifi_en_lock);
	ret = aml_bind_wifi_en_gpio();
	if (ret) {
		mutex_unlock(&wifi_en_lock);
		return;
	}

	if (is_on) {
		gpiod_set_value_cansleep(wifi_en_gpio, 0);
		pr_info(STUB_TAG "power on: reset deasserted\n");
	} else {
		gpiod_set_value_cansleep(wifi_en_gpio, 1);
		pr_info(STUB_TAG "power off: reset asserted\n");
	}
	mutex_unlock(&wifi_en_lock);
}

void extern_bt_set_enable(int is_on)
{
	extern_wifi_set_enable(is_on);
}

static char *sdio_host;
module_param(sdio_host, charp, 0444);
MODULE_PARM_DESC(sdio_host,
		 "Platform device name of the Wi-Fi SDIO host controller or of its slot (default: sd_emmc_a, then sd_emmc_b)");

static unsigned int sdio_f_max;
module_param(sdio_f_max, uint, 0444);
MODULE_PARM_DESC(sdio_f_max,
		 "Limit the SDIO host clock to this rate in Hz on card reset (0: no limit)");

/*
 * Find the mmc_host class device below a controller. A multi-slot
 * controller has one platform device per slot in between, so look one
 * level further down. Prefer a non-removable host, which is where an
 * on-board module sits.
 */
static int aml_match_mmc_host(struct device *dev, void *data)
{
	struct mmc_host **found = data;
	struct mmc_host *host;

	if (!dev->class || strcmp(dev->class->name, "mmc_host"))
		return device_for_each_child(dev, data, aml_match_mmc_host);

	host = container_of(dev, struct mmc_host, class_dev);
	if (!*found || (host->caps & MMC_CAP_NONREMOVABLE &&
			!((*found)->caps & MMC_CAP_NONREMOVABLE)))
		*found = host;
	return 0;
}

static struct mmc_host *aml_find_mmc_host(const char *name)
{
	struct mmc_host *host = NULL;
	struct device *dev;

	dev = bus_find_device_by_name(&platform_bus_type, NULL, name);
	if (!dev)
		return NULL;
	device_for_each_child(dev, &host, aml_match_mmc_host);
	put_device(dev);
	if (!host)
		return NULL;

	pr_info(STUB_TAG "found mmc host via %s (%s)\n",
		name, mmc_hostname(host));
	return host;
}

static struct mmc_host *aml_get_mmc0_host(void)
{
	/*
	 * Depending on the device tree the module sits on sd_emmc_a or on
	 * sd_emmc_b.  Disabled controllers
	 * have no platform device, so probing sd_emmc_a first is safe on both
	 * and never picks sd_emmc_b while it serves the SD card slot.
	 */
	static const char * const names[] = {
		"ffe03000.mmc", "mmc@ffe03000", "ffe05000.mmc", "d0070000.mmc",
	};
	struct mmc_host *host;
	unsigned int i;

	if (sdio_host && *sdio_host)
		return aml_find_mmc_host(sdio_host);

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		host = aml_find_mmc_host(names[i]);
		if (host)
			return host;
	}
	return NULL;
}

void sdio_reinit(void)
{
	struct mmc_host *host;
	int ret;

	host = aml_get_mmc0_host();
	if (!host) {
		pr_err(STUB_TAG "%s: SDIO host not found\n", __func__);
		return;
	}
	if (!host->card) {
		pr_info(STUB_TAG "%s: no card, mmc_detect_change\n", __func__);
		mmc_detect_change(host, msecs_to_jiffies(100));
		msleep(500);
		return;
	}
	/*
	 * SDR104 at the DT max-frequency (200 MHz) is not reliable on every
	 * board; the vendor Android image runs this module at 166 MHz.  The
	 * new limit takes effect when the card is re-initialised below.
	 */
	if (sdio_f_max && sdio_f_max < host->f_max) {
		pr_info(STUB_TAG "%s: limit %s f_max %u -> %u\n", __func__,
			mmc_hostname(host), host->f_max, sdio_f_max);
		host->f_max = sdio_f_max;
	}
	pr_info(STUB_TAG "%s: mmc_hw_reset(%s)\n", __func__, mmc_hostname(host));
	/* mmc_hw_reset needs claimed host */
	if (host->card && host->card->sdio_func[0])
		sdio_claim_host(host->card->sdio_func[0]);
	ret = mmc_hw_reset(host->card);
	if (host->card && host->card->sdio_func[0])
		sdio_release_host(host->card->sdio_func[0]);
	pr_info(STUB_TAG "%s: mmc_hw_reset ret=%d\n", __func__, ret);
	msleep(100);
}

void sdio_clk_always_on(int on)
{
	pr_debug(STUB_TAG "%s(%d): no-op on Meson\n", __func__, on);
}

void sdio_set_max_reqsz(unsigned int size)
{
	pr_debug(STUB_TAG "%s(%u): no-op on Meson\n", __func__, size);
}

int wifi_irq_num(void)
{
	return 0;
}

int wifi_irq_trigger_level(void)
{
	return GPIO_IRQ_HIGH;
}
