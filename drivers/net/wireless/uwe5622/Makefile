# Unisoc UWE5621/5622: firmware path uwe5621ds; Amlogic boards use AML_WIFI_DEVICE_UWE5621
ifneq ($(CONFIG_AW_WIFI_DEVICE_UWE5622)$(CONFIG_RK_WIFI_DEVICE_UWE5622)$(CONFIG_AML_WIFI_DEVICE_UWE5621)$(CFG_AML_WIFI_DEVICE_UWE5621)$(CFG_AML_WIFI_DEVICE_UWE5622),)
obj-y += unisocwcn/
endif
obj-$(CONFIG_WLAN_UWE5622)    += unisocwifi/
obj-$(CONFIG_TTY_OVERY_SDIO)  += tty-sdio/

UNISOCWCN_DIR := $(shell cd $(src)/unisocwcn/ && /bin/pwd)
UNISOC_BSP_INCLUDE := $(UNISOCWCN_DIR)/include
export UNISOC_BSP_INCLUDE

UNISOC_FW_PATH_CONFIG := "/lib/firmware/uwe5621ds/"
export UNISOC_FW_PATH_CONFIG

UNISOC_WIFI_CUS_CONFIG := "/lib/firmware/uwe5621ds/"
export UNISOC_WIFI_CUS_CONFIG
