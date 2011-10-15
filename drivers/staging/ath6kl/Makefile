#------------------------------------------------------------------------------
# Copyright (c) 2004-2010 Atheros Communications Inc.
# All rights reserved.
#
# 
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#
#
# Author(s): ="Atheros"
#------------------------------------------------------------------------------

ccflags-y += -I$(obj)/include
ccflags-y += -I$(obj)/include/common
ccflags-y += -I$(obj)/wlan/include
ccflags-y += -I$(obj)/os/linux/include
ccflags-y += -I$(obj)/os
ccflags-y += -I$(obj)/bmi/include
ccflags-y += -I$(obj)/include/common/AR6002/hw4.0

ifeq ($(CONFIG_AR600x_SD31_XXX),y)
ccflags-y += -DAR600x_SD31_XXX
endif

ifeq ($(CONFIG_AR600x_WB31_XXX),y)
ccflags-y += -DAR600x_WB31_XXX
endif

ifeq ($(CONFIG_AR600x_SD32_XXX),y)
ccflags-y += -DAR600x_SD32_XXX
endif

ifeq ($(CONFIG_AR600x_CUSTOM_XXX),y)
ccflags-y += -DAR600x_CUSTOM_XXX
endif

ifeq ($(CONFIG_ATH6KL_ENABLE_COEXISTENCE),y)
ccflags-y += -DENABLE_COEXISTENCE
endif

ifeq ($(CONFIG_AR600x_DUAL_ANTENNA),y)
ccflags-y += -DAR600x_DUAL_ANTENNA
endif

ifeq ($(CONFIG_AR600x_SINGLE_ANTENNA),y)
ccflags-y += -DAR600x_SINGLE_ANTENNA
endif

ifeq ($(CONFIG_AR600x_BT_QCOM),y)
ccflags-y += -DAR600x_BT_QCOM
endif

ifeq ($(CONFIG_AR600x_BT_CSR),y)
ccflags-y += -DAR600x_BT_CSR
endif

ifeq ($(CONFIG_AR600x_BT_AR3001),y)
ccflags-y += -DAR600x_BT_AR3001
endif

ifeq ($(CONFIG_ATH6KL_HCI_BRIDGE),y)
ccflags-y += -DATH_AR6K_ENABLE_GMBOX
ccflags-y += -DHCI_TRANSPORT_SDIO
ccflags-y += -DSETUPHCI_ENABLED
ccflags-y += -DSETUPBTDEV_ENABLED
ar6000-y += htc2/AR6000/ar6k_gmbox.o
ar6000-y += htc2/AR6000/ar6k_gmbox_hciuart.o
ar6000-y += miscdrv/ar3kconfig.o
ar6000-y += miscdrv/ar3kps/ar3kpsconfig.o
ar6000-y += miscdrv/ar3kps/ar3kpsparser.o
endif

ifeq ($(CONFIG_ATH6KL_CONFIG_GPIO_BT_RESET),y)
ccflags-y += -DATH6KL_CONFIG_GPIO_BT_RESET
endif

ifeq ($(CONFIG_ATH6KL_CFG80211),y)
ccflags-y += -DATH6K_CONFIG_CFG80211
ar6000-y += os/linux/cfg80211.o
endif

ifeq ($(CONFIG_ATH6KL_HTC_RAW_INTERFACE),y)
ccflags-y += -DHTC_RAW_INTERFACE
endif

ifeq ($(CONFIG_ATH6KL_ENABLE_HOST_DEBUG),y)
ccflags-y += -DDEBUG
ccflags-y += -DATH_DEBUG_MODULE
endif

ifeq ($(CONFIG_ATH6KL_ENABLE_TARGET_DEBUG_PRINTS),y)
ccflags-y += -DENABLEUARTPRINT_SET
endif

ifeq ($(CONFIG_ATH6KL_DISABLE_TARGET_DBGLOGS),y)
ccflags-y += -DATH6KL_DISABLE_TARGET_DBGLOGS
endif

ifeq ($(CONFIG_ATH6KL_VIRTUAL_SCATTER_GATHER),y)
ccflags-y += -DATH6KL_CONFIG_HIF_VIRTUAL_SCATTER
endif

ifeq ($(CONFIG_ATH6KL_SKIP_ABI_VERSION_CHECK),y)
ccflags-y += -DATH6KL_SKIP_ABI_VERSION_CHECK
endif

ccflags-y += -DLINUX -DKERNEL_2_6
ccflags-y += -DTCMD
ccflags-y += -DSEND_EVENT_TO_APP
ccflags-y += -DUSER_KEYS
ccflags-y += -DNO_SYNC_FLUSH
ccflags-y += -DHTC_EP_STAT_PROFILING
ccflags-y += -DATH_AR6K_11N_SUPPORT
ccflags-y += -DWAPI_ENABLE
ccflags-y += -DCHECKSUM_OFFLOAD
ccflags-y += -DWLAN_HEADERS
ccflags-y += -DINIT_MODE_DRV_ENABLED
ccflags-y += -DBMIENABLE_SET

## BEGIN: Additions for GT-I5500
ccflags-y += -DANDROID_ENV -D__linux__
ccflags-y += -DTARGET_EUROPA
ccflags-y += -DSOFTMAC_FILE_USED
## END: Additions for GT-I5500

CFLAGS_REMOVE_ioctl.o = -Werror

obj-$(CONFIG_ATH6K_LEGACY) := ar6000.o
ar6000-y += htc2/AR6000/ar6k.o
ar6000-y += htc2/AR6000/ar6k_events.o
ar6000-y += htc2/htc_send.o
ar6000-y += htc2/htc_recv.o
ar6000-y += htc2/htc_services.o
ar6000-y += htc2/htc.o
ar6000-y += bmi/src/bmi.o
ar6000-y += os/linux/ar6000_android.o
ar6000-y += os/linux/ar6000_drv.o
ar6000-y += os/linux/ar6000_raw_if.o
ar6000-y += os/linux/ar6000_pm.o
ar6000-y += os/linux/netbuf.o
ar6000-y += os/linux/wireless_ext.o
ar6000-y += os/linux/ioctl.o
ar6000-y += os/linux/hci_bridge.o
ar6000-y += os/linux/ar6k_pal.o
ar6000-y += miscdrv/common_drv.o
ar6000-y += miscdrv/credit_dist.o
ar6000-y += wmi/wmi.o
ar6000-y += reorder/rcv_aggr.o
ar6000-y += wlan/src/wlan_node.o
ar6000-y += wlan/src/wlan_recv_beacon.o
ar6000-y += wlan/src/wlan_utils.o			 

# ATH_HIF_TYPE := sdio
ccflags-y += -I$(obj)/hif/sdio/linux_sdio/include
ccflags-y += -DSDIO
ar6000-y += hif/sdio/linux_sdio/src/hif.o
ar6000-y += hif/sdio/linux_sdio/src/hif_scatter.o
