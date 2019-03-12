NAME := linkkitapp

$(NAME)_MBINS_TYPE := app
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := linkkitapp

$(NAME)_SOURCES :=   app_entry.c
$(NAME)_COMPONENTS += linkkit_sdk_c

$(NAME)_COMPONENTS += netmgr cjson



# LINKKITAPP_CONFIG_DEPRECATED ?= n
# LINKKITAPP_CONFIG_PRINT_HEAP ?= n
# LINKKITAPP_CONFIG_COMBOAPP ?= n


ifeq ($(AOS_COMP_OTA),y)
$(NAME)_COMPONENTS += ota
GLOBAL_DEFINES += ENABLE_AOS_OTA
endif


ifeq ($(LINKKITAPP_CONFIG_CASE_SOLO),y)
ifeq ($(LINKKITAPP_CONFIG_DEPRECATED),y)
$(NAME)_SOURCES += deprecated/solo.c
GLOBAL_DEFINES += DEPRECATED_LINKKIT
else
$(NAME)_SOURCES += linkkit_example_solo.c
endif
endif

#case cntdown
ifeq ($(LINKKITAPP_CONFIG_CASE_CNTDOWN),y)
ifeq ($(LINKKITAPP_CONFIG_DEPRECATED),y)
$(NAME)_SOURCES += deprecated/cntdown.c
GLOBAL_DEFINES += DEPRECATED_LINKKIT
else
$(NAME)_SOURCES += linkkit_example_cntdown.c
endif
endif

#case sched
ifeq ($(LINKKITAPP_CONFIG_CASE_SCHED),y)
ifeq ($(LINKKITAPP_CONFIG_DEPRECATED),y)
$(NAME)_SOURCES += deprecated/sched.c
GLOBAL_DEFINES += DEPRECATED_LINKKIT
else
$(NAME)_SOURCES += linkkit_example_sched.c
endif
endif

ifeq ($(LINKKITAPP_CONFIG_PRINT_HEAP),y)
$(NAME)_DEFINES += CONFIG_PRINT_HEAP
endif


ifeq ($(AOS_MCU_RDA5981X),y)
GLOBAL_DEFINES += SUPPORT_DEV_AP
endif

ifneq ($(HOST_MCU_FAMILY),mcu_esp8266)
$(NAME)_COMPONENTS  += cli
else
GLOBAL_DEFINES += ESP8266_CHIPSET
endif

ifeq ($(LINKKITAPP_CONFIG_COMBOAPP),y)
ifneq ($(CONFIG_COMP_BZ_EN_AWSS), y)
$(error need enable AWSS in breeze settings first)
endif
$(NAME)_COMPONENTS += breeze breeze_hal
$(NAME)_SOURCES += combo/combo_net.c
GLOBAL_DEFINES += EN_COMBO_NET
GLOBAL_INCLUDES += combo
bz_en_auth := 1
bz_en_awss := 1
ble := 1
endif


GLOBAL_INCLUDES += ./

