NAME := board_ap80a0

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := configuration for board ap80a0
MODULE             := 1062
HOST_ARCH          := Cortex-M3
HOST_MCU_FAMILY    := mcu_mvs_ap80xx
SUPPORT_MBINS      := no
HOST_MCU_NAME      := mvs_ap80xx
ENABLE_VFP         := 0

$(NAME)_COMPONENTS += $(HOST_MCU_FAMILY) newlib_stub

$(NAME)_SOURCES += config/k_config.c \
                   startup/board.c   \
                   startup/startup.c \
                   drivers/IntProc.c

ifeq ($(COMPILER), armcc)
$(NAME)_SOURCES    += startup/startup_keil.c
$(NAME)_LINK_FILES := startup/startup_keil.o
GLOBAL_LDFLAGS += -L --scatter=board/ap80a0/ap80xx.sct
endif

GLOBAL_INCLUDES += .    \
                   config/   \
                   drivers/  \
                   startup/  \

# Keil project support
$(NAME)_KEIL_VENDOR = ARM
$(NAME)_KEIL_DEVICE = Cortex-M3
