NAME := board_hk32f103rb_evb

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := configuration for board hk32f103rb_evb
MODULE             := 1062
HOST_ARCH          := Cortex-M3
HOST_MCU_FAMILY    := mcu_hk32f103
SUPPORT_MBINS      := no
HOST_MCU_NAME      := hk32f103
ENABLE_VFP         := 0

$(NAME)_COMPONENTS += $(HOST_MCU_FAMILY) newlib_stub

$(NAME)_SOURCES += config/k_config.c \
                   startup/board.c   \
                   startup/startup.c \
                   drivers/hk32f1xx_it.c \
                   drivers/hk32f1xx_hal_msp.c

ifeq ($(COMPILER), armcc)
$(NAME)_SOURCES    += startup/startup_hk32f103xb_keil.s
$(NAME)_LINK_FILES := startup/startup_hk32f103xb_keil.o
GLOBAL_LDFLAGS += -L --scatter=board/hk32f103rb_evb/hk32f103rb_evb.sct
endif

GLOBAL_INCLUDES += .    \
                   config/   \
                   drivers/  \
                   startup/  \

# Keil project support
$(NAME)_KEIL_VENDOR = ARM
$(NAME)_KEIL_DEVICE = ARMCM3
