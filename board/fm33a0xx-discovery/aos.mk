NAME := board_fm33a0xx-discovery

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := configuration for board fm33a0xx-discovery
MODULE             := 1062
HOST_ARCH          := Cortex-M0
HOST_MCU_FAMILY    := mcu_fm33a0xx
SUPPORT_MBINS      := no
HOST_MCU_NAME      := FM33A0XX
ENABLE_VFP         := 0

$(NAME)_COMPONENTS += $(HOST_MCU_FAMILY) newlib_stub kernel_init

$(NAME)_SOURCES += config/k_config.c \
				   config/partition_conf.c \
                   startup/board.c   \
                   startup/startup.c

ifeq ($(COMPILER), armcc)
$(NAME)_SOURCES    += startup/startup_keil.s
$(NAME)_LINK_FILES := startup/startup_keil.o
GLOBAL_LDFLAGS += -L --scatter=board/fm33a0xx-discovery/fm33a0.sct
endif

GLOBAL_INCLUDES += .    \
                   config/   \
                   startup/  \

# Keil project support
$(NAME)_KEIL_VENDOR = FMSH
$(NAME)_KEIL_DEVICE = FM33A04XX
