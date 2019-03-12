NAME := arch_linux

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := arch for linux

$(NAME)_SOURCES :=

ifneq ($(osal),posix)
$(NAME)_SOURCES += cpu_impl.c
ifeq ($(PLATFORM),linuxhost)
$(NAME)_SOURCES += swap.S
endif
endif

GLOBAL_INCLUDES += .
