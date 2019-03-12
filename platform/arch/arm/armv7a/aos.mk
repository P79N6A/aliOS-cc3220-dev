NAME := arch_armv7a

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := arch for armv7a

$(NAME)_SOURCES := common/k_vector.c
$(NAME)_SOURCES += common/k_cache.c
$(NAME)_SOURCES += common/k_mmu_sd.c
$(NAME)_SOURCES += common/panic.c
$(NAME)_SOURCES += common/port_c.c
GLOBAL_INCLUDES += common

ifeq ($(COMPILER),armcc)

GLOBAL_ASMFLAGS += --cpreproc
$(NAME)_SOURCES += common/k_cache_armcc.S

ifeq ($(HOST_ARCH),Cortex-A5)
$(NAME)_SOURCES += armcc/a5/port_s.S
$(NAME)_SOURCES += armcc/a5/vector_s.S
else ifeq ($(HOST_ARCH),Cortex-A7)
$(NAME)_SOURCES += armcc/a7/port_s.S
$(NAME)_SOURCES += armcc/a7/vector_s.S
else ifeq ($(HOST_ARCH),Cortex-A9)
$(NAME)_SOURCES += armcc/a9/port_s.S
$(NAME)_SOURCES += armcc/a9/vector_s.S

endif

else ifeq ($(COMPILER),iar)

$(NAME)_SOURCES += common/k_cache_iccarm.S

ifeq ($(HOST_ARCH),Cortex-A5)
$(NAME)_SOURCES += iccarm/a5/port_s.S
$(NAME)_SOURCES += iccarm/a5/vector_s.S
else ifeq ($(HOST_ARCH),Cortex-A7)
$(NAME)_SOURCES += iccarm/a7/port_s.S
$(NAME)_SOURCES += iccarm/a7/vector_s.S
else ifeq ($(HOST_ARCH),Cortex-A9)
$(NAME)_SOURCES += iccarm/a9/port_s.S
$(NAME)_SOURCES += iccarm/a9/vector_s.S
endif

else

$(NAME)_SOURCES += common/k_cache_gcc.S

ifeq ($(HOST_ARCH),Cortex-A5)
$(NAME)_SOURCES += gcc/a5/port_s.S
$(NAME)_SOURCES += gcc/a5/vector_s.S
else ifeq ($(HOST_ARCH),Cortex-A7)
$(NAME)_SOURCES += gcc/a7/port_s.S
$(NAME)_SOURCES += gcc/a7/vector_s.S
else ifeq ($(HOST_ARCH),Cortex-A9)
$(NAME)_SOURCES += gcc/a9/port_s.S
$(NAME)_SOURCES += gcc/a9/vector_s.S
endif

endif
