NAME := arch_cskyv2-l

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := arch for cskyv2-l

GLOBAL_INCLUDES += .

$(NAME)_SOURCES += cpu_impl.c     \
                   port_s_novic.S \
                   port_c.c       \
                   entry.S
