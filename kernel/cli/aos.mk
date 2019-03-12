NAME := cli

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := Command Line Interface

$(NAME)_SOURCES := cli.c cli_adapt.c cli_default_command.c cli_dumpsys.c

ifeq ($(COMPILER),armcc)
else ifeq ($(COMPILER),rvct)
else ifeq ($(COMPILER),iar)
else
$(NAME)_CFLAGS  += -Wall -Werror
endif

$(NAME)_COMPONENTS += ulog debug

GLOBAL_INCLUDES += include
GLOBAL_DEFINES  += AOS_COMP_CLI

AOS_COMP_CLI ?= y
