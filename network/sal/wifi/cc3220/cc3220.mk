NAME := device_sal_cc3220

GLOBAL_DEFINES += DEV_SAL_CC3220

$(NAME)_COMPONENTS += yloop

$(NAME)_SOURCES += cc3220_wifi.c
