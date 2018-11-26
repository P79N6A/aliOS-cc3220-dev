HOST_OPENOCD := CC3220
NAME := cc3220s

$(NAME)_TYPE := kernel

$(NAME)_COMPONENTS += platform/arch/arm/armv7m
$(NAME)_COMPONENTS += libc rhino hal common cli

sal ?= 0
module ?= wifi.cc3220

GLOBAL_CFLAGS += -DCORE_M4
GLOBAL_CFLAGS += -fmessage-length=0
GLOBAL_CFLAGS += -fno-builtin -ffunction-sections -fdata-sections -fno-common -std=gnu99 -nostdlib
GLOBAL_CFLAGS += -mcpu=cortex-m4 -mlittle-endian -mthumb -mthumb-interwork -march=armv7e-m 

GLOBAL_CFLAGS += -DRHINO_CONFIG_TASK_STACK_CUR_CHECK=1
GLOBAL_CFLAGS += -DSL_SUPPORT_IPV6
GLOBAL_CFLAGS += -DDISABLE_DEBUGGER_RECONNECT
GLOBAL_CFLAGS += -DSL_PLATFORM_MULTI_THREADED
GLOBAL_DEFINES += USE_HAL_DRIVER

GLOBAL_CFLAGS += -DSIMPLELINK_CC32XX
GLOBAL_CFLAGS += -DCONFIG_NO_TCPIP
GLOBAL_CFLAGS += -DALLOW_PARSING__TEMPLATE
GLOBAL_CFLAGS += -DALLOW_PARSING__JSON
GLOBAL_CFLAGS += -D__GCC__

GLOBAL_CFLAGS += -DCONFIG_YWSS

GLOBAL_DEFINES += IOTX_WITHOUT_TLS
GLOBAL_CFLAGS += -DMQTT_DIRECT=1

GLOBAL_INCLUDES += ../../arch/arm/armv7m/gcc/m4/

#GLOBAL_LDFLAGS += -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -march=armv7e-m -mlittle-endian -mthumb-interwork -nostartfiles
GLOBAL_LDFLAGS += -mcpu=cortex-m4        \
                  -mthumb -mthumb-interwork \
                  -mlittle-endian \
                  -nostartfiles \
                  $(CLIB_LDFLAGS_NANO_FLOAT)

$(NAME)_CFLAGS  += -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-implicit-function-declaration
$(NAME)_CFLAGS  += -Wno-type-limits -Wno-sign-compare -Wno-pointer-sign -Wno-uninitialized
$(NAME)_CFLAGS  += -Wno-return-type -Wno-unused-function -Wno-unused-but-set-variable
$(NAME)_CFLAGS  += -Wno-unused-value -Wno-strict-aliasing
GLOBAL_CFLAGS += -Wno-format -Wno-incompatible-pointer-types
$(NAME)_SOURCES     :=

#$(NAME)_SOURCES     += ../../arch/arm/armv7m/gcc/m4/port_c.c
#$(NAME)_SOURCES     += ../../arch/arm/armv7m/gcc/m4/port_s.S

$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/adc.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/camera.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/crc.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/flash.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/hwspinlock.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/i2s.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/pin.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/sdhost.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/spi.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/timer.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/udma.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/wdt.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/aes.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/cpu.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/des.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/gpio.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/i2c.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/interrupt.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/prcm.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/shamd5.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/systick.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/uart.c
$(NAME)_SOURCES     += ./ti/devices/cc32xx/driverlib/utils.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/nonos.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/device.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/driver.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/netapp.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/flowcont.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/sl_socket.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/fs.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/netutil.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/spawn.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/socket.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/netdb.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/wlan.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/source/netcfg.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/eventreg.c
#$(NAME)_SOURCES     += ./ti/drivers/net/wifi/slnetif/slnetifwifi.c
$(NAME)_SOURCES     += ./ti/drivers/net/wifi/porting/cc_pal.c
$(NAME)_SOURCES     += ./startup_cc32xx_gcc.c
$(NAME)_SOURCES     += ./ti/drivers/spi/SPICC32XXDMA.c
$(NAME)_SOURCES     += ./ti/drivers/gpio/GPIOCC32XX.c
$(NAME)_SOURCES     += ./ti/drivers/power/PowerCC32XX.c
$(NAME)_SOURCES     += ./ti/drivers/dma/UDMACC32XX.c
$(NAME)_SOURCES     += ./ti/drivers/utils/List.c
$(NAME)_SOURCES     += ./utils/PowerCC32XX_AliOS.c
ifeq ($(SL_TARGET),CC3220SF)
$(NAME)_SOURCES     += ./ti/boards/CC3220SF_LAUNCHXL/CC3220SF_LAUNCHXL.c
else
$(NAME)_SOURCES     += ./ti/boards/CC3220S_LAUNCHXL/CC3220S_LAUNCHXL.c
endif

$(NAME)_SOURCES     += ./ti/drivers/SPI.c
#$(NAME)_SOURCES     += ./hal/hal_wifi_nwp.c
$(NAME)_SOURCES     += ./hal/hwi_alios.c
$(NAME)_SOURCES     += ./hal/wifi_port.c
$(NAME)_SOURCES     += ./hal/hal_uart.c
$(NAME)_SOURCES     += ./hal/hal_flash.c
$(NAME)_SOURCES     += ./hal/csp_log.c
$(NAME)_SOURCES     += ./hal/hal_ota.c
$(NAME)_SOURCES     += ./aos/aos.c
$(NAME)_SOURCES     += ./aos/soc_impl.c
#$(NAME)_SOURCES     += ./aos/aos_awss.c
$(NAME)_SOURCES     += ./utils/nwp_util.c
#$(NAME)_SOURCES     += ./cc3220_handler.c
$(NAME)_SOURCES     += ./ti/net/ota/source/OtaArchive.c
$(NAME)_SOURCES     += ./ti/net/ota/source/OtaJson.c
$(NAME)_SOURCES     += ./ti/drivers/crypto/CryptoCC32XX.c
$(NAME)_SOURCES     += ./ti/net/json/source/json.c
$(NAME)_SOURCES     += ./ti/net/json/source/json_engine.c
$(NAME)_SOURCES     += ./ti/net/json/source/parse_common.c
$(NAME)_SOURCES     += ./ti/net/json/source/utils.c

GLOBAL_INCLUDES     += ./ti/net/json/include
GLOBAL_INCLUDES     += ./ti/devices/cc32xx/driverlib
GLOBAL_INCLUDES     += ./ti/devices/cc32xx
GLOBAL_INCLUDES     += .


GLOBAL_LDFLAGS += -lm
ifndef OVERRIDE_LD_FILE
ifeq ($(SLTARGET),CC3220S)
GLOBAL_LDFLAGS += -T platform/mcu/cc3220/CC3220S_LAUNCHXL_AOS.lds
GLOBAL_CFLAGS += -DCC32XX_ROM_DEVICE
else
GLOBAL_LDFLAGS += -T platform/mcu/cc3220/CC3220SF_LAUNCHXL_AOS.lds
GLOBAL_CFLAGS += -DCC32XX_FLASH_DEVICE
endif
endif







