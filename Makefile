BUILD=build

LD = $(TOOLCHAIN)arm-none-eabi-ld
AS = $(TOOLCHAIN)arm-none-eabi-as
CC = $(TOOLCHAIN)arm-none-eabi-gcc
CPP = $(TOOLCHAIN)arm-none-eabi-g++
OBJCOPY = $(TOOLCHAIN)arm-none-eabi-objcopy

INC_FLAGS = \
	-I src/apps/LoRaMac/classA/NucleoL073/ \
	-I src/apps/LoRaMac/classB/NucleoL073/ \
	-I src/apps/LoRaMac/classC/NucleoL073/ \
	-I src/apps/LoRaMac/common/ \
	-I src/boards/ \
	-I src/boards/NucleoL073/ \
	-I src/boards/NucleoL073/cmsis/ \
	-I src/boards/mcu/stm32/ \
	-I src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Inc/ \
	-I src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Inc/Legacy/ \
	-I src/boards/mcu/stm32/STM32_USB_Device_Library/Class/CDC/Inc/ \
	-I src/boards/mcu/stm32/STM32_USB_Device_Library/Class/DFU/Inc/ \
	-I src/boards/mcu/stm32/STM32_USB_Device_Library/Core/Inc/ \
	-I src/boards/mcu/stm32/cmsis/ \
	-I src/mac/ \
	-I src/mac/region/ \
	-I src/peripherals/ \
	-I src/peripherals/soft-se/ \
	-I src/radio/ \
	-I src/radio/sx126x/ \
	-I src/radio/sx1272/ \
	-I src/radio/sx1276/ \
	-I src/system/ \

UNUSED=\
	src/apps/LoRaMac/classB/NucleoL073/main.c \
	src/apps/LoRaMac/classC/NucleoL073/main.c \
	src/apps/ping-pong/NucleoL073/main.c \
	src/apps/rx-sensi/NucleoL073/main.c \
	src/apps/tx-cw/NucleoL073/main.c \
	src/boards/mcu/stm32/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
	src/boards/mcu/stm32/STM32_USB_Device_Library/Class/DFU/Src/usbd_dfu.c \
	src/boards/mcu/stm32/STM32_USB_Device_Library/Core/Src/usbd_core.c \
	src/boards/mcu/stm32/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
	src/boards/mcu/stm32/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_adc.c \
	src/boards/NucleoL073/adc-board.c \
	src/boards/NucleoL073/sx1261dvk1bas-board.c \
	src/boards/NucleoL073/sx1262dvk1cas-board.c \
	src/boards/NucleoL073/sx1262dvk1das-board.c \
	src/boards/NucleoL073/sx1272mb2das-board.c \
	src/boards/NucleoL073/sx1276mb1mas-board.c \
	src/peripherals/sx1509.c \
	src/peripherals/sx9500.c \
	src/radio/sx126x/sx126x.c \
	src/radio/sx1272/sx1272.c \
	src/boards/NucleoL073/i2c-board.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_i2c.c \
	src/peripherals/gpio-ioe.c \
	src/peripherals/mag3110.c \
	src/peripherals/mma8451.c \
	src/peripherals/mpl3115.c \
	src/peripherals/pam7q.c \

BOARD_SRC = \
	src/boards/NucleoL073/board.c \
	src/boards/NucleoL073/delay-board.c \
	src/boards/NucleoL073/eeprom-board.c \
	src/boards/NucleoL073/gpio-board.c \
	src/boards/NucleoL073/lpm-board.c \
	src/boards/NucleoL073/rtc-board.c \
	src/boards/NucleoL073/spi-board.c \
	src/boards/NucleoL073/uart-board.c \


SRCS= $(BOARD_SRC) \
	src/apps/LoRaMac/classA/NucleoL073/main.c \
	src/apps/LoRaMac/common/NvmCtxMgmt.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_comp.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_comp_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cortex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_crc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_crc_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cryp.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cryp_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_dac.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_dac_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_dma.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_firewall.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ramfunc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_gpio.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2s.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_irda.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_iwdg.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_lcd.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_lptim.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_msp_template.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pcd.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pcd_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rng.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_smartcard.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_smartcard_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_smbus.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_spi.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tsc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart_ex.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_usart.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_wwdg.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_comp.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_crc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_crs.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_dac.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_dma.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_exti.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_gpio.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_lptim.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_lpuart.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_pwr.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rcc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rng.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rtc.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_spi.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_tim.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_usart.c \
	src/boards/mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_utils.c \
	src/boards/mcu/stm32/sysIrqHandlers.c \
	src/boards/mcu/utilities.c \
	src/boards/NucleoL073/cmsis/system_stm32l0xx.c \
	src/mac/LoRaMac.c \
	src/mac/LoRaMacAdr.c \
	src/mac/LoRaMacClassB.c \
	src/mac/LoRaMacCommands.c \
	src/mac/LoRaMacConfirmQueue.c \
	src/mac/LoRaMacCrypto.c \
	src/mac/LoRaMacFCntHandler.c \
	src/mac/LoRaMacParser.c \
	src/mac/LoRaMacSerializer.c \
	src/mac/region/Region.c \
	src/mac/region/RegionCommon.c \
	src/mac/region/RegionUS915.c \
	src/peripherals/soft-se/aes.c \
	src/peripherals/soft-se/cmac.c \
	src/peripherals/soft-se/soft-se.c \
	src/radio/sx1276/sx1276.c \
	src/boards/NucleoL073/sx1276mb1las-board.c \
	src/system/delay.c \
	src/system/eeprom.c \
	src/system/fifo.c \
	src/system/gpio.c \
	src/system/nvmm.c \
	src/system/systime.c \
	src/system/timer.c \
	src/system/uart.c \
	src/desperation.c

EXTRA=\
	src/system/adc.c \
	src/system/gps.c \
	src/system/i2c.c \

NUC = \
	src/boards/NucleoL073/board.c \
	src/boards/NucleoL073/delay-board.c \
	src/boards/NucleoL073/eeprom-board.c \
	src/boards/NucleoL073/gpio-board.c \
	src/boards/NucleoL073/lpm-board.c \
	src/boards/NucleoL073/rtc-board.c \
	src/boards/NucleoL073/spi-board.c \
	src/boards/NucleoL073/uart-board.c \
	src/boards/NucleoL073/../mcu/stm32/sysIrqHandlers.c \
	src/boards/NucleoL073/../mcu/utilities.c \
	src/boards/NucleoL073/cmsis/system_stm32l0xx.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc_ex.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cortex.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_dma.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ex.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_gpio.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c_ex.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr_ex.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc_ex.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc_ex.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_spi.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart.c \
	src/boards/NucleoL073/../mcu/stm32/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart_ex.c \
	src/boards/NucleoL073/sx1276mb1las-board.c \
	src/radio/sx1276/sx1276.c \
	src/system/delay.c \
	src/system/eeprom.c \
	src/system/fifo.c \
	src/system/gpio.c \
	src/system/nvmm.c \
	src/system/systime.c \
	src/system/timer.c \
	src/system/uart.c \
	src/apps/LoRaMac/classA/NucleoL073/main.c \
	src/apps/LoRaMac/common/NvmCtxMgmt.c \
	src/mac/LoRaMac.c \
	src/mac/LoRaMacAdr.c \
	src/mac/LoRaMacClassB.c \
	src/mac/LoRaMacCommands.c \
	src/mac/LoRaMacConfirmQueue.c \
	src/mac/LoRaMacCrypto.c \
	src/mac/LoRaMacFCntHandler.c \
	src/mac/LoRaMacParser.c \
	src/mac/LoRaMacSerializer.c \

#SRCS=$(NUC)
OBJ := $(patsubst %.cpp,%.o,$(SRCS))
OBJ := $(addprefix $(BUILD)/, $(patsubst %.c,%.o,$(OBJ)))
OBJ += $(BUILD)/src/boards/NucleoL073/cmsis/arm-gcc/startup_stm32l073xx.o

#MCU_FLAGS = -DSTM32L073X -DSTM32L073xx -mthumb -mcpu=cortex-m0plus -mlittle-endian \
#                  -mfloat-abi=soft

MCU_FLAGS = -DSTM32L073X -DSTM32L073xx \
	-Og -g -mthumb -g2 -fno-builtin -mcpu=cortex-m0plus -Wall \
	-ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs \
	-fno-unroll-loops -ffast-math -ftree-vectorize

LORA_FLAGS = -DUSE_HAL_DRIVER -D SX1276MB1LAS -D ACTIVE_REGION=LORAMAC_REGION_US915 -D REGION_US915
CFLAGS = -std=gnu99 $(MCU_FLAGS) -D USE_HAL_DRIVER -g $(LORA_FLAGS)

LINKER_SCRIPT = src/boards/NucleoL073/cmsis/arm-gcc/stm32l073xx_flash.ld

LINKER_FLAGS = -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -mthumb -g2 -mcpu=cortex-m0plus -mabi=aapcs -T${LINKER_SCRIPT}

OUTPUT_BIN = build/node.bin

OPENOCD = killall openocd ; ~/tools/openocd/src/openocd -s ~/tools/openocd/tcl -f interface/stlink-v2-1.cfg -f target/stm32l0dual192k.cfg -c init

$(BUILD)/%.o : %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(LOCAL_CFLAGS) $(CFLAGS_ARM) $(C_SYNT_FLAGS) \
	       $(INC_FLAGS) \
	-o $@ -c $<


$(BUILD)/%.o : %.s
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(LOCAL_CFLAGS) $(CFLAGS_ARM) $(C_SYNT_FLAGS) \
	       $(INC_FLAGS) \
	-o $@ -c $<

$(BUILD)/node.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LOCAL_CFLAGS) $(CFLAGS_ARM) $(C_SYNT_FLAGS) \
	       $(LINKER_FLAGS) -o $@ $(OBJ)

$(BUILD)/node.bin: $(BUILD)/node.elf
	$(OBJCOPY) -O binary $< $@

flash: $(OUTPUT_BIN)
	$(OPENOCD)  -c reset -c halt -c "flash write_image erase $(OUTPUT_BIN) 0x8000000" -c "sleep 1000" -c reset -c exit
