SRC_ROOT=../
TARGET = loracommander

INCLUDES = -I"../" -I"../config" -I"../examples" -I"../hal/include" -I"../hal/utils/include" -I"../hpl/core" -I"../hpl/dmac" -I"../hpl/eic" -I"../hpl/gclk" -I"../hpl/pm" -I"../hpl/port" -I"../hpl/rtc" -I"../hpl/sercom" -I"../hpl/sysctrl" -I"../hpl/usb" -I"../hri" -I"../" -I"../config" -I"../thirdparty/RTOS" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/include" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/GCC/ARM_CM0" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/module_config" -I"../" -I"../config" -I"../usb" -I"../usb/class/cdc" -I"../usb/class/cdc/device" -I"../usb/device" -I"../" -I"../CMSIS/Include" -I"../samd21a/include"
#~ CPPFLAGS = -x c -mthumb -DDEBUG -Os -ffunction-sections -mlong-calls -g3 -Wall -c -std=gnu99 -D__SAMD21G18A__ -mcpu=cortex-m0plus $(INCLUDES)
CPPFLAGS = -mthumb -DDEBUG -Os -ffunction-sections -mlong-calls -g3 -Wall -D__SAMD21G18A__ -mcpu=cortex-m0plus $(INCLUDES)
#~ -std=c99, -std=gnu99, -std=c11 or -std=gnu11
CXXFLAGS = -fno-exceptions -std=c++11 

CFLAGS = -std=gnu99

LDFLAGS = -lstdc++_nano -Wl,--start-group -lm -Wl,--end-group -mthumb -Wl,-Map="$(TARGET).map" --specs=nano.specs -Wl,--gc-sections -mcpu=cortex-m0plus -T"../samd21a/gcc/gcc/samd21g18a_flash.ld" -L"../samd21a/gcc/gcc"
#~ LIBS = -lm


CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

#~ PROJECT_FILES := $(wildcard *.c)

PROJECT_OBJS = $(SRC_ROOT)main.o	#~tickless.o

ASF4_OBJS = $(SRC_ROOT)hal/src/hal_io.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/list.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/MemMang/heap_1.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/event_groups.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/croutine.o \
			$(SRC_ROOT)samd21a/gcc/gcc/startup_samd21.o \
			$(SRC_ROOT)hpl/eic/hpl_eic.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/stream_buffer.o \
			$(SRC_ROOT)usb/class/cdc/device/cdcdf_acm.o \
			$(SRC_ROOT)hal/utils/src/utils_syscalls.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/queue.o \
			$(SRC_ROOT)hal/src/hal_spi_m_sync.o \
			$(SRC_ROOT)hal/src/hal_timer.o \
			$(SRC_ROOT)hpl/usb/hpl_usb.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/timers.o \
			$(SRC_ROOT)hal/src/hal_delay.o \
			$(SRC_ROOT)hpl/sysctrl/hpl_sysctrl.o \
			$(SRC_ROOT)hpl/core/hpl_init.o \
			$(SRC_ROOT)hal/utils/src/utils_list.o \
			$(SRC_ROOT)hpl/core/hpl_core_m0plus_base.o \
			$(SRC_ROOT)hal/utils/src/utils_assert.o \
			$(SRC_ROOT)usb_start.o \
			$(SRC_ROOT)hpl/pm/hpl_pm.o \
			$(SRC_ROOT)hal/src/hal_usart_sync.o \
			$(SRC_ROOT)hpl/sercom/hpl_sercom.o \
			$(SRC_ROOT)hpl/gclk/hpl_gclk.o \
			$(SRC_ROOT)usb/usb_protocol.o \
			$(SRC_ROOT)hal/src/hal_init.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/rtos_port.o \
			$(SRC_ROOT)hal/src/hal_usb_device.o \
			$(SRC_ROOT)rtos_start.o \
			$(SRC_ROOT)samd21a/gcc/system_samd21.o \
			$(SRC_ROOT)hpl/rtc/hpl_rtc.o \
			$(SRC_ROOT)examples/driver_examples.o \
			$(SRC_ROOT)driver_init.o \
			$(SRC_ROOT)hal/src/hal_ext_irq.o \
			$(SRC_ROOT)hal/src/hal_gpio.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/GCC/ARM_CM0/port.o \
			$(SRC_ROOT)hal/utils/src/utils_event.o \
			$(SRC_ROOT)hal/src/hal_sleep.o \
			$(SRC_ROOT)hpl/dmac/hpl_dmac.o \
			$(SRC_ROOT)atmel_start.o \
			$(SRC_ROOT)usb/device/usbdc.o \
			$(SRC_ROOT)hal/src/hal_atomic.o \
			$(SRC_ROOT)thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/tasks.o

#~ OBJS := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o)
OBJS := $(ASF4_OBJS) $(PROJECT_OBJS)

all: $(TARGET).hex

$(TARGET).elf: $(OBJS) $(MCU_LD)
	arm-none-eabi-gcc -o $(TARGET).elf $(OBJS) $(LDFLAGS) 

%.hex: %.elf
	$(SIZE) $<
	arm-none-eabi-objcopy -O binary $(TARGET).elf $(TARGET).bin
	arm-none-eabi-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature $(TARGET).elf $(TARGET).hex
	arm-none-eabi-objcopy -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary $(TARGET).elf $(TARGET).eep || exit 0
	arm-none-eabi-objdump -h -S $(TARGET).elf > $(TARGET).lss
	arm-none-eabi-size $(TARGET).elf

-include $(OBJS:.o=.d)

clean:
	rm -f $(OBJS) *.o *.d $(TARGET).elf $(TARGET).hex
	rm -f $(TARGET).*

flash: $(TARGET).hex
	/home/jeroen/.arduino15/packages/arduino/tools/bossac/1.7.0/bossac -i  -U true --port=/dev/loraflash -e -w $(TARGET).bin -R
