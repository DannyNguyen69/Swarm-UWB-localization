# UWB firmware for DWM1001 (nRF52832 + DW1000), bare-metal, GCC.
# One image for both roles; role / node id are written to UICR at flash time.

PROJECT  := uwb
BUILD    := build

ROOT     := $(abspath ..)
SDK      ?= $(ROOT)/dwm1001-examples/nRF5_SDK_14.2.0
DECA     ?= $(ROOT)/dwm1001-examples/deca_driver

CROSS    ?= arm-none-eabi-
CC       := $(CROSS)gcc
OBJCOPY  := $(CROSS)objcopy
SIZE     := $(CROSS)size

FW_MAJOR := 0
FW_MINOR := 7

SRCS := \
  src/main.c \
  src/hw.c \
  src/uart.c \
  src/dw_port.c \
  src/proto.c \
  src/config.c \
  src/radio.c \
  src/twr.c \
  src/anchor.c \
  src/tag.c \
  src/syscalls.c \
  $(DECA)/deca_device.c \
  $(DECA)/deca_params_init.c \
  $(SDK)/components/toolchain/system_nrf52.c

ASRCS := $(SDK)/components/toolchain/gcc/gcc_startup_nrf52.S

INC := \
  -Isrc \
  -I$(DECA) \
  -I$(SDK)/components/device \
  -I$(SDK)/components/toolchain \
  -I$(SDK)/components/toolchain/cmsis/include

CPU := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

DEFS := -DNRF52832_XXAA -DNRF52 -DFLOAT_ABI_HARD -DNDEBUG \
        -DFW_MAJOR=$(FW_MAJOR) -DFW_MINOR=$(FW_MINOR)

CFLAGS := $(CPU) $(DEFS) $(INC) -std=gnu11 -O2 -g3 \
          -Wall -Wno-unused-but-set-variable -Wno-unused-function \
          -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin

ASFLAGS := $(CPU) $(DEFS) -D__HEAP_SIZE=0 -D__STACK_SIZE=4096 -x assembler-with-cpp

LDFLAGS := $(CPU) -Tnrf52832.ld -L$(SDK)/components/toolchain/gcc \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(PROJECT).map --specs=nano.specs

LIBS := -lm -lc -lnosys

# Driver sources from Decawave are not warning-clean; keep their noise out of our build log.
$(BUILD)/deca/%.o: CFLAGS += -w
$(BUILD)/sdk/%.o: CFLAGS += -w

OBJS := $(patsubst src/%.c,$(BUILD)/%.o,$(filter src/%,$(SRCS))) \
        $(patsubst $(DECA)/%.c,$(BUILD)/deca/%.o,$(filter $(DECA)/%,$(SRCS))) \
        $(patsubst $(SDK)/%.c,$(BUILD)/sdk/%.o,$(filter $(SDK)/%,$(SRCS))) \
        $(BUILD)/sdk/startup.o

all: $(BUILD)/$(PROJECT).hex
	@$(SIZE) $(BUILD)/$(PROJECT).elf

$(BUILD)/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

$(BUILD)/deca/%.o: $(DECA)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

$(BUILD)/sdk/%.o: $(SDK)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

$(BUILD)/sdk/startup.o: $(ASRCS)
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(PROJECT).elf: $(OBJS) nrf52832.ld
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(PROJECT).hex: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)

.PHONY: all clean
