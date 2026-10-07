################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/subdir.mk
-include Core/ThreadSafe/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Common/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.elf \

MAP_FILES += \
RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.list \

OBJCOPY_HEX += \
RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.hex \

OBJCOPY_BIN += \
RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.bin \


# All Target
all: main-build

# Main-build Target
main-build: RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.elf secondary-outputs

# Tool invocations
RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.elf RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.map: $(OBJS) $(USER_OBJS) C:\Users\kylej\OneDrive\ -\ Texas\ Tech\ University\TTU\RAS\LiquidEngineDAQ-Control\firmware\RAS_Liquid_Controls_Nucleo144-H755ZI-Q\RAS_Liquid_Controls_Nucleo144-H755ZI-Q\CM4\STM32H755ZITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\kylej\OneDrive - Texas Tech University\TTU\RAS\LiquidEngineDAQ-Control\firmware\RAS_Liquid_Controls_Nucleo144-H755ZI-Q\RAS_Liquid_Controls_Nucleo144-H755ZI-Q\CM4\STM32H755ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.list"
	@echo 'Finished building: $@'
	@echo ' '

RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.hex"
	@echo 'Finished building: $@'
	@echo ' '

RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.bin RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.elf RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.hex RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.list RAS_Liquid_Controls_Nucleo144-H755ZI-Q_CM4.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
