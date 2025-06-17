################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := security_system
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: security_system.axf

# Tool invocations
security_system.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\nikod\Downloads\E13_V7_final\E13_alarm_V3\Lib_CMSISv1p30_LPC17xx\Release" -L"C:\Users\nikod\Downloads\E13_V7_final\E13_alarm_V3\Lib_MCU\Release" -L"C:\Users\nikod\Downloads\E13_V7_final\E13_alarm_V3\Lib_EaBaseBoard\Release" -Xlinker --gc-sections -Xlinker -Map=security_system.map -mcpu=cortex-m3 -mthumb -T security_system_Release.ld -o "security_system.axf" $(OBJS) $(USER_OBJS) $(LIBS) -lCMSISv1p30_LPC17xx -lLib_EaBaseBoard -lLib_MCU
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) security_system.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size security_system.axf; # arm-none-eabi-objdump -h -S security_system.axf >security_system.lss
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
