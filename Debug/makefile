################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include drivers/subdir.mk
-include device/subdir.mk
-include component/uart/subdir.mk
-include component/serial_manager/subdir.mk
-include component/lists/subdir.mk
-include board/subdir.mk
-include CM4F_RTCESL_4.5.1_MCUX/MCDRV/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: DrivePOC_Controller_NXP.axf

# Tool invocations
DrivePOC_Controller_NXP.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/MLIB" -L"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GDFLIB" -L"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GMCLIB" -L"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/AMCLIB" -L"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GFLIB" -L"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/MCDRV" -Xlinker -Map="DrivePOC_Controller_NXP.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -T DrivePOC_Controller_NXP_Debug.ld -u _printf_float -u _scanf_float -o "DrivePOC_Controller_NXP.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) DrivePOC_Controller_NXP.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "DrivePOC_Controller_NXP.axf"; # arm-none-eabi-objcopy -v -O binary "DrivePOC_Controller_NXP.axf" "DrivePOC_Controller_NXP.bin" ; # checksum -p MKV31F512xxx12 -d "DrivePOC_Controller_NXP.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
