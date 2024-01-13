################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/serial_manager/fsl_component_serial_manager.c \
../component/serial_manager/fsl_component_serial_port_uart.c 

OBJS += \
./component/serial_manager/fsl_component_serial_manager.o \
./component/serial_manager/fsl_component_serial_port_uart.o 

C_DEPS += \
./component/serial_manager/fsl_component_serial_manager.d \
./component/serial_manager/fsl_component_serial_port_uart.d 


# Each subdirectory must supply rules for building sources it contributes
component/serial_manager/%.o: ../component/serial_manager/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DPRINTF_ADVANCED_ENABLE -DCPU_MKV31F512VLL12 -DCPU_MKV31F512VLL12_cm4 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSERIAL_PORT_TYPE_UART=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -D__NEWLIB__ -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/drivers" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CMSIS" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/device" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/drivers" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CMSIS" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/device" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/utilities" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/component/serial_manager" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/component/uart" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/component/lists" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/board" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/source" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/MLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GDFLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GMCLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/AMCLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GFLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/MCDRV" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -specs=nano.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

