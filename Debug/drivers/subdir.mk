################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_adc16.c \
../drivers/fsl_clock.c \
../drivers/fsl_cmp.c \
../drivers/fsl_common.c \
../drivers/fsl_dac.c \
../drivers/fsl_dmamux.c \
../drivers/fsl_dspi.c \
../drivers/fsl_dspi_edma.c \
../drivers/fsl_edma.c \
../drivers/fsl_ftm.c \
../drivers/fsl_gpio.c \
../drivers/fsl_i2c.c \
../drivers/fsl_lpuart.c \
../drivers/fsl_pit.c \
../drivers/fsl_smc.c \
../drivers/fsl_uart.c 

OBJS += \
./drivers/fsl_adc16.o \
./drivers/fsl_clock.o \
./drivers/fsl_cmp.o \
./drivers/fsl_common.o \
./drivers/fsl_dac.o \
./drivers/fsl_dmamux.o \
./drivers/fsl_dspi.o \
./drivers/fsl_dspi_edma.o \
./drivers/fsl_edma.o \
./drivers/fsl_ftm.o \
./drivers/fsl_gpio.o \
./drivers/fsl_i2c.o \
./drivers/fsl_lpuart.o \
./drivers/fsl_pit.o \
./drivers/fsl_smc.o \
./drivers/fsl_uart.o 

C_DEPS += \
./drivers/fsl_adc16.d \
./drivers/fsl_clock.d \
./drivers/fsl_cmp.d \
./drivers/fsl_common.d \
./drivers/fsl_dac.d \
./drivers/fsl_dmamux.d \
./drivers/fsl_dspi.d \
./drivers/fsl_dspi_edma.d \
./drivers/fsl_edma.d \
./drivers/fsl_ftm.d \
./drivers/fsl_gpio.d \
./drivers/fsl_i2c.d \
./drivers/fsl_lpuart.d \
./drivers/fsl_pit.d \
./drivers/fsl_smc.d \
./drivers/fsl_uart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DPRINTF_ADVANCED_ENABLE -DCPU_MKV31F512VLL12 -DCPU_MKV31F512VLL12_cm4 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSERIAL_PORT_TYPE_UART=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -D__NEWLIB__ -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/drivers" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CMSIS" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/device" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/drivers" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CMSIS" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/device" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/utilities" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/component/serial_manager" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/component/uart" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/component/lists" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/board" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/source" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/MLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GDFLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GMCLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/AMCLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/GFLIB/Include" -I"/home/sai_pavan/Documents/Agnite SVPWM UART-20230302T180339Z-001/Agnite SVPWM UART/DrivePOC_Controller_NXP/CM4F_RTCESL_4.5.1_MCUX/MCDRV" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -specs=nano.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


