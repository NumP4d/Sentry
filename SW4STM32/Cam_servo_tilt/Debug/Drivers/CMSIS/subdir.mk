################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32l4xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32l4xx.o: C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/CMSIS/system_stm32l4xx.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


