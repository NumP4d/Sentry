################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/freertos.c \
C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/main.c \
C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/stm32l4xx_hal_msp.c \
C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/stm32l4xx_hal_timebase_TIM.c \
C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/stm32l4xx_it.c 

OBJS += \
./Application/User/freertos.o \
./Application/User/main.o \
./Application/User/stm32l4xx_hal_msp.o \
./Application/User/stm32l4xx_hal_timebase_TIM.o \
./Application/User/stm32l4xx_it.o 

C_DEPS += \
./Application/User/freertos.d \
./Application/User/main.d \
./Application/User/stm32l4xx_hal_msp.d \
./Application/User/stm32l4xx_hal_timebase_TIM.d \
./Application/User/stm32l4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/freertos.o: C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/freertos.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/freertos.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/main.o: C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/main.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32l4xx_hal_msp.o: C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/stm32l4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/stm32l4xx_hal_msp.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32l4xx_hal_timebase_TIM.o: C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/stm32l4xx_hal_timebase_TIM.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/stm32l4xx_hal_timebase_TIM.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32l4xx_it.o: C:/Users/User/Documents/STM32\ workspace/L476/CAM/Cam_servo_tilt/Src/stm32l4xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Drivers/CMSIS/Include" -I"C:/Users/User/Documents/STM32 workspace/L476/CAM/Cam_servo_tilt/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/stm32l4xx_it.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


