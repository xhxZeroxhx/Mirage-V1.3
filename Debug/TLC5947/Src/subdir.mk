################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TLC5947/Src/tlc5947.c 

OBJS += \
./TLC5947/Src/tlc5947.o 

C_DEPS += \
./TLC5947/Src/tlc5947.d 


# Each subdirectory must supply rules for building sources it contributes
TLC5947/Src/tlc5947.o: ../TLC5947/Src/tlc5947.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/Marta/STM32CubeIDE/workspace_1.0.1/Mirage-V1.3/TLC5947/Inc" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"TLC5947/Src/tlc5947.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

