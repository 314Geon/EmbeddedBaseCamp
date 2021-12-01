################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SST25VF016B_Lib/SST25VF016B.c 

OBJS += \
./SST25VF016B_Lib/SST25VF016B.o 

C_DEPS += \
./SST25VF016B_Lib/SST25VF016B.d 


# Each subdirectory must supply rules for building sources it contributes
SST25VF016B_Lib/%.o: ../SST25VF016B_Lib/%.c SST25VF016B_Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Daniel/Study/GL_Basecamp/EmbeddedBaseCamp/Task7/SST25VF016B_Lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

