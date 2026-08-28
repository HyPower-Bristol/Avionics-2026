################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ads131m0x/ads131m0x.c 

OBJS += \
./ads131m0x/ads131m0x.o 

C_DEPS += \
./ads131m0x/ads131m0x.d 


# Each subdirectory must supply rules for building sources it contributes
ads131m0x/%.o ads131m0x/%.su ads131m0x/%.cyclo: ../ads131m0x/%.c ads131m0x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ads131m0x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ads131m0x

clean-ads131m0x:
	-$(RM) ./ads131m0x/ads131m0x.cyclo ./ads131m0x/ads131m0x.d ./ads131m0x/ads131m0x.o ./ads131m0x/ads131m0x.su

.PHONY: clean-ads131m0x

