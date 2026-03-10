################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Module/Src/motor.cpp \
../Module/Src/uart_interface.cpp 

C_SRCS += \
../Module/Src/ws2812.c 

C_DEPS += \
./Module/Src/ws2812.d 

OBJS += \
./Module/Src/motor.o \
./Module/Src/uart_interface.o \
./Module/Src/ws2812.o 

CPP_DEPS += \
./Module/Src/motor.d \
./Module/Src/uart_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Module/Src/%.o Module/Src/%.su Module/Src/%.cyclo: ../Module/Src/%.cpp Module/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Module/Inc -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Module/Src/%.o Module/Src/%.su Module/Src/%.cyclo: ../Module/Src/%.c Module/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Module/Inc -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Module-2f-Src

clean-Module-2f-Src:
	-$(RM) ./Module/Src/motor.cyclo ./Module/Src/motor.d ./Module/Src/motor.o ./Module/Src/motor.su ./Module/Src/uart_interface.cyclo ./Module/Src/uart_interface.d ./Module/Src/uart_interface.o ./Module/Src/uart_interface.su ./Module/Src/ws2812.cyclo ./Module/Src/ws2812.d ./Module/Src/ws2812.o ./Module/Src/ws2812.su

.PHONY: clean-Module-2f-Src

