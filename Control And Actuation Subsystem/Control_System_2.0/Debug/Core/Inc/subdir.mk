################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/FlashStore.c \
../Core/Inc/Sonar.c \
../Core/Inc/ToF.c \
../Core/Inc/battery.c \
../Core/Inc/mission_rx.c \
../Core/Inc/telemetry.c \
../Core/Inc/temp_sensor.c 

OBJS += \
./Core/Inc/FlashStore.o \
./Core/Inc/Sonar.o \
./Core/Inc/ToF.o \
./Core/Inc/battery.o \
./Core/Inc/mission_rx.o \
./Core/Inc/telemetry.o \
./Core/Inc/temp_sensor.o 

C_DEPS += \
./Core/Inc/FlashStore.d \
./Core/Inc/Sonar.d \
./Core/Inc/ToF.d \
./Core/Inc/battery.d \
./Core/Inc/mission_rx.d \
./Core/Inc/telemetry.d \
./Core/Inc/temp_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/FlashStore.cyclo ./Core/Inc/FlashStore.d ./Core/Inc/FlashStore.o ./Core/Inc/FlashStore.su ./Core/Inc/Sonar.cyclo ./Core/Inc/Sonar.d ./Core/Inc/Sonar.o ./Core/Inc/Sonar.su ./Core/Inc/ToF.cyclo ./Core/Inc/ToF.d ./Core/Inc/ToF.o ./Core/Inc/ToF.su ./Core/Inc/battery.cyclo ./Core/Inc/battery.d ./Core/Inc/battery.o ./Core/Inc/battery.su ./Core/Inc/mission_rx.cyclo ./Core/Inc/mission_rx.d ./Core/Inc/mission_rx.o ./Core/Inc/mission_rx.su ./Core/Inc/telemetry.cyclo ./Core/Inc/telemetry.d ./Core/Inc/telemetry.o ./Core/Inc/telemetry.su ./Core/Inc/temp_sensor.cyclo ./Core/Inc/temp_sensor.d ./Core/Inc/temp_sensor.o ./Core/Inc/temp_sensor.su

.PHONY: clean-Core-2f-Inc

