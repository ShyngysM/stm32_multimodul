################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: my 
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/STEPPER/STEPPER.c \
../ECUAL/STEPPER/STEPPER_cfg.c 

OBJS += \
./ECUAL/STEPPER/STEPPER.o \
./ECUAL/STEPPER/STEPPER_cfg.o 

C_DEPS += \
./ECUAL/STEPPER/STEPPER.d \
./ECUAL/STEPPER/STEPPER_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/STEPPER/%.o ECUAL/STEPPER/%.su: ../ECUAL/STEPPER/%.c ECUAL/STEPPER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ECUAL-2f-STEPPER

clean-ECUAL-2f-STEPPER:
	-$(RM) ./ECUAL/STEPPER/STEPPER.d ./ECUAL/STEPPER/STEPPER.o ./ECUAL/STEPPER/STEPPER.su ./ECUAL/STEPPER/STEPPER_cfg.d ./ECUAL/STEPPER/STEPPER_cfg.o ./ECUAL/STEPPER/STEPPER_cfg.su

.PHONY: clean-ECUAL-2f-STEPPER

