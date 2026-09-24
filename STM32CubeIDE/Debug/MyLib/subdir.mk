################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/OneDrive\ -\ student.tdtu.edu.vn/Project/Da\ Nang\ 2026/2\ mat\ tu\ xa/Test_code_h7/MyLib/BMI088.c 

OBJS += \
./MyLib/BMI088.o 

C_DEPS += \
./MyLib/BMI088.d 


# Each subdirectory must supply rules for building sources it contributes
MyLib/BMI088.o: D:/OneDrive\ -\ student.tdtu.edu.vn/Project/Da\ Nang\ 2026/2\ mat\ tu\ xa/Test_code_h7/MyLib/BMI088.c MyLib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../MyLib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MyLib

clean-MyLib:
	-$(RM) ./MyLib/BMI088.cyclo ./MyLib/BMI088.d ./MyLib/BMI088.o ./MyLib/BMI088.su

.PHONY: clean-MyLib

