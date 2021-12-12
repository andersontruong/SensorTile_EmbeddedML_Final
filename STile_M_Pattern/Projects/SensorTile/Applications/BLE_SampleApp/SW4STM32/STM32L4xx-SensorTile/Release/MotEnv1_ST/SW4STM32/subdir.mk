################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/thinkpad/Documents/GitHub/SensorTile_EmbeddedML_Final/STile_M_Pattern/Projects/SensorTile/Applications/BLE_SampleApp/SW4STM32/startup_stm32l476xx.s 

C_SRCS += \
C:/Users/thinkpad/Documents/GitHub/SensorTile_EmbeddedML_Final/STile_M_Pattern/Projects/SensorTile/Applications/BLE_SampleApp/Src/syscalls.c 

OBJS += \
./MotEnv1_ST/SW4STM32/startup_stm32l476xx.o \
./MotEnv1_ST/SW4STM32/syscalls.o 

S_DEPS += \
./MotEnv1_ST/SW4STM32/startup_stm32l476xx.d 

C_DEPS += \
./MotEnv1_ST/SW4STM32/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
MotEnv1_ST/SW4STM32/startup_stm32l476xx.o: C:/Users/thinkpad/Documents/GitHub/SensorTile_EmbeddedML_Final/STile_M_Pattern/Projects/SensorTile/Applications/BLE_SampleApp/SW4STM32/startup_stm32l476xx.s MotEnv1_ST/SW4STM32/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/STM32L4xx_Nucleo -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../Middlewares/ST/STM32_OSX_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_OSX_MotionFX_Library -I../../../Inc -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
MotEnv1_ST/SW4STM32/syscalls.o: C:/Users/thinkpad/Documents/GitHub/SensorTile_EmbeddedML_Final/STile_M_Pattern/Projects/SensorTile/Applications/BLE_SampleApp/Src/syscalls.c MotEnv1_ST/SW4STM32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=c99 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -c -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

