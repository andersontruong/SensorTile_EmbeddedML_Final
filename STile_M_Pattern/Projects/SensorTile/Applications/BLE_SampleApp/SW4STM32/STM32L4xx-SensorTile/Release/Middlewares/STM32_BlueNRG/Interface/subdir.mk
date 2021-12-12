################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/thinkpad/Documents/GitHub/SensorTile_EmbeddedML_Final/STile_M_Pattern/Middlewares/ST/STM32_BlueNRG/Interface/bluenrg_itf_template.c 

OBJS += \
./Middlewares/STM32_BlueNRG/Interface/bluenrg_itf_template.o 

C_DEPS += \
./Middlewares/STM32_BlueNRG/Interface/bluenrg_itf_template.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/STM32_BlueNRG/Interface/bluenrg_itf_template.o: C:/Users/thinkpad/Documents/GitHub/SensorTile_EmbeddedML_Final/STile_M_Pattern/Middlewares/ST/STM32_BlueNRG/Interface/bluenrg_itf_template.c Middlewares/STM32_BlueNRG/Interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=c99 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -c -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

