################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f446xx_gpio_driver.c \
../drivers/src/stm32f446xx_i2c_driver.c \
../drivers/src/stm32f446xx_spi_driver.c \
../drivers/src/stm32f446xx_usart_driver.c 

OBJS += \
./drivers/src/stm32f446xx_gpio_driver.o \
./drivers/src/stm32f446xx_i2c_driver.o \
./drivers/src/stm32f446xx_spi_driver.o \
./drivers/src/stm32f446xx_usart_driver.o 

C_DEPS += \
./drivers/src/stm32f446xx_gpio_driver.d \
./drivers/src/stm32f446xx_i2c_driver.d \
./drivers/src/stm32f446xx_spi_driver.d \
./drivers/src/stm32f446xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -I"/home/bhavya/workspace/stm32/drivers/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


