################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Src/Startup/startup_stm32f446retx.s 

OBJS += \
./Src/Startup/startup_stm32f446retx.o 

S_DEPS += \
./Src/Startup/startup_stm32f446retx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Startup/startup_stm32f446retx.o: ../Src/Startup/startup_stm32f446retx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Src/Startup/startup_stm32f446retx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

