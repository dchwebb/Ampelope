################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/SerialHandler.cpp \
../src/USB.cpp \
../src/envelope.cpp \
../src/initialisation.cpp \
../src/main.cpp \
../src/uartHandler.cpp 

S_SRCS += \
../src/startup_stm32g431rbtx.s 

C_SRCS += \
../src/syscalls.c \
../src/sysmem.c \
../src/system_stm32g4xx.c 

S_DEPS += \
./src/startup_stm32g431rbtx.d 

C_DEPS += \
./src/syscalls.d \
./src/sysmem.d \
./src/system_stm32g4xx.d 

OBJS += \
./src/SerialHandler.o \
./src/USB.o \
./src/envelope.o \
./src/initialisation.o \
./src/main.o \
./src/startup_stm32g431rbtx.o \
./src/syscalls.o \
./src/sysmem.o \
./src/system_stm32g4xx.o \
./src/uartHandler.o 

CPP_DEPS += \
./src/SerialHandler.d \
./src/USB.d \
./src/envelope.d \
./src/initialisation.d \
./src/main.d \
./src/uartHandler.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o src/%.su src/%.cyclo: ../src/%.cpp src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++17 -g3 -DDEBUG -DSTM32G431xx -c -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -Wno-unused-variable -ffast-math -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/%.o: ../src/%.s src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
src/%.o src/%.su src/%.cyclo: ../src/%.c src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32G431xx -c -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src

clean-src:
	-$(RM) ./src/SerialHandler.cyclo ./src/SerialHandler.d ./src/SerialHandler.o ./src/SerialHandler.su ./src/USB.cyclo ./src/USB.d ./src/USB.o ./src/USB.su ./src/envelope.cyclo ./src/envelope.d ./src/envelope.o ./src/envelope.su ./src/initialisation.cyclo ./src/initialisation.d ./src/initialisation.o ./src/initialisation.su ./src/main.cyclo ./src/main.d ./src/main.o ./src/main.su ./src/startup_stm32g431rbtx.d ./src/startup_stm32g431rbtx.o ./src/syscalls.cyclo ./src/syscalls.d ./src/syscalls.o ./src/syscalls.su ./src/sysmem.cyclo ./src/sysmem.d ./src/sysmem.o ./src/sysmem.su ./src/system_stm32g4xx.cyclo ./src/system_stm32g4xx.d ./src/system_stm32g4xx.o ./src/system_stm32g4xx.su ./src/uartHandler.cyclo ./src/uartHandler.d ./src/uartHandler.o ./src/uartHandler.su

.PHONY: clean-src

