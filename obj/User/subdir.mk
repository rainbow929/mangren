################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32v30x_it.c \
../User/gpio.c \
../User/i2c.c \
../User/imu_mpu6050.c \
../User/lcd_st7789.c \
../User/main.c \
../User/ov.c \
../User/system_ch32v30x.c \
../User/timer.c 

OBJS += \
./User/ch32v30x_it.o \
./User/gpio.o \
./User/i2c.o \
./User/imu_mpu6050.o \
./User/lcd_st7789.o \
./User/main.o \
./User/ov.o \
./User/system_ch32v30x.o \
./User/timer.o 

C_DEPS += \
./User/ch32v30x_it.d \
./User/gpio.d \
./User/i2c.d \
./User/imu_mpu6050.d \
./User/lcd_st7789.d \
./User/main.d \
./User/ov.d \
./User/system_ch32v30x.d \
./User/timer.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g -I"F:\qianrushi\opench-chitu-game-demos-master\mangren\mangren\Debug" -I"F:\qianrushi\opench-chitu-game-demos-master\mangren\mangren\Core" -I"F:\qianrushi\opench-chitu-game-demos-master\mangren\mangren\User" -I"F:\qianrushi\opench-chitu-game-demos-master\mangren\mangren\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

