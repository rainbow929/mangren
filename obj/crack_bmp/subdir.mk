################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../crack_bmp/crack_EN.c \
../crack_bmp/crack_ES.c \
../crack_bmp/crack_NE.c \
../crack_bmp/crack_NW.c \
../crack_bmp/crack_SE.c \
../crack_bmp/crack_SW.c \
../crack_bmp/crack_WN.c \
../crack_bmp/crack_WS.c 

OBJS += \
./crack_bmp/crack_EN.o \
./crack_bmp/crack_ES.o \
./crack_bmp/crack_NE.o \
./crack_bmp/crack_NW.o \
./crack_bmp/crack_SE.o \
./crack_bmp/crack_SW.o \
./crack_bmp/crack_WN.o \
./crack_bmp/crack_WS.o 

C_DEPS += \
./crack_bmp/crack_EN.d \
./crack_bmp/crack_ES.d \
./crack_bmp/crack_NE.d \
./crack_bmp/crack_NW.d \
./crack_bmp/crack_SE.d \
./crack_bmp/crack_SW.d \
./crack_bmp/crack_WN.d \
./crack_bmp/crack_WS.d 


# Each subdirectory must supply rules for building sources it contributes
crack_bmp/%.o: ../crack_bmp/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g -I"F:\qianrushi\opench-chitu-game-demos-master\mangren\mangren\Debug" -I"F:\qianrushi\opench-chitu-game-demos-master\mangren\mangren\Core" -I"F:\qianrushi\opench-chitu-game-demos-master\mangren\mangren\User" -I"F:\qianrushi\opench-chitu-game-demos-master\mangren\mangren\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

