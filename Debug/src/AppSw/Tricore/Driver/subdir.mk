################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AppSw/Tricore/Driver/LQ_ADC.c \
../src/AppSw/Tricore/Driver/LQ_CCU6.c \
../src/AppSw/Tricore/Driver/LQ_DMA.c \
../src/AppSw/Tricore/Driver/LQ_EEPROM.c \
../src/AppSw/Tricore/Driver/LQ_GPIO.c \
../src/AppSw/Tricore/Driver/LQ_GPSR.c \
../src/AppSw/Tricore/Driver/LQ_GPT12_ENC.c \
../src/AppSw/Tricore/Driver/LQ_GTM.c \
../src/AppSw/Tricore/Driver/LQ_QSPI.c \
../src/AppSw/Tricore/Driver/LQ_SOFTI2C.c \
../src/AppSw/Tricore/Driver/LQ_SPI.c \
../src/AppSw/Tricore/Driver/LQ_STM.c \
../src/AppSw/Tricore/Driver/LQ_UART.c 

OBJS += \
./src/AppSw/Tricore/Driver/LQ_ADC.o \
./src/AppSw/Tricore/Driver/LQ_CCU6.o \
./src/AppSw/Tricore/Driver/LQ_DMA.o \
./src/AppSw/Tricore/Driver/LQ_EEPROM.o \
./src/AppSw/Tricore/Driver/LQ_GPIO.o \
./src/AppSw/Tricore/Driver/LQ_GPSR.o \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.o \
./src/AppSw/Tricore/Driver/LQ_GTM.o \
./src/AppSw/Tricore/Driver/LQ_QSPI.o \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.o \
./src/AppSw/Tricore/Driver/LQ_SPI.o \
./src/AppSw/Tricore/Driver/LQ_STM.o \
./src/AppSw/Tricore/Driver/LQ_UART.o 

COMPILED_SRCS += \
./src/AppSw/Tricore/Driver/LQ_ADC.src \
./src/AppSw/Tricore/Driver/LQ_CCU6.src \
./src/AppSw/Tricore/Driver/LQ_DMA.src \
./src/AppSw/Tricore/Driver/LQ_EEPROM.src \
./src/AppSw/Tricore/Driver/LQ_GPIO.src \
./src/AppSw/Tricore/Driver/LQ_GPSR.src \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.src \
./src/AppSw/Tricore/Driver/LQ_GTM.src \
./src/AppSw/Tricore/Driver/LQ_QSPI.src \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.src \
./src/AppSw/Tricore/Driver/LQ_SPI.src \
./src/AppSw/Tricore/Driver/LQ_STM.src \
./src/AppSw/Tricore/Driver/LQ_UART.src 

C_DEPS += \
./src/AppSw/Tricore/Driver/LQ_ADC.d \
./src/AppSw/Tricore/Driver/LQ_CCU6.d \
./src/AppSw/Tricore/Driver/LQ_DMA.d \
./src/AppSw/Tricore/Driver/LQ_EEPROM.d \
./src/AppSw/Tricore/Driver/LQ_GPIO.d \
./src/AppSw/Tricore/Driver/LQ_GPSR.d \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.d \
./src/AppSw/Tricore/Driver/LQ_GTM.d \
./src/AppSw/Tricore/Driver/LQ_QSPI.d \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.d \
./src/AppSw/Tricore/Driver/LQ_SPI.d \
./src/AppSw/Tricore/Driver/LQ_STM.d \
./src/AppSw/Tricore/Driver/LQ_UART.d 


# Each subdirectory must supply rules for building sources it contributes
src/AppSw/Tricore/Driver/%.src: ../src/AppSw/Tricore/Driver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc37x -I"E:\ADS\Camr\LQ_TC377TP_LIB_ADS\src\AppSw\Tricore\Camera_m" -I"E:\ADS\Camr\LQ_TC377TP_LIB_ADS\src\AppSw\Tricore\UI" -I"E:\ADS\Camr\LQ_TC377TP_LIB_ADS\src\AppSw\Tricore\APP" -I"E:\ADS\Camr\LQ_TC377TP_LIB_ADS\src\AppSw\Tricore\MIne" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/src/AppSw/Tricore/Main" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/src/AppSw/Tricore/Driver" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/src/AppSw/Tricore/User" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/src/AppSw" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Tom/PwmHl" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Platform/Tricore/Compilers" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Platform" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Iom/Iom" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/If/Ccu6If" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Iom/Driver" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Sent" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gpt12/IncrEnc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/I2c" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Psi5s/Psi5s" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Comm" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Math" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Platform/Tricore" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Sent/Sent" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Msc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Ccu6/Icu" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Qspi" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Ssw/TC37A" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Qspi/SpiMaster" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/_Lib" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Dma/Dma" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/SysSe" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Ccu6/TPwm" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gpt12/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/If" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Msc/Msc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/_Lib/DataHandling" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Asclin/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Edsadc/Edsadc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Sent/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Ccu6/Timer" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gpt12" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Convctrl/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Psi5/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Stm" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Edsadc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Sfr/TC37A" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Ssw/TC37A/Tricore" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Psi5" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Sfr/TC37A/_Reg" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Flash" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Scu" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Eray" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Hssl" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Msc/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Asclin" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Fce" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Smu/Smu" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Port/Io" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Convctrl" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Geth/Eth" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Tom" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Sfr" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Ccu6/TimerWithTrigger" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/_PinMap" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/_Lib/InternalMux" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Bsp" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/General" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Smu" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Evadc/Adc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Tim/Timer" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Port" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Configurations" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Flash/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Cpu/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Tom/Pwm" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Asclin/Lin" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Fce/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Port/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Dts" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Src" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Dma" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Stm/Timer" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Atom/PwmHl" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Ccu6/PwmHl" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Smu/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Iom" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Ccu6" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Evadc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Hssl/Hssl" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Tim" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Time" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Geth" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Qspi/SpiSlave" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Atom" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Pms/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Pms" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Asclin/Spi" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Eray/Eray" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Hssl/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/I2c/I2c" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Dts/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Can/Can" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Mtu/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Eray/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Cpu" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Scu/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Stm/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Atom/Pwm" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Psi5s" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Cpu/Irq" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Ccu6/PwmBc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Cpu/Trap" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Infra/Ssw" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/StdIf" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Trig" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Mtu" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Edsadc/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Fce/Crc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Psi5/Psi5" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Tim/In" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Qspi/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Configurations/Debug" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Can/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Src/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Dma/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Psi5s/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/Service/CpuGeneric/_Utilities" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Atom/Timer" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Iom/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Asclin/Asc" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/I2c/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Ccu6/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Geth/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Gtm/Tom/Timer" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Can" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Dts/Dts" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/Evadc/Std" -I"E:/ADS/Camr/LQ_TC377TP_LIB_ADS/Libraries/iLLD/TC37A/Tricore/_Impl" --iso=99 --c++14 --language=+volatile --anachronisms --fp-model=3 --fp-model=c --fp-model=f --fp-model=l --fp-model=n --fp-model=r --fp-model=z -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -o "$@"  "$<"  -cs --dep-file=$(@:.src=.d) --misrac-version=2012 -N0 -Z0 -Y0 2>&1; sed -i -e '/ctc\\include/d' -e '/Libraries\\iLLD/d' -e '/Libraries\\Infra/d' -e 's/\(.*\)".*\\LQ_TC377TP_LIB_ADS\(\\.*\)"/\1\.\.\2/g' -e 's/\\/\//g' $(@:.src=.d) && \
	echo $(@:.src=.d) generated
	@echo 'Finished building: $<'
	@echo ' '

src/AppSw/Tricore/Driver/%.o: ./src/AppSw/Tricore/Driver/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


