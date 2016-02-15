################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/chad/vmshare/project/ece544/ip_repo/tsl235r_1.0/drivers/tsl235r_v1_0/src/tsl235r.c \
/home/chad/vmshare/project/ece544/ip_repo/tsl235r_1.0/drivers/tsl235r_v1_0/src/tsl235r_selftest.c 

OBJS += \
./temp_drivers/src/tsl235r.o \
./temp_drivers/src/tsl235r_selftest.o 

C_DEPS += \
./temp_drivers/src/tsl235r.d \
./temp_drivers/src/tsl235r_selftest.d 


# Each subdirectory must supply rules for building sources it contributes
temp_drivers/src/tsl235r.o: /home/chad/vmshare/project/ece544/ip_repo/tsl235r_1.0/drivers/tsl235r_v1_0/src/tsl235r.c
	@echo 'Building file: $<'
	@echo 'Invoking: MicroBlaze gcc compiler'
	mb-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -I../../standalone_bsp_0/microblaze_0/include -mlittle-endian -mcpu=v9.5 -mxl-soft-mul -Wl,--no-relax -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

temp_drivers/src/tsl235r_selftest.o: /home/chad/vmshare/project/ece544/ip_repo/tsl235r_1.0/drivers/tsl235r_v1_0/src/tsl235r_selftest.c
	@echo 'Building file: $<'
	@echo 'Invoking: MicroBlaze gcc compiler'
	mb-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -I../../standalone_bsp_0/microblaze_0/include -mlittle-endian -mcpu=v9.5 -mxl-soft-mul -Wl,--no-relax -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


