################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := F:/MCU/11_TI_MSP/CCS_ide/install/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./ball_controller.o" \
"./ball_estimator.o" \
"./ball_fast_estimator.o" \
"./ball_pid_bank.o" \
"./ball_quiet_observer.o" \
"./ball_ui.o" \
"./curve_control_q2.o" \
"./curve_control_v4.o" \
"./curve_control_v5.o" \
"./curve_pid.o" \
"./delay.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./huidu.o" \
"./main.o" \
"./mode_q2.o" \
"./mode_q3.o" \
"./mode_q4.o" \
"./mode_q5.o" \
"./pid.o" \
"./pid_q2.o" \
"./driver/encoder.o" \
"./driver/key.o" \
"./driver/motor.o" \
"./driver/oled.o" \
"./driver/servo.o" \
"./driver/uart.o" \
"./driver/mpu6050/inv_mpu.o" \
"./driver/mpu6050/inv_mpu_dmp_motion_driver.o" \
"./driver/mpu6050/mpu6050.o" \
"./driver/mpu6050/mpu_port.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include driver/subdir_vars.mk
-include driver/mpu6050/subdir_vars.mk
-include subdir_rules.mk
-include driver/subdir_rules.mk
-include driver/mpu6050/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
balance_car_final.out 

EXE_OUTPUTS__QUOTED += \
"balance_car_final.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "balance_car_final.out"

# Tool invocations
balance_car_final.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"F:/MCU/11_TI_MSP/CCS_ide/install/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wall -Wl,-m"balance_car_final.map" -Wl,-i"C:/TI/mspm0_sdk_2_10_00_04/source" -Wl,-i"F:/MCU/11_TI_MSP/CCS_project/2026project/balance_car_final" -Wl,-i"F:/MCU/11_TI_MSP/CCS_project/2026project/balance_car_final/Debug/syscfg" -Wl,-i"F:/MCU/11_TI_MSP/CCS_ide/install/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="balance_car_final_linkInfo.xml" -Wl,--rom_model -o "balance_car_final.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ball_controller.o" "ball_estimator.o" "ball_fast_estimator.o" "ball_pid_bank.o" "ball_quiet_observer.o" "ball_ui.o" "curve_control_q2.o" "curve_control_v4.o" "curve_control_v5.o" "curve_pid.o" "delay.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "huidu.o" "main.o" "mode_q2.o" "mode_q3.o" "mode_q4.o" "mode_q5.o" "pid.o" "pid_q2.o" "driver\encoder.o" "driver\key.o" "driver\motor.o" "driver\oled.o" "driver\servo.o" "driver\uart.o" "driver\mpu6050\inv_mpu.o" "driver\mpu6050\inv_mpu_dmp_motion_driver.o" "driver\mpu6050\mpu6050.o" "driver\mpu6050\mpu_port.o" 
	-$(RM) "ball_controller.d" "ball_estimator.d" "ball_fast_estimator.d" "ball_pid_bank.d" "ball_quiet_observer.d" "ball_ui.d" "curve_control_q2.d" "curve_control_v4.d" "curve_control_v5.d" "curve_pid.d" "delay.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "huidu.d" "main.d" "mode_q2.d" "mode_q3.d" "mode_q4.d" "mode_q5.d" "pid.d" "pid_q2.d" "driver\encoder.d" "driver\key.d" "driver\motor.d" "driver\oled.d" "driver\servo.d" "driver\uart.d" "driver\mpu6050\inv_mpu.d" "driver\mpu6050\inv_mpu_dmp_motion_driver.d" "driver\mpu6050\mpu6050.d" "driver\mpu6050\mpu_port.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

