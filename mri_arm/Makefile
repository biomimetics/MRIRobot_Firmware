
######################################
# target
######################################
TARGET = Main
TARGET_EX = Main_TXRX
#######################################
# paths
#######################################
# Build path
BUILD_DIR = bin
SRC_GEN = src-gen
INCL = include

UNAME_S := $(shell uname -s)
OPENOCD_PATH 		:= 
	ifeq ($(UNAME_S),Darwin) 
		OPENOCD_PATH = /Users/$(USER)/openocd
	endif
	ifeq ($(UNAME_S),Linux)
		OPENOCD_PATH = /home/$(USER)/openocd
	endif

#######################################
# clean up
#######################################
clean: 
	-rm -rf $(SRC_GEN)/
	-rm -rf $(BUILD_DIR)/
	-rm -rf $(INCL)/
	-rm -rf build/
	-rm -rf target/

#######################################
# run everything
#######################################
build_dev:
	lfc-dev src/Main.lf

build:
	lfc src/Main.lf

#######################################
# run everything
#######################################
build_test:
	-rm -fR src-gen/Main/build
	cd src-gen/Main && cmake . -Bbuild
	cd src-gen/Main/build && make

#######################################
# flash the controller
#######################################
just_flash:
	openocd -f $(OPENOCD_PATH)/tcl/interface/stlink.cfg -f $(OPENOCD_PATH)/tcl/target/stm32f4x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
	
flash: build_dev just_flash	

flash_ex: 
	lfc-dev src/Main_TXRX.lf 
	openocd -f $(OPENOCD_PATH)/tcl/interface/stlink.cfg -f $(OPENOCD_PATH)/tcl/target/stm32f4x.cfg -c "program $(BUILD_DIR)/$(TARGET_EX).elf verify reset exit"	



#######################################
# For runing scripts
#######################################
intf:
	python3 scripts/joint_interface.py

clean_data:
	-rm -rf scripts/data/*

#######################################
# For running communication debugging programs
#######################################
comms:
	gcc src_c/uart_comms_example.c src_c/linux_comms.c src_c/stm_comms.c -o uart_comms_example

comms_logger:
	gcc src_c/uart_comms_logger.c src_c/linux_comms.c src_c/stm_comms.c -o uart_logger

comms_listener:
	gcc src_c/uart_comms_listener.c src_c/linux_comms.c src_c/stm_comms.c -o uart_listener