
######################################
# target
######################################
TARGET = Main

#######################################
# paths
#######################################
# Build path
BUILD_DIR = bin
SRC_GEN = src-gen
INCL = include

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
	openocd -f /Users/$(USER)/openocd/tcl/interface/stlink.cfg -f /Users/$(USER)/openocd/tcl/target/stm32f4x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

flash: build_dev
	openocd -f /Users/$(USER)/openocd/tcl/interface/stlink.cfg -f /Users/$(USER)/openocd/tcl/target/stm32f4x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
