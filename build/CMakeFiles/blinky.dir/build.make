# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bhavya/Work/stm32f44x

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bhavya/Work/stm32f44x/build

# Include any dependencies generated for this target.
include CMakeFiles/blinky.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/blinky.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/blinky.dir/flags.make

CMakeFiles/blinky.dir/src/001LEDtoggle.c.obj: CMakeFiles/blinky.dir/flags.make
CMakeFiles/blinky.dir/src/001LEDtoggle.c.obj: ../src/001LEDtoggle.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bhavya/Work/stm32f44x/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/blinky.dir/src/001LEDtoggle.c.obj"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blinky.dir/src/001LEDtoggle.c.obj   -c /home/bhavya/Work/stm32f44x/src/001LEDtoggle.c

CMakeFiles/blinky.dir/src/001LEDtoggle.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blinky.dir/src/001LEDtoggle.c.i"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/bhavya/Work/stm32f44x/src/001LEDtoggle.c > CMakeFiles/blinky.dir/src/001LEDtoggle.c.i

CMakeFiles/blinky.dir/src/001LEDtoggle.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blinky.dir/src/001LEDtoggle.c.s"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/bhavya/Work/stm32f44x/src/001LEDtoggle.c -o CMakeFiles/blinky.dir/src/001LEDtoggle.c.s

CMakeFiles/blinky.dir/startup/syscalls.c.obj: CMakeFiles/blinky.dir/flags.make
CMakeFiles/blinky.dir/startup/syscalls.c.obj: ../startup/syscalls.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bhavya/Work/stm32f44x/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/blinky.dir/startup/syscalls.c.obj"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blinky.dir/startup/syscalls.c.obj   -c /home/bhavya/Work/stm32f44x/startup/syscalls.c

CMakeFiles/blinky.dir/startup/syscalls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blinky.dir/startup/syscalls.c.i"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/bhavya/Work/stm32f44x/startup/syscalls.c > CMakeFiles/blinky.dir/startup/syscalls.c.i

CMakeFiles/blinky.dir/startup/syscalls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blinky.dir/startup/syscalls.c.s"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/bhavya/Work/stm32f44x/startup/syscalls.c -o CMakeFiles/blinky.dir/startup/syscalls.c.s

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.obj: CMakeFiles/blinky.dir/flags.make
CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.obj: ../drivers/src/stm32f446xx_gpio_driver.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bhavya/Work/stm32f44x/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.obj"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.obj   -c /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_gpio_driver.c

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.i"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_gpio_driver.c > CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.i

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.s"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_gpio_driver.c -o CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.s

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.obj: CMakeFiles/blinky.dir/flags.make
CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.obj: ../drivers/src/stm32f446xx_rcc_driver.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bhavya/Work/stm32f44x/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.obj"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.obj   -c /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_rcc_driver.c

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.i"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_rcc_driver.c > CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.i

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.s"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_rcc_driver.c -o CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.s

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.obj: CMakeFiles/blinky.dir/flags.make
CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.obj: ../drivers/src/stm32f446xx_spi_driver.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bhavya/Work/stm32f44x/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.obj"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.obj   -c /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_spi_driver.c

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.i"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_spi_driver.c > CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.i

CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.s"
	/usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/bhavya/Work/stm32f44x/drivers/src/stm32f446xx_spi_driver.c -o CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.s

# Object files for target blinky
blinky_OBJECTS = \
"CMakeFiles/blinky.dir/src/001LEDtoggle.c.obj" \
"CMakeFiles/blinky.dir/startup/syscalls.c.obj" \
"CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.obj" \
"CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.obj" \
"CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.obj"

# External object files for target blinky
blinky_EXTERNAL_OBJECTS =

blinky: CMakeFiles/blinky.dir/src/001LEDtoggle.c.obj
blinky: CMakeFiles/blinky.dir/startup/syscalls.c.obj
blinky: CMakeFiles/blinky.dir/drivers/src/stm32f446xx_gpio_driver.c.obj
blinky: CMakeFiles/blinky.dir/drivers/src/stm32f446xx_rcc_driver.c.obj
blinky: CMakeFiles/blinky.dir/drivers/src/stm32f446xx_spi_driver.c.obj
blinky: CMakeFiles/blinky.dir/build.make
blinky: CMakeFiles/blinky.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bhavya/Work/stm32f44x/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C executable blinky"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/blinky.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/blinky.dir/build: blinky

.PHONY : CMakeFiles/blinky.dir/build

CMakeFiles/blinky.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/blinky.dir/cmake_clean.cmake
.PHONY : CMakeFiles/blinky.dir/clean

CMakeFiles/blinky.dir/depend:
	cd /home/bhavya/Work/stm32f44x/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bhavya/Work/stm32f44x /home/bhavya/Work/stm32f44x /home/bhavya/Work/stm32f44x/build /home/bhavya/Work/stm32f44x/build /home/bhavya/Work/stm32f44x/build/CMakeFiles/blinky.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/blinky.dir/depend
