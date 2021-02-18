cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/arm-gcc-cortex-m4.cmake")
endif()

include(${CMAKE_SOURCE_DIR}/cmake/utilities.cmake)

project(blinky)

set(SOURCES 
	src/001LEDtoggle.c
	startup/startup_stm32.s
	startup/syscalls.c
	#	startup/sysmem.c

)
set(DRIVERS
	drivers/src/stm32f446xx_gpio_driver.c
	drivers/src/stm32f446xx_rcc_driver.c
	#drivers/src/stm32f446xx_usart_driver.c
	#drivers/src/stm32f446xx_i2c_driver.c
	drivers/src/stm32f446xx_spi_driver.c
)
set(HEADER_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/drivers/inc/stm32f446xx.h
	${CMAKE_CURRENT_SOURCE_DIR}/drivers/inc/stm32f446xx_gpio_driver.h
	#${CMAKE_CURRENT_SOURCE_DIR}/drivers/inc/stm32f446xx_i2c_driver.h
	${CMAKE_CURRENT_SOURCE_DIR}/drivers/inc/stm32f446xx_spi_driver.h
	${CMAKE_CURRENT_SOURCE_DIR}/drivers/inc/stm32f446xx_rcc_driver.h
	#${CMAKE_CURRENT_SOURCE_DIR}/drivers/inc/stm32f446xx_usart_driver.h
)
set(LINKER_SCRIPT "LinkerScript.ld")
set_target_properties(${TARGET_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

add_executable(blinky ${SOURCES} ${DRIVERS})

