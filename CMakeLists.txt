cmake_minimum_required(VERSION 3.19)

#set(PICO_CYW43_SUPPORTED=1) #Required for pico w libraries
include(pico_sdk_import.cmake)
project(main)

set(PICO_EXAMPLE_PATH ${PROJECT_SOURCE_DIR})


pico_sdk_init()

add_executable(main
		main.c
)


target_link_libraries(main 
	pico_stdlib
	hardware_gpio
	hardware_spi
	hardware_pwm
	
)

pico_add_extra_outputs(main)