cmake_minimum_required(VERSION 3.19)

include(pico_sdk_import.cmake)
project(blink)

set(PICO_EXAMPLE_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()

add_executable(blink
		main.c
)

target_link_libraries(blink 
	pico_stdlib
	hardware_gpio
	hardware_pwm
)

pico_add_extra_outputs(blink)