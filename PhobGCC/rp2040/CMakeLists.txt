cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(phobgcc_rp2040 CXX C)

pico_sdk_init()

include_directories(
	include
	../common
	../extras
	)

add_executable(phobgcc_rp2040
	src/main.cpp
	src/storage.cpp
	src/comms/joybus.cpp
	)

pico_generate_pio_header(phobgcc_rp2040 ${CMAKE_CURRENT_LIST_DIR}/pio/joybus.pio)

add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/joybus.pio.h
	DEPENDS ${CMAKE_CURRENT_LIST_DIR}/joybus.pio
	COMMAND Pioasm ${CMAKE_CURRENT_LIST_DIR}/joybus.pio ${CMAKE_CURRENT_LIST_DIR}/generated/joybus.pio.h
	)

target_link_libraries(phobgcc_rp2040 pico_stdlib hardware_adc hardware_flash hardware_pio hardware_pwm hardware_spi hardware_sync)

pico_add_extra_outputs(phobgcc_rp2040)
