# Copyright (c) 2023 Intel Corporation.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(llext_hello_world_test)

# TODO check which architecture is being used
if(CONFIG_ARM)
    set(CMAKE_C_FLAGS "-mlong-calls" "-mthumb")

    add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/hello_world.llext
	COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAGS} -c
		-o ${PROJECT_BINARY_DIR}/hello_world.llext
		${PROJECT_SOURCE_DIR}/hello_world.c
    )
elseif(CONFIG_XTENSA)
    set(CMAKE_C_FLAGS "-shared" "-fPIC" "-nostdlib" "-nodefaultlibs")

    add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/hello_world.llext
	COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAGS}
		-o ${PROJECT_BINARY_DIR}/hello_world.pre.llext
		${PROJECT_SOURCE_DIR}/hello_world.c
	COMMAND ${CROSS_COMPILE}strip -R .xt.*
		-o ${PROJECT_BINARY_DIR}/hello_world.llext
		${PROJECT_BINARY_DIR}/hello_world.pre.llext
    )
endif()

set(HELLO_WORLD_LLEXT ${PROJECT_BINARY_DIR}/hello_world.llext PARENT_SCOPE)

add_llext_target(hello_world
  OUTPUT  ${llext_bin_file}
  SOURCES ${llext_src_file}
)

generate_inc_file_for_target(app ${llext_bin_file} ${llext_inc_file})
