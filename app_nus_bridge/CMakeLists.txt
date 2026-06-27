#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

set(BOARD_FLASH_RUNNER nrfutil)
set(BOARD_ROOT $ENV{ZEPHYR_BASE} CACHE PATH "" FORCE)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(app_nus_bridge)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common/CMakeLists.txt)

target_sources(app PRIVATE src/main.cpp)
