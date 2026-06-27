#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

set(BOARD_FLASH_RUNNER nrfutil)
set(BOARD_ROOT $ENV{ZEPHYR_BASE} CACHE PATH "" FORCE)
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../foot_common)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(app_esb_foot_v2_left)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../common/CMakeLists.txt)

target_sources(app PRIVATE
    ../foot_common/src/main.cpp
)

exo_add_bno08x_i2c(app)
exo_add_ads122c04_i2c(app)
exo_add_pcap01_spi(app)
