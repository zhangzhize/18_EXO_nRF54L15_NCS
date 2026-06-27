#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# Shared CMake, included by each app's CMakeLists.txt after find_package().
# Keep this file limited to reusable code. App-specific protocols and helpers
# are exposed as opt-in functions below.

set(EXO_COMMON_DIR ${CMAKE_CURRENT_LIST_DIR})

target_include_directories(app PRIVATE ${EXO_COMMON_DIR}/inc)
zephyr_include_directories(${EXO_COMMON_DIR}/inc)

# Always-available project data and board helpers.
target_sources(app PRIVATE
    ${EXO_COMMON_DIR}/src/exo_data.c
    ${EXO_COMMON_DIR}/src/bsp_led.c
)

# Reusable protocol parsers and app utilities. Apps opt in explicitly.
function(exo_add_ads1220_spi target)
    target_sources(${target} PRIVATE ${EXO_COMMON_DIR}/src/bsp_ads1220.cpp)
endfunction()

function(exo_add_pcap01_spi target)
    target_sources(${target} PRIVATE ${EXO_COMMON_DIR}/src/pcap01.cpp)
endfunction()

function(exo_add_mlx90393_i2c target)
    target_sources(${target} PRIVATE ${EXO_COMMON_DIR}/src/bsp_mlx90393.cpp)
endfunction()

function(exo_add_ads122c04_i2c target)
    target_sources(${target} PRIVATE ${EXO_COMMON_DIR}/src/SparkFun_ADS122C04_Library.cpp)
endfunction()

function(exo_add_bno08x_i2c target)
    target_sources(${target} PRIVATE
        ${EXO_COMMON_DIR}/src/SparkFun_BNO08x_Library.cpp
        ${EXO_COMMON_DIR}/src/sh2.c
        ${EXO_COMMON_DIR}/src/sh2_SensorValue.c
        ${EXO_COMMON_DIR}/src/sh2_util.c
        ${EXO_COMMON_DIR}/src/shtp.c
    )
endfunction()

# Hardware drivers selected by Kconfig.
if(CONFIG_ADC)
    target_sources(app PRIVATE ${EXO_COMMON_DIR}/src/bsp_adc.c)
endif()

if(CONFIG_ESB)
    target_sources(app PRIVATE ${EXO_COMMON_DIR}/src/bsp_esb.c)
endif()

if(CONFIG_BT)
    target_sources(app PRIVATE ${EXO_COMMON_DIR}/src/bsp_ble.c)
endif()

if(CONFIG_BT_CENTRAL)
    target_sources(app PRIVATE ${EXO_COMMON_DIR}/src/bsp_ble_central.c)
endif()

if(CONFIG_BT_PERIPHERAL)
    target_sources(app PRIVATE ${EXO_COMMON_DIR}/src/bsp_ble_periph.c)
endif()

if(CONFIG_SERIAL)
    target_sources(app PRIVATE ${EXO_COMMON_DIR}/src/bsp_uart.c)
endif()
