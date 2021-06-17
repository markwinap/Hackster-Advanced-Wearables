# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(BOARD nrf5340dk_nrf5340_cpuapp)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(threads)

# target_sources(app PRIVATE
#     src/main.c
#     src/audio_soundboard_functions.c
#     src/minmea.c
#     )

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})