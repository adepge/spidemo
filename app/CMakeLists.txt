# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(spi-demo)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR})

include_directories(${CMAKE_BINARY_DIR})
