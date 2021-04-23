# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(prac2)

target_sources(app PRIVATE src/scu_hci.c src/scu_hcsr04.c src/scu_lsm6dsl.c src/scu_lis3mdl.c src/scu_hts221.c src/scu_vl53l0x.c src/scu_lps22hb.c src/main.c)
target_include_directories(app PRIVATE inc)