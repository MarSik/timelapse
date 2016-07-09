# Example cmake project file used with stm32l1 toolchain
cmake_minimum_required(VERSION 2.8)
PROJECT(timelapse C)

# Set used CPU 
SET(CPU msp430g2553)
SET(F_CPU 8000000)

# Target name must match project name !!
ADD_EXECUTABLE(${PROJECT_NAME} main.c)

# Project specific toolchain initialization
INCLUDE(${TOOLCHAIN_INIT})

# Additional defines and properties

