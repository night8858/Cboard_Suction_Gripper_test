cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(FLOAT_ABI "hard")

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Cboard_Suction_Gripper_test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "-u _printf_float"
    "-Wl,-u,_printf_float"
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    hardware_device/Motor/DM_motor/DM_motor.c
    hardware_device/Motor/DJI_motor/DJI_motor.c
    hardware_device/Motor/BM_motor/BM_motor.c
    hardware_device/Servo/FEETECH/SCSLib/SCS.c
    hardware_device/Servo/FEETECH/SCSLib/SCSCL.c
    hardware_device/Servo/FEETECH/SCSLib/SCSerail.c
    hardware_device/Servo/FEETECH/SCSLib/SMS_STS.c
    hardware_device/remote_control/DT7.c
    bsp/can/bsp_can.c
    bsp/usart/bsp_usart.c
    bsp/led/bsp_led.c
    bsp/led/bsp_led_strip.c
    bsp/solenoid/bsp_solenoid.c
    application/RTOS_TASK/arm_control_task.c
    application/RTOS_TASK/pump_control_task.c
    application/RTOS_TASK/usart_debug_task.c
    application/RTOS_TASK/led_indicate_task.c
    application/RTOS_TASK/solenoid_control_task.c
    application/RTOS_TASK/led_strip_task.c
    application/RTOS_TASK/heartbeat_monitor_task.c
    application/Gripper/arm_control.c
    application/Gripper/Planar_Robot_Arm.c
    application/Gripper/pump_control.c
    application/Gripper/route_planner.c
    application/variables/variables.c
    application/Algorithm/PID.c
    application/Algorithm/user_lib.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    hardware_device/Motor/DM_motor
    hardware_device/Motor/DJI_motor
    hardware_device/Motor/BM_motor
    hardware_device/Servo
    hardware_device/Servo/FEETECH
    hardware_device/Servo/FEETECH/SCSLib
    hardware_device/remote_control
    application/RTOS_TASK
    application/Gripper
    application/variables
    application/Algorithm
    bsp/can
    bsp/usart
    bsp/led
    bsp/solenoid
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
