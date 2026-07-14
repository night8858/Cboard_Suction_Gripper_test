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
    # Add user defined library search paths
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-Wl,-u,_printf_float"
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
    bsp/can/bsp_can.c
    bsp/usart/bsp_usart.c
    bsp/led/bsp_led.c
    bsp/led/bsp_led_strip.c 
    bsp/virtual_serial_port/virtual_serial_port.c   
    application/RTOS_TASK/arm_control_task.c
    application/RTOS_TASK/pump_control_task.c
    application/RTOS_TASK/led_indicate_task.c
    application/RTOS_TASK/led_indicate_state.c
    application/RTOS_TASK/update_task.c
    application/RTOS_TASK/gimbal_task.c
    application/Gripper/pneumatic_control.c
    application/Gripper/command_decode_4dof.c
    application/Gripper/gimbal.c
    application/Gripper/block_inspect.c
    application/Gripper/action_scheduler_4dof.c
    application/Gripper/pc_action_executor_4dof.c
    application/Gripper/Dof4_Arm.c
    application/Gripper/Dof4_Arm_Calibration.c
    application/Gripper/Dof4_Collision.c
    application/Gripper/Trajectory_Planning.c
    application/variables/variables.c
    application/Algorithm/PID.c
    application/Algorithm/user_lib.c
    hardware_device/voice_module/xiao_R/usart_interface.c
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
    application/RTOS_TASK
    application/Gripper
    application/variables
    application/Algorithm
    bsp/can
    bsp/usart
    bsp/led
    bsp/solenoid
    bsp/virtual_serial_port
    bsp/virtual_serial_port
    hardware_device/voice_module/xiao_R
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
