set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Resolve the GNU Arm Embedded Toolchain from an explicit override first, then
# fall back to the user's PATH. This keeps the project independent of local
# package managers and versioned installation directories.
set(STM32_TOOLCHAIN_PATH "" CACHE PATH "Path to the STM32 arm-none-eabi toolchain bin directory")

set(_STM32_TOOLCHAIN_CANDIDATES)
if(DEFINED ENV{STM32_TOOLCHAIN_PATH} AND NOT "$ENV{STM32_TOOLCHAIN_PATH}" STREQUAL "")
    list(APPEND _STM32_TOOLCHAIN_CANDIDATES "$ENV{STM32_TOOLCHAIN_PATH}")
endif()
if(STM32_TOOLCHAIN_PATH)
    list(APPEND _STM32_TOOLCHAIN_CANDIDATES "${STM32_TOOLCHAIN_PATH}")
endif()

set(_STM32_SELECTED_TOOLCHAIN_PATH "")
foreach(_STM32_TOOLCHAIN_CANDIDATE IN LISTS _STM32_TOOLCHAIN_CANDIDATES)
    if(EXISTS "${_STM32_TOOLCHAIN_CANDIDATE}/arm-none-eabi-gcc")
        set(_STM32_SELECTED_TOOLCHAIN_PATH "${_STM32_TOOLCHAIN_CANDIDATE}")
        break()
    endif()
endforeach()

if(NOT _STM32_SELECTED_TOOLCHAIN_PATH)
    find_program(_STM32_SYSTEM_ARM_NONE_EABI_GCC arm-none-eabi-gcc)
    if(_STM32_SYSTEM_ARM_NONE_EABI_GCC)
        get_filename_component(_STM32_SELECTED_TOOLCHAIN_PATH "${_STM32_SYSTEM_ARM_NONE_EABI_GCC}" DIRECTORY)
    endif()
endif()

if(NOT _STM32_SELECTED_TOOLCHAIN_PATH)
    message(FATAL_ERROR
        "Could not find arm-none-eabi-gcc. Install GNU Arm Embedded Toolchain "
        "and add its bin directory to PATH, or set STM32_TOOLCHAIN_PATH to the "
        "directory containing arm-none-eabi-gcc."
    )
endif()

set(STM32_TOOLCHAIN_PATH "${_STM32_SELECTED_TOOLCHAIN_PATH}" CACHE PATH "Path to the STM32 arm-none-eabi toolchain bin directory" FORCE)
set(TOOLCHAIN_PREFIX "${STM32_TOOLCHAIN_PATH}/arm-none-eabi-")

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
