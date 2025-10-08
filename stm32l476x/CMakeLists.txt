cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(stm32l476x_setup C ASM)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

set(PROCESSOR_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

if(HAL)
    set(COMPILATION_DEFINE "-DSTM32L476xx -DHAL")
else()
    set(COMPILATION_DEFINE "-DSTM32L476xx ")
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${PROCESSOR_FLAGS} ${COMPILATION_DEFINE} -Wall -Wextra -g -O0")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32L476RGTX_FLASH.ld)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -specs=nosys.specs -T${LINKER_SCRIPT}")

if(HAL)
    message("[INFO] st_hal build")

    file(GLOB_RECURSE SOURCE "src/*.c"
                             "./src/drivers/*.c"
                             "./src/device/*.s"
                             "./src/device/*.c"
                             "./lib/RTT/SEGGER_RTT.c"
                             "./lib/RTT/SEGGER_RTT_printf.c")
else() 
    file(GLOB_RECURSE SOURCE "src/*.c"
                             "./src/device/*.c"
                             "./src/device/*.s"
                             "./lib/RTT/SEGGER_RTT.c"
                             "./lib/RTT/SEGGER_RTT_printf.c")
endif()

include_directories(src)
include_directories(src/drivers)
include_directories(src/device)
include_directories(lib/RTT)

add_executable(${PROJECT_NAME}.elf ${SOURCE})

