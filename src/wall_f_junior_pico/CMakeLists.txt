cmake_minimum_required(VERSION 3.8)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(wall_f_junior_pico LANGUAGES C CXX)

pico_sdk_init()

link_directories(libmicroros)

add_executable(pico_micro_ros_example
    src/pico_micro_ros_example.c
    src/pico_uart_transport.c
)

target_link_libraries(pico_micro_ros_example
    pico_stdlib
    microros
)

target_include_directories(pico_micro_ros_example PUBLIC
    include
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_micro_ros_example 1)
pico_enable_stdio_uart(pico_micro_ros_example 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_micro_ros_example)