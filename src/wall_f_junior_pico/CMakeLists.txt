cmake_minimum_required(VERSION 3.8)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(wall_f_junior_pico LANGUAGES C CXX)

option(STDIO_USB "Use Pico USB IO" ON)
option(STDIO_UART "Use Pico uart IO" OFF)
option(BUILD_TEST "Build test" OFF)

pico_sdk_init()

link_directories(libmicroros)

include_directories(
    include
    libmicroros/include
)

add_executable(driver_motor_agent
    src/driver_motor_agent.c
    src/pico_uart_transport.c
)

target_link_libraries(driver_motor_agent
    pico_stdlib
    hardware_pwm
    microros
)

if (BUILD_TEST)
    add_executable(pico_micro_ros_example
        src/test/pico_micro_ros_example.c
        src/pico_uart_transport.c
    )

    target_link_libraries(pico_micro_ros_example
        pico_stdlib
        microros
    )

    add_executable(test_driver_motor
        src/test/test_driver_motor.c
    )

    target_link_libraries(test_driver_motor
        pico_stdlib
        hardware_pwm
    )
endif()

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_enable_stdio_usb(driver_motor_agent STDIO_USB)
pico_enable_stdio_uart(driver_motor_agent STDIO_UART)

# Configure Pico
if (BUILD_TEST)
    pico_enable_stdio_usb(pico_micro_ros_example STDIO_USB)
    pico_enable_stdio_uart(pico_micro_ros_example STDIO_UART)
    pico_enable_stdio_usb(test_driver_motor STDIO_USB)
    pico_enable_stdio_uart(test_driver_motor STDIO_UART)
endif()

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(driver_motor_agent)

if (BUILD_TEST)
    pico_add_extra_outputs(pico_micro_ros_example)
    pico_add_extra_outputs(test_driver_motor)
endif()
