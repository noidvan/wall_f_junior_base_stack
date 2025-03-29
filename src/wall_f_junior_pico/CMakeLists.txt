cmake_minimum_required(VERSION 3.8)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(wall_f_junior_pico LANGUAGES C CXX)

option(STDIO_USB "Use Pico USB IO" 1)
option(STDIO_UART "Use Pico uart IO" 1)
option(BUILD_TEST "Build test" false)

set(BME68X_PORT     "i2c1"                 CACHE STRING "i2c-port name")
set(BME68X_SDA      "2"                    CACHE STRING "SDA pin-number for BME68X")
set(BME68X_SCL      "3"                    CACHE STRING "SCL pin-number for BME68X")
set(BME68X_ADDR     "BME68X_I2C_ADDR_LOW"  CACHE STRING "BME68x address constant")

pico_sdk_init()

add_subdirectory(pico-bme688)
link_directories(libmicroros)

include_directories(
    include
    libmicroros/include
)

add_executable(rear_agent
    src/rear_agent.c
    src/pico_uart_transport.c
)

target_link_libraries(rear_agent
    pico_stdlib
    microros
    hardware_adc
    hardware_i2c
    bme68x
    hardware_pwm
    pico_stdio_uart
)

add_executable(front_agent
    src/front_agent.c
    src/pico_uart_transport.c
)

target_link_libraries(front_agent
    pico_stdlib
    microros
    hardware_i2c
    pico_stdio_uart
)

add_executable(first_section_agent
    src/first_section_agent.c
    src/pico_uart_transport.c
)

target_link_libraries(first_section_agent
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

    add_executable(test_air_sensor
        src/test/test_air_sensor.c
        src/pico_uart_transport.c
    )

    target_link_libraries(test_air_sensor
        pico_stdlib
        microros
        hardware_i2c
        bme68x
    )

    add_executable(test_soil_sensor
        src/test/test_soil_sensor.c
        src/pico_uart_transport.c
    )

    target_link_libraries(test_soil_sensor
        pico_stdlib
        microros
        hardware_adc
    )

    add_executable(test_driver_motor_agent
        src/test/test_driver_motor_agent.c
        src/pico_uart_transport.c
    )

    target_link_libraries(test_driver_motor_agent
        pico_stdlib
        hardware_pwm
        microros
    )

    add_executable(test_stepper_motor_agent
        src/test/test_stepper_motor_agent.c
        src/pico_uart_transport.c
    )

    target_link_libraries(test_stepper_motor_agent
        pico_stdlib
        microros
    )

    add_executable(test_imu
        src/test/test_imu.c
        src/pico_uart_transport.c
    )

    target_link_libraries(test_imu
        pico_stdlib
        microros
        hardware_i2c
    )
endif()

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_enable_stdio_usb(rear_agent STDIO_USB)
pico_enable_stdio_uart(rear_agent STDIO_UART)

pico_enable_stdio_usb(front_agent STDIO_USB)
pico_enable_stdio_uart(front_agent STDIO_UART)

pico_enable_stdio_usb(first_section_agent STDIO_USB)
pico_enable_stdio_uart(first_section_agent STDIO_UART)

# Configure Pico
if (BUILD_TEST)
    pico_enable_stdio_usb(pico_micro_ros_example STDIO_USB)
    pico_enable_stdio_uart(pico_micro_ros_example STDIO_UART)
    pico_enable_stdio_usb(test_driver_motor STDIO_USB)
    pico_enable_stdio_uart(test_driver_motor STDIO_UART)
    pico_enable_stdio_usb(test_air_sensor STDIO_USB)
    pico_enable_stdio_uart(test_air_sensor STDIO_UART)
    pico_enable_stdio_usb(test_soil_sensor STDIO_USB)
    pico_enable_stdio_uart(test_soil_sensor STDIO_UART)
    pico_enable_stdio_usb(test_driver_motor_agent STDIO_USB)
    pico_enable_stdio_uart(test_driver_motor_agent STDIO_UART)
    pico_enable_stdio_usb(test_stepper_motor_agent STDIO_USB)
    pico_enable_stdio_uart(test_stepper_motor_agent STDIO_UART)
    pico_enable_stdio_usb(test_imu STDIO_USB)
    pico_enable_stdio_uart(test_imu STDIO_UART)
endif()

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2

if (BUILD_TEST)
    pico_add_extra_outputs(pico_micro_ros_example)
    pico_add_extra_outputs(test_driver_motor)
    pico_add_extra_outputs(test_air_sensor)
    pico_add_extra_outputs(test_soil_sensor)
    pico_add_extra_outputs(test_stepper_motor_agent)
    pico_add_extra_outputs(test_driver_motor_agent)
    pico_add_extra_outputs(test_imu)
endif()

pico_add_extra_outputs(rear_agent)
pico_add_extra_outputs(front_agent)
pico_add_extra_outputs(first_section_agent)