# Generated cmake Pico project file.
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Project details.
set(PROJECT JubileeDepthLogTemperature)
set(MAJOR_VERSION 0)
set(MINOR_VERSION 1)
set(PICO_SDK_PATH /usr/share/pico-sdk)
set(PICO_BOARD pico_w)

# Directories.
set(BUILD "${CMAKE_SOURCE_DIR}/build")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Declare project.
project(${PROJECT} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Make project data accessible to compiler
add_compile_definitions(PROJECT=${PROJECT})
add_compile_definitions(MAJOR_VERSION=${MAJOR_VERSION})
add_compile_definitions(MINOR_VERSION=${MINOR_VERSION})

# Create executable.
add_executable(${PROJECT})

# Specify sources for the executable.
target_sources(${PROJECT} PRIVATE
    src/${PROJECT}.cpp
)

# Specify link libraries.
target_link_libraries(
    ${PROJECT}
    pico_stdlib
    pico_cyw43_arch_lwip_poll
    json
    pico_one_wire
    hardware_gpio
)

target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

# Enable stdio outputs as required.
pico_enable_stdio_usb(${PROJECT} 1)
pico_enable_stdio_uart(${PROJECT} 0)

# Create additional executables.
pico_add_extra_outputs(${PROJECT})

# Add required subdirectories for the libraries required.
add_subdirectory(external/json)
message("json library added.")
add_subdirectory(external/pico-onewire)
message("pico-onewire library added.")



