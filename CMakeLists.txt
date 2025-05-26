
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
set(FREERTOS_KERNEL_PATH "C:/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(painel C CXX ASM)
pico_sdk_init()


include_directories(${CMAKE_SOURCE_DIR}/libs)


add_executable(painel  
        painel.c
        libs/ssd1306.c # Biblioteca para o display OLED
        libs/display.c # Biblioteca para controle do display
        libs/buzzer.c  # Biblioteca para buzzer
        libs/pio.c
        )

target_include_directories(painel PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries(painel 
        pico_stdlib 
        hardware_gpio
        hardware_i2c
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4
        hardware_pwm
        hardware_pio
        )

pico_generate_pio_header(painel ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

pico_enable_stdio_usb(painel 1)
pico_enable_stdio_uart(painel 0)

pico_add_extra_outputs(painel)