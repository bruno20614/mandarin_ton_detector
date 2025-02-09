cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
set(PICOTOOL_PATH "/usr/local/bin/picotool")
project(display_oled C CXX ASM)

# =========== Configurações CMSIS-DSP ===========
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Include
)

# Adiciona os arquivos necessários do CMSIS-DSP
add_library(cmsis_dsp STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/CommonTables/arm_const_structs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.c
)
# Define a macro que ativa a geração de arm_bitreversal_32
target_compile_definitions(cmsis_dsp PUBLIC ARM_MATH_LOOPUNROLL)
# ===============================================

# Inicializa o Pico SDK
pico_sdk_init()

# Configurações do projeto
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

# Executável principal
add_executable(display_oled 
    display_oled.c
    inc/ssd1306_i2c.c
)

# Define para o processador Cortex-M0+ (RP2040)
target_compile_definitions(display_oled PRIVATE ARM_MATH_CM0PLUS)

pico_enable_stdio_usb(display_oled 1)
pico_enable_stdio_uart(display_oled 0)

pico_set_program_name(display_oled "display_oled")
pico_set_program_version(display_oled "0.1")

target_link_libraries(display_oled
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_dma
    cmsis_dsp
    m
)

target_include_directories(display_oled PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/src/rp2_common/hardware_adc/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_dma/include
    inc/
)

pico_add_extra_outputs(display_oled)
