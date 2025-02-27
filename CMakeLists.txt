cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Inicializar o SDK do Raspberry Pi Pico
set(PICO_SDK_PATH /caminho/para/pico-sdk)  # Se não estiver configurado no ambiente

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Importar o SDK do Raspberry Pi Pico
include(pico_sdk_import.cmake)

project(tarefa_final C CXX ASM)

# Inicializar o SDK
pico_sdk_init()

# Adicionar o executável
add_executable(tarefa_final tarefa_final.c inc/ssd1306_i2c.c display_oled.c)

# Configuração do nome e versão do programa
pico_set_program_name(tarefa_final "tarefa_final")
pico_set_program_version(tarefa_final "0.1")

# Configuração de saída padrão (USB, desabilitar UART)
pico_enable_stdio_uart(tarefa_final 0)
pico_enable_stdio_usb(tarefa_final 1)

# Adicionar a biblioteca padrão
target_link_libraries(tarefa_final
    pico_stdlib
    hardware_i2c
    hardware_timer
)

# Incluir diretórios de cabeçalhos
target_include_directories(tarefa_final PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/inc
)

# Adicionar saídas extras (ex: .uf2, .bin)
pico_add_extra_outputs(tarefa_final)
