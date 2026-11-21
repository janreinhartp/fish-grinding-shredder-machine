cmake_minimum_required(VERSION 3.16)
project(fish_shredder_host C)
enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
add_library(firmware_host STATIC
    ${ROOT}/main/common/crc16.c
    ${ROOT}/main/machine/machine_settings.c
    ${ROOT}/main/machine/machine_core.c
    ${ROOT}/main/settings/settings_codec.c
    ${ROOT}/main/modbus/modbus_master.c
    ${ROOT}/main/device/scale_decode.c
    ${ROOT}/main/device/profile.c
    ${ROOT}/main/ui/encoder_decode.c
)
target_include_directories(firmware_host PUBLIC ${ROOT}/main)

function(add_host_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE firmware_host)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_crc16)
add_host_test(test_settings)
add_host_test(test_machine_core)
add_host_test(test_modbus_master)
add_host_test(test_scale_decode)
add_host_test(test_encoder_decode)
