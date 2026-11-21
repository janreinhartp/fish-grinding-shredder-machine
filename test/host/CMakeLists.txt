cmake_minimum_required(VERSION 3.16)
project(fish_shredder_host C)
enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
add_library(firmware_host STATIC
    ${ROOT}/main/common/crc16.c
)
target_include_directories(firmware_host PUBLIC ${ROOT}/main)

add_executable(test_crc16 test_crc16.c)
target_link_libraries(test_crc16 PRIVATE firmware_host)
add_test(NAME test_crc16 COMMAND test_crc16)
