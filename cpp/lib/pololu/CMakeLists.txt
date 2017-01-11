cmake_minimum_required(VERSION 2.8.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

project(driver_pololu)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

option(USE_LCM "Build drake-designer with lcm dependency." ON)

find_package(LCM REQUIRED)
include_directories(${LCM_INCLUDE_DIRS})

set(SRCS
    driver_pololu.cpp)

set(SRCS_Test
    driver_pololu_test.cpp)

include_directories("${CMAKE_SOURCE_DIR}/../../build/include/")

# Executable
add_executable(driver_pololu ${SRCS})
target_link_libraries(driver_pololu serial ${LCM_LIBRARIES} ${CMAKE_SOURCE_DIR}/../../build/lib/liblcmtypes_NJORD.a)

# Test App
add_executable(driver_pololu_test ${SRCS_Test})
target_link_libraries(driver_pololu_test m ${LCM_LIBRARIES} ${CMAKE_SOURCE_DIR}/../../build/lib/liblcmtypes_NJORD.a)

# a testing function for pololu debugging
add_executable(test_pololu test_pololu.cpp)
target_link_libraries(test_pololu serial ${LCM_LIBRARIES} ${CMAKE_SOURCE_DIR}/../../build/lib/liblcmtypes_NJORD.a)


