cmake_minimum_required(VERSION 3.12)
project(Tearing_test VERSION 1.0)

find_package(Sofa.Testing REQUIRED)

set(SOURCE_FILES
    TearingEngine_test.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")

target_link_libraries(${PROJECT_NAME} Tearing Sofa.Testing)

add_definitions("-DSOFA_TEARING_TEST_SCENES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/../scenes/\"")

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
