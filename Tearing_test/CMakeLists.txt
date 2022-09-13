cmake_minimum_required(VERSION 3.12)
project(Tearing_test)


set(SOURCE_FILES
    TearingEngine_test.cpp
)

add_definitions("-DSOFA_TEARING_TEST_SCENES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/../scenes/\"")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} Tearing Sofa.Testing Sofa.Component.Topology.Testing)
 
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
