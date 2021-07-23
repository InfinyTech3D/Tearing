cmake_minimum_required(VERSION 3.12)
project(tearing VERSION 1.0)

# SOFA packages containing our dependencies
find_package(SofaFramework REQUIRED) # Needed to use SofaCore, SofaHelper and SofaDefaultType
find_package(SofaGeneral REQUIRED) # Needed to use SofaBaseTopology

set(TEARING_SRC_DIR src/tearing)

# Our nice new SOFA Components
set(HEADER_FILES
    ${TEARING_SRC_DIR}/config.h.in
    ${TEARING_SRC_DIR}/StressDetector.h
    ${TEARING_SRC_DIR}/TearingAlgorithms.h
    ${TEARING_SRC_DIR}/TearingAlgorithms.inl
    ${TEARING_SRC_DIR}/VolumeTearingAlgorithms.h
    ${TEARING_SRC_DIR}/VolumeTearingAlgorithms.inl

    ${TEARING_SRC_DIR}/TearingEngine.h
    ${TEARING_SRC_DIR}/TearingEngine.inl
    
    ${TEARING_SRC_DIR}/VolumeTearingEngine.h
    ${TEARING_SRC_DIR}/VolumeTearingEngine.inl
)

set(SOURCE_FILES
    ${TEARING_SRC_DIR}/inittearing.cpp
    ${TEARING_SRC_DIR}/StressDetector.cpp
    ${TEARING_SRC_DIR}/TearingAlgorithms.cpp
    ${TEARING_SRC_DIR}/VolumeTearingAlgorithms.cpp
    
    ${TEARING_SRC_DIR}/TearingEngine.cpp
    ${TEARING_SRC_DIR}/VolumeTearingEngine.cpp
)

# SOFA plugin are basically libraries
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

# Dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
    SofaCore
    SofaHelper
    SofaDefaultType
    SofaBaseTopology
    SofaMiscFem
    SofaBoundaryCondition
    SofaLoader
    SofaGeneralSimpleFem
    )

# nested-namespace-definition requires compiler flag '/std:c++17'
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    )

# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
sofa_generate_package(
    NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME}
    VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "tearing"
    RELOCATABLE "plugins"
    )

