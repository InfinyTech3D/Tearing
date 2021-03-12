cmake_minimum_required(VERSION 3.12)
project(tearing VERSION 1.0)

# SOFA packages containing our dependencies
find_package(SofaFramework REQUIRED) # Needed to use SofaCore, SofaHelper and SofaDefaultType
find_package(SofaGeneral REQUIRED) # Needed to use SofaBaseTopology

# Config files (tell SOFA this is a plugin)
set(CONFIG_FILES
    src/tearing/config.h
    src/tearing/inittearing.cpp
    )
# Our nice new SOFA Components
set(HEADER_FILES
    src/tearing/StressDetector.h
    #TearingEngine.h
    #TearingEngine.inl
    )
set(SOURCE_FILES
    src/tearing/StressDetector.cpp
    #TearingEngine.cpp
    )

# SOFA plugin are basically libraries
add_library(${PROJECT_NAME} SHARED ${CONFIG_FILES} ${HEADER_FILES} ${SOURCE_FILES})

# Dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
    SofaCore
    SofaHelper
    SofaDefaultType
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

