cmake_minimum_required(VERSION 3.12)
project(tearing VERSION 1.0)

# SOFA packages containing our dependencies
find_package(Sofa.Component.Topology.Container.Dynamic REQUIRED)
find_package(Sofa.Component.SolidMechanics.FEM.Elastic REQUIRED)
find_package(Sofa.Component.MechanicalLoad REQUIRED)


set(TEARING_SRC_DIR src/tearing)

# Our nice new SOFA Components
set(HEADER_FILES
    ${TEARING_SRC_DIR}/config.h.in
    ${TEARING_SRC_DIR}/TearingAlgorithms.h
    ${TEARING_SRC_DIR}/TearingAlgorithms.inl

    ${TEARING_SRC_DIR}/TearingEngine.h
    ${TEARING_SRC_DIR}/TearingEngine.inl
)

set(SOURCE_FILES
    ${TEARING_SRC_DIR}/inittearing.cpp
    ${TEARING_SRC_DIR}/TearingAlgorithms.cpp
    ${TEARING_SRC_DIR}/TearingEngine.cpp
)

set(README_FILES
    README.md
)

find_package(MeshRefinement QUIET)
if (MeshRefinement_FOUND)
    message("MeshRefinement plugin found - Adding it to ${PROJECT_NAME}")

    list(APPEND HEADER_FILES 
        ${TEARING_SRC_DIR}/VolumeTearingAlgorithms.h
        ${TEARING_SRC_DIR}/VolumeTearingAlgorithms.inl
        ${TEARING_SRC_DIR}/VolumeTearingEngine.h
        ${TEARING_SRC_DIR}/VolumeTearingEngine.inl
    )
    
    list(APPEND SOURCE_FILES 
        ${TEARING_SRC_DIR}/VolumeTearingAlgorithms.cpp
        ${TEARING_SRC_DIR}/VolumeTearingEngine.cpp
    )
endif()

# SOFA plugin are basically libraries
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_TEARING_PLUGIN")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} Sofa.Component.Topology.Container.Dynamic 
    Sofa.Core Sofa.Component.SolidMechanics.FEM.Elastic 
    Sofa.Component.MechanicalLoad)

# Dependencies
if (MeshRefinement_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC MeshRefinement)
endif()


# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
    