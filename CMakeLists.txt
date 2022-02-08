
PROJECT(manus)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1 FATAL_ERROR)

include(GNUInstallDirs)

set(PROJECT_VERSION 0.1.0)

SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
SET(CMAKE_CXX_STANDARD 11)

option(BUILD_DEBUG "Enable debug output" OFF)

if(BUILD_DEBUG)
    add_definitions(-DMANUS_DEBUG)
    add_definitions(-DMANUS_SOURCE_COMPILE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/src/")
endif()

FIND_PROGRAM(ECHOGEN echogen)

ADD_CUSTOM_TARGET(manus_messages ALL
                DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/messages/manus/messages.h
                COMMENT "Generating message wrappers")

ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/messages/manus/messages.h
                COMMAND python3 -m echolib.messages
                --python-outdir ${CMAKE_CURRENT_BINARY_DIR}/python/manus/
                --cpp-outdir ${CMAKE_CURRENT_BINARY_DIR}/messages/manus/
                -p ${CMAKE_SOURCE_DIR}/msg/
                ${CMAKE_SOURCE_DIR}/msg/messages.msg
                DEPENDS 
                ${CMAKE_SOURCE_DIR}/msg/messages.msg
                ${CMAKE_SOURCE_DIR}/msg/manipulator.msg 
                ${CMAKE_SOURCE_DIR}/msg/apps.msg
                ${CMAKE_SOURCE_DIR}/msg/privileged.msg
                ${CMAKE_SOURCE_DIR}/msg/markers.msg
                COMMENT "Processing message descriptions")

FIND_PACKAGE(echolib REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src/ ${CMAKE_CURRENT_BINARY_DIR}/messages/ ${echolib_INCLUDE_DIRS} include/)

ADD_LIBRARY(manus SHARED src/debug.cpp src/files.cpp src/manipulator.cpp)
TARGET_LINK_LIBRARIES(manus ${echolib_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(manus manus_messages)

ADD_EXECUTABLE(manus_dummy_manipulator src/dummy/main.cpp)
TARGET_LINK_LIBRARIES(manus_dummy_manipulator manus ${echolib_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(manus_dummy_manipulator manus_messages)

if(BUILD_DEBUG)
FIND_PACKAGE(OpenCV REQUIRED core imgproc video calib3d)
else()
FIND_PACKAGE(OpenCV REQUIRED core imgproc video calib3d highgui)
endif()

ADD_EXECUTABLE(manus_camera src/camera/cameraserver.cpp)
TARGET_LINK_LIBRARIES(manus_camera manus ${echolib_LIBRARIES} ${OpenCV_LIBS})

FIND_PACKAGE(Ary REQUIRED)
ADD_EXECUTABLE(manus_artrack src/camera/artrack.cpp)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${ary_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(manus_artrack manus ${echolib_LIBRARIES} ${OpenCV_LIBS} ${Ary_LIBRARIES})

find_path(EIGEN3_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library PATHS ${CMAKE_INSTALL_PREFIX}/include PATH_SUFFIXES eigen3 eigen)
FIND_PACKAGE(orocos_kdl REQUIRED)
ADD_EXECUTABLE(manus_planner src/planner/planner.cpp src/planner/voxelgrid.cpp src/files.cpp)
TARGET_INCLUDE_DIRECTORIES(manus_planner PUBLIC ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(manus_planner manus ${echolib_LIBRARIES} ${orocos_kdl_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(manus_planner manus_messages)

ADD_SUBDIRECTORY(python)

# CMake config file
include(CMakePackageConfigHelpers)

set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})


configure_package_config_file(manus-config.cmake.in
    ${PROJECT_BINARY_DIR}/manus-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/manus
    PATH_VARS LIB_INSTALL_DIR INCLUDE_INSTALL_DIR)

write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/manus-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/manus-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/manus-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/manus)

install(
    EXPORT manus_targets
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/manus
    FILE manus-targets.cmake
)

set_target_properties(manus PROPERTIES VERSION 0.0.0 SOVERSION 0)

install(TARGETS 
        manus_dummy_manipulator
        manus_planner
        manus_artrack
        manus_camera)

install(TARGETS 
        manus
        EXPORT manus_targets
        DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/messages/manus/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/manus/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/manus
)

# install(DIRECTORY
#     ${CMAKE_CURRENT_SOURCE_DIR}/resources/
#     DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/manus
# )