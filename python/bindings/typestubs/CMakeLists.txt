# -*- coding: utf-8 -*-

cmake_minimum_required(VERSION 2.8)

project(openravepytypestubs)

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

message(STATUS "Using cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" )
# https://cmake.org/cmake/help/cmake2.6docs.html#policy:CMP0002
cmake_policy(SET CMP0002 NEW)
# https://cmake.org/cmake/help/cmake2.6docs.html#policy:CMP0003
cmake_policy(SET CMP0003 NEW)
# Proper handling for generated sources
cmake_policy(SET CMP0118 NEW)

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# When building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(LIB_SUFFIX CACHE STRING "suffix for the library directory need for x86-64 systems that use lib64 ")

# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")

# Add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ---

find_package(OpenRAVE 0.151.0 REQUIRED)
set(BLA_VENDOR "${OpenRAVE_BLA_VENDOR}")

if (OpenRAVE_PYTHON3)
    message(STATUS "Found openrave's python3 dir: ${OpenRAVE_PYTHON3_DIR}")
    find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter Development NumPy)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import distutils.sysconfig as sysconf; import os; \
            print(os.path.relpath(sysconf.get_python_lib(1, prefix='${CMAKE_INSTALL_PREFIX}'), '${CMAKE_INSTALL_PREFIX}'))"
        OUTPUT_VARIABLE OPENRAVE_PYTHON3_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    )
endif()

# ----

if("${CMAKE_VERSION}" VERSION_GREATER_EQUAL "3.26")
    set(COPY_COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different)
else()
    set(COPY_COMMAND cp -rp)
endif()

find_program(PYBIND11_STUBGEN pybind11-stubgen)
find_file(OPENRAVEPY_INT_SO_FILE openravepy_int.so PATHS ${CMAKE_INSTALL_PREFIX}/${OPENRAVE_PYTHON3_INSTALL_DIR}/openravepy/_openravepy_)
if (PYBIND11_STUBGEN)
    # Openravecommon stubs
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/__init__.pyi
        COMMAND bash -c "${PYBIND11_STUBGEN} openravepy -o ${CMAKE_CURRENT_BINARY_DIR}/python"
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_
        COMMAND ${COPY_COMMAND} ${CMAKE_CURRENT_BINARY_DIR}/python/_openravepy_-stubs/. ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_ import *" > ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/__init__.pyi
        DEPENDS ${OPENRAVEPY_INT_SO_FILE}
    )
    add_custom_target(openravepy_int_stubs ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/__init__.pyi)
    install(
        DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs
        DESTINATION ${OPENRAVE_PYTHON3_INSTALL_DIR}
        FILES_MATCHING PATTERN "*.pyi"
    )
else ()
    message(WARNING "Could not find pybind11-stubgen.")
endif (PYBIND11_STUBGEN)
