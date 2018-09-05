#.rst:
# FindPCAP
# --------------
#
# Find PCAP (or WinPCAP) include dirs, library dirs, libraries.
#
# Use this module by invoking find_package with the form::
#
#    find_package( PCAP [REQUIRED] )
#
# Results for users are reported in following variables::
#
#    PCAP_FOUND                - Return "TRUE" when PCAP (or WinPCAP) found. Otherwise, Return "FALSE".
#    PCAP_INCLUDE_DIRS         - PCAP (or WinPCAP) include directories.
#    PCAP_LIBRARY_DIRS         - PCAP (or WinPCAP) library directories.
#    PCAP_LIBRARIES            - PCAP (or WinPCAP) library files.
#
# This module reads hints about search locations from following environment variables::
#
#    PCAP_DIR                  - PCAP (or WinPCAP) root directory.
#
# Example to find PCAP (or WinPCAP)::
#
#    cmake_minimum_required( VERSION 2.8 )
#
#    project( project )
#    add_executable( project main.cpp )
#
#    # Find package using this module.
#    find_package( PCAP REQUIRED )
#
#    if(PCAP_FOUND)
#      # Include Directories
#      include_directories( ${PCAP_INCLUDE_DIRS} )
#
#      # Library Directories (Option)
#      link_directories( ${PCAP_LIBRARY_DIRS} )
#
#      # Dependencies
#      target_link_libraries( project ${PCAP_LIBRARIES} )
#    endif()
#
# This module refers to the following::
#
#    https://github.com/PointCloudLibrary/pcl/blob/master/cmake/Modules/FindPcap.cmake
#
# =============================================================================
#
# Copyright (c) 2017 Tsukasa SUGIURA
# Distributed under the MIT License.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# =============================================================================

if(WIN32)
  set(_PCAP_LIBRARY_NAME wpcap)
else()
  set(_PCAP_LIBRARY_NAME pcap)
endif()

set(_TARGET_PLATFORM)
if(MSVC AND CMAKE_CL_64)
  set(_TARGET_PLATFORM x64)
endif()

if(EXISTS $ENV{PCAP_DIR})
  find_path(
    PCAP_INCLUDE_DIR
    NAMES pcap/pcap.h pcap.h
    PATHS $ENV{PCAP_DIR} $ENV{PCAP_DIR}/include
    NO_DEFAULT_PATH
  )

  find_path(
    PCAP_LIBRARY_DIR
    NAMES ${_PCAP_LIBRARY_NAME}.a ${_PCAP_LIBRARY_NAME}.so ${_PCAP_LIBRARY_NAME}.lib
    PATHS $ENV{PCAP_DIR} $ENV{PCAP_DIR}/lib/ $ENV{PCAP_DIR}/Lib
    PATH_SUFFIXES ${_TARGET_PLATFORM}
    NO_DEFAULT_PATH
  )

  find_library(
    PCAP_LIBRARY
    NAMES ${_PCAP_LIBRARY_NAME}
    PATHS $ENV{PCAP_DIR} $ENV{PCAP_DIR}/lib/ $ENV{PCAP_DIR}/Lib
    PATH_SUFFIXES ${_TARGET_PLATFORM}
    NO_DEFAULT_PATH
  )
else()
  find_path(
    PCAP_INCLUDE_DIR
    NAMES pcap/pcap.h pcap.h
  )

  find_path(
    PCAP_LIBRARY_DIR
    NAMES ${_PCAP_LIBRARY_NAME}.a ${_PCAP_LIBRARY_NAME}.so ${_PCAP_LIBRARY_NAME}.lib
  )

  find_library(
    PCAP_LIBRARY
    NAMES ${_PCAP_LIBRARY_NAME}
  )
endif()

set(PCAP_INCLUDE_DIRS ${PCAP_INCLUDE_DIR})
set(PCAP_LIBRARIES ${PCAP_LIBRARY})

include(CheckFunctionExists)
set(CMAKE_REQUIRED_INCLUDES ${PCAP_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${PCAP_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCAP DEFAULT_MSG PCAP_LIBRARIES)

mark_as_advanced(PCAP_INCLUDE_DIRS PCAP_LIBRARIES)
