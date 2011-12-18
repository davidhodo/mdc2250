## Project Setup
cmake_minimum_required(VERSION 2.4.6)

project(mdc2250)

## Configurations

# Use clang if available
IF(EXISTS /usr/bin/clang)
  set(CMAKE_CXX_COMPILER /usr/bin/clang++)
ENDIF(EXISTS /usr/bin/clang)

option(MDC2250_BUILD_TESTS "Build all of the mdc2250 tests." OFF)
option(MDC2250_BUILD_EXAMPLES "Build all of the mdc2250 examples." OFF)

# Allow for building shared libs override
IF(NOT BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS OFF)
ENDIF(NOT BUILD_SHARED_LIBS)

# Allow for build type override
IF(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
ENDIF(NOT CMAKE_BUILD_TYPE)

# set the default path for built executables to the "bin" directory
IF(NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
ENDIF(NOT EXECUTABLE_OUTPUT_PATH)
# set the default path for built libraries to the "lib" directory
IF(NOT LIBRARY_OUTPUT_PATH)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF(NOT LIBRARY_OUTPUT_PATH)

# Find Boost, if it hasn't already been found
IF(NOT Boost_FOUND OR NOT Boost_SYSTEM_FOUND OR NOT Boost_FILESYSTEM_FOUND OR NOT Boost_THREAD_FOUND)
    find_package(Boost COMPONENTS system filesystem thread REQUIRED)
ENDIF(NOT Boost_FOUND OR NOT Boost_SYSTEM_FOUND OR NOT Boost_FILESYSTEM_FOUND OR NOT Boost_THREAD_FOUND)

## Configure build system

# Add the include folder to the include path
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/vendor)

# Add default source files
set(MDC2250_SRCS src/mdc2250.cc include/mdc2250/mdc2250.h include/mdc2250/mdc2250_types.h)
#set(ROBOTEQ_API_DIR ${PROJECT_SOURCE_DIR}/vendor/roboteq_api)
#IF(WIN32)
 # list(APPEND MDC2250_SRCS ${ROBOTEQ_API_DIR}/windows/RoboteqDevice.cpp)
#ELSE(WIN32)
#  list(APPEND MDC2250_SRCS ${ROBOTEQ_API_DIR}/linux/RoboteqDevice.cpp)
#ENDIF(WIN32)

# Add header files
set(MDC2250_HEADERS ${PROJECT_SOURCE_DIR}/include/mdc2250/mdc2250_types.h)
list(APPEND MDC2250_HEADERS ${PROJECT_SOURCE_DIR}/include/mdc2250/mdc2250.h)
#IF(WIN32)
#  set(ROBOTEQ_API_HEADERS ${ROBOTEQ_API_DIR}/windows/Constants.h
#                          ${ROBOTEQ_API_DIR}/windows/ErrorCodes.h
#                          ${ROBOTEQ_API_DIR}/windows/RoboteqDevice.h)
#ELSE(WIN32)
#  set(ROBOTEQ_API_HEADERS ${ROBOTEQ_API_DIR}/linux/Constants.h
#                          ${ROBOTEQ_API_DIR}/linux/ErrorCodes.h
#                          ${ROBOTEQ_API_DIR}/linux/RoboteqDevice.h)
#ENDIF(WIN32)

## Build the library

set(SERIAL_LINK_LIBS serial
                     ${Boost_SYSTEM_LIBRARY}
                     ${Boost_FILESYSTEM_LIBRARY}
                     ${Boost_THREAD_LIBRARY})

# Compile the libmdc2250 Library
add_library(mdc2250 ${MDC2250_SRCS})
target_link_libraries(mdc2250 ${SERIAL_LINK_LIBS})
## Build Examples

# If specified
IF(MDC2250_BUILD_EXAMPLES)
  # Compile the Example program
  add_executable(mdc2250_example examples/mdc2250_example.cc)
  # Link the Test program to the mdc2250 library
  target_link_libraries(mdc2250_example mdc2250 ${SERIAL_LINK_LIBS})
ENDIF(MDC2250_BUILD_EXAMPLES)

## Build Tests

# If specified
IF(MDC2250_BUILD_TESTS)
  # Find Google Test                                                                                                                                                               
  enable_testing()
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
  
  # Compile the Test program
  add_executable(mdc2250_tests tests/mdc2250_tests.cc)
  # Link the Test program to the mdc2250 library
  target_link_libraries(mdc2250_tests mdc2250 ${GTEST_BOTH_LIBRARIES})
  add_test(AllTestsIntest_mdc2250 mdc2250_tests)
ENDIF(MDC2250_BUILD_TESTS)

# Configure make install
IF(NOT CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX /usr/local)
ENDIF(NOT CMAKE_INSTALL_PREFIX)

## Install setup

# Install the library
IF(NOT MDC2250_DONT_SETUP_INSTALL)
    INSTALL(
            TARGETS mdc2250
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
    )
    
    INSTALL(FILES ${MDC2250_HEADERS} DESTINATION include/mdc2250)
    INSTALL(FILES ${ROBOTEQ_API_HEADERS} DESTINATION include/roboteq_api)
    
    IF(NOT CMAKE_FIND_INSTALL_PATH)
        set(CMAKE_FIND_INSTALL_PATH ${CMAKE_ROOT})
    ENDIF(NOT CMAKE_FIND_INSTALL_PATH)
    
    INSTALL(FILES Findmdc2250.cmake DESTINATION 
            ${CMAKE_FIND_INSTALL_PATH}/Modules/)
    
    ADD_CUSTOM_TARGET(uninstall @echo uninstall package)
    
    IF(UNIX)
      ADD_CUSTOM_COMMAND(
                         COMMENT "uninstall package"
                         COMMAND xargs ARGS rm < install_manifest.txt
                         TARGET  uninstall
      )
    ELSE(UNIX)
      ADD_CUSTOM_COMMAND(
                         COMMENT "uninstall only implemented in unix"
                         TARGET  uninstall
      )
    ENDIF(UNIX)
ENDIF(NOT MDC2250_DONT_SETUP_INSTALL)
