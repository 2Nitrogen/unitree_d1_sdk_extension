cmake_minimum_required(VERSION 3.16)
project(unitree_arm VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(UNITREE_ARM_BUILD_EXAMPLES "Build the example programs" ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_library(unitree_arm STATIC
  # Cyclone DDS IDL-generated type support.
  src/idl/ArmString_.cpp
  src/idl/PubServoInfo_.cpp
  src/idl/SetServoAngle_.cpp
  src/idl/SetServoDumping_.cpp
  # Protocol codec.
  src/d1_protocol.cpp
)

set_target_properties(unitree_arm PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CXX
)

target_include_directories(unitree_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(unitree_arm PUBLIC unitree_sdk2)



include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS unitree_arm
  EXPORT unitree_armTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT unitree_armTargets
  FILE unitree_armTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_arm
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/unitree_armConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/unitree_armConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_arm
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/unitree_armConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/unitree_armConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/unitree_armConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_arm
)

if(UNITREE_ARM_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
