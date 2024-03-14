cmake_minimum_required(VERSION 3.8)
project(transporter_sdk)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
if(LIBUSB1_FOUND)
  include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})
else()
  message( FATAL_ERROR "Failed to find libusb-1.0" )
endif()

# imu sensor sdk
add_library(transporter_sdk SHARED
  src/usbcdc_transporter.cpp
)
target_link_libraries(transporter_sdk
    ${LIBUSB1_LIBRARIES}
)
target_include_directories(transporter_sdk PUBLIC
  include
)

# Install libraries
install(TARGETS transporter_sdk
  EXPORT transporter_sdk
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
