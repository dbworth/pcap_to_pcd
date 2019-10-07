cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcap_to_pcd
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE RelWithDebInfo) # Debug, Release, RelWithDebInfo
# Note: 'Debug' causes segfault in boost::math::lanczos::lanczos17m64::lanczos_sum

# Need libPCAP for network packet capture
find_package(PCAP REQUIRED)

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Prevent the C++ compiler from printing
# a warning when linking to PCL:
# <command-line>:0:15: warning: missing whitespace after the macro name
# because the definition "-DDISABLE_LIBUSB-1.0" contains a dash character.
remove_definitions(-DDISABLE_LIBUSB-1.0)

# Enable PCAP within HDL Grabber class
add_definitions(-DHAVE_PCAP)

add_executable(main main.cpp
                    hdl_grabber.cpp
                    vlp_grabber.cpp)
target_link_libraries(main ${PCL_LIBRARIES}
                           ${PCAP_LIBRARY})
