cmake_minimum_required(VERSION 3.14)
include(FetchContent)
FetchContent_Declare(
  kinova_binary_api
  URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip
  URL_HASH MD5=64bd86e7ab8bda90ef1fc7d6a356e080 #checksum to ensure integrity
)
FetchContent_MakeAvailable(kinova_binary_api)

project(kortex_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) #Adds compile time warnings
endif()

# this is needed by the kortex_api to tell it we are compiling for linux
add_definitions(-D_OS_UNIX=1)

# find dependencies
find_package(ament_cmake REQUIRED) # Build system used by ROS2 for cmake packages
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kortex_api REQUIRED)

add_library(KortexApiCpp STATIC IMPORTED) # Already compiled library
set_target_properties(KortexApiCpp PROPERTIES
  IMPORTED_LOCATION ${kinova_binary_api_SOURCE_DIR}/lib/release/libKortexApiCpp.a # Path to the library file
)
target_link_libraries(KortexApiCpp INTERFACE pthread) # Kortex API uses multi-threading internally
add_dependencies(KortexApiCpp kortex_api)

## COMPILE
# Target
add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface.cpp
  src/kortex_math_util.cpp
)
target_link_libraries(${PROJECT_NAME} KortexApiCpp)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE # include is only exposed to the target and not its links
  include
  ${kinova_binary_api_SOURCE_DIR}/include
  ${kinova_binary_api_SOURCE_DIR}/include/client
  ${kinova_binary_api_SOURCE_DIR}/include/client_stubs
  ${kinova_binary_api_SOURCE_DIR}/include/common
  ${kinova_binary_api_SOURCE_DIR}/include/messages
  ${kinova_binary_api_SOURCE_DIR}/include/google/protobuf

)

ament_target_dependencies(
  ${PROJECT_NAME} # Trusted dependencies
  SYSTEM kortex_api 
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  kortex_api
  pluginlib
  rclcpp
)
ament_package() #register the package with the build system
