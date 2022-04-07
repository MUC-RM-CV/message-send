cmake_minimum_required(VERSION 3.1)

project(message)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

# Messaging Library
add_library(message INTERFACE)

# Messaging Library Includes
target_include_directories(message INTERFACE src)

# Messaging Library Link Libraries
target_link_libraries(message INTERFACE ${OpenCV_LIBS})
