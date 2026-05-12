cmake_minimum_required(VERSION 3.18)

project(web-ifc-api-folder LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Allows CLion to open this api folder directly while still using the real
# C++ project and target definitions from src/cpp/CMakeLists.txt.
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." "${CMAKE_BINARY_DIR}/web-ifc-cpp")
