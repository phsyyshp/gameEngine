cmake_minimum_required(VERSION 3.16)
project(GameEngine LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(GameEngine src/main.cpp)
target_link_libraries(GameEngine PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(GameEngine PRIVATE cxx_std_17)

install(TARGETS GameEngine)
