cmake_minimum_required(VERSION 3.16)
project(GameEngine LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(MAIN_SOURCES
core/src/main.cpp
core/src/entity.cpp
core/src/collidible.cpp)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(GameEngine ${MAIN_SOURCES})
target_link_libraries(GameEngine PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(GameEngine PRIVATE cxx_std_17)

# Add the include directory
target_include_directories(GameEngine PRIVATE core/include)

install(TARGETS GameEngine)

