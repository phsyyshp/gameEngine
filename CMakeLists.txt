cmake_minimum_required(VERSION 3.16)
project(GameEngine LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)


add_executable(GameEngine src/main.cpp)
target_link_libraries(GameEngine PRIVATE sfml-graphics)
target_compile_features(GameEngine PRIVATE cxx_std_17)

install(TARGETS GameEngine)