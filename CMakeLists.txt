cmake_minimum_required(VERSION 3.16)
project(GameEngine LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Sanitizer option
option(ENABLE_SANITIZERS "Enable Sanitizers (UBSan, ASan, LSAN)" OFF)

# Conditionally add sanitizer flags
if(ENABLE_SANITIZERS)
    add_compile_options(-fsanitize=undefined,address,leak)
    add_link_options(-fsanitize=undefined,address,leak)
endif()

# Set optimization flags if sanitizers are not enabled
if(NOT ENABLE_SANITIZERS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

if(ENABLE_PROFILING)
    add_compile_options(-pg)
    add_link_options(-pg)
endif()
# Option for building shared libraries
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Define sources for ColliderSphereDemo
set(COLLIDER_SPHERE_DEMO_SOURCES
    demos/colliderSphereSphereDemo.cpp
    core/src/collisionHandler.cpp
    core/src/utils.cpp
    core/src/manifold.cpp
    core/src/contact.cpp
    core/src/rigidBody2D.cpp
    core/src/forceGeneration.cpp
    core/src/visuals.cpp
    core/src/world.cpp)

add_executable(ColliderSphereDemo ${COLLIDER_SPHERE_DEMO_SOURCES})
target_link_libraries(ColliderSphereDemo PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(ColliderSphereDemo PRIVATE cxx_std_20)
target_include_directories(ColliderSphereDemo PRIVATE core/include)
set(benchMark_SOURCES
    demos/benchMark.cpp
    core/src/collisionHandler.cpp
    core/src/utils.cpp
    core/src/contact.cpp
    core/src/rigidBody2D.cpp
    core/src/forceGeneration.cpp
    core/src/visuals.cpp
    core/src/manifold.cpp
    core/src/world.cpp)

add_executable(benchMark ${benchMark_SOURCES})
target_link_libraries(benchMark PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(benchMark PRIVATE cxx_std_20)
target_include_directories(benchMark PRIVATE core/include)

# Define sources for gjkTests
set(gjkTest_SOURCES
    demos/gjkTests.cpp
    core/src/collisionHandler.cpp
    core/src/utils.cpp
    core/src/contact.cpp
    core/src/rigidBody2D.cpp
    core/src/forceGeneration.cpp
    core/src/visuals.cpp
    core/src/manifold.cpp
    core/src/world.cpp)

add_executable(gjkTests ${gjkTest_SOURCES})
target_link_libraries(gjkTests PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(gjkTests PRIVATE cxx_std_20)
target_include_directories(gjkTests PRIVATE core/include)

