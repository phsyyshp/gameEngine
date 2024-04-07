cmake_minimum_required(VERSION 3.16)
project(GameEngine LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(MAIN_SOURCES
demos/main.cpp
core/src/entity.cpp
core/src/rigidBody2D.cpp
core/src/forceGeneration.cpp
core/src/world.cpp)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(GameEngine ${MAIN_SOURCES})
target_link_libraries(GameEngine PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(GameEngine PRIVATE cxx_std_17)

# Add the include directory
target_include_directories(GameEngine PRIVATE core/include)

install(TARGETS GameEngine)
#add another build for colliderspherespheredem
set(COLLIDER_SPHERE_DEMO_SOURCES
demos/colliderSphereSphereDemo.cpp
core/src/entity.cpp
core/src/collisionHandler.cpp
core/src/utils.cpp
core/src/contact.cpp
core/src/rigidBody2D.cpp
core/src/forceGeneration.cpp
core/src/world.cpp)

add_executable(ColliderSphereDemo ${COLLIDER_SPHERE_DEMO_SOURCES})
target_link_libraries(ColliderSphereDemo PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(ColliderSphereDemo PRIVATE cxx_std_17)

# Add the include directory
target_include_directories(ColliderSphereDemo PRIVATE core/include)

install(TARGETS ColliderSphereDemo)
