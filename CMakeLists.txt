cmake_minimum_required(VERSION 3.16)
project(GameEngine LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)


# #add another build for colliderspherespheredem
set(COLLIDER_SPHERE_DEMO_SOURCES
demos/colliderSphereSphereDemo.cpp
core/src/entity.cpp
core/src/collisionHandler.cpp
core/src/utils.cpp
core/src/contact.cpp
core/src/rigidBody2D.cpp
core/src/forceGeneration.cpp
core/src/contactResolver.cpp
core/src/world.cpp)

add_executable(ColliderSphereDemo ${COLLIDER_SPHERE_DEMO_SOURCES})
target_link_libraries(ColliderSphereDemo PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(ColliderSphereDemo PRIVATE cxx_std_20)

# Add the include directory
target_include_directories(ColliderSphereDemo PRIVATE core/include)

install(TARGETS ColliderSphereDemo )


# set(CollisionTestDemo_SOURCES
# demos/collisionTest.cpp
# core/src/entity.cpp
# core/src/collisionHandler.cpp
# core/src/utils.cpp
# core/src/contact.cpp
# core/src/rigidBody2D.cpp
# core/src/forceGeneration.cpp
# core/src/contactResolver.cpp
# core/src/world.cpp)

# add_executable(collisionTest ${CollisionTestDemo_SOURCES})
# target_link_libraries(collisionTest PRIVATE sfml-graphics sfml-window sfml-system)
# target_compile_features(collisionTest PRIVATE cxx_std_20)

# # Add the include directory
# target_include_directories(collisionTest PRIVATE core/include)

# install(TARGETS collisionTest)
set(gjkTest_SOURCES
demos/gjkTests.cpp
core/src/entity.cpp
core/src/collisionHandler.cpp
core/src/utils.cpp
core/src/contact.cpp
core/src/rigidBody2D.cpp
core/src/forceGeneration.cpp
core/src/contactResolver.cpp
core/src/world.cpp)

add_executable(gjkTests ${gjkTest_SOURCES})
target_link_libraries(gjkTests PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_features(gjkTests PRIVATE cxx_std_20)

# Add the include directory
target_include_directories(gjkTests PRIVATE core/include)

install(TARGETS gjkTests)