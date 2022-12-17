cmake_minimum_required(VERSION 3.25)
project(w05)

SET(CMAKE_CXX_FLAGS -pthread)
set(CMAKE_CXX_STANDARD 20)

include_directories(include/)
include_directories(/home/bohm/.mujoco/mujoco-2.3.1/include/)
link_directories(/home/bohm/.mujoco/mujoco-2.3.1/lib/)

find_package(glfw3 REQUIRED)

add_executable(w05 src/main.cpp include/swing_controller.h src/swing_controller.cpp)
target_link_libraries(w05 libmujoco.so libglfw.so)


