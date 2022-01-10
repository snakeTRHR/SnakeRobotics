cmake_minimum_required(VERSION 3.10.2)
project(snake)
 
set(CMAKE_CXX_STANDARD 11)
 
find_package(PythonLibs 2.7 REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

add_executable(frenet_serret frenet_serret.cpp)
add_executable(snake_joy main.cpp joystick.cpp)
add_executable(snake_key control_keyboard.cpp)
add_executable(snake_dwa snake_dwa.cpp)
add_executable(dwa dwa.cpp)
add_executable(circle circle.cpp)
target_link_libraries(frenet_serret ${PYTHON_LIBRARIES})
target_link_libraries(snake_joy ${PYTHON_LIBRARIES})
target_link_libraries(snake_key ${PYTHON_LIBRARIES})
target_link_libraries(snake_dwa ${PYTHON_LIBRARIES})
target_link_libraries(dwa ${PYTHON_LIBRARIES})
target_link_libraries(circle ${PYTHON_LIBRARIES})
