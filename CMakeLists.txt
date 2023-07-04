cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(miko)

include_directories(include)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

add_executable(test_graph src/test_graph.cpp src/custom_graph.cpp src/geometry.cpp)
target_link_libraries(test_graph
    PRIVATE
        ${PYTHON_LIBRARIES}
        Python3::NumPy
)

add_executable(test_geom  src/test_geometry.cpp src/geometry.cpp)


add_executable(test_tsp  src/test_tsp.cpp src/tspnn.cpp src/geometry.cpp src/custom_graph.cpp)


add_executable(test_rrg src/tspnn.cpp src/test_rrg.cpp src/rrg.cpp src/geometry.cpp src/custom_graph.cpp)
target_link_libraries(test_rrg
    PRIVATE
        ${PYTHON_LIBRARIES}
        Python3::NumPy
)
