cmake_minimum_required(VERSION 3.7)
project(ShapeDeformer)

set(CMAKE_CXX_STANDARD 11)

# Include source files
include_directories(${PROJECT_SOURCE_DIR}/src)
file(GLOB_RECURSE SOURCE_FILES
    src/*.hpp
    src/*.cpp
)

# Include the eigen header library
include_directories(${PROJECT_SOURCE_DIR}/lib/Eigen)

add_executable(ShapeDeformer ${SOURCE_FILES} ${EXTERNAL_FILES})
