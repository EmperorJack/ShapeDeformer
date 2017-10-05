cmake_minimum_required(VERSION 3.7)
project(ShapeDeformer)

set(CMAKE_CXX_STANDARD 11)

# Setup LLVM paths (needed for openmp on OSX)
if (APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L/usr/local/opt/llvm/lib -I/usr/local/opt/llvm/include")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

# Include source files
include_directories(${PROJECT_SOURCE_DIR}/src)
file(GLOB_RECURSE SOURCE_FILES
    src/*.hpp
    src/*.cpp
)

# Include the eigen header library
include_directories(${PROJECT_SOURCE_DIR}/lib/Eigen)

add_executable(ShapeDeformer ${SOURCE_FILES} ${EXTERNAL_FILES})

if (APPLE)
    add_custom_command(TARGET ${PROJECT_NAME}
            POST_BUILD COMMAND
            ${CMAKE_INSTALL_NAME_TOOL} -add_rpath /usr/local/opt/llvm/lib
            $<TARGET_FILE:${PROJECT_NAME}>
    )
endif()