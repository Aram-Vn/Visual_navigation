cmake_minimum_required(VERSION 3.10)

project(Visual_navigation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SOURCE_FILES 
     src/*.cpp
)

file(GLOB_RECURSE HEADER_FILES 
     include/*.h
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC 
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

set(COMMON_COMPILE_OPTIONS
    -Wall
    -Wextra
    -Wshadow
    -Wswitch
    -pedantic
    -Wformat=2
    -Wnull-dereference
    -Wunused-parameter
    -Wunreachable-code
    -Wimplicit-fallthrough
)

set(COMMON_ERROR_OPTIONS
    -Werror
    -Werror=return-type
    -Werror=uninitialized
    -Werror=unused-result
    -Werror=strict-overflow
)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${COMMON_COMPILE_OPTIONS}
    ${COMMON_ERROR_OPTIONS}
)
