cmake_minimum_required(VERSION 3.30)
project(stlToOptimizedGlb)

set(CMAKE_CXX_STANDARD 17)

include_directories(external/tinygltf)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(assimp REQUIRED)
find_package(Threads REQUIRED)
find_package(tinygltf REQUIRED)

add_executable(stlToOptimizedGlb main.cpp
        converter.cpp)

target_link_libraries(stlToOptimizedGlb
        PRIVATE
        Boost::system
        assimp::assimp
        Threads::Threads
        tinygltf
)

target_include_directories(stlToOptimizedGlb
        PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${ASSIMP_INCLUDE_DIRS}
)

if(MSVC)
    target_compile_options(stlToOptimizedGlb PRIVATE /W4)
else()
    target_compile_options(stlToOptimizedGlb PRIVATE -Wall -Wextra)
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(stlToOptimizedGlb PRIVATE stdc++fs)
endif()
