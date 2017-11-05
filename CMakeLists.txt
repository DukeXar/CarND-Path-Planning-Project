project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif(NOT CMAKE_BUILD_TYPE)

add_definitions(-std=c++11)

# This does not seem to work with CMAKE_BUILD_TYPE
#set(CXX_FLAGS "-Wall")
#set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/map.cpp src/planner.cpp src/trajectory.cpp src/world.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.12.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(${CMAKE_SOURCE_DIR}/src/Eigen-3.3/Eigen)

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
