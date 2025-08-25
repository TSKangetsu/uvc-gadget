cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(THREADS_PREFER_PTHREAD_FLAG ON)

project(v4l2-gadget)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -s -Wstack-usage=1000")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -s -Wstack-usage=1000")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_EXE_LINKER_FLAGS "-static -static-libstdc++ -static-libgcc")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")

add_compile_definitions(DEBUG)

find_package(Threads REQUIRED)

add_executable(uvc-gadget uvc-gadget.c)
target_link_libraries(uvc-gadget Threads::Threads)
target_link_libraries(uvc-gadget gcc_pic) 
