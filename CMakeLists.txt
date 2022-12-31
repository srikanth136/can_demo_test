cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 14)

project(coursework C CXX)

# turns out that g++ will allow variable length arrays but I 
# dont' want students getting dependant on non-standard features
 # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wvla -Werror=vla")

# limit number of errors shown to 1
 # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors")

# location of source code files
include_directories(coursework)

# tell cmake where to put the executables that it creates
file(MAKE_DIRECTORY bin)    
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin ) 

# where to put the object files it creates
file(MAKE_DIRECTORY lib)
SET(LIBRARY_OUTPUT_PATH lib)

# add tests
# enable_testing() 
# add_subdirectory(Testing)
# add_subdirectory(Lecture)

 add_library( candata candata.c )

add_executable( main socketcan.cpp )
target_link_libraries( main candata )