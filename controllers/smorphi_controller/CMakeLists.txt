cmake_minimum_required(VERSION 3.8)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

# Set the  Webots home path (change it according to your installation method)
set(WEBOTS_HOME "/usr/local/webots")
#set(WEBOTS_HOME "/snap/webots/current/usr/share/webots")
include_directories(include)
# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES} smorphi_controller.cpp)
target_link_libraries(${PROJECT} ${LIBRARIES} zmq)

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)