# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(opencv_example_project)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# Declare the executable target built from your sources
add_executable(opencv_example src/example.cpp)

# Link your application with OpenCV libraries
target_link_libraries(opencv_example ${OpenCV_LIBS})

add_executable(uv-stereo src/uv-stereo.cpp src/umap_util.cpp src/wp_planning.cpp)
target_link_libraries(uv-stereo ${OpenCV_LIBS})

add_executable(uv-stereo-raw src/uv-stereo-raw.cpp src/umap_util.cpp src/wp_planning.cpp)
target_link_libraries(uv-stereo-raw ${OpenCV_LIBS})

add_executable(stereo-calib src/stereo-calib.cpp)
target_link_libraries(stereo-calib ${OpenCV_LIBS})