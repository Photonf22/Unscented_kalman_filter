cmake_minimum_required(VERSION 3.14)
#set(CMAKE_CXX_STANDARD 14)
add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}" "${CMAKE_CXX_FLAGS}" "-pthread" "-lgdi32" "-D_GLIBCXX_USE_CXX11_ABI=1")
#set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=1")
project(playback)

find_package(PCL 1.13 REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
if (TARGET Eigen3::Eigen)
  # Use the imported target
endif (TARGET Eigen3::Eigen)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (ukf_highway src/main.cpp src/ukf.cpp src/tools.cpp src/render/render.cpp)
target_link_libraries (ukf_highway ${PCL_LIBRARIES})
