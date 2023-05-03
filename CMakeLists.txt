cmake_minimum_required(VERSION 3.10)
project(drake_cmake_installed_apt)

#include(CTest)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

#find_package(Threads MODULE REQUIRED)

if(CMAKE_VERSION VERSION_LESS 3.12)
  list(INSERT CMAKE_MODULE_PATH 0
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/cmake/3.12/Modules"
  )
endif()

#find_package(Python 3.6 MODULE REQUIRED)
#set(PYTHON_VERSION_MAJOR_MINOR
  #"${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}"
#)

find_package(drake CONFIG REQUIRED PATHS /opt/drake)

#For matplotlibcpp
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

#get_filename_component(DRAKE_PYTHONPATH "${drake_DIR}" DIRECTORY)
#get_filename_component(DRAKE_PYTHONPATH "${DRAKE_PYTHONPATH}" DIRECTORY)
#set(DRAKE_PYTHONPATH
  #"${DRAKE_PYTHONPATH}/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages"
#)

#add_subdirectory(src)

add_executable(main src/main.cpp src/RobotSystem.cpp src/Utils.cpp src/Optimize.cpp)
target_link_libraries(main drake::drake Python3::Python)

add_executable(replay src/replay.cpp src/RobotSystem.cpp src/Utils.cpp)
target_link_libraries(replay drake::drake Python3::Python)