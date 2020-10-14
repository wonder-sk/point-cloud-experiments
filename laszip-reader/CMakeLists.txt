cmake_minimum_required(VERSION 3.5)

project(laszip-reader LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(Qt5Core)

INCLUDE_DIRECTORIES(/home/martin/inst/laz-perf/cpp/laz-perf)

add_executable(laszip-reader
  main.cpp
)
target_link_libraries(laszip-reader /home/martin/inst/LASzip-inst/lib/liblaszip_api.so dl)
