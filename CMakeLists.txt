cmake_minimum_required(VERSION 3.5)

project(pdal-reader LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/martin/inst/PDAL-inst/lib/cmake) #/PDAL)
SET(CMAKE_PREFIX_PATH /home/martin/inst/PDAL-inst/lib/cmake)

find_package(PDAL)

find_package(Qt5Core)
find_package(Qt5Gui)

add_executable(pdal-reader
  #main.cpp
  pdaltest.cpp
  indexedpointcloud.cpp
  pointcloudrenderer.cpp
)

include_directories(${PDAL_INCLUDE_DIRS} /home/martin/inst/LASzip-inst/include)
target_link_libraries(pdal-reader ${PDAL_LIBRARIES}  Qt5::Core Qt5::Gui)

