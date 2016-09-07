cmake_minimum_required ( VERSION 2.8)
project( examples )

MESSAGE(STATUS “OSG_ROOT = $ENV{OSG_ROOT}”)
MESSAGE(STATUS “OSG_LIBRARY_PATH = $ENV{OSG_LIBRARY_PATH}”)

set(SRCEXT "cpp")

find_package ( OpenCV REQUIRED )
find_package ( PCL 1.7 )
find_package( osg )
find_package( osgDB )
find_package( osgViewer )

macro( config_project PROJNAME LIBNAME )
	include_directories( ${${LIBNAME}_INCLUDE_DIR} )
	target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

if (PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif (PCL_FOUND)

#set link directories for lgc
add_library(lgc SHARED IMPORTED)
set_target_properties(lgc PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../src/liblgc.a")

add_library(io SHARED IMPORTED)
set_target_properties(io PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../io/build/libio.a")

add_executable(osgexample osgexample.cpp)
target_link_libraries(
    osgexample
    lgc
    io
    ${OpenCV_LIBS}
    ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
)

config_project(osgexample OSG )
config_project(osgexample OSGDB )
config_project(osgexample OSGVIEWER )

add_executable(opencvexample opencvexample.cpp)

target_link_libraries(
    opencvexample
    lgc
    io
    ${OpenCV_LIBS}
    ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
)
message("CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
