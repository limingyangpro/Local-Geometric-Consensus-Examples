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

#tracking target 2D
add_executable(textureTracking textureTracking.cpp)

#link_directories ( ${ARMaps_BINARY_DIR}/basiclib ) 
# linking other libraries
target_link_libraries(
    textureTracking	
    io
    lgc
    ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
)

macro( config_project PROJNAME LIBNAME )
	include_directories( ${${LIBNAME}_INCLUDE_DIR} )
	target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

#augmentation
add_executable(augmentingDrawings augmentingDrawings.cpp )
target_link_libraries(
    augmentingDrawings	
    io
    lgc
    ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${OpenCV_LIBS}
)
config_project(augmentingDrawings OPENTHREADS )
config_project(augmentingDrawings OSG )
config_project(augmentingDrawings OSGDB )
config_project(augmentingDrawings OSGUTIL )
config_project(augmentingDrawings OSGVIEWER )


message("CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")


