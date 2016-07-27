cmake_minimum_required ( VERSION 2.8)
project( inputoutput )

set(SRCEXT "cpp")

# searching for source files : files in the SRCDIR directory with the SRCEXT extension.
file(
    GLOB_RECURSE
    source_files
    *.${SRCEXT}
)

find_package ( OpenCV REQUIRED )
find_package( osg )
find_package( osgDB )
find_package( osgViewer )

add_library( io ${source_files})

macro( config_project PROJNAME LIBNAME )
	include_directories( ${${LIBNAME}_INCLUDE_DIR} )
	target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

config_project(io OSG )
config_project(io OSGDB )
config_project(io OSGUTIL )
config_project(io OSGVIEWER )

