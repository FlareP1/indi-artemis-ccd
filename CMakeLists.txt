# define the project name
project(indi-artemis-ccd C CXX)
cmake_minimum_required(VERSION 2.8)

include(GNUInstallDirs)

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# find our required packages
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
find_package(ZLIB REQUIRED)
find_package(GSL REQUIRED)

#find_library(ARTEMIS_LIBRARY
#    NAMES libartemis
#    HINTS "/home/astroberry/libartemis"
#    PATH_SUFFIXES build include
#)

# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 1)
set(CDRIVER_VERSION_MINOR 0)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

# do the replacement in the driver's xml file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_artemis_ccd.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_artemis_ccd.xml
)

# set our include directories to look for header files
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${NOVA_INCLUDE_DIR})
include_directories( ${EV_INCLUDE_DIR})

include(CMakeCommon)

#Add the compile defines required by the wxWidgets library
add_definitions(-DNOPCH -D__WXDEBUG__ -D_DEBUG -D__WX__ )
 
#Set Compiler Flags
# from wx-config --version=2.8 --toolkit=gtk2 --debug --cxxflags
SET(GCC_COVERAGE_COMPILE_FLAGS "-I${PROJECT_SOURCE_DIR}/../libartemis -I${PROJECT_SOURCE_DIR}/../wxWidgets-2.8.12/include/ -I${PROJECT_SOURCE_DIR}/../wxWidgets-2.8.12/build-debug/lib/wx/include/gtk2-ansi-debug-2.8/ -I/usr/local/lib/wx/include/gtk2-ansi-debug-2.8 -I/usr/local/include/wx-2.8 -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -D__WXDEBUG__ -D__WXGTK__ -pthread -Wno-unused-local-typedefs -Wno-ignored-qualifiers -Wno-deprecated")
# from wx-config  --version=2.8 --toolkit=gtk2 --debug --libs
SET(GCC_COVERAGE_LINK_FLAGS  "-L${PROJECT_SOURCE_DIR}/../libartemis/build -lartemis -L{PROJECT_SOURCE_DIR}/../wxWidgets-2.8.12/build-debug/lib -pthread -lwx_gtk2d_richtext-2.8 -lwx_gtk2d_aui-2.8 -lwx_gtk2d_xrc-2.8 -lwx_gtk2d_qa-2.8 -lwx_gtk2d_html-2.8 -lwx_gtk2d_adv-2.8 -lwx_gtk2d_core-2.8 -lwx_based_xml-2.8 -lwx_based_net-2.8 -lwx_based-2.8 -lusb")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

# tell cmake to build our executable
add_executable(
    indi_artemis_ccd
    indi_artemis_ccd.cpp
)

# and link it to these libraries
target_link_libraries(
    indi_artemis_ccd
 #   ${ARTEMIS_LIBRARY}
    ${INDI_LIBRARIES}
    ${NOVA_LIBRARIES}
    ${GSL_LIBRARIES}
)

# tell cmake where to install our executable
install(TARGETS indi_artemis_ccd RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_artemis_ccd.xml
    DESTINATION ${INDI_DATA_DIR}
)
