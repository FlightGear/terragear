cmake_minimum_required (VERSION 2.6.4)

include (CheckFunctionExists)
include (CheckCSourceCompiles)
include (CheckCXXSourceCompiles)
include (CheckIncludeFile)
include (CPack)

project(TerraGear)

set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually d on windows")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")
set(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")
set(CMAKE_MINSIZEREL_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")

# We have some custom .cmake scripts not in the official distribution.
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

# Change the default build type to something fast
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)


if (MSVC)
  GET_FILENAME_COMPONENT(PARENT_DIR ${PROJECT_SOURCE_DIR} PATH)
  if (CMAKE_CL_64)
    SET(TEST_3RDPARTY_DIR "${PARENT_DIR}/3rdparty.x64")
  else (CMAKE_CL_64)
    SET(TEST_3RDPARTY_DIR "${PARENT_DIR}/3rdparty")
  endif (CMAKE_CL_64)
  if (EXISTS ${TEST_3RDPARTY_DIR})
    set(MSVC_3RDPARTY_ROOT ${PARENT_DIR} CACHE PATH "Location where the third-party dependencies are extracted")
  else (EXISTS ${TEST_3RDPARTY_DIR})
    set(MSVC_3RDPARTY_ROOT NOT_FOUND CACHE PATH "Location where the third-party dependencies are extracted")
  endif (EXISTS ${TEST_3RDPARTY_DIR})
else (MSVC)
  set(MSVC_3RDPARTY_ROOT NOT_FOUND CACHE PATH "Location where the third-party dependencies are extracted")
endif (MSVC)

if (MSVC AND MSVC_3RDPARTY_ROOT)
  message(STATUS "3rdparty files located in ${MSVC_3RDPARTY_ROOT}")
  set( OSG_MSVC "msvc" )
  if (${MSVC_VERSION} EQUAL 1600)
      set( OSG_MSVC ${OSG_MSVC}100 )
  else (${MSVC_VERSION} EQUAL 1600)
      set( OSG_MSVC ${OSG_MSVC}90 )
  endif (${MSVC_VERSION} EQUAL 1600)
  if (CMAKE_CL_64)
      set( OSG_MSVC ${OSG_MSVC}-64 )
	  set( MSVC_3RDPARTY_DIR 3rdParty.x64 )
  else (CMAKE_CL_64)
	  set( MSVC_3RDPARTY_DIR 3rdParty )
  endif (CMAKE_CL_64)

  set (CMAKE_LIBRARY_PATH ${MSVC_3RDPARTY_ROOT}/${MSVC_3RDPARTY_DIR}/lib ${MSVC_3RDPARTY_ROOT}/install/${OSG_MSVC}/OpenScenegraph/lib ${MSVC_3RDPARTY_ROOT}/install/${OSG_MSVC}/SimGear/lib )
  set (CMAKE_INCLUDE_PATH ${MSVC_3RDPARTY_ROOT}/${MSVC_3RDPARTY_DIR}/include ${MSVC_3RDPARTY_ROOT}/install/${OSG_MSVC}/OpenScenegraph/include ${MSVC_3RDPARTY_ROOT}/install/${OSG_MSVC}/SimGear/include)
  set (BOOST_ROOT ${MSVC_3RDPARTY_ROOT}/boost_1_44_0)
  set (OPENAL_INCLUDE_DIR ${MSVC_3RDPARTY_ROOT}/${MSVC_3RDPARTY_DIR}/include)
  set (ALUT_INCLUDE_DIR ${MSVC_3RDPARTY_ROOT}/${MSVC_3RDPARTY_DIR}/include)
  set (OPENAL_LIBRARY_DIR ${MSVC_3RDPARTY_ROOT}/${MSVC_3RDPARTY_DIR}/lib)
endif (MSVC AND MSVC_3RDPARTY_ROOT)

option(ENABLE_GDAL "Set to ON to build TerraGear with GDAL support" ON)

# check required dependencies
find_package(Boost REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(SimGear 2.5.0 REQUIRED)
find_package(GPC REQUIRED)
find_package(TIFF) # needed for SRTM
if (CMAKE_CL_64)
	find_package(JPEG) # needed for SRTM
endif (CMAKE_CL_64)
find_package(NewMat11)

if(ENABLE_GDAL)
    find_package(GDAL) # build OGRDecode
endif(ENABLE_GDAL)

check_include_file(unistd.h HAVE_UNISTD_H)
check_include_file(sys/time.h HAVE_SYS_TIME_H)
check_include_file(windows.h HAVE_WINDOWS_H)

check_function_exists (rint HAVE_RINT)

set(PKGDATADIR ${CMAKE_INSTALL_PREFIX}/share/TerraGear)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(WARNING_FLAGS -Wall)
endif(CMAKE_COMPILER_IS_GNUCXX)

if(WIN32)

    if(MSVC)
        # turn off various warnings
        # foreach(warning 4244 4251 4267 4275 4290 4786 4305 4996)
        #     SET(WARNING_FLAGS "${WARNING_FLAGS} /wd${warning}")
        # endforeach(warning)
        
        set(MSVC_FLAGS "-DNOMINMAX -D_USE_MATH_DEFINES -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS -D__CRT_NONSTDC_NO_WARNINGS -DNO_TIMER")
        if (${MSVC_VERSION} EQUAL 1600)
		  set( MSVC_LD_FLAGS "/FORCE:MULTIPLE" )
        endif (${MSVC_VERSION} EQUAL 1600)
    endif(MSVC)

    set(NOMINMAX 1)
endif(WIN32)    

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${WARNING_FLAGS} ${MSVC_FLAGS} -D_REENTRANT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_FLAGS} ${MSVC_FLAGS} -D_REENTRANT -DNO_OPENSCENEGRAPH_INTERFACE")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MSVC_LD_FLAGS}")

include_directories(
	${Boost_INCLUDE_DIRS}
	${ZLIB_INCLUDE_DIR}
	${SIMGEAR_INCLUDE_DIR}
	)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_BINARY_DIR}/src/Include)

add_definitions(-DHAVE_CONFIG_H)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/src/Include/config_cmake.h.in"
  "${PROJECT_BINARY_DIR}/src/Include/config.h"
  )

add_subdirectory(src)
#add_subdirectory(gismodules)

#-----------------------------------------------------------------------------
### uninstall target
#-----------------------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

