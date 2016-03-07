include_directories(${GDAL_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src/Lib)
add_executable(tgChopperTest tgChopperTest.cxx)

target_link_libraries(tgChopperTest
    ${GDAL_LIBRARY}
    terragear
    ${ZLIB_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS tgChopperTest RUNTIME DESTINATION bin)
