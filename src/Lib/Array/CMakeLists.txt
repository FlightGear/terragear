# array library, and test_array app

add_library(Array STATIC 
    array.cxx array.hxx
)

include_directories(${PROJECT_SOURCE_DIR}/src/Lib/terragear)

install( TARGETS Array
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib 
         )
install( FILES array.hxx DESTINATION include/tg )

add_executable(test_array testarray.cxx)

add_executable(rectify_height rectify_height.cxx)

target_link_libraries(test_array 
    Array
    terragear
	${ZLIB_LIBRARY}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

target_link_libraries(rectify_height 
    Array
    terragear
	${ZLIB_LIBRARY}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS test_array rectify_height RUNTIME DESTINATION bin)
if (MSVC)
    set_target_properties( test_array PROPERTIES DEBUG_POSTFIX d )
endif ()
