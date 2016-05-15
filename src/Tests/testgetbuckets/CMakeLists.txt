add_executable(testgetbuckets
    testgetbuckets.cxx
)

target_link_libraries(testgetbuckets
    terragear
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS testgetbuckets RUNTIME DESTINATION bin)
