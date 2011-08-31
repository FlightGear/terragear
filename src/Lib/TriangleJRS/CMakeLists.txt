

add_library(TriangleJRS STATIC 
    tri_support.c
    tri_support.h
    triangle.c
    triangle.h
)

set_target_properties(TriangleJRS PROPERTIES
        COMPILE_DEFINITIONS TRILIBRARY )

#add_executable(test_triangle test_triangle.c)

#target_link_libraries(test_triangle 
#    TriangleJRS
#	)
