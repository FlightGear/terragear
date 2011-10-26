add_library(Terra STATIC 
    Array.h Geom.h 
    GreedyInsert.cc GreedyInsert.h 
    Heap.cc Heap.h 
    Map.cc Map.h
    Mask.cc Mask.h 
    Quadedge.cc Quadedge.h 
    Subdivision.cc Subdivision.h 
    Vec2.h Vec3.h
	getopt.c getopt.h 
)

add_executable(terra_bin 
    cmdline.cc greedy.cc output.cc terra.cc terra.h version.h)

target_link_libraries(terra_bin Terra)
