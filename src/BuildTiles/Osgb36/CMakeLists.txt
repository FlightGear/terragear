

add_library(Osgb36 STATIC 
    osgb36.cxx osgb36.hxx osgbtc.cxx osgbtc.hxx uk.cxx uk.hxx
)

add_executable(testosgb36 testosgb36.cxx)
target_link_libraries(testosgb36 Osgb36)
