

add_library(shape STATIC 
    dbfopen.c shpopen.c shapefil.h 
)

foreach( c 
		shpcreate
		shpadd
		shpdump
		shputils
		shptest
		dbfdump
		dbfadd
		dbfcreate
    )

    add_executable(${c} ${c}.c)
    target_link_libraries(${c} shape)
endforeach( c )


