/*
 *  file:  mkdir.cpp
 */
#include "simgear/config.h"
#include "simgear/compiler.h"

#include <direct.h>
#include <io.h>
#include <string.h>
#include <string>

using namespace std;

static char SEP[] = "/\\";

void fg_mkdir( const char *path )
{

	char *r, *token, tmp_path[256];
	string dir;
	struct _finddata_t de;
	//struct stat stat_buf;

	strcpy( tmp_path, path );
	r = strchr( SEP, path[0] );  // is first char a seperator?
	token = strtok( tmp_path, SEP );
	if ( r != NULL )
		dir = --token;	// include first char
	else
		dir = token;

	while ( token != NULL )
		{
		//if ( stat( dir.c_str, &stat_buf) != 0 && errno == ENOENT )
		if ( _findfirst( dir.c_str(), &de ) == -1 && token[1] != ':' ) 
			{  // does not exist - create it
			mkdir( dir.c_str() );
			//printf( "mkdir %s\n", dir.c_str() );
			}  // end if
		token = strtok( NULL, SEP );
		if ( token != NULL )
			//dir = dir + DIR_SEP + token;
			dir = dir + "/" + token;
		}  // end while

}  // end fg_mkdir
