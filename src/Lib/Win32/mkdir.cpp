//
// file:  mkdir.cpp
//
// A window mkdir function. Windows 9x system mkdir command will only
// create a single directory at a time. This function will parse the
// path and create each individual directory.

// Written by Bruce Finney
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <direct.h>
#include <io.h>
#include <string.h>
#include <string>

using namespace std;

// NOTE: the system mkdir will accept either a "/" or a "\"
//       command.com ( the shell ) only accepts "\" in the path.
static char SEP[] = "/\\";

void fg_mkdir( const char *path )
{
    char *r, *token, tmp_path[256];
    string dir;
    struct _finddata_t de;

    strcpy( tmp_path, path );
    r = strchr( SEP, path[0] );  // is first char a seperator?
    token = strtok( tmp_path, SEP );
    if ( r != NULL ) {
	dir = --token;	// include first char
    } else {
	dir = token;
    }

    while ( token != NULL ) {
	if ( _findfirst( dir.c_str(), &de ) == -1 && token[1] != ':' ) {
	    // does not exist - create it
	    mkdir( dir.c_str() );
	}  // end if
	token = strtok( NULL, SEP );
	if ( token != NULL ) {
	    dir = dir + "/" + token;
	}
    }  // end while

}  // end fg_mkdir
