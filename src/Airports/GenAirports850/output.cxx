// output.cxx -- routines to output index files of an airport
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999 - 2000  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <cstdlib>
#include <fstream>
#include <set>
#include <string>

#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>

using std::string;

// update index file (list of objects to be included in final scenery build)
void write_index_object( const string& base, const SGBucket& b, const string& name )
{
    string dir = base + "/" + b.gen_base_path();
    SGPath sgp( dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    string file = dir + "/" + b.gen_index_str() + ".ind";
    SG_LOG( SG_GENERAL, SG_DEBUG, "Writing object to " << file );

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "a" )) == NULL ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "ERROR: opening " << file << " for writing!" );
        exit(-1);
    }

    fprintf( fp, "OBJECT %s\n", name.c_str() );
    fclose( fp );
}


// update index file (list of shared objects to be included in final scenery build)
void write_index_object_shared( const string &base, const SGBucket &b,
                         const SGGeod &p, const string& name,
                         const double &heading )
{
    string dir = base + "/" + b.gen_base_path();
    SGPath sgp( dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    string file = dir + "/" + b.gen_index_str() + ".ind";
    SG_LOG( SG_GENERAL, SG_DEBUG, "Writing shared object to " << file );

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "a" )) == NULL ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "ERROR: opening " << file << " for writing!" );
        exit(-1);
    }

    fprintf( fp, "OBJECT_SHARED %s %.6f %.6f %.1f %.2f\n", name.c_str(),
             p.getLongitudeDeg(), p.getLatitudeDeg(), p.getElevationM(), heading );
    fclose( fp );
}


// update index file (list of shared objects to be included in final scenery build)
void write_index_object_sign( const string &base, const SGBucket &b,
                        const SGGeod &p, const string& sign,
                        const double &heading, const int &size)
{
    string dir = base + "/" + b.gen_base_path();
    SGPath sgp( dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    string file = dir + "/" + b.gen_index_str() + ".ind";
    SG_LOG( SG_GENERAL, SG_DEBUG, "Writing sign to " << file );

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "a" )) == NULL ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "ERROR: opening " << file << " for writing!" );
        exit(-1);
    }

    fprintf( fp, "OBJECT_SIGN %s %.6f %.6f %.1f %.2f %d\n", sign.c_str(),
             p.getLongitudeDeg(), p.getLatitudeDeg(), p.getElevationM(), heading, size );
    fclose( fp );
}


// purge the existing index file when it already exists
void truncate_index_file( const std::string& fileName )
{
    if (static_cast<bool>(std::ifstream(fileName)))
    {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Truncating file " << fileName );

        std::ofstream fsIndex;
        fsIndex.open(fileName, std::ofstream::out | std::ofstream::trunc);
        fsIndex.close();
    }
}
