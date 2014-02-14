// fillvoids.cxx -- fill voids in one array from data points of another array.
//
// Written by Curtis Olson, started November 2005.
//
// Copyright (C) 2005  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: fillvoids.cxx,v 1.1 2005-11-08 16:30:10 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <string>
#include <iostream>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_path.hxx>

#include <Array/array.hxx>

#include <stdlib.h>

using std::cout;
using std::endl;
using std::string;


int main(int argc, char **argv) {
    sglog().setLogLevels( SG_ALL, SG_WARN );

    if ( argc != 3 ) {
	cout << "Usage " << argv[0] << " <src_array> <fill_array_base>"
             << endl;
	exit(-1);
    }

    string src_array_path = argv[1];
    string fill_base_path = argv[2];

    // compute the fill array path
    SGPath tmp1( src_array_path );
    string tmp2 = tmp1.base(); // strip .gz
    cout << "tmp2 = " << tmp2 << endl;
    string tmp3 = SGPath(tmp2).base(); // strip .arr
    cout << "tmp3 = " << tmp3 << endl;
    string file = SGPath(tmp3).file();
    cout << "file = " << file << endl;
    long int index = atoi(file.c_str());
    SGBucket bucket( index );
    SGPath tmp4( fill_base_path );
    tmp4.append( bucket.gen_base_path() );
    tmp4.append( file );
    string tmp5 = tmp1.dir();
    string tmp6 = SGPath(tmp5).dir();
    string tmp7 = SGPath(tmp6).dir();
    cout << "tmp7 = " << tmp7 << endl;

    cout << "Index = " << index << endl;
    cout << "fill array = " << tmp4.str() << endl;

    // open the source array
    TGArray src_array;
    src_array.open( tmp3 );
    src_array.parse( bucket );
    if ( !src_array.is_open() ) {
      cout << "Unable to open source array " << tmp3 << endl;
      return -1;
    }

    // open the fill array
    TGArray fill_array;
    fill_array.open( tmp4.str() );
    fill_array.parse( bucket );
    if ( !fill_array.is_open() ) {
      cout << "no fill array, nothing to do " << tmp4.str() << endl;
      return 0;
    }

    // traverse the source array and lookup replacement values for any voids
    bool has_void = false;
    for ( int i = 0; i < src_array.get_cols(); ++i ) {
      for ( int j = 0; j < src_array.get_rows(); ++j ) {
	int src_elev = src_array.get_array_elev(i, j);
	if ( src_elev < -9000 ) {
	  has_void = true;
	  cout << "We have a void = " << src_elev << endl;
	  int fill_elev = fill_array.get_array_elev(i, j);
	  cout << "Replacing with " << fill_elev << endl;
	  src_array.set_array_elev(i, j, fill_elev);
	}
      }
    }

    // write out the new data file if we filled any voids
    if ( has_void ) {
      cout << "Has voids, writing file ..." << endl;
      bool result = src_array.write( tmp7, bucket );
      if ( result ) {
	// filled data written to new file name, now replace old file
	SGPath tmp_file(tmp3);
	tmp_file.concat(".arr.new.gz");
	SGPath orig_file(tmp3);
	orig_file.concat(".arr.gz");
        orig_file.rename(tmp_file);
      }
    } else {
      cout << "no voids" << endl;
    }

    return 0;
}


