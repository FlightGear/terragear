/*
-------------------------------------------------------------------------
 Merger of Flight Gear base scenery data
 Losely based on the testclipper by Curtis Olson

 Written by Alexei Novikov, Oct. 1999.

 Copyright (C) 1999 Alexei Novikov, anovikov@heron.itep.ru

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
--------------------------------------------------------------------------
*/
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <dirent.h>

#include "merger.hxx"

SG_USING_STD(cout);
SG_USING_STD(endl);

int main( int argc, char **argv ) {
  point2d global_min, global_max;
  
  sglog().setLogLevels( SG_ALL, SG_DEBUG );
  
  global_min.x = global_min.y = 200;
  global_max.y = global_max.x = -200;
  
  FGMerger clipper;
  clipper.init();

  FGPolyList subject;

  if ( argc < 2 ) {
    SG_LOG( SG_CLIPPER, SG_ALERT, "Usage: " << argv[0] 
	    << " file1 file2 ... dir_to_put" );
    exit(-1);
  }
  string base_name;
  // process all specified polygon files
  string cur_dir=argv[0];
  DIR *d;
  // struct dirent *de;
  if ( (d = opendir( cur_dir.c_str() )) == NULL ) {
        cout << "cannot open directory " << cur_dir << "\n";
        return 0;
  }

  //  string file;
  //vector<char> files;

  //  while ( (de = readdir(d)) != NULL ) {
  // file = de->d_name;
  // if (file == ".." || file == ".") continue;
  // files.push_back(file);
  //}
  
  
    
  for ( int i = 1; i < argc-1; i++ ) {
    string full_path = argv[i];
    
    // determine bucket for this polygon
    int pos = full_path.rfind("/");
    string file_name = full_path.substr(pos + 1);
    cout << "file name = " << file_name << endl;
    
    pos = file_name.find(".");
    base_name = file_name.substr(0, pos);
    cout << "base_name = " << base_name << endl;
    
    long int index;
    sscanf( base_name.c_str(), "%ld", &index);
    SGBucket b(index);
    cout << "bucket = " << b << endl;
    
    // calculate bucket dimensions
    point2d c, min, max;
    
    c.x = b.get_center_lon();
    c.y = b.get_center_lat();
    double span = sg_bucket_span(c.y);
    
    if ( (c.y >= -89.0) && (c.y < 89.0) ) {
      min.x = c.x - span / 2.0;
      max.x = c.x + span / 2.0;
      min.y = c.y - SG_HALF_BUCKET_SPAN;
      max.y = c.y + SG_HALF_BUCKET_SPAN;
    } else if ( c.y < -89.0) {
      min.x = -90.0;
      max.x = -89.0;
      min.y = -180.0;
      max.y = 180.0;
    } else if ( c.y >= 89.0) {
      min.x = 89.0;
      max.x = 90.0;
      min.y = -180.0;
      max.y = 180.0;
    } else {
      SG_LOG ( SG_GENERAL, SG_ALERT, 
	       "Out of range latitude in clip_and_write_poly() = " 
	       << c.y );
    }
    
    if ( min.x < global_min.x ) global_min.x = min.x;
    if ( min.y < global_min.y ) global_min.y = min.y;
    if ( max.x > global_max.x ) global_max.x = max.x;
    if ( max.y > global_max.y ) global_max.y = max.y;
    
    // finally, load the polygon(s) from this file
    clipper.load_polys( full_path, subject);
  }
  //  FGPolyList subject;
  //subject=clipper.get_polys_clipped();
  
  clipper.merge(subject);
  const long int index=atoi(base_name.c_str());
  SGBucket b(index);
  string root=argv[argc-1];
  string path = root + "/Scenery/" + b.gen_base_path();
  string command = "mkdir -p " + path;
  system( command.c_str() );
  string polyfile = path + "/" + base_name;
  clipper.write(subject, polyfile);
  // do the clipping

}
