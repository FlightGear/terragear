/*
-------------------------------------------------------------------------
 Library for the clipper of Flight Gear base scenery data
 Losely based on the library libClipper by Curtis Olson

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
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/fgstream.hxx>

#include <Polygon/names.hxx>
#include <Polygon/index.hxx>

#include <string>
#include <stdio.h>

#include "merger.hxx"

// Constructor
FGMerger::FGMerger( void ) {
}

// Destructor
FGMerger::~FGMerger( void ) {
}

bool FGMerger::init() {
  for ( int i = 0; i < FG_MAX_AREA_TYPES; ++i ) {
    polys_subject.polys[i].clear();
  }
  
  return true;
}


bool FGMerger::load_polys(const string& path, FGPolyList& subject) {
  string poly_name;
  AreaType poly_type = DefaultArea;
  int contours, count, i, j;
  int hole_flag;
  double startx, starty, x, y, lastx, lasty;
  
  FG_LOG( FG_CLIPPER, FG_INFO, "Loading " << path << " ..." );
  
  fg_gzifstream in( path );
  
  if ( !in ) {
    FG_LOG( FG_CLIPPER, FG_ALERT, "Cannot open file: " << path );
    exit(-1);
  }
  
  FGPolygon poly;
  
  Point3D p;
  in >> skipcomment;
  while ( !in.eof() ) {
    in >> poly_name;
    cout << "poly name = " << poly_name << endl;
    poly_type = get_area_type( poly_name );
    cout << "poly type (int) = " << (int)poly_type << endl;
    in >> contours;
    cout << "num contours = " << contours << endl;
    
    poly.erase();
    
    for ( i = 0; i < contours; ++i ) {
      in >> count;
      
      if ( count < 3 ) {
	FG_LOG( FG_CLIPPER, FG_ALERT, 
		"Polygon with less than 3 data points." );
	exit(-1);
      }
      
      in >> hole_flag;
      
      in >> startx;
      in >> starty;
      p = Point3D(startx, starty, 0.0);
      poly.add_node( i, p );
      /* FG_LOG( FG_CLIPPER, FG_BULK, "0 = " 
	      << startx << ", " << starty );
      */
      for ( j = 1; j < count - 1; ++j ) {
	in >> x;
	in >> y;
	p = Point3D( x, y, 0.0 );
	poly.add_node( i, p );
	FG_LOG( FG_CLIPPER, FG_BULK, j << " = " << x << ", " << y );
      }
      
      in >> lastx;
      in >> lasty;
      
      if ( (fabs(startx - lastx) < FG_EPSILON) 
	   && (fabs(starty - lasty) < FG_EPSILON) ) {
	// last point same as first, discard
      } else {
	p = Point3D( lastx, lasty, 0.0 );
	poly.add_node( i, p );
	FG_LOG( FG_CLIPPER, FG_BULK, count - 1 << " = " 
		<< lastx << ", " << lasty );
      }
      
      // gpc_add_contour( poly, &v_list, hole_flag );
    }
    
    in >> skipcomment;
  }
  
  int area = (int)poly_type;

  // if ( area == OceanArea ) {
  // TEST - Ignore
  // } else 
  
  if ( area < FG_MAX_AREA_TYPES ) {
    subject.polys[area].push_back(poly);
  } else {
    FG_LOG( FG_CLIPPER, FG_ALERT, "Polygon type out of range = " 
	    << (int)poly_type);
    exit(-1);
  }
  
  //FILE *ofp= fopen("outfile", "w");
  //gpc_write_polygon(ofp, &polys);
  
  return true;
}

void FGMerger::merge( FGPolyList& clipped ) {
  
  FGPolygon poly, result, sliver;
  bool done;
  done=false;
  
  for ( int area = 0; area < FG_MAX_AREA_TYPES && !done; ++area ) {
    /* cout << "  testing area = " << area << " with " 
       << clipped.polys[area].size() << " polys" << endl; */
    if (clipped.polys[area].size() > 0) {
      result = clipped.polys[area][0];
      for ( int j = 1; 
	    j < (int)clipped.polys[area].size() && !done;
	    ++j )
	{
	  cout << "  polygon = " << j << endl;
	  
	  poly = clipped.polys[area][j];
	  result = polygon_union( poly, result );
	  done=true;
	}
      clipped.polys[area].clear();
      clipped.polys[area].push_back(result); 
    }
  }
    
  if ( !done ) {
    cout << "no suitable polys found for merge" << endl;
  }
}


void FGMerger::clip(FGPolyList& subject, FGPolyList& clip) {
  FGPolygon poly, result, cliped, difference;
  int max_a[FG_MAX_AREA_TYPES];
  int max_area;
  int default_indx=(int)get_area_type("Default");
  
  difference = clip.polys[default_indx][0];

  for ( int area = 0; area < FG_MAX_AREA_TYPES; ++area ) {
    //cout << "  testing area = " << area << endl;
      if ((int)subject.polys[area].size() > 0) {
	cout << "  Clipping polygon with area = " << area << endl;
	poly=subject.polys[area][0];
	cliped=clip.polys[default_indx][0];
	result = polygon_int(poly, cliped);
	difference = polygon_diff(difference, result);
	subject.polys[area][0]=result;
	max_a[area] +=result.contour_size(0); // let's hope we have only 1 contour polygons (first approximation)
      }
  }
  /* OK we have leftovers in difference where have we assign it. Simply add
     it to the last area type */
  int max=0;
  for ( int area = 0; area < FG_MAX_AREA_TYPES; ++area ) {
    if ( max_a[area] > max) {
      max_area=area;
    }
  }
  
  
  subject.polys[max_area][0] = polygon_union(subject.polys[max_area][0], difference);  
}

void FGMerger::write(FGPolyList& subject, string& file) {
  FGPolygon poly;
  char tile_name[256], poly_index[256];
  
  for ( int area = 0; area < FG_MAX_AREA_TYPES; ++area ) {
    //cout << "  testing area = " << area << endl;
    string file2 = file;
    string counter_file = "./../poly_counter";
    poly_index_init( counter_file );
    if ((int)subject.polys[area].size() > 0) {
      poly=subject.polys[area][0];
      cout << "  found a polygon" << endl;
      long int index = poly_index_next();
      sprintf(poly_index, "%ld", index);
      file2 += "."; 
      file2 += poly_index;
      cout << "  Dumping to file " << file2 << endl;
      FILE *fp = fopen( file2.c_str(), "w" );
      string area_name=get_area_name( (AreaType)area );
      fprintf(fp,"%s\n", area_name.c_str());
      const int cont= poly.contours();
      fprintf(fp,"%d\n", cont);
      for ( int i = 0; i < cont; ++i ) {
	fprintf(fp,"%d\n", poly.contour_size(i));
	fprintf(fp,"%d\n", poly.get_hole_flag(i));
	for ( int j = 0; j < (int)poly.contour_size(i); ++j ) {
	  fprintf(fp, "%.6f %.6f\n", (poly.get_pt(i,j)).x(), (poly.get_pt(i,j)).y());
	}
	//fprintf(fp, "%.6f %.6f\n", (poly.get_pt(i,0)).x(), (poly.get_pt(i,0)).y());
      }
      fclose(fp);
    }
  }
  
}

