// clipper.cxx -- top level routines to take a series of arbitrary areas and
//                produce a tight fitting puzzle pieces that combine to make a
//                tile
//
// Written by Curtis Olson, started February 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$
 


#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/fgstream.hxx>

#include <Polygon/names.hxx>

#include "clipper.hxx"

SG_USING_STD(cout);


#define MASK_CLIP 1


// Constructor.
FGClipper::FGClipper() {
}


// Destructor.
FGClipper::~FGClipper() {
}


// Initialize the clipper (empty all the polygon buckets.)
bool FGClipper::init() {
    for ( int i = 0; i < FG_MAX_AREA_TYPES; ++i ) {
	polys_in.polys[i].clear();
    }

    return true;
}


// Load a polygon definition file.
bool FGClipper::load_polys(const string& path) {
    string poly_name;
    AreaType poly_type = DefaultArea;
    int contours, count, i, j;
    int hole_flag;
    double startx, starty, x, y, lastx, lasty;

    SG_LOG( SG_CLIPPER, SG_INFO, "Loading " << path << " ..." );

    fg_gzifstream in( path );

    if ( !in ) {
        SG_LOG( SG_CLIPPER, SG_ALERT, "Cannot open file: " << path );
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
		SG_LOG( SG_CLIPPER, SG_ALERT, 
			"Polygon with less than 3 data points." );
		exit(-1);
	    }

	    in >> hole_flag;

	    in >> startx;
	    in >> starty;
	    p = Point3D(startx, starty, 0.0);
	    // cout << "poly pt = " << p << endl;
	    poly.add_node( i, p );
	    SG_LOG( SG_CLIPPER, SG_BULK, "0 = " 
		    << startx << ", " << starty );

	    for ( j = 1; j < count - 1; ++j ) {
		in >> x;
		in >> y;
		p = Point3D( x, y, 0.0 );
		// cout << "poly pt = " << p << endl;
		poly.add_node( i, p );
	    }

	    in >> lastx;
	    in >> lasty;

	    if ( (fabs(startx - lastx) < SG_EPSILON) 
		 && (fabs(starty - lasty) < SG_EPSILON) ) {
		// last point same as first, discard
	    } else {
		p = Point3D( lastx, lasty, 0.0 );
		// cout << "poly pt = " << p << endl;
		poly.add_node( i, p );
	    }
	}

	in >> skipcomment;
    }

    int area = (int)poly_type;

    add_poly(area, poly);

    // FILE *ofp= fopen("outfile", "w");
    // gpc_write_polygon(ofp, &polys.landuse);

    return true;
}

// Add a polygon to the clipper.
void FGClipper::add_poly( int area, const FGPolygon &poly )
{
    if ( area < FG_MAX_AREA_TYPES ) {
	polys_in.polys[area].push_back(poly);
    } else {
	SG_LOG( SG_CLIPPER, SG_ALERT, "Polygon type out of range = " 
		<< area);
	exit(-1);
    }
}


// Move slivers from in polygon to out polygon.
void FGClipper::move_slivers( FGPolygon& in, FGPolygon& out ) {
    // traverse each contour of the polygon and attempt to identify
    // likely slivers

    // cout << "Begin move slivers" << endl;

    int i;

    out.erase();

    double angle_cutoff = 10.0 * SGD_DEGREES_TO_RADIANS;
    double area_cutoff = 0.000008;
    double min_angle;
    double area;

    point_list contour;
    int hole_flag;

    // process contours in reverse order so deleting a contour doesn't
    // foul up our sequence
    for ( i = in.contours() - 1; i >= 0; --i ) {
	// cout << "contour " << i << endl;

	min_angle = in.minangle_contour( i );
	area = in.area_contour( i );

	/* cout << "  min_angle (rad) = " 
	     << min_angle << endl;
	cout << "  min_angle (deg) = " 
	     << min_angle * 180.0 / SG_PI << endl;
	cout << "  area = " << area << endl; */

	if ( ((min_angle < angle_cutoff) && (area < area_cutoff)) ||
	     ( area < area_cutoff / 10.0) )
	{
	    // cout << "      WE THINK IT'S A SLIVER!" << endl;

	    // check if this is a hole
	    hole_flag = in.get_hole_flag( i );

	    if ( hole_flag ) {
		// just delete/eliminate/remove sliver holes
		// cout << "just deleting a sliver hole" << endl;
		in.delete_contour( i );
	    } else {
		// move sliver contour to out polygon
		contour = in.get_contour( i );
		in.delete_contour( i );
		out.add_contour( contour, hole_flag );
	    }
	}
    }
}


// Attempt to merge slivers into a list of polygons.
//
// For each sliver contour, see if a union with another polygon yields
// a polygon with no increased contours (i.e. the sliver is adjacent
// and can be merged.)  If so, replace the clipped polygon with the
// new polygon that has the sliver merged in.
void FGClipper::merge_slivers( FGPolyList& clipped, FGPolygon& slivers ) {
    FGPolygon poly, result, sliver;
    point_list contour;
    int original_contours, result_contours;
    bool done;
    int area, i, j, k;

    for ( i = 0; i < slivers.contours(); ++i ) {
	// cout << "Merging sliver = " << i << endl;

	// make the sliver polygon
	contour = slivers.get_contour( i );
	sliver.erase();
	sliver.add_contour( contour, 0 );
	done = false;

	for ( area = 0; area < FG_MAX_AREA_TYPES && !done; ++area ) {

	    if ( area == HoleArea ) {
		// don't merge a non-hole sliver in with a hole
		continue;
	    }

	    // cout << "  testing area = " << area << " with " 
	    //      << clipped.polys[area].size() << " polys" << endl;
	    for ( j = 0; 
		  j < (int)clipped.polys[area].size() && !done;
		  ++j )
	    {
		// cout << "  polygon = " << j << endl;

		poly = clipped.polys[area][j];
		original_contours = poly.contours();
		result = polygon_union( poly, sliver );
		result_contours = result.contours();

		if ( original_contours == result_contours ) {
		    // cout << "    FOUND a poly to merge the sliver with" << endl;
		    clipped.polys[area][j] = result;
		    done = true;
		    // poly.write("orig");
		    // sliver.write("sliver");
		    // result.write("result");
		    // cout << "press return: ";
		    // string input;
		    // cin >> input;
		} else {
		    /* cout << "    poly not a match" << endl;
		    cout << "    original = " << original_contours
			 << " result = " << result_contours << endl;
		    cout << "    sliver = " << endl; */
		    for ( k = 0; k < (int)contour.size(); ++k ) {
			// cout << "      " << contour[k].x() << ", "
			//      << contour[k].y() << endl;
		    }
		}
	    }
	}
	if ( !done ) {
	    // cout << "no suitable polys found for sliver merge" << endl;
	}
    }
}


// Clip all the polygons against each other in a priority scheme based
// on order of the polygon type in the polygon type enum.
bool FGClipper::clip_all(const point2d& min, const point2d& max) {
    FGPolygon accum, tmp;
    FGPolygon slivers, remains;
    int i, j;

    // gpcpoly_iterator current, last;

    SG_LOG( SG_CLIPPER, SG_INFO, "Running master clipper" );

    accum.erase();

    cout << "  (" << min.x << "," << min.y << ") (" 
	 << max.x << "," << max.y << ")" << endl;

    // set up clipping tile
    polys_in.safety_base.erase();
    polys_in.safety_base.add_node( 0, Point3D(min.x, min.y, 0.0) );
    polys_in.safety_base.add_node( 0, Point3D(max.x, min.y, 0.0) );
    polys_in.safety_base.add_node( 0, Point3D(max.x, max.y, 0.0) );
    polys_in.safety_base.add_node( 0, Point3D(min.x, max.y, 0.0) );

    // set up land mask, we clip most things to this since it is our
    // best representation of land vs. ocean.  If we have other less
    // accurate data that spills out into the ocean, we want to just
    // clip it.
    FGPolygon land_mask;
    land_mask.erase();
    for ( i = 0; i < (int)polys_in.polys[DefaultArea].size(); ++i ) {
	land_mask =
	  polygon_union( land_mask, polys_in.polys[DefaultArea][i] );
    }

    // set up island mask, for cutting holes in lakes
    FGPolygon island_mask;
    island_mask.erase();
    for ( i = 0; i < (int)polys_in.polys[IslandArea].size(); ++i ) {
	island_mask =
	  polygon_union( island_mask, polys_in.polys[IslandArea][i] );
    }

// no longer needed, should be handle by polygon priority scheme
#if 0
    // set up pond mask, for cutting holes in islands
    FGPolygon pond_mask;
    pond_mask.erase();
    for ( i = 0; i < (int)polys_in.polys[PondArea].size(); ++i ) {
	pond_mask =
	  polygon_union( pond_mask, polys_in.polys[PondArea][i] );
    }
#endif

    // process polygons in priority order
    for ( i = 0; i < FG_MAX_AREA_TYPES; ++i ) {
	cout << "num polys of type (" << i << ") = " 
	     << polys_in.polys[i].size() << endl;
	// current = polys_in.polys[i].begin();
	// last = polys_in.polys[i].end();
	// for ( ; current != last; ++current ) {
	for( j = 0; j < (int)polys_in.polys[i].size(); ++j ) {
	    FGPolygon current = polys_in.polys[i][j];
	    SG_LOG( SG_CLIPPER, SG_DEBUG, get_area_name( (AreaType)i ) 
		    << " = " << current.contours() );

	    // if not a hole, clip the area to the land_mask
	    if ( i != HoleArea ) {
		// clip to land mask
		tmp = polygon_int( current, land_mask );
	    } else {
		tmp = current;
	    }

	    // if a water area, cut out potential islands
	    if ( i == LakeArea || i == IntLakeArea || i == ReservoirArea ||
		 i == IntReservoirArea || i == StreamArea || i == CanalArea ||
		 i == OceanArea ) {
	        // clip against island mask
	        tmp = polygon_diff( tmp, island_mask );
	    }

// no longer needed, should be handle by polygon priority scheme
#if 0
	    // if an island area, cut out potential ponds
	    } else if ( i == IslandArea ) {
	        // clip to pond mask
	        tmp = polygon_diff( tmp, pond_mask );
	    }
#endif

	    FGPolygon result_union, result_diff;

	    if ( accum.contours() == 0 ) {
		result_diff = tmp;
		result_union = tmp;
	    } else {
   		// cout << "DIFF: tmp.num_contours = " << tmp.num_contours
		//      << " accum.num_contours = " << accum.num_contours
		//      << endl;
		// tmp output accum

		// FILE *ofp= fopen("tmp-debug", "w");
		// gpc_write_polygon(ofp, 1, &tmp);
		// fclose(ofp);

		// ofp= fopen("accum-debug", "w");
		// gpc_write_polygon(ofp, 1, &accum);
		// fclose(ofp);

		result_diff = polygon_diff( tmp, accum);
		result_union = polygon_union( tmp, accum);
	    }

	    /*
	      cout << "original contours = " << tmp.num_contours << endl;

	      for ( j = 0; j < tmp.num_contours; j++ ) {
	        for ( k = 0;k < tmp.contour[j].num_vertices;k++ ) {
	          cout << tmp.contour[j].vertex[k].x << ","
	               << tmp.contour[j].vertex[k].y << endl;
	        }
	      }

	      cout << "clipped contours = " << result_diff->num_contours << endl;

	      for ( j = 0; j < result_diff->num_contours; j++ ) {
	        for ( k = 0; k < result_diff->contour[j].num_vertices; k++ ) {
	          cout << result_diff->contour[j].vertex[k].x << ","
	               << result_diff->contour[j].vertex[k].y << endl;
	        }
	      }
	      */

	    // only add to output list if the clip left us with a polygon
	    if ( result_diff.contours() > 0 ) {
		// move slivers from result_diff polygon to slivers polygon
		move_slivers(result_diff, slivers);
		// cout << "  After sliver move:" << endl;
		// cout << "    result_diff = " << result_diff.contours() << endl;
		// cout << "    slivers = " << slivers.contours() << endl;

		// merge any slivers with previously clipped
		// neighboring polygons
		if ( slivers.contours() > 0 ) {
		    merge_slivers(polys_clipped, slivers);
		}

		// add the sliverless result polygon (from after the
		// move_slivers) to the clipped polys list
		if ( result_diff.contours() > 0  ) {
		    polys_clipped.polys[i].push_back(result_diff);
		}

		// char filename[256];
		// sprintf(filename, "next-result-%02d", count++);
		// FILE *tmpfp= fopen(filename, "w");
		// gpc_write_polygon(tmpfp, 1, result_diff);
		// fclose(tmpfp);
	    }
	    accum = result_union;
	}
    }

    // finally, what ever is left over goes to ocean

    // clip to accum against original base tile
    // remains = new gpc_polygon;
    // remains->num_contours = 0;
    // remains->contour = NULL;
    remains = polygon_diff( polys_in.safety_base, accum );

    if ( remains.contours() > 0 ) {
	// cout << "remains contours = " << remains.contours() << endl;
	// move slivers from remains polygon to slivers polygon
	move_slivers(remains, slivers);
	// cout << "  After sliver move:" << endl;
	// cout << "    remains = " << remains.contours() << endl;
	// cout << "    slivers = " << slivers.contours() << endl;

	// merge any slivers with previously clipped
	// neighboring polygons
	if ( slivers.contours() > 0 ) {
	    merge_slivers(polys_clipped, slivers);
	}

	if ( remains.contours() > 0 ) {
	    polys_clipped.polys[(int)OceanArea].push_back(remains);
	}
    }

#if 0
    FILE *ofp;

    // tmp output accum
    if ( accum.num_contours ) {
	ofp = fopen("accum", "w");
	gpc_write_polygon(ofp, 1, &accum);
	fclose(ofp);
    }

    // tmp output safety_base
    if ( remains->num_contours ) {
	ofp= fopen("remains", "w");
	gpc_write_polygon(ofp, 1, remains);
	fclose(ofp);
    }
#endif

    return true;
}


