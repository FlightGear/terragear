// clipper.cxx -- top level routines to take a series of arbitrary areas and
//                produce a tight fitting puzzle pieces that combine to make a
//                tile
//
// Written by Curtis Olson, started February 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: clipper.cxx,v 1.33 2006-11-29 22:19:33 curt Exp $
 

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>

#include <Clipper/priorities.hxx>
#include <Osgb36/osgb36.hxx>

#include "clipper.hxx"

#include STL_IOSTREAM

SG_USING_STD(cout);
SG_USING_STD(endl);


#define MASK_CLIP 1


// Constructor.
TGClipper::TGClipper() {
        nudge=0.0;
}


// Destructor.
TGClipper::~TGClipper() {
}


// Initialize the clipper (empty all the polygon buckets.)
bool TGClipper::init() {
    for ( int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
	polys_in.polys[i].clear();
    }
    fixed_elevations.clear();

    return true;
}


// Load a polygon definition file.
bool TGClipper::load_polys(const string& path) {
    bool poly3d = false;
    string first_line;
    string poly_name;
    AreaType poly_type = DefaultArea;
    int contours, count, i, j;
    int hole_flag;
    double startx, starty, startz, x, y, z, lastx, lasty, lastz;

    SG_LOG( SG_CLIPPER, SG_INFO, "Loading " << path << " ..." );

    sg_gzifstream in( path );

    if ( !in ) {
        SG_LOG( SG_CLIPPER, SG_ALERT, "Cannot open file: " << path );
	exit(-1);
    }

    TGPolygon poly;

    Point3D p;
    // (this could break things, why is it here) in >> skipcomment;
    while ( !in.eof() ) {
        in >> first_line;
        if ( first_line == "#2D" ) {
            poly3d = false;
            in >> poly_name;
        } else if ( first_line == "#3D" ) {
            poly3d = true;
            in >> poly_name;
        } else {
            // support old format (default to 2d)
            poly3d = false;
            poly_name = first_line;
        }
	cout << "poly name = " << poly_name << endl;
	poly_type = get_area_type( poly_name );
	cout << "poly type (int) = " << (int)poly_type << endl;
	in >> contours;
	cout << "num contours = " << contours << endl;

	poly.erase();

	for ( i = 0; i < contours; ++i ) {
	    in >> count;
            // cout << "Contour = " << i << " size = " << count << endl;

	    if ( count < 3 ) {
		SG_LOG( SG_CLIPPER, SG_ALERT, 
			"Polygon with less than 3 data points." );
		exit(-1);
	    }

	    in >> hole_flag;

	    in >> startx;
	    in >> starty;
            if ( poly3d ) {
                in >> startz;
            } else {
                startz = -9999.0;
            }
	    p = Point3D(startx+nudge, starty+nudge, startz);
	    poly.add_node( i, p );
            if ( poly3d ) {
                fixed_elevations.unique_add( p );
            }

	    for ( j = 1; j < count - 1; ++j ) {
		in >> x;
		in >> y;
                if ( poly3d ) {
                    in >> z;
                } else {
                    z = -9999.0;
                }
		p = Point3D( x+nudge, y+nudge, z );
		poly.add_node( i, p );
                if ( poly3d ) {
                    fixed_elevations.unique_add( p );
                }
	    }

	    in >> lastx;
	    in >> lasty;
            if ( poly3d ) {
                in >> lastz;
            } else {
                lastz = -9999.0;
            }

	    if ( (fabs(startx - lastx) < SG_EPSILON) 
		 && (fabs(starty - lasty) < SG_EPSILON)
                 && (fabs(startz - lastz) < SG_EPSILON) ) {
		// last point same as first, discard
	    } else {
		p = Point3D( lastx+nudge, lasty+nudge, lastz );
		poly.add_node( i, p );
                if ( poly3d ) {
                    fixed_elevations.unique_add( p );
                }
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


// Load a polygon definition file containing osgb36 Eastings and Northings
// and convert them to WGS84 Latitude and Longitude
bool TGClipper::load_osgb36_polys(const string& path) {
//   cout << "Loading osgb36 poly\n";
    string poly_name;
    AreaType poly_type = DefaultArea;
    int contours, count, i, j;
    int hole_flag;
    double startx, starty, x, y, lastx, lasty;

    SG_LOG( SG_CLIPPER, SG_INFO, "Loading " << path << " ..." );

    sg_gzifstream in( path );

    if ( !in ) {
        SG_LOG( SG_CLIPPER, SG_ALERT, "Cannot open file: " << path );
	exit(-1);
    }

    // gpc_polygon *poly = new gpc_polygon;
    // poly->num_contours = 0;
    // poly->contour = NULL;
    TGPolygon poly;

    Point3D p;
    Point3D OSRef;
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
	    OSRef = Point3D(startx, starty, -9999.0);
	    
            //Convert from OSGB36 Eastings/Northings to WGS84 Lat/Lon
            //Note that startx and starty themselves must not be altered since we compare them with unaltered lastx and lasty later
	    p = OSGB36ToWGS84(OSRef);
	    
	    poly.add_node( i, p );
	    SG_LOG( SG_CLIPPER, SG_BULK, "0 = "
		    << startx << ", " << starty );

	    for ( j = 1; j < count - 1; ++j ) {
		in >> x;
		in >> y;
		OSRef = Point3D( x, y, -9999.0 );
		p = OSGB36ToWGS84(OSRef);
		
		poly.add_node( i, p );
		SG_LOG( SG_CLIPPER, SG_BULK, j << " = " << x << ", " << y );
	    }

	    in >> lastx;
	    in >> lasty;

	    if ( (fabs(startx - lastx) < SG_EPSILON)
		 && (fabs(starty - lasty) < SG_EPSILON) ) {
		// last point same as first, discard
	    } else {
		OSRef = Point3D( lastx, lasty, -9999.0 );
		p = OSGB36ToWGS84(OSRef);
		
		poly.add_node( i, p );
		SG_LOG( SG_CLIPPER, SG_BULK, count - 1 << " = "
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

    if ( area < TG_MAX_AREA_TYPES ) {
	polys_in.polys[area].push_back(poly);
    } else {
	SG_LOG( SG_CLIPPER, SG_ALERT, "Polygon type out of range = "
		<< (int)poly_type);
	exit(-1);
    }

    // FILE *ofp= fopen("outfile", "w");
    // gpc_write_polygon(ofp, &polys.landuse);

    return true;
}

// Add a polygon to the clipper.
void TGClipper::add_poly( int area, const TGPolygon &poly )
{
    if ( area < TG_MAX_AREA_TYPES ) {
	polys_in.polys[area].push_back(poly);
    } else {
	SG_LOG( SG_CLIPPER, SG_ALERT, "Polygon type out of range = " 
		<< area);
	exit(-1);
    }
}


// Move slivers from in polygon to out polygon.
void TGClipper::move_slivers( TGPolygon& in, TGPolygon& out ) {
    // traverse each contour of the polygon and attempt to identify
    // likely slivers

    // cout << "Begin move slivers" << endl;

    int i;

    out.erase();

    double angle_cutoff = 10.0 * SGD_DEGREES_TO_RADIANS;
    double area_cutoff = 0.00000008;
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
	     << min_angle * 180.0 / SGD_PI << endl;
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
void TGClipper::merge_slivers( TGPolyList& clipped, TGPolygon& slivers ) {
    TGPolygon poly, result, sliver;
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

	for ( area = 0; area < TG_MAX_AREA_TYPES && !done; ++area ) {

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
		result = tgPolygonUnion( poly, sliver );
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


static bool
is_water_area (AreaType type)
{
    switch (type) {
    case PondArea:
    case LakeArea:
    case DryLakeArea:
    case IntLakeArea:
    case ReservoirArea:
    case IntReservoirArea:
    case StreamArea:
    case IntStreamArea:
    case CanalArea:
    case OceanArea:
    case BogArea:
    case MarshArea:
    case LittoralArea:
    case WaterBodyCover:
        return true;
    default:
        return false;
    }
}


// Clip all the polygons against each other in a priority scheme based
// on order of the polygon type in the polygon type enum.
bool TGClipper::clip_all(const point2d& min, const point2d& max) {
    TGPolygon accum, tmp;
    TGPolygon slivers, remains;
    int i, j;

    // gpcpoly_iterator current, last;

    SG_LOG( SG_CLIPPER, SG_INFO, "Running master clipper" );

    accum.erase();

    cout << "  (" << min.x << "," << min.y << ") (" 
	 << max.x << "," << max.y << ")" << endl;

    // set up clipping tile
    polys_in.safety_base.erase();
    polys_in.safety_base.add_node( 0, Point3D(min.x, min.y, -9999.0) );
    polys_in.safety_base.add_node( 0, Point3D(max.x, min.y, -9999.0) );
    polys_in.safety_base.add_node( 0, Point3D(max.x, max.y, -9999.0) );
    polys_in.safety_base.add_node( 0, Point3D(min.x, max.y, -9999.0) );

    // set up land mask, we clip most things to this since it is our
    // best representation of land vs. ocean.  If we have other less
    // accurate data that spills out into the ocean, we want to just
    // clip it.
    TGPolygon land_mask;
    land_mask.erase();
    for ( i = 0; i < (int)polys_in.polys[DefaultArea].size(); ++i ) {
	land_mask =
	  tgPolygonUnion( land_mask, polys_in.polys[DefaultArea][i] );
    }

    // set up a mask for all water.
    TGPolygon water_mask;
    water_mask.erase();
    for ( i = 0; i < TG_MAX_AREA_TYPES; i++ ) {
        if (is_water_area(AreaType(i))) {
            for (unsigned int j = 0; j < polys_in.polys[i].size(); j++) {
                water_mask =
                    tgPolygonUnion( water_mask, polys_in.polys[i][j] );
            }
        }
    }

    // set up island mask, for cutting holes in lakes
    TGPolygon island_mask;
    island_mask.erase();
    for ( i = 0; i < (int)polys_in.polys[IslandArea].size(); ++i ) {
	island_mask =
	  tgPolygonUnion( island_mask, polys_in.polys[IslandArea][i] );
    }

    // process polygons in priority order
    for ( i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
	cout << "num polys of type (" << i << ") = " 
	     << polys_in.polys[i].size() << endl;
	for( j = 0; j < (int)polys_in.polys[i].size(); ++j ) {
	    TGPolygon current = polys_in.polys[i][j];
	    SG_LOG( SG_CLIPPER, SG_INFO, get_area_name( (AreaType)i ) 
		    << " = " << current.contours() );

            tmp = current;

            // if not a hole, clip the area to the land_mask
            if ( i != HoleArea ) {
                tmp = tgPolygonInt( tmp, land_mask );
            }

            // Airport areas are limited to existing land mass and
            // never override water.
	    //
	    // 9/26/2005 - CLO: We are going to add the ability to
	    // manually define airport areas when the default area
	    // isn't sufficient.  It is clear that it is impossible to
	    // auto-generate correct airport areas in all cases.  For
	    // now we default to topologically continuous scenery and
	    // wait for people to submit manual fixes.
	    //
            // if ( i == AirportArea ) {
            //     tmp = tgPolygonInt( tmp, land_mask );
            //     tmp = tgPolygonDiff( tmp, water_mask );
            // }

	    // if a water area, cut out potential islands
	    if ( is_water_area(AreaType(i)) ) {
	        // clip against island mask
	        tmp = tgPolygonDiff( tmp, island_mask );
	    }

	    TGPolygon result_union, result_diff;

	    if ( accum.contours() == 0 ) {
		result_diff = tmp;
		result_union = tmp;
	    } else {
		result_diff = tgPolygonDiff( tmp, accum);
		result_union = tgPolygonUnion( tmp, accum);
	    }

	    // only add to output list if the clip left us with a polygon
	    if ( result_diff.contours() > 0 ) {
		// move slivers from result_diff polygon to slivers polygon
		move_slivers(result_diff, slivers);

		// merge any slivers with previously clipped
		// neighboring polygons
		if ( slivers.contours() > 0 ) {
		    merge_slivers(polys_clipped, slivers);
		}

		// add the sliverless result polygon (from after the
		// move_slivers) to the clipped polys list
		if ( result_diff.contours() > 0  ) {
		    polys_clipped.polys[i].push_back(result_diff);

                    // static int count = 0;
                    // cout << "Writing clipped polygon to next-result" << count
                    //      << endl;
                    // char filename[256];
                    // sprintf(filename, "next-result-%02d-%02d", i, count++);
                    // result_diff.write(filename);
		}
	    }
	    accum = result_union;
	}
    }

    // finally, what ever is left over goes to ocean

    // clip to accum against original base tile
    // remains = new gpc_polygon;
    // remains->num_contours = 0;
    // remains->contour = NULL;
    remains = tgPolygonDiff( polys_in.safety_base, accum );

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

    SG_LOG( SG_CLIPPER, SG_INFO, "  master clipper finished." );

    return true;
}


