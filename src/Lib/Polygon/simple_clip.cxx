// simple_clip.cxx -- simplistic polygon clipping routine.  Returns
//                    the portion of a polygon that is above or below
//                    a horizontal line of y = a.  Only works with
//                    single contour polygons (no holes.)
//
// Written by Curtis Olson, started August 1999.
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: simple_clip.cxx,v 1.7 2004-11-19 22:25:50 curt Exp $


#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/structure/exception.hxx>

#include "simple_clip.hxx"

#define CLIP_EPSILON 0.000000000001


// Given a line segment specified by two endpoints p1 and p2, return
// the x value of a point on the line that intersects with the
// horizontal line through y.  Return true if an intersection is found,
// false otherwise.
static bool intersects_y( Point3D p0, Point3D p1, double y, Point3D *result )
{
    // sort the end points
    if ( p0.y() > p1.y() ) {
        Point3D tmp = p0;
        p0 = p1;
        p1 = tmp;
    }

    if ( (y < p0.y()) || (y > p1.y()) )
        // out of range of line segment, bail right away
        return false;

    // equation of a line through (x0,y0) and (x1,y1):
    //
    //     y = y1 + (x - x1) * (y0 - y1) / (x0 - x1)
    //     x = x1 + (y - y1) * (x0 - x1) / (y0 - y1)

    double x;

    if ( fabs(p0.y() - p1.y()) > CLIP_EPSILON )
        x = p1.x() + (y - p1.y()) * (p0.x() - p1.x()) / (p0.y() - p1.y());
    else
        return false;
    result->setx(x);
    result->sety(y);

    if ( p0.x() <= p1.x() ) {
        if ( (p0.x() <= x) && (x <= p1.x()) )
            return true;
    } else if ( (p0.x() >= x) && (x >= p1.x()) )
        return true;


    return false;
}


// find the smallest point in contour 0 of poly such that x > min_x
// and y = y.  Returns index of the found point, -1 if no match found.
static int find_point( const TGPolygon& poly, double min_x, double y )
{
    Point3D p, save;
    int index = -1;

    save.setx( 361.0 );

    for ( int i = 0; i < poly.contour_size( 0 ); ++i ) {
        p = poly.get_pt( 0, i );
        if ( p.y() == y ) {
            // printf("(%d) p.y() = %.12f  y = %.12f\n", i, p.y(), y);
            // SG_LOG(SG_GENERAL, SG_DEBUG, "  " << p);
            if ( p.x() > min_x ) {
                if ( p.x() < save.x() ) {
                    save = p;
                    index = i;
                }
            }
        }
    }

    return index;
}


// return if interesection is valid (not in the ignore list)
static bool valid_intersection( int intersection, const int_list& ignore_ints )
{
    for ( int i = 0; i < (int)ignore_ints.size(); ++i )
        if ( intersection == ignore_ints[i] )
            return false;
    return true;
}


// return index of next valid intersection
static int next_intersection( const int_list& keep_ints,
                              const int_list& ignore_ints,
                              const int beginning_at )
{
    // SG_LOG(SG_GENERAL, SG_DEBUG, "[ni] start_int = " << beginning_at);
    int i = beginning_at;

    if ( i < 0 ) i = 0; while ( i < (int)keep_ints.size() ) {
        // SG_LOG(SG_GENERAL, SG_DEBUG, "     i = " << i);
        if ( keep_ints[i] != -1 )
            if ( valid_intersection(keep_ints[i], ignore_ints) )
                return i;
        ++i;
    }

    return -1;
}


// return true if point.y() touches or is inside of line, else return false
inline bool is_on_or_inside( double line, Point3D p, fgSideType side )
{
    if ( side == Above ) {
        if ( p.y() >= line )
            return true;
    } else if ( side == Below )
        if ( p.y() <= line )
            return true;

    return false;
}


// return true if point.y() is inside of line, else return false
inline bool is_inside( double line, Point3D p, fgSideType side )
{
    if ( side == Above ) {
        if ( p.y() > line )
            return true;
    } else if ( side == Below )
        if ( p.y() < line )
            return true;

    return false;
}


// Walk through the input polygon and split it into the
// portion that is inside the clip region
static bool simple_clip( const TGPolygon& in, const double y,
                         const fgSideType side,
                         TGPolygon& result )
{
    Point3D p, p_last, p_int;
    int i;

    result.erase();

    SG_LOG(SG_GENERAL, SG_DEBUG, "input poly size = " << in.total_size());

    p_last = in.get_pt( 0, in.contour_size(0) - 1 );

    for ( i = 0; i < (int)in.contour_size(0); ++i ) {
        p = in.get_pt( 0, i );

        if ( (fabs(p.x() - p_last.x()) < CLIP_EPSILON) &&
             (fabs(p.y() - p_last.y()) < CLIP_EPSILON) &&
             (i > 0) ) {
            // SG_LOG(SG_GENERAL, SG_WARN,
            //        "WARNING: p and p_last are identical at index = " << i);
        }

        if ( is_on_or_inside(y, p, side) ) {
            if ( is_on_or_inside(y, p_last, side) )
                // SG_LOG(SG_GENERAL, SG_DEBUG, "inside & inside " << i << " " << p);
                result.add_node( 0, p );
            else {
                if ( !intersects_y(p, p_last, y, &p_int) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Huh, this should have intersected!");
                    return false;
                } else {
                    // SG_LOG(SG_GENERAL, SG_DEBUG,
                    //        "intersection outside to inside " << i << " "
                    //        << p_int "  i - 1 = " << in.get_pt( 0, i-1 ));
                    // SG_LOG(SG_GENERAL, SG_DEBUG, "  i     = " << in.get_pt( 0, i ));
                    // SG_LOG(SG_GENERAL, SG_DEBUG, "  i + 1 = " << in.get_pt( 0, i+1 ));
                    result.add_node( 0, p_int );
                    if ( (fabs(p.x() - p_int.x()) < CLIP_EPSILON) &&
                         (fabs(p.y() - p_int.y()) < CLIP_EPSILON) ) {
                        // SG_LOG(SG_GENERAL, SG_DEBUG,
                        //        "WARNING: p and p_int are identical, "
                        //        << "omitting p");
                    } else {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "adding intersection" << i << " " << p);
                        result.add_node( 0, p );
                    }
                }
            }
        } else {
            if ( is_inside(y, p_last, side) ) {
                if ( !intersects_y(p, p_last, y, &p_int) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Huh, this should have intersected!");
                    return false;
                } else {
                    // SG_LOG(SG_GENERAL, SG_DEBUG,
                    //        "intersection inside to outside " << i << " "
                    //        << p_int);
                    if ( (fabs(p.x() - p_int.x()) < CLIP_EPSILON) &&
                         (fabs(p.y() - p_int.y()) < CLIP_EPSILON) ) {
                        SG_LOG(SG_GENERAL, SG_DEBUG,
                               "WARNING: p and p_int are identical, "
                               << "omitting p");
                    } else
                        result.add_node( 0, p_int );
                }
            }
        }

        p_last = p;
    }

    return true;
}


// build the list of intersections
static bool build_intersections( const TGPolygon& arcs, double line,
                                 fgSideType side,
                                 int_list& keep_ints,
                                 int_list& ignore_ints )
{
    keep_ints.clear();
    ignore_ints.clear();

    int index = 0;
    double current_x = -181.0;

    while ( index >= 0 ) {
        index = find_point( arcs, current_x, line );
        if ( index >= 0 ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "intersection at " << index << " = "
                                                            << arcs.get_pt( 0, index ));
            keep_ints.push_back( index );
            current_x = arcs.get_pt( 0, index ).x();

            int before = index - 1;
            if ( before < 0 ) before += arcs.contour_size(0); int after = (index + 1) % arcs.contour_size(0);
            SG_LOG(SG_GENERAL, SG_DEBUG, "  before = "
                   << arcs.get_pt(0, before));
            SG_LOG(SG_GENERAL, SG_DEBUG, "  int    = "
                   << arcs.get_pt(0, index));
            SG_LOG(SG_GENERAL, SG_DEBUG, "  after  = "
                   << arcs.get_pt(0, after));
            if ( side == Above ) {
                if ( (arcs.get_pt(0, before).y() > line) &&
                     (arcs.get_pt(0, after).y() > line) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "side = above");
                    SG_LOG(SG_GENERAL, SG_DEBUG,
                           "V intersection with clip line from above");
                    SG_LOG(SG_GENERAL, SG_DEBUG,
                           "Adding intersection to ignore_ints");
                    ignore_ints.push_back( index );
                }
                if ( (arcs.get_pt(0, before).y() <= line) &&
                     (arcs.get_pt(0, after).y() <= line) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "side = above");
                    SG_LOG(SG_GENERAL, SG_DEBUG,
                           "V intersection with clip line from BELOW\n"
                            << "or an extra in-clip-line intersection.\n"
                            << "Adding intersection to ignore_ints");
                    ignore_ints.push_back( index );
                }
            } else if ( side == Below ) {
                if ( (arcs.get_pt(0, before).y() >= line) &&
                     (arcs.get_pt(0, after).y() >= line) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "side = below");
                    SG_LOG(SG_GENERAL, SG_DEBUG, "V intersection with clip line from above");
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Adding intersection to ignore_ints");
                    ignore_ints.push_back( index );
                }
                if ( (arcs.get_pt(0, before).y() < line) &&
                     (arcs.get_pt(0, after).y() < line) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "side = below");
                    SG_LOG(SG_GENERAL, SG_DEBUG, "V intersection with clip line from BELOW");
                    SG_LOG(SG_GENERAL, SG_DEBUG, "or an extra in-clip-line intersection.");
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Adding intersection to ignore_ints");
                    ignore_ints.push_back( index );
                }
            }
        }
    }

    return true;
}


// test for duplicate nodes
TGPolygon fix_dups( TGPolygon& in )
{
    TGPolygon result;

    double x_last = -20000.0;
    double y_last = -20000.0;
    double x, y;

    for ( int i = 0; i < (int)in.contour_size(0); ++i ) {
        x = in.get_pt(0, i).x();
        y = in.get_pt(0, i).y();
        if ( (x == x_last) && (y == y_last) ) {
            // ignore
        } else
            result.add_node(0, in.get_pt(0, i));
        x_last = x;
        y_last = y;
    }

    return result;
}


// simple polygon clipping routine.  Returns the portion of a polygon
// that is above and below a horizontal line of y = a.  Only works
// with single contour polygons (no holes.)  Returns true if routine
// thinks it was successful.

static bool clip_contour( const TGPolygon& in, const double y,
                          const fgSideType side,
                          TGPolygon& result )
{
    TGPolygon result_arcs, arcs;
    int i, i1, i2, index;


    // Step 1: sanity checks
    if ( (int)in.contours() != 1 ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "we only handle single contour polygons");
        return false;
    }

    if ( (int)in.contour_size( 0 ) < 3 ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "we must have at least three vertices to work");
        return false;
    }


    // Step 2: walk through the input polygon and split it into the
    // portion that is on or inside the clip line

    if ( simple_clip( in, y, side, result_arcs ) ) {
        if ( result_arcs.contours() > 0 )
            SG_LOG(SG_GENERAL, SG_DEBUG, "result_arcs size = "
                   << result_arcs.total_size());
        else
            SG_LOG(SG_GENERAL, SG_DEBUG, "empty result");
    } else
        throw sg_exception("simple_clip_above() failed!");


    // Step 3: check for trivial cases

    result.erase();

    // trivial -- nothing inside of clip line
    if ( result_arcs.contours() == 0 ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "trivially empty");
        return true;
    }

    // trivial -- everything inside of clip line
    i1 = find_point( result_arcs, -181.0, y );
    if ( i1 < 0 ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "trivially full");
        result = result_arcs;
        return true;
    }

    // trivial -- single clip line intersection (polygon just nicks
    // it) -- everything inside
    i2 = find_point( result_arcs, result_arcs.get_pt(0, i1).x(), y );
    if ( i2 < 0 ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "trivially full (clip line nicks edge)");
        result = result_arcs;
        return true;
    }


    // Step 4: If we are finding the "below" clip, reverse the points
    // before extracting the cycles.  (and remove duplicates)

    TGPolygon tmp;
    tmp.erase();

    if ( side == Below ) {
        for ( i = result_arcs.contour_size(0) - 1; i >= 0; --i ) {
            Point3D p = result_arcs.get_pt( 0, i );
            tmp.add_node( 0, p );
        }
    } else
        tmp = result_arcs;

    arcs = fix_dups( tmp );

    // Step 5: Build the intersections lists

    int_list keep_ints, ignore_ints;
    build_intersections( arcs, y, side, keep_ints, ignore_ints );
    SG_LOG(SG_GENERAL, SG_DEBUG, "total keep_ints = " << keep_ints.size());
    SG_LOG(SG_GENERAL, SG_DEBUG, "total ignore_ints = " << ignore_ints.size());


    // Step 6: Walk through the result_arcs and extract the cycles (or
    // individual contours.)

    int start_int = next_intersection( keep_ints, ignore_ints, 0 );
    int next_int = next_intersection( keep_ints, ignore_ints, start_int + 1 );
    SG_LOG(SG_GENERAL, SG_DEBUG, "start_int = " << start_int);
    SG_LOG(SG_GENERAL, SG_DEBUG, "next_int = " << next_int);

    int count = 0;

    // while we have keep_ints left to process
    while ( start_int >= 0 ) {
        point_list contour;
        contour.clear();

        index = keep_ints[next_int];
        keep_ints[next_int] = -1;
        SG_LOG(SG_GENERAL, SG_DEBUG, "\nstarting at point = "
               << arcs.get_pt(0, index));

        while ( index != keep_ints[start_int] ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "index = " << index
                                                    << " start_int = " << start_int
                                                    << " keep_ints[start_int] = " << keep_ints[start_int]);

            // start with the 2nd item in the intersection list and
            // traverse until we find another intersection
            contour.push_back( arcs.get_pt(0, index) );
            index = (index + 1) % arcs.contour_size(0);

            while ( (arcs.get_pt(0, index).y() != y) ||
                    !valid_intersection(index, ignore_ints) ) {
                contour.push_back( arcs.get_pt(0, index) );
                index = (index + 1) % arcs.contour_size(0);
            }
            contour.push_back( arcs.get_pt(0, index) );
            SG_LOG(SG_GENERAL, SG_WARN, "exited at poly index = "
                   << index << " " << arcs.get_pt(0, index));

            // find which intersection we came out on in our list
            SG_LOG(SG_GENERAL, SG_DEBUG, "finding exit intersection for " << index);
            i = 0;
            while ( i < (int)keep_ints.size() ) {
                // SG_LOG(SG_GENERAL, SG_DEBUG, " keep_int[" << i << "] = " << keep_ints[i]);
                if ( index == keep_ints[i] ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  intersection index = " << i);
                    if ( index != keep_ints[start_int] ) {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "  not start index so keep going");
                        keep_ints[i] = -1;
                        next_int = next_intersection( keep_ints, ignore_ints,
                                                      i + 1 );
                        index = keep_ints[next_int];
                        keep_ints[next_int] = -1;
                        SG_LOG(SG_GENERAL, SG_DEBUG,
                               "   next_int = " << next_int << " index = "
                                                << index);
                    }
                    break;
                }
                ++i;
            }
            if ( i == (int)keep_ints.size() )
                throw sg_exception("oops, didn't find that intersection, you are screwed");
        }
        keep_ints[start_int] = -1;
        result.add_contour( contour, count );
        count++;

        // find next keep_ints
        start_int = next_intersection( keep_ints, ignore_ints, -1 );
        next_int = next_intersection( keep_ints, ignore_ints, start_int + 1 );

        SG_LOG(SG_GENERAL, SG_DEBUG, "start_int = " << start_int);
        SG_LOG(SG_GENERAL, SG_DEBUG, "next_int = " << next_int);
    }

    return true;
}


// simple polygon clipping routine.  Returns the portion of a polygon
// that is above and below a horizontal line of y = a.  Clips
// multi-contour polygons individually and then reassembles the
// results.  Doesn't work with holes.  Returns true if routine thinks
// it was successful.

TGPolygon horizontal_clip( const TGPolygon& in, const double y,
                           const fgSideType side )
{
    TGPolygon result;

    result.erase();

    // Step 1: sanity checks
    if ( (int)in.contours() == 0 ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Error: 0 contour polygon");
        return result;
    }

    // clip each contour individually
    TGPolygon tmp_poly, clip_poly;
    point_list contour;

    for ( int i = 0; i < in.contours(); ++i ) {
        if ( (int)in.contour_size( i ) < 3 ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "we must have at least three vertices to work");
            return result;
        }

        tmp_poly.erase();

        contour = in.get_contour( i );
        tmp_poly.add_contour( contour, 0 );

        clip_contour( tmp_poly, y, side, clip_poly );

        // add each clip_poly contour to the result poly
        for ( int j = 0; j < clip_poly.contours(); ++j ) {
            contour = clip_poly.get_contour( j );
            result.add_contour( contour, 0 );
        }
    }

    return result;
}
