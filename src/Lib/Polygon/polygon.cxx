// polygon.cxx -- polygon (with holes) management class
//
// Written by Curtis Olson, started March 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#include <iostream>
#include <fstream>
#include <limits.h>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#include <simgear/constants.h>
#include <simgear/threads/SGThread.hxx>
#include <simgear/threads/SGGuard.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/io/lowlevel.hxx>
#include <simgear/misc/texcoord.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>

#include "polygon.hxx"

#ifdef _MSC_VER
#   define LONG_LONG_MAX LLONG_MAX
#   define LONG_LONG_MIN LLONG_MIN
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NEW IMPLEMENTATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangle_2.h>
#include <iostream>



// TODO : Where does this belong
// Calculate theta of angle (a, b, c)
double tgPolygonCalcAngle(SGVec2d a, SGVec2d b, SGVec2d c) {
    SGVec2d u, v;
    double udist, vdist, uv_dot;

    // u . v = ||u|| * ||v|| * cos(theta)

    u = b-a;
    udist = dist(a,b);

    v = b-c;
    vdist = dist(b,c);

    uv_dot = dot(u,v);

    return acos(uv_dot / (udist * vdist));
}





// EXACT ( polygon list from contour )
typedef CGAL::Exact_predicates_exact_constructions_kernel         Kernel;
typedef Kernel::Point_2                                           Point_2;
typedef CGAL::Segment_2<Kernel>                                   Segment_2;


// INEXACT ( triangulate )

/* determining if a face is within the reulting poly */
struct FaceInfo2
{
  FaceInfo2() {}
  int nesting_level;

  bool in_domain(){
    return nesting_level%2 == 1;
  }
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Point                                                Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;
typedef CGAL::Triangle_2<K>                                       Triangle_2;


// GEOD HELPERS - maybe some math???

const double isEqual2D_Epsilon = 0.000001;

static SGGeod SGGeod_snap( const SGGeod& in, double grid )
{
    return SGGeod::fromDegM( grid * SGMisc<double>::round( in.getLongitudeDeg()/grid ),
                             grid * SGMisc<double>::round( in.getLatitudeDeg() /grid ),
                             grid * SGMisc<double>::round( in.getElevationM()  /grid ) );
}

static bool SGGeod_isEqual2D( const SGGeod& g0, const SGGeod& g1 )
{
    return ( (fabs( g0.getLongitudeDeg() - g1.getLongitudeDeg() ) < isEqual2D_Epsilon) &&
             (fabs( g0.getLatitudeDeg()  - g1.getLatitudeDeg() )  < isEqual2D_Epsilon ) );
}

static SGVec2d SGGeod_ToSGVec2d( const SGGeod& p )
{
    return SGVec2d( p.getLongitudeDeg(), p.getLatitudeDeg() );
}

double SGGeod_CalculateTheta( const SGGeod& p0, const SGGeod& p1, const SGGeod& p2 )
{
    SGVec2d v0, v1, v2;
    SGVec2d u, v;
    double  udist, vdist, uv_dot;

    v0 = SGGeod_ToSGVec2d( p0 );
    v1 = SGGeod_ToSGVec2d( p1 );
    v2 = SGGeod_ToSGVec2d( p2 );

    u  = v1 - v0;
    udist = norm(u);

    v = v1 - v2;
    vdist = norm(v);

    uv_dot = dot(u, v);

    return acos( uv_dot / (udist * vdist) );
}

bool FindIntermediateNode( const SGGeod& start, const SGGeod& end,
                           const std::vector<SGGeod>& nodes, SGGeod& result,
                           double bbEpsilon, double errEpsilon )
{
    bool found_node = false;
    double m, m1, b, b1, y_err, x_err, y_err_min, x_err_min;

    SGGeod p0 = start;
    SGGeod p1 = end;

    double xdist = fabs(p0.getLongitudeDeg() - p1.getLongitudeDeg());
    double ydist = fabs(p0.getLatitudeDeg()  - p1.getLatitudeDeg());

    x_err_min = xdist + 1.0;
    y_err_min = ydist + 1.0;

    if ( xdist > ydist ) {
        // sort these in a sensible order
        SGGeod p_min, p_max;
        if ( p0.getLongitudeDeg() < p1.getLongitudeDeg() ) {
            p_min = p0;
            p_max = p1;
        } else {
            p_min = p1;
            p_max = p0;
        }

        m = (p_min.getLatitudeDeg() - p_max.getLatitudeDeg()) / (p_min.getLongitudeDeg() - p_max.getLongitudeDeg());
        b = p_max.getLatitudeDeg() - m * p_max.getLongitudeDeg();

        for ( int i = 0; i < (int)nodes.size(); ++i ) {
            // cout << i << endl;
            SGGeod current = nodes[i];

            if ( (current.getLongitudeDeg() > (p_min.getLongitudeDeg() + (bbEpsilon))) && (current.getLongitudeDeg() < (p_max.getLongitudeDeg() - (bbEpsilon))) ) {
                y_err = fabs(current.getLatitudeDeg() - (m * current.getLongitudeDeg() + b));

                if ( y_err < errEpsilon ) {
                    found_node = true;
                    if ( y_err < y_err_min ) {
                        result = current;
                        y_err_min = y_err;
                    }
                }
            }
        }
    } else {
        // sort these in a sensible order
        SGGeod p_min, p_max;
        if ( p0.getLatitudeDeg() < p1.getLatitudeDeg() ) {
            p_min = p0;
            p_max = p1;
        } else {
            p_min = p1;
            p_max = p0;
        }

        m1 = (p_min.getLongitudeDeg() - p_max.getLongitudeDeg()) / (p_min.getLatitudeDeg() - p_max.getLatitudeDeg());
        b1 = p_max.getLongitudeDeg() - m1 * p_max.getLatitudeDeg();

        for ( int i = 0; i < (int)nodes.size(); ++i ) {
            SGGeod current = nodes[i];

            if ( (current.getLatitudeDeg() > (p_min.getLatitudeDeg() + (bbEpsilon))) && (current.getLatitudeDeg() < (p_max.getLatitudeDeg() - (bbEpsilon))) ) {

                x_err = fabs(current.getLongitudeDeg() - (m1 * current.getLatitudeDeg() + b1));

                if ( x_err < errEpsilon ) {
                    found_node = true;
                    if ( x_err < x_err_min ) {
                        result = current;
                        x_err_min = x_err;
                    }
                }
            }
        }
    }

    return found_node;
}

void AddIntermediateNodes( const SGGeod& p0, const SGGeod& p1, std::vector<SGGeod>& nodes, tgContour& result, double bbEpsilon, double errEpsilon )
{
    SGGeod new_pt;

    SG_LOG(SG_GENERAL, SG_BULK, "   " << p0 << " <==> " << p1 );

    bool found_extra = FindIntermediateNode( p0, p1, nodes, new_pt, bbEpsilon, errEpsilon );

    if ( found_extra ) {
        AddIntermediateNodes( p0, new_pt, nodes, result, bbEpsilon, errEpsilon  );

        result.AddNode( new_pt );
        SG_LOG(SG_GENERAL, SG_BULK, "    adding = " << new_pt);

        AddIntermediateNodes( new_pt, p1, nodes, result, bbEpsilon, errEpsilon  );
    }
}

#define CLIPPER_FIXEDPT (10000000000000000)
#define CLIPPER_FIXED1M (            90090)

static ClipperLib::IntPoint SGGeod_ToClipper( const SGGeod& p )
{
    ClipperLib::long64 x, y;

    x = (ClipperLib::long64)( p.getLongitudeDeg() * CLIPPER_FIXEDPT );
    y = (ClipperLib::long64)( p.getLatitudeDeg()  * CLIPPER_FIXEDPT );

    return ClipperLib::IntPoint( x, y );
}

static SGGeod SGGeod_FromClipper( const ClipperLib::IntPoint& p )
{
    double lon, lat;

    lon = (double)( ((double)p.X) / (double)CLIPPER_FIXEDPT );
    lat = (double)( ((double)p.Y) / (double)CLIPPER_FIXEDPT );

    return SGGeod::fromDeg( lon, lat );
}

static double Dist_ToClipper( double dist )
{
    return ( dist * ( CLIPPER_FIXEDPT / CLIPPER_FIXED1M ) );
}

static tgRectangle BoundingBox_FromClipper( const ClipperLib::Polygons& subject )
{
    ClipperLib::IntPoint min_pt, max_pt;
    SGGeod min, max;

    min_pt.X = min_pt.Y = LONG_LONG_MAX;
    max_pt.X = max_pt.Y = LONG_LONG_MIN;

    // for each polygon, we need to check the orientation, to set the hole flag...
    for (unsigned int i=0; i<subject.size(); i++)
    {
        for (unsigned int j = 0; j < subject[i].size(); j++)
        {
            if ( subject[i][j].X < min_pt.X ) {
                min_pt.X = subject[i][j].X;
            }
            if ( subject[i][j].Y < min_pt.Y ) {
                min_pt.Y = subject[i][j].Y;
            }

            if ( subject[i][j].X > max_pt.X ) {
                max_pt.X = subject[i][j].X;
            }
            if ( subject[i][j].Y > max_pt.Y ) {
                max_pt.Y = subject[i][j].Y;
            }
        }
    }

    min = SGGeod_FromClipper( min_pt );
    max = SGGeod_FromClipper( max_pt );

    return tgRectangle( min, max );
}


// tgContour static functions

tgContour tgContour::Snap( const tgContour& subject, double snap )
{
    tgContour result;
    SGGeod    pt;

    for (unsigned int i = 0; i < subject.GetSize(); i++) {
        pt = SGGeod_snap( subject.GetNode(i), snap );
        result.AddNode(pt);
    }
    result.SetHole( subject.GetHole() );

    return result;
}

double tgContour::GetMinimumAngle( void ) const
{
    unsigned int p1_index, p2_index, p3_index;
    SGVec2d      p1, p2, p3;
    double       angle;
    double       min_angle = 2.0 * SGD_PI;
    unsigned int size = node_list.size();

    SG_LOG(SG_GENERAL, SG_DEBUG, "  tgContour::GetMinimumAngle() : contour size is " << size );

    for ( unsigned int i = 0; i < size; i++ ) {
        if ( i == 0) {
            p1_index = size -1;
        } else {
            p1_index = i - 1;
        }

        p2_index = i;

        if ( i == size ) {
            p3_index = 0;
        } else {
            p3_index = i + 1;
        }

        SG_LOG(SG_GENERAL, SG_DEBUG, "  tgContour::GetMinimumAngle() iteration " << i << " of " << size << " p1 " << p1_index << " p2 " << p2_index << " p3 " << p3_index );

        p1 = SGGeod_ToSGVec2d( node_list[p1_index] );
        p2 = SGGeod_ToSGVec2d( node_list[p2_index] );
        p3 = SGGeod_ToSGVec2d( node_list[p3_index] );

        SG_LOG(SG_GENERAL, SG_DEBUG, "  tgContour::GetMinimumAngle() iteration " << i << " of " << size << " p1 " << p1 << " p2 " << p2 << " p3 " << p3 );

        angle = tgPolygonCalcAngle( p1, p2, p3 );

        SG_LOG(SG_GENERAL, SG_DEBUG, "  tgContour::GetMinimumAngle() iteration " << i << " of " << size << "angle is " << angle );

        if ( angle < min_angle ) {
            min_angle = angle;
        }
    }

    return min_angle;
}

double tgContour::GetArea( void ) const
{
    double area = 0.0;
    SGVec2d a, b;
    unsigned int i, j;

    if ( node_list.size() ) {
        j = node_list.size() - 1;
        for (i=0; i<node_list.size(); i++) {
            a = SGGeod_ToSGVec2d( node_list[i] );
            b = SGGeod_ToSGVec2d( node_list[j] );

            area += (b.x() + a.x()) * (b.y() - a.y());
            j=i;
        }
    }

    return fabs(area * 0.5);
}

tgContour tgContour::RemoveCycles( const tgContour& subject )
{
    tgContour result;
    bool      found;
    int       iters = 0;

    for ( unsigned int i = 0; i < subject.GetSize(); i++ )
    {
        result.AddNode( subject.GetNode(i) );
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "remove small cycles : original contour has " << result.GetSize() << " points" );

    do
    {
        found = false;

        // Step 1 - find a duplicate point
        for ( unsigned int i = 0; i < result.GetSize() && !found; i++ ) {
            // first check until the end of the vector
            for ( unsigned int j = i + 1; j < result.GetSize() && !found; j++ ) {
                if ( SGGeod_isEqual2D( result.GetNode(i), result.GetNode(j) ) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "detected a dupe: i = " << i << " j = " << j );

                    // We found a dupe - calculate the distance between them
                    if ( i + 4 > j ) {
                        // it's within target distance - remove the points in between, and start again
                        if ( j-i == 1 ) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "detected a small cycle: i = " << i << " j = " << j << " Erasing " << i );
                            result.RemoveNodeAt( i );
                        } else {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "detected a small cycle: i = " << i << " j = " << j << " Erasing from " << i << " to " << j-1 );
                            result.RemoveNodeRange( i, j-1 );
                        }
                        found = true;
                    }
                }
            }

            // then check from beginning to the first point (wrap around)
            for ( unsigned int j = 0; j < i && !found; j++ ) {
                if ( (i != j) && ( SGGeod_isEqual2D( result.GetNode(i), result.GetNode(j) ) ) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "detected a dupe: i = " << i << " j = " << j );

                    // We found a dupe - calculate the distance between them
                    if ( (result.GetSize() - i + j) < 4 ) {
                        // it's within target distance - remove from the end point to the end of the vector
                        if ( i == result.GetSize() - 1 ) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "detected a small cycle: i = " << i << " j = " << j << " Erasing " << result.GetSize()-1 );
                            result.RemoveNodeAt( result.GetSize()-1 );
                        } else {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "detected a small cycle: i = " << i << " j = " << j << " Erasing from " << i << " to " << result.GetSize()-1 );
                            result.RemoveNodeRange( i, result.GetSize()-1 );
                        }

                        // then remove from the beginning of the vector to the beginning point
                        if ( j == 1 ) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "detected a small cycle: i = " << i << " j = " << j << " Erasing " << j-1 );
                            result.RemoveNodeAt( 0 );
                        } else if ( j > 1) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "detected a small cycle: i = " << i << " j = " << j << " Erasing from 0 " <<  " to " <<  j-1 );
                            result.RemoveNodeRange( 0, j-1 );
                        }

                        found = true;
                    }
                }
            }
        }

        iters++;
        SG_LOG(SG_GENERAL, SG_DEBUG, "remove small cycles : after " << iters << " iterations, contour has " << result.GetSize() << " points" );
    } while( found );

    return result;
}

tgContour tgContour::RemoveDups( const tgContour& subject )
{
    tgContour result;

    int  iters = 0;
    bool found;

    for ( unsigned int i = 0; i < subject.GetSize(); i++ )
    {
        result.AddNode( subject.GetNode( i ) );
    }
    result.SetHole( subject.GetHole() );

    SG_LOG( SG_GENERAL, SG_DEBUG, "remove contour dups : original contour has " << result.GetSize() << " points" );

    do
    {
        SG_LOG( SG_GENERAL, SG_DEBUG, "remove_contour_dups: start new iteration" );
        found = false;

        // Step 1 - find a neighboring duplicate points
        SGGeod       cur, next;
        unsigned int cur_loc, next_loc;

        for ( unsigned int i = 0; i < result.GetSize() && !found; i++ ) {
            if (i == result.GetSize() - 1 ) {
                cur_loc = i;
                cur  = result.GetNode(i);

                next_loc = 0;
                next = result.GetNode(0);

                SG_LOG( SG_GENERAL, SG_DEBUG, " cur is last point: " << cur << " next is first point: " << next );
            } else {
                cur_loc = i;
                cur  = result.GetNode(i);

                next_loc = i+1;
                next = result.GetNode(i+1);
                SG_LOG( SG_GENERAL, SG_DEBUG, " cur is: " << cur << " next is : " << next );
            }

            if ( SGGeod_isEqual2D( cur, next ) ) {
                // keep the point with higher Z
                if ( cur.getElevationM() < next.getElevationM() ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "remove_contour_dups: erasing " << cur );
                    result.RemoveNodeAt( cur_loc );
                    // result.RemoveNodeAt( result.begin()+cur );
                } else {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "remove_contour_dups: erasing " << next );
                    result.RemoveNodeAt( next_loc );
                }
                found = true;
            }
        }

        iters++;
        SG_LOG(SG_GENERAL, SG_DEBUG, "remove_contour_dups : after " << iters << " iterations, contour has " << result.GetSize() << " points" );
    } while( found );

    return result;

}

tgContour tgContour::SplitLongEdges( const tgContour& subject, double max_len )
{
    SGGeod      p0, p1;
    double      dist;
    tgContour   result;

    for ( unsigned i = 0; i < subject.GetSize() - 1; i++ ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "point = " << i);

        p0 = subject.GetNode( i );
        p1 = subject.GetNode( i + 1 );

        SG_LOG(SG_GENERAL, SG_DEBUG, " " << p0 << "  -  " << p1);

        if ( fabs(p0.getLatitudeDeg()) < (90.0 - SG_EPSILON) ||
             fabs(p1.getLatitudeDeg()) < (90.0 - SG_EPSILON) )
        {
            dist = SGGeodesy::distanceM( p0, p1 );
            SG_LOG(SG_GENERAL, SG_DEBUG, "distance = " << dist);

            if ( dist > max_len ) {
                unsigned int segments = (int)(dist / max_len) + 1;
                SG_LOG(SG_GENERAL, SG_DEBUG, "segments = " << segments);

                double dx = (p1.getLongitudeDeg() - p0.getLongitudeDeg()) / segments;
                double dy = (p1.getLatitudeDeg()  - p0.getLatitudeDeg())  / segments;

                for ( unsigned int j = 0; j < segments; j++ ) {
                    SGGeod tmp = SGGeod::fromDeg( p0.getLongitudeDeg() + dx * j, p0.getLatitudeDeg() + dy * j );
                    SG_LOG(SG_GENERAL, SG_DEBUG, tmp);
                    result.AddNode( tmp );
                }
            } else {
                SG_LOG(SG_GENERAL, SG_DEBUG, p0);
                result.AddNode( p0 );
            }
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, p0);
            result.AddNode( p0 );
        }

        // end of segment is beginning of next segment
    }

    p0 = subject.GetNode( subject.GetSize() - 1 );
    p1 = subject.GetNode( 0 );

    dist = SGGeodesy::distanceM( p0, p1 );
    SG_LOG(SG_GENERAL, SG_DEBUG, "distance = " << dist);

    if ( dist > max_len ) {
        unsigned int segments = (int)(dist / max_len) + 1;
        SG_LOG(SG_GENERAL, SG_DEBUG, "segments = " << segments);

        double dx = (p1.getLongitudeDeg() - p0.getLongitudeDeg()) / segments;
        double dy = (p1.getLatitudeDeg()  - p0.getLatitudeDeg())  / segments;

        for ( unsigned int i = 0; i < segments; i++ ) {
            SGGeod tmp = SGGeod::fromDeg( p0.getLongitudeDeg() + dx * i, p0.getLatitudeDeg() + dy * i );
            SG_LOG(SG_GENERAL, SG_DEBUG, tmp);
            result.AddNode( tmp );
        }
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, p0);
        result.AddNode( p0 );
    }

    // maintain original hole flag setting
    result.SetHole( subject.GetHole() );

    SG_LOG(SG_GENERAL, SG_DEBUG, "split_long_edges() complete");

    return result;
}

tgContour tgContour::RemoveSpikes( const tgContour& subject )
{
    tgContour result;
    int       iters = 0;
    double    theta;
    bool      found;
    SGGeod    cur, prev, next;

    for ( unsigned int i = 0; i < subject.GetSize(); i++ )
    {
        result.AddNode( subject.GetNode( i ) );
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "remove contour spikes : original contour has " << result.GetSize() << " points" );

    do
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "remove_contour_spikes: start new iteration");
        found = false;

        // Step 1 - find a duplicate point
        for ( unsigned int i = 0; i < result.GetSize() && !found; i++ ) {
            if (i == 0) {
                SG_LOG(SG_GENERAL, SG_DEBUG, " cur is first point: " << i << ": " << result.GetNode(i) );
                cur  =  result.GetNode( 0 );
                prev =  result.GetNode( result.GetSize()-1 );
                next =  result.GetNode( 1 );
            } else if ( i == result.GetSize()-1 ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, " cur is last point: " << i << ": " << result.GetNode(i) );
                cur  =  result.GetNode( i );
                prev =  result.GetNode( i-1 );
                next =  result.GetNode( 0 );
            } else {
                SG_LOG(SG_GENERAL, SG_DEBUG, " cur is: " << i << ": " << result.GetNode(i) );
                cur  =  result.GetNode( i );
                prev =  result.GetNode( i-1 );
                next =  result.GetNode( i+1 );
            }

            theta = SGMiscd::rad2deg( SGGeod_CalculateTheta(prev, cur, next) );

            if ( abs(theta) < 0.1 ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "remove_contour_spikes: (theta is " << theta << ") erasing " << i << " prev is " << prev << " cur is " << cur << " next is " << next );
                result.RemoveNodeAt( i );
                found = true;
            }
        }

        iters++;
        SG_LOG(SG_GENERAL, SG_DEBUG, "remove_contour_spikes : after " << iters << " iterations, contour has " << result.GetSize() << " points" );
    } while( found );

    return result;
}

ClipperLib::Polygon tgContour::ToClipper( const tgContour& subject )
{
    ClipperLib::Polygon  contour;

    for ( unsigned int i=0; i<subject.GetSize(); i++)
    {
        SGGeod p = subject.GetNode( i );
        contour.push_back( SGGeod_ToClipper(p) );
    }

    if ( subject.GetHole() )
    {
        // holes need to be orientation: false
        if ( Orientation( contour ) ) {
            //SG_LOG(SG_GENERAL, SG_INFO, "Building clipper contour - hole contour needs to be reversed" );
            ReversePolygon( contour );
        }
    } else {
        // boundaries need to be orientation: true
        if ( !Orientation( contour ) ) {
            //SG_LOG(SG_GENERAL, SG_INFO, "Building clipper contour - boundary contour needs to be reversed" );
            ReversePolygon( contour );
        }
    }

    return contour;
}

tgContour tgContour::FromClipper( const ClipperLib::Polygon& subject )
{
    tgContour result;

    for (unsigned int i = 0; i < subject.size(); i++)
    {
        ClipperLib::IntPoint ip = ClipperLib::IntPoint( subject[i].X, subject[i].Y );
        //SG_LOG(SG_GENERAL, SG_INFO, "Building tgContour : Add point (" << ip.X << "," << ip.Y << ") );
        result.AddNode( SGGeod_FromClipper( ip ) );
    }

    if ( Orientation( subject ) ) {
        //SG_LOG(SG_GENERAL, SG_INFO, "Building tgContour as boundary " );
        result.SetHole(false);
    } else {
        //SG_LOG(SG_GENERAL, SG_INFO, "Building tgContour as hole " );
        result.SetHole(true);
    }

    return result;
}

tgContour tgContour::Expand( const tgContour& subject, double offset )
{
    tgPolygon poly;
    tgContour result;

    poly.AddContour( subject );
    ClipperLib::Polygons clipper_src, clipper_dst;

    clipper_src = tgPolygon::ToClipper( poly );

    // convert delta from meters to clipper units
    OffsetPolygons( clipper_src, clipper_dst, Dist_ToClipper(offset) );

    poly = tgPolygon::FromClipper( clipper_dst );

    if ( poly.Contours() == 1 ) {
        result = poly.GetContour( 0 );
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "Expanding contour resulted in more than 1 contour ! ");
        exit(0);
    }

    return result;
}

tgRectangle tgContour::GetBoundingBox( void ) const
{
    SGGeod min, max;

    double minx =  std::numeric_limits<double>::infinity();
    double miny =  std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();

    for (unsigned int i = 0; i < node_list.size(); i++) {
        SGGeod pt = GetNode(i);
        if ( pt.getLongitudeDeg() < minx ) { minx = pt.getLongitudeDeg(); }
        if ( pt.getLongitudeDeg() > maxx ) { maxx = pt.getLongitudeDeg(); }
        if ( pt.getLatitudeDeg()  < miny ) { miny = pt.getLatitudeDeg(); }
        if ( pt.getLatitudeDeg()  > maxy ) { maxy = pt.getLatitudeDeg(); }
    }

    min = SGGeod::fromDeg( minx, miny );
    max = SGGeod::fromDeg( maxx, maxy );

    return tgRectangle( min, max );
}

tgPolygon tgContour::Union( const tgContour& subject, tgPolygon& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.GetSize(); ++i ) {
        all_nodes.add( subject.GetNode(i) );
    }

    ClipperLib::Polygon clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygon(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    return result;
}

tgPolygon tgContour::Diff( const tgContour& subject, tgPolygon& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.GetSize(); ++i ) {
        all_nodes.add( subject.GetNode(i) );
    }

    ClipperLib::Polygon clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygon(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    return result;
}

tgPolygon tgPolygon::Union( const tgContour& subject, tgPolygon& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.GetSize(); ++i ) {
        all_nodes.add( subject.GetNode(i) );
    }

    ClipperLib::Polygon  clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygon(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    return result;
}

tgContour tgContour::AddColinearNodes( const tgContour& subject, UniqueSGGeodSet& nodes )
{
    SGGeod p0, p1;
    tgContour result;
    std::vector<SGGeod>& tmp_nodes = nodes.get_list();

    for ( unsigned int n = 0; n < subject.GetSize()-1; n++ ) {
        p0 = subject.GetNode( n );
        p1 = subject.GetNode( n+1 );

        // add start of segment
        result.AddNode( p0 );

        // add intermediate points
        AddIntermediateNodes( p0, p1, tmp_nodes, result, SG_EPSILON*10, SG_EPSILON*4 );
    }

    p0 = subject.GetNode( subject.GetSize() - 1 );
    p1 = subject.GetNode( 0 );

    // add start of segment
    result.AddNode( p0 );

    // add intermediate points
    AddIntermediateNodes( p0, p1, tmp_nodes, result, SG_EPSILON*10, SG_EPSILON*4 );

    // maintain original hole flag setting
    result.SetHole( subject.GetHole() );

    return result;
}

tgContour tgContour::AddColinearNodes( const tgContour& subject, std::vector<SGGeod>& nodes )
{
    SGGeod p0, p1;
    tgContour result;

    for ( unsigned int n = 0; n < subject.GetSize()-1; n++ ) {
        p0 = subject.GetNode( n );
        p1 = subject.GetNode( n+1 );

        // add start of segment
        result.AddNode( p0 );

        // add intermediate points
        AddIntermediateNodes( p0, p1, nodes, result, SG_EPSILON*10, SG_EPSILON*4 );
    }

    p0 = subject.GetNode( subject.GetSize() - 1 );
    p1 = subject.GetNode( 0 );

    // add start of segment
    result.AddNode( p0 );

    // add intermediate points
    AddIntermediateNodes( p0, p1, nodes, result, SG_EPSILON*10, SG_EPSILON*4 );

    // maintain original hole flag setting
    result.SetHole( subject.GetHole() );

    return result;
}

// this is the opposite of FindColinearNodes - it takes a single SGGeode,
// and tries to find the line segment the point is colinear with
bool tgContour::FindColinearLine( const tgContour& subject, const SGGeod& node, SGGeod& start, SGGeod& end )
{
    SGGeod p0, p1;
    SGGeod new_pt;
    std::vector<SGGeod> tmp_nodes;

    tmp_nodes.push_back( node );
    for ( unsigned int n = 0; n < subject.GetSize()-1; n++ ) {
        p0 = subject.GetNode( n );
        p1 = subject.GetNode( n+1 );

        // add intermediate points
        bool found_extra = FindIntermediateNode( p0, p1, tmp_nodes, new_pt, SG_EPSILON*10, SG_EPSILON*4 );
        if ( found_extra ) {
            start = p0;
            end   = p1;
            return true;
        }
    }

    // check last segment
    p0 = subject.GetNode( subject.GetSize() - 1 );
    p1 = subject.GetNode( 0 );

    // add intermediate points
    bool found_extra = FindIntermediateNode( p0, p1, tmp_nodes, new_pt, SG_EPSILON*10, SG_EPSILON*4 );
    if ( found_extra ) {
        start = p0;
        end   = p1;
        return true;
    }

    return false;
}

void tgContour::SaveToGzFile( gzFile& fp ) const
{
    // Save the nodelist
    sgWriteUInt( fp, node_list.size() );
    for (unsigned int i = 0; i < node_list.size(); i++) {
        sgWriteGeod( fp, node_list[i] );
    }

    // and the hole flag
    sgWriteInt( fp, (int)hole );
}

void tgContour::LoadFromGzFile( gzFile& fp )
{
    unsigned int count;
    SGGeod node;

    // Start Clean
    Erase();

    // Load the nodelist
    sgReadUInt( fp, &count );
    for (unsigned int i = 0; i < count; i++) {
        sgReadGeod( fp, node );
        node_list.push_back( node );
    }

    sgReadInt( fp, (int *)&hole );
}

std::ostream& operator<< ( std::ostream& output, const tgContour& subject )
{
    // Save the data
    output << "NumNodes: " << subject.node_list.size() << "\n";

    for( unsigned int n=0; n<subject.node_list.size(); n++) {
        output << subject.node_list[n] << "\n";
    }

    output << "Hole: " << subject.hole << "\n";

    return output;
}



// tgPolygon static functions
unsigned int tgPolygon::TotalNodes( void ) const
{
    unsigned int total_nodes = 0;

    for (unsigned int c = 0; c < contours.size(); c++) {
        total_nodes += contours[c].GetSize();
    }

    return total_nodes;
}

tgPolygon tgPolygon::Snap( const tgPolygon& subject, double snap )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for (unsigned int c = 0; c < subject.Contours(); c++) {
        result.AddContour( tgContour::Snap( subject.GetContour( c ), snap ) );
    }

    return result;

}

tgPolygon tgPolygon::RemoveDups( const tgPolygon& subject )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::RemoveDups( subject.GetContour( c ) ) );
    }

    return result;
}

tgPolygon tgPolygon::RemoveBadContours( const tgPolygon& subject )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        tgContour contour = subject.GetContour(c);
        if ( contour.GetSize() >= 3 ) {
            /* keeping the contour */
            result.AddContour( contour );
        }
    }

    return result;
}

tgPolygon tgPolygon::RemoveCycles( const tgPolygon& subject )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::RemoveCycles( subject.GetContour( c ) ) );
    }

    return result;
}

tgPolygon tgPolygon::SplitLongEdges( const tgPolygon& subject, double dist )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned c = 0; c < subject.Contours(); c++ )
    {
        result.AddContour( tgContour::SplitLongEdges( subject.GetContour(c), dist ) );
    }

    return result;
}

tgPolygon tgPolygon::StripHoles( const tgPolygon& subject )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    ClipperLib::Polygons clipper_result;
    ClipperLib::Clipper c;
    c.Clear();

    for ( unsigned int i = 0; i < subject.Contours(); i++ ) {
        tgContour contour = subject.GetContour( i );
        if ( !contour.GetHole() ) {
            c.AddPolygon( tgContour::ToClipper( contour ), ClipperLib::ptClip );
        }
    }
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

tgPolygon tgPolygon::Simplify( const tgPolygon& subject )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    ClipperLib::Polygons clipper_poly = tgPolygon::ToClipper( subject );
    SimplifyPolygons( clipper_poly );

    result = tgPolygon::FromClipper( clipper_poly );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

tgPolygon tgPolygon::RemoveTinyContours( const tgPolygon& subject )
{
    double min_area = SG_EPSILON*SG_EPSILON;
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        tgContour contour = subject.GetContour( c );
        double area = contour.GetArea();

        if ( area >= min_area) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "remove_tiny_contours NO - " << c << " area is " << area << " requirement is " << min_area);
            result.AddContour( contour );
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, "remove_tiny_contours " << c << " area is " << area << ": removing");
        }
    }

    return result;
}

tgPolygon tgPolygon::RemoveSpikes( const tgPolygon& subject )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::RemoveSpikes( subject.GetContour(c) ) );
    }

    return result;
}

ClipperLib::Polygons tgPolygon::ToClipper( const tgPolygon& subject )
{
    ClipperLib::Polygons result;

    for ( unsigned int i=0; i<subject.Contours(); i++ ) {
        result.push_back( tgContour::ToClipper( subject.GetContour(i) ) );
    }

    return result;
}

tgPolygon tgPolygon::FromClipper( const ClipperLib::Polygons& subject )
{
    tgPolygon result;

    // for each polygon, we need to check the orientation, to set the hole flag...
    for ( unsigned int i=0; i<subject.size(); i++)
    {
        result.AddContour( tgContour::FromClipper( subject[i] ) );
    }

    return result;
}

bool clipper_dump = false;
void tgPolygon::SetDump( bool dmp )
{
    clipper_dump = dmp;
}

tgPolygon tgPolygon::Expand( const tgPolygon& subject, double offset )
{
    ClipperLib::Polygons clipper_src, clipper_dst;
    clipper_src = tgPolygon::ToClipper( subject );
    tgPolygon result;

    // convert delta from meters to clipper units
    OffsetPolygons( clipper_src, clipper_dst, Dist_ToClipper(offset) );

    result = tgPolygon::FromClipper( clipper_dst );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

inline double CalculateTheta( const SGVec3d& dirCur, const SGVec3d& dirNext, const SGVec3d& cp )
{
    double dp = dot( dirCur, dirNext );

    return acos( dp );
}

SGGeod OffsetPointMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by, int& turn_dir )
{
    double  courseCur, courseNext, courseAvg, theta;
    SGVec3d dirCur, dirNext, dirAvg, cp;
    double  courseOffset, distOffset;
    SGGeod  pt;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find average angle for contour: prev (" << gPrev << "), "
                                                                  "cur (" << gCur  << "), "
                                                                 "next (" << gNext << ")" );

    // first, find if the line turns left or right ar src
    // for this, take the cross product of the vectors from prev to src, and src to next.
    // if the cross product is negetive, we've turned to the left
    // if the cross product is positive, we've turned to the right
    courseCur = SGGeodesy::courseDeg( gCur, gPrev );
    dirCur = SGVec3d( sin( courseCur*SGD_DEGREES_TO_RADIANS ), cos( courseCur*SGD_DEGREES_TO_RADIANS ), 0.0f );

    courseNext = SGGeodesy::courseDeg( gCur, gNext );
    dirNext = SGVec3d( sin( courseNext*SGD_DEGREES_TO_RADIANS ), cos( courseNext*SGD_DEGREES_TO_RADIANS ), 0.0f );

    // Now find the average
    dirAvg = normalize( dirCur + dirNext );
    courseAvg = SGMiscd::rad2deg( atan( dirAvg.x()/dirAvg.y() ) );
    if (courseAvg < 0) {
        courseAvg += 180.0f;
    }

    // check the turn direction
    cp    = cross( dirCur, dirNext );
    theta = SGMiscd::rad2deg(CalculateTheta( dirCur, dirNext, cp ) );

    if ( (abs(theta - 180.0) < 0.1) || (abs(theta) < 0.1) || (isnan(theta)) ) {
        // straight line blows up math - offset 90 degree and dist is as given
        courseOffset = SGMiscd::normalizePeriodic(0, 360, courseNext-90.0);
        distOffset   = offset_by;
    }  else  {
        // calculate correct distance for the offset point
        if (cp.z() < 0.0f) {
            courseOffset = SGMiscd::normalizePeriodic(0, 360, courseAvg+180);
            turn_dir = 0;
        } else {
            courseOffset = SGMiscd::normalizePeriodic(0, 360, courseAvg);
            turn_dir = 1;
        }
        distOffset = (offset_by)/sin(SGMiscd::deg2rad(courseNext-courseOffset));
    }

    // calculate the point from cur
    pt = SGGeodesy::direct(gCur, courseOffset, distOffset);
    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << courseOffset << " distance is " << distOffset << " point is (" << pt.getLatitudeDeg() << "," << pt.getLongitudeDeg() << ")" );

    return pt;
}

SGGeod OffsetPointMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by )
{
    int unused;
    return OffsetPointMiddle( gPrev, gCur, gNext, offset_by, unused );
}

SGGeod OffsetPointFirst( const SGGeod& cur, const SGGeod& next, double offset_by )
{
    double courseOffset;
    SGGeod pt;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at Start : cur (" << cur  << "), "
                                                            "next (" << next << ")" );

    // find the offset angle
    courseOffset = SGGeodesy::courseDeg( cur, next ) - 90;
    courseOffset = SGMiscd::normalizePeriodic(0, 360, courseOffset);

    // calculate the point from cur
    pt = SGGeodesy::direct( cur, courseOffset, offset_by );
    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << courseOffset << " distance is " << offset_by << " point is (" << pt.getLatitudeDeg() << "," << pt.getLongitudeDeg() << ")" );

    return pt;
}

SGGeod OffsetPointLast( const SGGeod& prev, const SGGeod& cur, double offset_by )
{
    double courseOffset;
    SGGeod pt;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at End   : prev (" << prev  << "), "
                                                              "cur (" << cur << ")" );

    // find the offset angle
    courseOffset = SGGeodesy::courseDeg( prev, cur ) - 90;
    courseOffset = SGMiscd::normalizePeriodic(0, 360, courseOffset);

    // calculate the point from cur
    pt = SGGeodesy::direct( cur, courseOffset, offset_by );
    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << courseOffset << " distance is " << offset_by << " point is (" << pt.getLatitudeDeg() << "," << pt.getLongitudeDeg() << ")" );

    return pt;
}

SGGeod midpoint( const SGGeod& p0, const SGGeod& p1 )
{
    return SGGeod::fromDegM( (p0.getLongitudeDeg() + p1.getLongitudeDeg()) / 2,
                             (p0.getLatitudeDeg()  + p1.getLatitudeDeg()) / 2,
                             (p0.getElevationM()   + p1.getElevationM()) / 2 );
}

bool getIntersection_cgal(const SGGeod &p0, const SGGeod &p1, const SGGeod& p2, const SGGeod& p3, SGGeod& intersection)
{
    Point_2 a1( p0.getLongitudeDeg(), p0.getLatitudeDeg() );
    Point_2 b1( p1.getLongitudeDeg(), p1.getLatitudeDeg() );
    Point_2 a2( p2.getLongitudeDeg(), p2.getLatitudeDeg() );
    Point_2 b2( p3.getLongitudeDeg(), p3.getLatitudeDeg() );

    Segment_2 seg1( a1, b1 );
    Segment_2 seg2( a2, b2 );

    CGAL::Object result = CGAL::intersection(seg1, seg2);
    if (const CGAL::Point_2<Kernel> *ipoint = CGAL::object_cast<CGAL::Point_2<Kernel> >(&result)) {
        // handle the point intersection case with *ipoint.
        return true;
    } else {
        if (const CGAL::Segment_2<Kernel> *iseg = CGAL::object_cast<CGAL::Segment_2<Kernel> >(&result)) {
            // handle the segment intersection case with *iseg.
            return false;
        } else {
            // handle the no intersection case.
            return false;
        }
    }
}

tgPolygon tgPolygon::Expand( const SGGeod& subject, double offset )
{
    tgPolygon result;
    tgContour contour;
    SGGeod    pt;

    pt = SGGeodesy::direct( subject, 90, offset/2.0 );
    double dlon = pt.getLongitudeDeg() - subject.getLongitudeDeg();

    pt = SGGeodesy::direct( subject, 0, offset/2.0 );
    double dlat = pt.getLatitudeDeg() - subject.getLatitudeDeg();

    contour.AddNode( SGGeod::fromDeg( subject.getLongitudeDeg() - dlon, subject.getLatitudeDeg() - dlat ) );
    contour.AddNode( SGGeod::fromDeg( subject.getLongitudeDeg() + dlon, subject.getLatitudeDeg() - dlat ) );
    contour.AddNode( SGGeod::fromDeg( subject.getLongitudeDeg() + dlon, subject.getLatitudeDeg() + dlat ) );
    contour.AddNode( SGGeod::fromDeg( subject.getLongitudeDeg() - dlon, subject.getLatitudeDeg() + dlat ) );
    contour.SetHole(false);
    
    result.AddContour( contour );

    return result;
}

tgpolygon_list tgContour::ExpandToPolygons( const tgContour& subject, double width )
{
    int turn_dir;

    SGGeod cur_inner;
    SGGeod cur_outer;
    SGGeod prev_inner;
    SGGeod prev_outer;
    SGGeod calc_inner;
    SGGeod calc_outer;

    double last_end_v = 0.0f;

    tgContour      expanded;
    tgPolygon      segment;
    tgAccumulator  accum;
    tgpolygon_list result;
    
    // generate poly and texparam lists for each line segment
    for (unsigned int i = 0; i < subject.GetSize(); i++)
    {
        last_end_v = 0.0f;
        turn_dir   = 0;

        sglog().setLogLevels( SG_ALL, SG_INFO );

        SG_LOG(SG_GENERAL, SG_DEBUG, "makePolygonsTP: calculating offsets for segment " << i);

        // for each point on the PointsList, generate a quad from
        // start to next, offset by 1/2 width from the edge
        if (i == 0)
        {
            // first point on the list - offset heading is 90deg
            cur_outer = OffsetPointFirst( subject.GetNode(i), subject.GetNode(i+1), -width/2.0f );
            cur_inner = OffsetPointFirst( subject.GetNode(i), subject.GetNode(i+1),  width/2.0f );
        }
        else if (i == subject.GetSize()-1)
        {
            // last point on the list - offset heading is 90deg
            cur_outer = OffsetPointLast( subject.GetNode(i-1), subject.GetNode(i), -width/2.0f );
            cur_inner = OffsetPointLast( subject.GetNode(i-1), subject.GetNode(i),  width/2.0f );
        }
        else
        {
            // middle section
            cur_outer = OffsetPointMiddle( subject.GetNode(i-1), subject.GetNode(i), subject.GetNode(i+1), -width/2.0f, turn_dir );
            cur_inner = OffsetPointMiddle( subject.GetNode(i-1), subject.GetNode(i), subject.GetNode(i+1),  width/2.0f, turn_dir );
        }

        if ( i > 0 )
        {
            SGGeod prev_mp = midpoint( prev_outer, prev_inner );
            SGGeod cur_mp  = midpoint( cur_outer,  cur_inner  );
            SGGeod intersect;
            double heading;
            double dist;
            double az2;

            SGGeodesy::inverse( prev_mp, cur_mp, heading, az2, dist );

            expanded.Erase();
            segment.Erase();

            expanded.AddNode( prev_inner );
            expanded.AddNode( prev_outer );

            // we need to extend one of the points so we're sure we don't create adjacent edges
            if (turn_dir == 0)
            {
                // turned right - offset outer

                if ( getIntersection_cgal( prev_inner, prev_outer, cur_inner, cur_outer, intersect ) )
                {
                    // yes - make a triangle with inner edge = 0
                    expanded.AddNode( cur_outer );
                    cur_inner = prev_inner;
                }
                else
                {
                    expanded.AddNode( cur_outer );
                    expanded.AddNode( cur_inner );
                }
            }
            else
            {
                // turned left - offset inner

                if ( getIntersection_cgal( prev_inner, prev_outer, cur_inner, cur_outer, intersect ) )
                {
                    // yes - make a triangle with outer edge = 0
                    expanded.AddNode( cur_inner );
                    cur_outer = prev_outer;
                }
                else
                {
                    expanded.AddNode( cur_outer );
                    expanded.AddNode( cur_inner );
                }
            }

            expanded.SetHole(false);
            segment.AddContour(expanded);
            segment.SetTexParams( prev_inner, width, 20.0f, heading );
            segment.SetTexLimits( 0, last_end_v, 1, 1 );
            segment.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1.0, 0.0, 1.0, 0.0 );
            result.push_back( segment );

            last_end_v = 1.0f - (fmod( (double)(dist - last_end_v), (double)1.0f ));
        }

        prev_outer = cur_outer;
        prev_inner = cur_inner;
    }

    sglog().setLogLevels( SG_ALL, SG_INFO );

    return result;
}

tgPolygon tgPolygon::Union( const tgPolygon& subject, tgPolygon& clip )
{
    tgPolygon       result;
    UniqueSGGeodSet all_nodes;
    std::ofstream   dmpfile;

    /* before union - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    if ( clipper_dump ) {
        dmpfile.open ("subject.txt");
        dmpfile << clipper_subject;
        dmpfile.close();

        dmpfile.open ("clip.txt");
        dmpfile << clipper_clip;
        dmpfile.close();
    }

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    if ( clipper_dump ) {
        dmpfile.open ("result.txt");
        dmpfile << clipper_result;
        dmpfile.close();
    }

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

tgPolygon tgPolygon::Union( const tgpolygon_list& polys )
{
    ClipperLib::Polygons clipper_result;
    ClipperLib::Clipper c;
    UniqueSGGeodSet all_nodes;
    tgPolygon  result;

    /* before union - gather all nodes */
    for ( unsigned int i=0; i<polys.size(); i++ ) {
        for ( unsigned int j = 0; j < polys[i].Contours(); ++j ) {
            for ( unsigned int k = 0; k < polys[i].ContourSize( j ); ++k ) {
                all_nodes.add( polys[i].GetNode(j, k) );
            }
        }
    }

    c.Clear();
    for (unsigned int i=0; i<polys.size(); i++) {
        ClipperLib::Polygons clipper_clip = tgPolygon::ToClipper( polys[i] );
        c.AddPolygons(clipper_clip, ClipperLib::ptSubject);
    }
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    return result;
}

tgPolygon tgPolygon::Diff( const tgPolygon& subject, tgPolygon& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

tgPolygon tgPolygon::Intersect( const tgPolygon& subject, const tgPolygon& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before intersect - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctIntersection, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

tgRectangle tgPolygon::GetBoundingBox( void ) const
{
    SGGeod min, max;

    double minx =  std::numeric_limits<double>::infinity();
    double miny =  std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();

    for ( unsigned int i = 0; i < Contours(); i++ ) {
        for (unsigned int j = 0; j < ContourSize(i); j++) {
            SGGeod pt = GetNode(i,j);
            if ( pt.getLongitudeDeg() < minx ) { minx = pt.getLongitudeDeg(); }
            if ( pt.getLongitudeDeg() > maxx ) { maxx = pt.getLongitudeDeg(); }
            if ( pt.getLatitudeDeg()  < miny ) { miny = pt.getLatitudeDeg(); }
            if ( pt.getLatitudeDeg()  > maxy ) { maxy = pt.getLatitudeDeg(); }
        }
    }

    min = SGGeod::fromDeg( minx, miny );
    max = SGGeod::fromDeg( maxx, maxy );

    return tgRectangle( min, max );
}

void clipperToShapefile( ClipperLib::Polygons polys, const std::string& path, const std::string&layer, const std::string& name )
{
    tgPolygon poly = tgPolygon::FromClipper(polys);
    tgPolygon::ToShapefile( poly, path, layer, name);
}

// Move slivers from in polygon to out polygon.
void tgPolygon::RemoveSlivers( tgPolygon& subject, tgcontour_list& slivers )
{
#if 0
    // traverse each contour of the polygon and attempt to identify
    // likely slivers
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygon::RemoveSlivers()");

    tgPolygon result;
    tgContour contour;
    int       i;

    double angle_cutoff = 10.0 * SGD_DEGREES_TO_RADIANS;
    double area_cutoff = 0.000000001;
    double min_angle;
    double area;

    // process contours in reverse order so deleting a contour doesn't
    // foul up our sequence
    for ( i = subject.Contours() - 1; i >= 0; --i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "contour " << i );

        contour   = subject.GetContour(i);

        SG_LOG(SG_GENERAL, SG_DEBUG, "  calc min angle for contour " << i);
        min_angle = contour.GetMinimumAngle();
        SG_LOG(SG_GENERAL, SG_DEBUG, "  min_angle (rad) = " << min_angle );

        area      = contour.GetArea();
        SG_LOG(SG_GENERAL, SG_DEBUG, "  area = " << area );

        if ( ((min_angle < angle_cutoff) && (area < area_cutoff)) ||
           ( area < area_cutoff / 10.0) )
        {
            if ((min_angle < angle_cutoff) && (area < area_cutoff))
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "      WE THINK IT'S A SLIVER! - min angle < 10 deg, and area < 10 sq meters");
            }
            else
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "      WE THINK IT'S A SLIVER! - min angle > 10 deg, but area < 1 sq meters");
            }

            // Remove the sliver from source
            subject.DeleteContourAt( i );

            // And add it to the slive list if it isn't a hole
            if ( !contour.GetHole() ) {
                // move sliver contour to sliver list
                SG_LOG(SG_GENERAL, SG_DEBUG, "      Found SLIVER!");

                slivers.push_back( contour );
            }
        }
    }
#endif
}

tgcontour_list tgPolygon::MergeSlivers( tgpolygon_list& polys, tgcontour_list& sliver_list ) {
    tgPolygon poly, result;
    tgContour sliver;
    tgContour contour;
    tgcontour_list unmerged;
    unsigned int original_contours, result_contours;
    bool done;

    for ( unsigned int i = 0; i < sliver_list.size(); i++ ) {
        sliver = sliver_list[i];
        SG_LOG(SG_GENERAL, SG_DEBUG, "Merging sliver = " << i );

        sliver.SetHole( false );

        done = false;

        // try to merge the slivers with the list of clipped polys
        for ( unsigned int j = 0; j < polys.size() && !done; j++ ) {
            poly = polys[j];
            original_contours = poly.Contours();
            result = tgPolygon::Union( sliver, poly );
            result_contours = result.Contours();

            if ( original_contours == result_contours ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "    FOUND a poly to merge the sliver with");
                result.SetMaterial( polys[j].GetMaterial() );
                result.SetTexParams( polys[j].GetTexParams() );
                polys[j] = result;
                done = true;
            }
        }

        if ( !done ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "couldn't merge sliver " << i );
            unmerged.push_back( sliver );
        }
    }

    return unmerged;
}

tgPolygon tgPolygon::AddColinearNodes( const tgPolygon& subject, UniqueSGGeodSet& nodes )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::AddColinearNodes( subject.GetContour(c), nodes ) );
    }

    return result;
}

tgPolygon tgPolygon::AddColinearNodes( const tgPolygon& subject, std::vector<SGGeod>& nodes )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::AddColinearNodes( subject.GetContour(c), nodes ) );
    }

    return result;
}

// this is the opposite of FindColinearNodes - it takes a single SGGeode,
// and tries to find the line segment the point is colinear with
bool tgPolygon::FindColinearLine( const tgPolygon& subject, SGGeod& node, SGGeod& start, SGGeod& end )
{
    bool found = false;

    for ( unsigned int c = 0; c < subject.Contours() && !found; c++ ) {
        found = tgContour::FindColinearLine( subject.GetContour(c), node, start, end );
    }

    return found;
}

SGGeod InterpolateElevation( const SGGeod& dst_node, const SGGeod& start, const SGGeod& end )
{
    double total_dist = SGGeodesy::distanceM( start, end );
    double inter_dist = SGGeodesy::distanceM( start, dst_node );
    double delta = inter_dist/total_dist;

    double dest_elevation = start.getElevationM() + (delta * ( end.getElevationM() - start.getElevationM() ));

    return SGGeod::fromDegM( dst_node.getLongitudeDeg(), dst_node.getLatitudeDeg(), dest_elevation );
}


void tgPolygon::InheritElevations( const tgPolygon& source )
{
    UniqueSGGeodSet     src_nodes;

    // build a list of points from the source polygon
    for ( unsigned int i = 0; i < source.Contours(); ++i ) {
        for ( unsigned int j = 0; j < source.ContourSize(i); ++j ) {
            src_nodes.add( source.GetNode( i, j ) );
        }
    }

    // traverse the dest polygon and build a mirror image but with
    // elevations from the source polygon
    for ( unsigned int i = 0; i < contours.size(); ++i ) {
        for ( unsigned int j = 0; j < contours[i].GetSize(); ++j ) {
            SGGeod dst_node = GetNode(i,j);
            int index = src_nodes.find( dst_node );
            if ( index >= 0 ) {
                SetNode( i, j, src_nodes.get_list()[index] );
            } else {
                /* node not is source - we need to find the two points to interpolate from */
                SGGeod start, end, result;
                if ( FindColinearLine( source, dst_node, start, end ) ) {
                    dst_node = InterpolateElevation( dst_node, start, end );
                    SetNode( i, j, dst_node );
                }
            }
        }
    }
}

void tgPolygon::Texture( void )
{
    SGGeod  p;
    SGVec2f t;
    double  x, y;
    float   tx, ty;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Texture Poly with material " << material << " method " << tp.method << " tpref " << tp.ref << " heading " << tp.heading );

    switch( tp.method ) {
        case TG_TEX_BY_GEODE:
        {
            // The Simgear General texture coordinate routine takes a fan.
            // Simgear could probably use a new function that just takes a Geod vector
            // For now, just create an identity fan...
            std::vector< int > node_idxs;
            for (int i = 0; i < 3; i++) {
                node_idxs.push_back(i);
            }

            for ( unsigned int i = 0; i < triangles.size(); i++ ) {
                std::vector< SGVec2f > tc_list;
                std::vector< SGGeod > nodes;

                nodes = triangles[i].GetNodeList();
                tc_list = sgCalcTexCoords( tp.center_lat, nodes, node_idxs );
                triangles[i].SetTexCoordList( tc_list );
            }
        }
        break;

        case TG_TEX_BY_TPS_NOCLIP:
        case TG_TEX_BY_TPS_CLIPU:
        case TG_TEX_BY_TPS_CLIPV:
        case TG_TEX_BY_TPS_CLIPUV:
        {
            for ( unsigned int i = 0; i < triangles.size(); i++ ) {
                for ( unsigned int j = 0; j < 3; j++ ) {
                    p = triangles[i].GetNode( j );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "point = " << p);

                    //
                    // 1. Calculate distance and bearing from the center of
                    // the poly
                    //

                    // given alt, lat1, lon1, lat2, lon2, calculate starting
                    // and ending az1, az2 and distance (s).  Lat, lon, and
                    // azimuth are in degrees.  distance in meters
                    double az1, az2, dist;
                    SGGeodesy::inverse( tp.ref, p, az1, az2, dist );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "basic course = " << az2);

                    //
                    // 2. Rotate this back into a coordinate system where Y
                    // runs the length of the poly and X runs crossways.
                    //

                    double course = SGMiscd::normalizePeriodic(0, 360, az2 - tp.heading);
                    SG_LOG( SG_GENERAL, SG_DEBUG,"  course = " << course << "  dist = " << dist );

                    //
                    // 3. Convert from polar to cartesian coordinates
                    //

                    x = sin( course * SGD_DEGREES_TO_RADIANS ) * dist;
                    y = cos( course * SGD_DEGREES_TO_RADIANS ) * dist;
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  x = " << x << " y = " << y);

                    //
                    // 4. Map x, y point into texture coordinates
                    //
                    float tmp;

                    tmp = (float)x / (float)tp.width;
                    tx = tmp * (float)(tp.maxu - tp.minu) + (float)tp.minu;
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ")");

                    // clip u?
                    if ( (tp.method == TG_TEX_BY_TPS_CLIPU) || (tp.method == TG_TEX_BY_TPS_CLIPUV) ) {
                        if ( tx < (float)tp.min_clipu ) { tx = (float)tp.min_clipu; }
                        if ( tx > (float)tp.max_clipu ) { tx = (float)tp.max_clipu; }
                    }

                    tmp = (float)y / (float)tp.length;
                    ty = tmp * (float)(tp.maxv - tp.minv) + (float)tp.minv;
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << ty << ")");

                    // clip v?
                    if ( (tp.method == TG_TEX_BY_TPS_CLIPV) || (tp.method == TG_TEX_BY_TPS_CLIPUV) ) {
                        if ( ty < (float)tp.min_clipv ) { ty = (float)tp.min_clipv; }
                        if ( ty > (float)tp.max_clipv ) { ty = (float)tp.max_clipv; }
                    }

                    t = SGVec2f( tx, ty );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ", " << ty << ")");

                    triangles[i].SetTexCoord( j, t );
                }
            }
        }
        break;
    }
}

void tgPolygon::ToShapefile( const tgPolygon& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void*          ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenDatasource returned " << (unsigned long)ds_id);
    
    OGRLayer*      l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);

    OGRPolygon*    polygon = new OGRPolygon();

    SG_LOG(SG_GENERAL, SG_DEBUG, "subject has " << subject.Contours() << " contours ");

    for ( unsigned int i = 0; i < subject.Contours(); i++ ) {
        bool skip_ring=false;
        tgContour contour = subject.GetContour( i );

        if (contour.GetSize() < 3) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points");
            skip_ring=true;
        }

        // FIXME: Current we ignore the hole-flag and instead assume
        //        that the first ring is not a hole and the rest
        //        are holes
        OGRLinearRing *ring=new OGRLinearRing();
        for (unsigned int pt = 0; pt < contour.GetSize(); pt++) {
            OGRPoint *point=new OGRPoint();

            point->setX( contour.GetNode(pt).getLongitudeDeg() );
            point->setY( contour.GetNode(pt).getLatitudeDeg() );
            point->setZ( 0.0 );
            ring->addPoint(point);
        }
        ring->closeRings();

        if (!skip_ring) {
            polygon->addRingDirectly(ring);
        }

        OGRFeature* feature = NULL;
        feature = new OGRFeature( l_id->GetLayerDefn() );
        feature->SetField("ID", description.c_str());
        feature->SetGeometry(polygon);
        if( l_id->CreateFeature( feature ) != OGRERR_NONE )
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
        }
        OGRFeature::DestroyFeature(feature);
    }

    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

tgPolygon tgPolygon::FromOGR( const OGRPolygon* subject )
{
    OGRLinearRing const *ring = subject->getExteriorRing();
    tgContour contour;
    tgPolygon result;

    for (int i = 0; i < ring->getNumPoints(); i++) {
        contour.AddNode( SGGeod::fromDegM( ring->getX(i), ring->getY(i), ring->getZ(i)) );
    }
    contour.SetHole( false );
    result.AddContour( contour );

    // then add the inner rings
    for ( int j = 0 ; j < subject->getNumInteriorRings(); j++ ) {
        ring = subject->getInteriorRing( j );
        contour.Erase();

        for (int i = 0; i < ring->getNumPoints(); i++) {
            contour.AddNode( SGGeod::fromDegM( ring->getX(i), ring->getY(i), ring->getZ(i)) );
        }
        contour.SetHole( true );
        result.AddContour( contour );
    }
    result.SetTexMethod( TG_TEX_BY_GEODE );
    
    return result;
}

void tgPolygon::SaveToGzFile( gzFile& fp ) const
{
    // Save the contours
    sgWriteUInt( fp, contours.size() );
    for (unsigned int i = 0; i < contours.size(); i++) {
        contours[i].SaveToGzFile( fp );
    }

    // Save the triangles
    sgWriteUInt( fp, triangles.size() );
    for (unsigned int i = 0; i < triangles.size(); i++) {
        triangles[i].SaveToGzFile( fp );
    }

    // Save the tex params
    tp.SaveToGzFile( fp );

    // and the rest
    sgWriteString( fp, material.c_str() );
    sgWriteString( fp, flag.c_str() );
    sgWriteInt( fp, (int)preserve3d );
}

void tgPolygon::LoadFromGzFile( gzFile& fp )
{
    unsigned int count;
    tgContour contour;
    tgTriangle triangle;
    char *strbuff;

    // Start clean
    Erase();

    // Load the contours
    sgReadUInt( fp, &count );
    for (unsigned int i = 0; i < count; i++) {
        contour.LoadFromGzFile( fp );
        AddContour(contour);
    }

    // load the triangles
    sgReadUInt( fp, &count );
    for (unsigned int i = 0; i < count; i++) {
        triangle.LoadFromGzFile( fp );
        AddTriangle(triangle);
    }

    // Load the tex params
    tp.LoadFromGzFile( fp );

    // and the rest
    sgReadString( fp, &strbuff );
    if ( strbuff ) {
        material = strbuff;
        delete strbuff;
    }

    sgReadString( fp, &strbuff );
    if ( strbuff ) {
        flag = strbuff;
        delete strbuff;
    }

    sgReadInt( fp, (int *)&preserve3d );
}

#if 0
////////////////////////////// CHOP ////////////////////////////////

// initialize the unique polygon index counter stored in path
static long int poly_index = 0;
static std::string poly_path;

bool tgPolygon::ChopIdxInit( const std::string& path )
{
    poly_path = path;
    FILE *fp = fopen( poly_path.c_str(), "r" );

    if ( fp == NULL ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Warning: cannot open " << path);
        poly_index = 0;
        return false;
    }

    fscanf( fp, "%ld", &poly_index );
    fclose( fp );

    return true;
}
#endif

/************************ TESSELATION ***********************************/

void tg_mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border )
{
    if(start->info().nesting_level != -1) {
        return;
    }

    std::list<CDT::Face_handle> queue;
    queue.push_back(start);

    while( !queue.empty() ){
        CDT::Face_handle fh = queue.front();
        queue.pop_front();
        if(fh->info().nesting_level == -1) {
            fh->info().nesting_level = index;
            for(int i = 0; i < 3; i++) {
                CDT::Edge e(fh,i);
                CDT::Face_handle n = fh->neighbor(i);
                if(n->info().nesting_level == -1) {
                    if(ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void tg_mark_domains(CDT& cdt)
{
    for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
        it->info().nesting_level = -1;
    }

    int index = 0;
    std::list<CDT::Edge> border;
    tg_mark_domains(cdt, cdt.infinite_face(), index++, border);
    while(! border.empty()) {
        CDT::Edge e = border.front();
        border.pop_front();
        CDT::Face_handle n = e.first->neighbor(e.second);
        if(n->info().nesting_level == -1) {
            tg_mark_domains(cdt, n, e.first->info().nesting_level+1, border);
        }
    }
}

void tg_insert_polygon(CDT& cdt,const Polygon_2& polygon)
{
    if ( polygon.is_empty() ) return;

    CDT::Vertex_handle v_prev=cdt.insert(*CGAL::cpp0x::prev(polygon.vertices_end()));
    for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin(); vit!=polygon.vertices_end();++vit) {
        CDT::Vertex_handle vh=cdt.insert(*vit);
        cdt.insert_constraint(vh,v_prev);
        v_prev=vh;
    }
}

void tgPolygon::Tesselate( const std::vector<SGGeod>& extra )
{
    CDT       cdt;

    SG_LOG( SG_GENERAL, SG_DEBUG, "Tess with extra" );

    // Bail right away if polygon is empty
    if ( contours.size() != 0 ) {
        // First, convert the extra points to cgal Points
        std::vector<Point> points;
        points.reserve(extra.size());
        for (unsigned int n = 0; n < extra.size(); n++) {
            points.push_back( Point(extra[n].getLongitudeDeg(), extra[n].getLatitudeDeg() ) );
        }

        cdt.insert(points.begin(), points.end());

        // then insert each polygon as a constraint into the triangulation
        for ( unsigned int c = 0; c < contours.size(); c++ ) {
            tgContour contour = contours[c];
            Polygon_2 poly;

            for (unsigned int n = 0; n < contour.GetSize(); n++ ) {
                SGGeod node = contour.GetNode(n);
                poly.push_back( Point( node.getLongitudeDeg(), node.getLatitudeDeg() ) );
            }

            tg_insert_polygon(cdt, poly);
        }

        tg_mark_domains( cdt );

        int count=0;
        for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end(); ++fit) {
            if ( fit->info().in_domain() ) {
                Triangle_2 tri = cdt.triangle(fit);

                SGGeod p0 = SGGeod::fromDeg( tri.vertex(0).x(), tri.vertex(0).y() );
                SGGeod p1 = SGGeod::fromDeg( tri.vertex(1).x(), tri.vertex(1).y() );
                SGGeod p2 = SGGeod::fromDeg( tri.vertex(2).x(), tri.vertex(2).y() );

                AddTriangle( p0, p1, p2 );

                ++count;
            }
        }
    }
}

void tgPolygon::Tesselate()
{
    CDT       cdt;

    SG_LOG( SG_GENERAL, SG_DEBUG, "Tess" );

    // Bail right away if polygon is empty
    if ( contours.size() != 0 ) {
        // insert each polygon as a constraint into the triangulation
        for ( unsigned int c = 0; c < contours.size(); c++ ) {
            tgContour contour = contours[c];
            Polygon_2 poly;

            for (unsigned int n = 0; n < contour.GetSize(); n++ ) {
                SGGeod node = contour.GetNode(n);
                SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : Adding GEOD " << node);
                poly.push_back( Point( node.getLongitudeDeg(), node.getLatitudeDeg() ) );
            }

            tg_insert_polygon(cdt, poly);
        }

        tg_mark_domains( cdt );

        int count=0;
        for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end(); ++fit) {
            if ( fit->info().in_domain() ) {
                SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : face   in domain");

                Triangle_2 tri = cdt.triangle(fit);

                SGGeod p0 = SGGeod::fromDeg( tri.vertex(0).x(), tri.vertex(0).y() );
                SGGeod p1 = SGGeod::fromDeg( tri.vertex(1).x(), tri.vertex(1).y() );
                SGGeod p2 = SGGeod::fromDeg( tri.vertex(2).x(), tri.vertex(2).y() );

                AddTriangle( p0, p1, p2 );

                ++count;
            } else {
                SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : face not in domain");
            }
        }
    }
    else
    {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : no contours" );
    }
}

// Friends for serialization
std::ostream& operator<< ( std::ostream& output, const tgPolygon& subject )
{
    // Save the data
    output << "NumContours: " << subject.contours.size() << "\n";

    for( unsigned int c=0; c<subject.contours.size(); c++) {
        output << subject.contours[c];
    }

    output << "NumTriangles: " << subject.triangles.size() << "\n";
    for( unsigned int t=0; t<subject.triangles.size(); t++) {
        output << subject.triangles[t];
    }

    output << "Material: " << subject.material;
    output << "Flag: " << subject.flag;
    output << subject.tp;

    return output;
}

std::ostream& operator<< ( std::ostream& output, const tgTriangle& subject )
{
    output << "nodes\n";
    if ( subject.node_list.size() == 3 ) {
        output << subject.node_list[0] << ", " << subject.node_list[1] << ", " << subject.node_list[2] << "\n";
    } else {
        output << "empty\n";
    }

    output << "normals\n";
    if ( subject.norm_list.size() == 3 ) {
        output << subject.norm_list[0] << ", " << subject.norm_list[1] << ", " << subject.norm_list[2] << "\n";
    } else {
        output << "empty\n";
    }

    output << "texture coords\n";
    if ( subject.tc_list.size() == 3 ) {
        output << subject.tc_list[0] << ", " << subject.tc_list[1] << ", " << subject.tc_list[2] << "\n";
    } else {
        output << "empty\n";
    }

    output << "node indexes\n";
    if ( subject.idx_list.size() == 3 ) {
        output << subject.idx_list[0] << ", " << subject.idx_list[1] << ", " << subject.idx_list[2] << "\n";
    } else {
        output << "empty\n";
    }

    output << "Face normal: " << subject.face_normal << "\n";
    output << "Face area: "   << subject.face_area << "\n";

    return output;
}

std::ostream& operator<< ( std::ostream& output, const tgTexParams& subject )
{
    // Save the data
    output << "Ref    : " << subject.ref;
    output << "Width  : " << subject.width;
    output << "Length : " << subject.length;
    output << "Heading: " << subject.heading;

    output << "u: (" << subject.minu << "," << subject.maxu << ")";
    output << "v: (" << subject.minv << "," << subject.maxv << ")";

    output << "method: " << subject.method;
    
    return output;
}

tgPolygon tgAccumulator::Diff( const tgContour& subject )
{
    tgPolygon  result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.GetSize(); ++i ) {
        all_nodes.add( subject.GetNode(i) );
    }

    unsigned int  num_hits = 0;
    tgRectangle box1 = subject.GetBoundingBox();

    ClipperLib::Polygon  clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();

    c.AddPolygon(clipper_subject, ClipperLib::ptSubject);

    // clip result against all polygons in the accum that intersect our bb
    for (unsigned int i=0; i < accum.size(); i++) {
        tgRectangle box2 = BoundingBox_FromClipper( accum[i] );

        if ( box2.intersects(box1) )
        {
            c.AddPolygons(accum[i], ClipperLib::ptClip);
            num_hits++;
        }
    }

    if (num_hits) {
        if ( !c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE" );
            exit(-1);
        }
        result = tgPolygon::FromClipper( clipper_result );
        result = tgPolygon::AddColinearNodes( result, all_nodes );
    } else {
        result.AddContour( subject );
    }

    return result;
}

void tgAccumulator::Add( const tgContour& subject )
{
    tgPolygon poly;
    poly.AddContour( subject );

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( poly );
    accum.push_back( clipper_subject );
}

void tgAccumulator::ToShapefiles( const std::string& path, const std::string& layer_prefix )
{
    char shapefile[16];
    char layer[16];

    for (unsigned int i=0; i < accum.size(); i++) {
        sprintf( layer, "%s_%d", layer_prefix.c_str(), i );
        sprintf( shapefile, "accum_%d", i );
        clipperToShapefile( accum[i], path, layer, std::string(shapefile) );
    }
}

tgPolygon tgAccumulator::Diff( const tgPolygon& subject )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    unsigned int  num_hits = 0;
    tgRectangle box1 = subject.GetBoundingBox();

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();

    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);

    // clip result against all polygons in the accum that intersect our bb
    for (unsigned int i=0; i < accum.size(); i++) {
        tgRectangle box2 = BoundingBox_FromClipper( accum[i] );

        if ( box2.intersects(box1) )
        {
            c.AddPolygons(accum[i], ClipperLib::ptClip);
            num_hits++;
        }
    }

    if (num_hits) {
        if ( !c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE" );
            exit(-1);
        }

        result = tgPolygon::FromClipper( clipper_result );
        result = tgPolygon::AddColinearNodes( result, all_nodes );

        // Make sure we keep texturing info
        result.SetMaterial( subject.GetMaterial() );
        result.SetTexParams( subject.GetTexParams() );
    } else {
        result = subject;
    }

    return result;
}

void tgAccumulator::Add( const tgPolygon& subject )
{
    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    accum.push_back( clipper_subject );
}

void tgTriangle::SaveToGzFile( gzFile& fp ) const
{
    // Save the three nodes, and their attributes
    for (unsigned int i = 0; i < 3; i++) {
        sgWriteGeod( fp, node_list[i] );
        // sgWriteVec2( fp, tc_list[i] );
        // sgWritedVec3( fp, norm_list[i] ); // not calculated until stage 3
        sgWriteInt( fp, idx_list[i] );
    }

    // and the area, and face normal
    // sgWriteVec3( fp, face_normal ); // not calculated until stage3
    // sgWriteDouble( fp, face_area ); // not calculated until stage3
}

void tgTriangle::LoadFromGzFile( gzFile& fp )
{
    // Load the nodelist
    for (unsigned int i = 0; i < 3; i++) {
        sgReadGeod( fp, node_list[i] );
        // sgReadVec2( fp, tc_list[i] );
        // sgReaddVec3( fp, norm_list[i] );
        sgReadInt( fp, &idx_list[i] );
    }

    // and the area, and face normal
    // sgReadVec3( fp, face_normal );
    // sgReadDouble( fp, &face_area );
}

void tgTexParams::SaveToGzFile( gzFile& fp ) const
{
    // Save the parameters
    sgWriteInt( fp, (int)method );

    if ( method == TG_TEX_BY_GEODE ) {
        sgWriteDouble( fp, center_lat );
    } else {
        sgWriteGeod( fp, ref );
        sgWriteDouble( fp, width );
        sgWriteDouble( fp, length );
        sgWriteDouble( fp, heading );

        sgWriteDouble( fp, minu );
        sgWriteDouble( fp, maxu );
        sgWriteDouble( fp, minv );
        sgWriteDouble( fp, maxv );

        if ( (method == TG_TEX_BY_TPS_CLIPU) ||
             (method == TG_TEX_BY_TPS_CLIPUV) ) {
            sgWriteDouble( fp, min_clipu );
            sgWriteDouble( fp, max_clipu );
        }

        if ( (method == TG_TEX_BY_TPS_CLIPV) ||
             (method == TG_TEX_BY_TPS_CLIPUV) ) {
            sgWriteDouble( fp, min_clipv );
            sgWriteDouble( fp, max_clipv );
        }
    }
}

void tgTexParams::LoadFromGzFile( gzFile& fp )
{
    // Load the parameters
    sgReadInt( fp, (int*)&method );

    if ( method == TG_TEX_BY_GEODE ) {
        sgReadDouble( fp, &center_lat );
    } else {
        sgReadGeod( fp, ref );
        sgReadDouble( fp, &width );
        sgReadDouble( fp, &length );
        sgReadDouble( fp, &heading );

        sgReadDouble( fp, &minu );
        sgReadDouble( fp, &maxu );
        sgReadDouble( fp, &minv );
        sgReadDouble( fp, &maxv );

        if ( (method == TG_TEX_BY_TPS_CLIPU) ||
             (method == TG_TEX_BY_TPS_CLIPUV) ) {
            sgReadDouble( fp, &min_clipu );
            sgReadDouble( fp, &max_clipu );
        }

        if ( (method == TG_TEX_BY_TPS_CLIPV) ||
             (method == TG_TEX_BY_TPS_CLIPUV) ) {
            sgReadDouble( fp, &min_clipv );
            sgReadDouble( fp, &max_clipv );
        }
    }
}

// CHOPPER
void tgChopper::Clip( const tgPolygon& subject,
                      const std::string& type,
                      SGBucket& b )
{
    // p;

    SGGeod min, max;
    SGGeod c    = b.get_center();
    double span = b.get_width();
    tgPolygon base, result;

    // calculate bucket dimensions
    if ( (c.getLatitudeDeg() >= -89.0) && (c.getLatitudeDeg() < 89.0) ) {
        min = SGGeod::fromDeg( c.getLongitudeDeg() - span/2.0, c.getLatitudeDeg() - SG_HALF_BUCKET_SPAN );
        max = SGGeod::fromDeg( c.getLongitudeDeg() + span/2.0, c.getLatitudeDeg() + SG_HALF_BUCKET_SPAN );
    } else if ( c.getLatitudeDeg() < -89.0) {
        min = SGGeod::fromDeg( -90.0, -180.0 );
        max = SGGeod::fromDeg( -89.0,  180.0 );
    } else if ( c.getLatitudeDeg() >= 89.0) {
        min = SGGeod::fromDeg(  89.0, -180.0 );
        max = SGGeod::fromDeg(  90.0,  180.0 );
    } else {
        SG_LOG( SG_GENERAL, SG_ALERT,  "Out of range latitude in clip_and_write_poly() = " << c.getLatitudeDeg() );
    }

    SG_LOG( SG_GENERAL, SG_DEBUG, "  (" << min << ") (" << max << ")" );

    // set up clipping tile
    base.AddNode( 0, SGGeod::fromDeg( min.getLongitudeDeg(), min.getLatitudeDeg()) );
    base.AddNode( 0, SGGeod::fromDeg( max.getLongitudeDeg(), min.getLatitudeDeg()) );
    base.AddNode( 0, SGGeod::fromDeg( max.getLongitudeDeg(), max.getLatitudeDeg()) );
    base.AddNode( 0, SGGeod::fromDeg( min.getLongitudeDeg(), max.getLatitudeDeg()) );

    SG_LOG(SG_GENERAL, SG_DEBUG, "shape contours = " << subject.Contours() );
    for ( unsigned int ii = 0; ii < subject.Contours(); ii++ ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "   hole = " << subject.GetContour(ii).GetHole() );
    }

    result = tgPolygon::Intersect( subject, base );

    SG_LOG(SG_GENERAL, SG_DEBUG, "result contours = " << result.Contours() );
    for ( unsigned int ii = 0; ii < result.Contours(); ii++ ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  hole = " << result.GetContour(ii).GetHole() );
    }

    if ( subject.GetPreserve3D() ) {
        result.InheritElevations( subject );
    }

    if ( result.Contours() > 0 ) {
        result.SetPreserve3D( subject.GetPreserve3D() );
        result.SetTexParams( subject.GetTexParams() );
        if ( subject.GetTexMethod() == TG_TEX_BY_GEODE ) {
            // need to set center latitude for geodetic texturing
            result.SetTexMethod( TG_TEX_BY_GEODE, b.get_center_lat() );
        }
        result.SetFlag(type);

        lock.lock();
        bp_map[b.gen_index()].push_back( result );
        lock.unlock();
    }
}

void tgChopper::Add( const tgPolygon& subject, const std::string& type )
{
    tgRectangle bb;
    SGGeod p;

    // bail out immediately if polygon is empty
    if ( subject.Contours() == 0 )
        return;

    bb = subject.GetBoundingBox();

    SG_LOG( SG_GENERAL, SG_DEBUG, "  min = " << bb.getMin() << " max = " << bb.getMax() );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( bb.getMin() );
    SGBucket b_max( bb.getMax() );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket min = " << b_min );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket max = " << b_max );

    if ( b_min == b_max ) {
        // shape entirely contained in a single bucket, write and bail
        Clip( subject, type, b_min );
        return;
    }

    SGBucket b_cur;
    int dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_DEBUG, "  polygon spans tile boundaries" );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  dx = " << dx << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy <= 1 ) {
        // we are down to at most two rows, write each column and then bail
        double min_center_lat = b_min.get_center_lat();
        double min_center_lon = b_min.get_center_lon();
        for ( int j = 0; j <= dy; ++j ) {
            for ( int i = 0; i <= dx; ++i ) {
                b_cur = sgBucketOffset(min_center_lon, min_center_lat, i, j);
                Clip( subject, type, b_cur );
            }
        }
        return;
    }

    // we have two or more rows left, split in half (along a
    // horizontal dividing line) and recurse with each half

    // find mid point (integer math)
    int mid = (dy + 1) / 2 - 1;

    // determine horizontal clip line
    SGBucket b_clip = sgBucketOffset( bb.getMin().getLongitudeDeg(), bb.getMin().getLatitudeDeg(), 0, mid);
    double clip_line = b_clip.get_center_lat();
    if ( (clip_line >= -90.0 + SG_HALF_BUCKET_SPAN)
         && (clip_line < 90.0 - SG_HALF_BUCKET_SPAN) )
        clip_line += SG_HALF_BUCKET_SPAN;
    else if ( clip_line < -89.0 )
        clip_line = -89.0;
    else if ( clip_line >= 89.0 )
        clip_line = 90.0;
    else {
        SG_LOG( SG_GENERAL, SG_ALERT, "Out of range latitude in clip_and_write_poly() = " << clip_line );
    }

    {
        //
        // Crop bottom area (hopefully by putting this in it's own
        // scope we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG, "Generating bottom half (" << bb.getMin().getLatitudeDeg() << "-" << clip_line << ")" );

        tgPolygon bottom, bottom_clip;

        bottom.AddNode( 0, SGGeod::fromDeg(-180.0, bb.getMin().getLatitudeDeg()) );
        bottom.AddNode( 0, SGGeod::fromDeg( 180.0, bb.getMin().getLatitudeDeg()) );
        bottom.AddNode( 0, SGGeod::fromDeg( 180.0, clip_line) );
        bottom.AddNode( 0, SGGeod::fromDeg(-180.0, clip_line) );

        bottom_clip = tgPolygon::Intersect( subject, bottom );

        // the texparam should be constant over each clipped poly.
        // when they are reassembled, we want the texture map to
        // be seamless
        Add( bottom_clip, type );
    }

    {
        //
        // Crop top area (hopefully by putting this in it's own scope
        // we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG, "Generating top half (" << clip_line << "-" << bb.getMax().getLatitudeDeg() << ")" );

        tgPolygon top, top_clip;

        top.AddNode( 0, SGGeod::fromDeg(-180.0, clip_line) );
        top.AddNode( 0, SGGeod::fromDeg( 180.0, clip_line) );
        top.AddNode( 0, SGGeod::fromDeg( 180.0, bb.getMax().getLatitudeDeg()) );
        top.AddNode( 0, SGGeod::fromDeg(-180.0, bb.getMax().getLatitudeDeg()) );

        top_clip = tgPolygon::Intersect( subject, top );

        if ( top_clip.TotalNodes() == subject.TotalNodes() ) {
            SG_LOG( SG_GENERAL, SG_DEBUG, "Generating top half - total nodes is the same after clip" << subject.TotalNodes() );
            exit(0);
        }

        Add( top_clip, type );
    }
}

long int tgChopper::GenerateIndex( std::string path )
{
    std::string index_file = path + "/chop.idx";
    long int index = 0;

    //Open or create the named mutex
    boost::interprocess::named_mutex mutex(boost::interprocess::open_or_create, "tgChopper_index2");
    {
//        SG_LOG(SG_GENERAL, SG_ALERT, "getting lock");
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mutex);
//        SG_LOG(SG_GENERAL, SG_ALERT, " - got it");

        /* first try to read the file */
        FILE *fp = fopen( index_file.c_str(), "r+" );
        if ( fp == NULL ) {
            /* doesn't exist - create it */
            fp = fopen( index_file.c_str(), "w" );
            if ( fp == NULL ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Error cannot open Index file " << index_file << " for writing");
                boost::interprocess::named_mutex::remove("tgChopper_index2");
                exit( 0 );
            }
        } else {
            fread( (void*)&index, sizeof(long int), 1, fp );
//            SG_LOG(SG_GENERAL, SG_ALERT, " SUCCESS READING INDEX FILE - READ INDEX " << index );
        }

        index++;

        rewind( fp );
        fwrite( (void*)&index, sizeof(long int), 1, fp );
        fclose( fp );
    }

    boost::interprocess::named_mutex::remove("tgChopper_index2");

    return index;
}

void tgChopper::Save( void )
{
    // traverse the bucket list
    bucket_polys_map_interator it;
    char tile_name[16];
    char poly_ext[16];

    for (it=bp_map.begin(); it != bp_map.end(); it++) {
        SGBucket b( (*it).first );
        tgpolygon_list const& polys = (*it).second;

        std::string path = root_path + "/" + b.gen_base_path();
        sprintf( tile_name, "%ld", b.gen_index() );

        std::string polyfile = path + "/" + tile_name;

        SGPath sgp( polyfile );
        sgp.create_dir( 0755 );

        long int poly_index = GenerateIndex( path );

        sprintf( poly_ext, "%ld", poly_index );
        polyfile = polyfile + "." + poly_ext;

        gzFile fp;
        if ( (fp = gzopen( polyfile.c_str(), "wb9" )) == NULL ) {
            SG_LOG( SG_GENERAL, SG_INFO, "ERROR: opening " << polyfile.c_str() << " for writing!" );
            return;
        }

        /* Write polys to the file */
        sgWriteUInt( fp, polys.size() );
        for ( unsigned int i=0; i<polys.size(); i++ ) {
            polys[i].SaveToGzFile( fp );
        }

        gzclose( fp );
    }
}

const char* format_name="ESRI Shapefile";

void tgShapefile::Init( void )
{
    OGRRegisterAll();
}

void* tgShapefile::OpenDatasource( const char* datasource_name )
{
    OGRDataSource *datasource;
    OGRSFDriver   *ogrdriver;

    ogrdriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(format_name);
    if (!ogrdriver) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Unknown datasource format driver: " << format_name);
        exit(1);
    }

    datasource = ogrdriver->Open(datasource_name, TRUE);

    if (!datasource) {
        datasource = ogrdriver->CreateDataSource(datasource_name, NULL);
    }

    if (!datasource) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Unable to open or create datasource: " << datasource_name);
        exit(1);
    }

    return (void*)datasource;
}

void* tgShapefile::OpenLayer( void* ds_id, const char* layer_name ) {
    OGRDataSource* datasource = (OGRDataSource *)ds_id;
    OGRLayer* layer;

    OGRSpatialReference srs;
    srs.SetWellKnownGeogCS("WGS84");

    layer = datasource->GetLayerByName(layer_name);

    if (!layer) {
        layer = datasource->CreateLayer( layer_name, &srs, wkbPolygon25D, NULL);

        OGRFieldDefn descriptionField("ID", OFTString);
        descriptionField.SetWidth(128);

        if( layer->CreateField( &descriptionField ) != OGRERR_NONE ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Creation of field 'Description' failed");
        }
    }

    if (!layer) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Creation of layer '" << layer_name << "' failed");
        return NULL;
    }

    return (void*)layer;
}

void tgShapefile::CloseLayer( void* l_id )
{
    //OGRLayer::DestroyLayer( layer );
}

void* tgShapefile::CloseDatasource( void* ds_id )
{
    OGRDataSource* datasource = (OGRDataSource *)ds_id;

    OGRDataSource::DestroyDataSource( datasource );

    return (void *)-1;
}

void clipper_to_shapefile( ClipperLib::Polygons polys, char* ds )
{
#if 0
    ClipperLib::Polygons contour;
    TGPolygon tgcontour;
    char layer[32];

    void*       ds_id = tgShapefile::OpenDatasource( ds );

    for (unsigned int i = 0; i < polys.size(); ++i) {
        if  ( Orientation( polys[i] ) ) {
            sprintf( layer, "%04d_boundary", i );
        } else {
            sprintf( layer, "%04d_hole", i );
        }

        void* l_id  = tgShapefile::OpenLayer( ds_id, layer );
        contour.clear();
        contour.push_back( polys[i] );

        tgcontour.erase();
        make_tg_poly_from_clipper( contour, &tgcontour );

        tgShapefile::CreateFeature( ds_id, l_id, tgcontour, "contour" );
    }

    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
#endif
}
