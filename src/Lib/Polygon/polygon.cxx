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

#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <Geometry/point3d.hxx>
#include <Geometry/poly_support.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/structure/exception.hxx>

#include <Geometry/trinodes.hxx>

#include "polygon.hxx"

#ifdef _MSC_VER
#   define LONG_LONG_MAX LLONG_MAX
#   define LONG_LONG_MIN LLONG_MIN
#endif

using std::endl;
using std::cout;

// Constructor 
TGPolygon::TGPolygon( void )
{
}


// Destructor
TGPolygon::~TGPolygon( void ) {
}

void TGPolygon::get_bounding_box( SGGeod& min, SGGeod& max ) const
{
    double minx = std::numeric_limits<double>::infinity();
    double miny = std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();

    for ( int i = 0; i < contours(); i++ ) {
        for (unsigned int j = 0; j < poly[i].size(); j++) {
            SGGeod pt = poly[i][j].toSGGeod();
            if ( pt.getLongitudeDeg() < minx ) { minx = pt.getLongitudeDeg(); }
            if ( pt.getLongitudeDeg() > maxx ) { maxx = pt.getLongitudeDeg(); }
            if ( pt.getLatitudeDeg() < miny ) { miny = pt.getLatitudeDeg(); }
            if ( pt.getLatitudeDeg() > maxy ) { maxy = pt.getLatitudeDeg(); }
        }
    }

    min = SGGeod::fromDeg( minx, miny );
    max = SGGeod::fromDeg( maxx, maxy );
}


// Set the elevations of points in the current polgyon based on the
// elevations of points in source.  For points that are not found in
// source, propogate the value from the nearest matching point.
void TGPolygon::inherit_elevations( const TGPolygon &source ) {
    TGTriNodes nodes;
    nodes.clear();

    int i, j;

    // build a list of points from the source and dest polygons

    for ( i = 0; i < source.contours(); ++i ) {
        for ( j = 0; j < source.contour_size(i); ++j ) {
            Point3D p = source.get_pt( i, j );
            nodes.unique_add( p );
        }
    }

    // traverse the dest polygon and build a mirror image but with
    // elevations from the source polygon

    for ( i = 0; i < (int)poly.size(); ++i ) {
        for ( j = 0; j < (int)poly[i].size(); ++j ) {
            Point3D p = poly[i][j];
            int index = nodes.find( p );
            if ( index >= 0 ) {
                Point3D ref = nodes.get_node( index );
                poly[i][j].setz( ref.z() );
            }
        }
    }

    // now post process result to catch any nodes that weren't updated
    // (because the clipping process may have added points which
    // weren't in the original.)

    double last = -9999.0;
    for ( i = 0; i < (int)poly.size(); ++i ) {
        // go front ways
        last = -9999.0;
        for ( j = 0; j < (int)poly[i].size(); ++j ) {
            Point3D p = poly[i][j];
            if ( p.z() > -9000 ) {
                last = p.z();
            } else {
               if ( last > -9000 ) {
                   poly[i][j].setz( last );
               }
            }
        }

        // go back ways
        last = -9999.0;
        for ( j = poly[i].size() - 1; j >= 0; --j ) {
            Point3D p = poly[i][j];
            if ( p.z() > -9000 ) {
                last = p.z();
            } else {
               if ( last > -9000 ) {
                   poly[i][j].setz( last );
               }
            }
        }
    }
}


// Set the elevations of all points to the specified values
void TGPolygon::set_elevations( double elev ) {
    for ( unsigned i = 0; i < poly.size(); ++i ) {
        for ( unsigned int j = 0; j < poly[i].size(); ++j ) {
            poly[i][j].setz( elev );
        }
    }
}

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

// return the perimeter of a contour (assumes simple polygons,
// i.e. non-self intersecting.)
//
// negative areas indicate counter clockwise winding
// positive areas indicate clockwise winding.

double TGPolygon::area_contour( const int contour ) const {
    point_list c = poly[contour];
    double area = 0.0;
    unsigned int i, j;

    if (c.size()) {
        j = c.size() - 1;
        for (i=0; i<c.size(); i++) {
            area += (c[j].x() + c[i].x()) * (c[j].y() - c[i].y()); 
            j=i; 
        }
    }

    return fabs(area * 0.5); 
}

// return the smallest interior angle of the contour
double TGPolygon::minangle_contour( const int contour ) {
    point_list c = poly[contour];
    int size = c.size();
    int p1_index, p2_index, p3_index;
    SGVec2d p1, p2, p3;
    double angle;
    double min_angle = 2.0 * SGD_PI;

    for ( int i = 0; i < size; ++i ) {
        p1_index = i - 1;
        if ( p1_index < 0 ) {
            p1_index += size;
        }

        p2_index = i;

        p3_index = i + 1;
        if ( p3_index >= size ) {
            p3_index -= size;
        }

        p1.x() = c[p1_index].x();
        p1.y() = c[p1_index].y();

        p2.x() = c[p2_index].x();
        p2.y() = c[p2_index].y();

        p3.x() = c[p3_index].x();
        p3.y() = c[p3_index].y();

        angle = tgPolygonCalcAngle( p1, p2, p3 );

        if ( angle < min_angle ) {
            min_angle = angle;
        }
    }

    return min_angle;
}

// return true if contour A is inside countour B
bool TGPolygon::is_inside( int a, int b ) const {
    // make polygons from each specified contour
    TGPolygon A, B;
    point_list pl;
    A.erase();
    B.erase();

    pl = get_contour( a );
    A.add_contour( pl, 0 );

    pl = get_contour( b );
    B.add_contour( pl, 0 );

    // SG_LOG(SG_GENERAL, SG_DEBUG, "A size = " << A.total_size());
    // A.write( "A" );
    // SG_LOG(SG_GENERAL, SG_DEBUG, "B size = " << B.total_size());
    // B.write( "B" );

    // A is "inside" B if the polygon_diff( A, B ) is null.
    TGPolygon result = tgPolygonDiff( A, B );
    // SG_LOG(SG_GENERAL, SG_DEBUG, "result size = " << result.total_size());

    // char junk;
    // cin >> junk;

    if ( result.contours() == 0 ) {
	// SG_LOG(SG_GENERAL, SG_DEBUG, "  " << a << " is_inside() " << b);
	return true;
    }

    // SG_LOG(SG_GENERAL, SG_DEBUG, "  " << a << " not is_inside() " << b);
    return false;
}


// shift every point in the polygon by lon, lat
void TGPolygon::shift( double lon, double lat ) {
    for ( int i = 0; i < (int)poly.size(); ++i ) {
        for ( int j = 0; j < (int)poly[i].size(); ++j ) {
            poly[i][j].setx( poly[i][j].x() + lon );
            poly[i][j].sety( poly[i][j].y() + lat );
        }
    }
}


// output
void TGPolygon::write( const std::string& file ) const {
    FILE *fp = fopen( file.c_str(), "w" );

    fprintf(fp, "%ld\n", poly.size());
    for ( int i = 0; i < (int)poly.size(); ++i ) {
        fprintf(fp, "%ld\n", poly[i].size());
        for ( int j = 0; j < (int)poly[i].size(); ++j ) {
            fprintf(fp, "%.6f %.6f\n", poly[i][j].x(), poly[i][j].y());
        }
        fprintf(fp, "%.6f %.6f\n", poly[i][0].x(), poly[i][0].y());
    }

    fclose(fp);
}


// Move slivers from in polygon to out polygon.
void tgPolygonFindSlivers( TGPolygon& in, poly_list& slivers ) 
{
    // traverse each contour of the polygon and attempt to identify
    // likely slivers

    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonFindSlivers()");

    TGPolygon out;
    int i;

    out.erase();

    double angle_cutoff = 10.0 * SGD_DEGREES_TO_RADIANS;
    double area_cutoff = 0.000000001;
    double min_angle;
    double area;

    point_list contour;
    int hole_flag;

    // process contours in reverse order so deleting a contour doesn't
    // foul up our sequence
    for ( i = in.contours() - 1; i >= 0; --i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "contour " << i );

        min_angle = in.minangle_contour( i );
        area = in.area_contour( i );

        SG_LOG(SG_GENERAL, SG_DEBUG, "  min_angle (rad) = " << min_angle );
        SG_LOG(SG_GENERAL, SG_DEBUG, "  min_angle (deg) = " << min_angle * 180.0 / SGD_PI );
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

            // check if this is a hole
            hole_flag = in.get_hole_flag( i );

            if ( hole_flag ) {
                // just delete/eliminate/remove sliver holes
                // cout << "just deleting a sliver hole" << endl;
                in.delete_contour( i );
            } else {
                // move sliver contour to out polygon
                SG_LOG(SG_GENERAL, SG_INFO, "      Found SLIVER!");

                contour = in.get_contour( i );
                in.delete_contour( i );
                out.add_contour( contour, hole_flag );
            }
        }
    }

    if ( out.contours() )
    {
        slivers.push_back( out );
    }
}



// output
void TGPolygon::write_contour( const int contour, const std::string& file ) const {
    FILE *fp = fopen( file.c_str(), "w" );
    
    for ( int j = 0; j < (int)poly[contour].size(); ++j ) {
	fprintf(fp, "%.6f %.6f\n", poly[contour][j].x(), poly[contour][j].y());
    }

    fclose(fp);
}

// Set operation type
typedef enum {
    POLY_DIFF,			// Difference
    POLY_INT,			// Intersection
    POLY_XOR,			// Exclusive or
    POLY_UNION			// Union
} clip_op;


#define FIXEDPT (10000000000000000)
#define FIXED1M (            90090)


static ClipperLib::IntPoint MakeClipperPoint( Point3D pt )
{
	ClipperLib::long64 x, y;

	x = (ClipperLib::long64)( pt.x() * FIXEDPT );
	y = (ClipperLib::long64)( pt.y() * FIXEDPT );

	return ClipperLib::IntPoint( x, y );
}

static Point3D MakeTGPoint( ClipperLib::IntPoint pt )
{
    Point3D tg_pt;
	double x, y;

	x = (double)( ((double)pt.X) / (double)FIXEDPT );
	y = (double)( ((double)pt.Y) / (double)FIXEDPT );

    tg_pt = Point3D( x, y, -9999.0f);

	return tg_pt;
}

static SGGeod MakeSGGeod( ClipperLib::IntPoint pt )
{
    double x, y;
    x = (double)( ((double)pt.X) / (double)FIXEDPT );
    y = (double)( ((double)pt.Y) / (double)FIXEDPT );

    return SGGeod::fromDeg(x, y);
}

double MakeClipperDelta( double mDelta )
{
    double cDelta = mDelta * ( FIXEDPT / FIXED1M );

    // SG_LOG(SG_GENERAL, SG_INFO, "mdelta:" << mDelta << " is " << cDelta );
    
    return( cDelta );
}

static void make_clipper_poly( const TGPolygon& in, ClipperLib::Polygons *out ) 
{
    ClipperLib::Polygon contour;
    Point3D  p;
    int      i, j;	

    for (i=0; i<in.contours(); i++) {
        // create a clipper contour
        contour.clear();
	    for (j=0; j<in.contour_size(i); ++j) 
        {
            p = in.get_pt( i, j );
            contour.push_back(MakeClipperPoint(p));
        }

        if ( in.get_hole_flag( i ) )
        {
            // holes need to be orientation: false 
            if ( Orientation( contour ) ) {
                //SG_LOG(SG_GENERAL, SG_INFO, "Building clipper poly - hole contour needs to be reversed" );
                ReversePolygon( contour );
            }
        } else {
            // boundaries need to be orientation: true
            if ( !Orientation( contour ) ) {
                //SG_LOG(SG_GENERAL, SG_INFO, "Building clipper poly - boundary contour needs to be reversed" );
                ReversePolygon( contour );
            }
        }
        out->push_back(contour);
    }
}

void make_tg_poly_from_clipper( const ClipperLib::Polygons& in, TGPolygon *out )
{
	out->erase();

    // for each polygon, we need to check the orientation, to set the hole flag...
    for (unsigned int i=0; i<in.size(); i++)
    {
        ClipperLib::IntPoint ip;

        for (unsigned int j = 0; j < in[i].size(); j++)
        {
            ip = ClipperLib::IntPoint( in[i][j].X, in[i][j].Y );
            //SG_LOG(SG_GENERAL, SG_INFO, "Building TG Poly : Add point (" << ip.X << "," << ip.Y << ") to contour " << i );
            out->add_node( i, MakeTGPoint(ip) );
        }

        if ( Orientation( in[i] ) ) {
            //SG_LOG(SG_GENERAL, SG_INFO, "Building TG Poly : contour " << i << " is boundary " );
            out->set_hole_flag(i, 0);
        } else {
            //SG_LOG(SG_GENERAL, SG_INFO, "Building TG Poly : contour " << i << " is hole " );
            out->set_hole_flag(i, 1);
        }
    }
}


void get_Clipper_bounding_box( const ClipperLib::Polygons& in, SGGeod& min, SGGeod& max )
{
    ClipperLib::IntPoint min_pt, max_pt;

    min_pt.X = min_pt.Y = LONG_LONG_MAX;
    max_pt.X = max_pt.Y = LONG_LONG_MIN;

    // for each polygon, we need to check the orientation, to set the hole flag...
    for (unsigned int i=0; i<in.size(); i++)
    {
        for (unsigned int j = 0; j < in[i].size(); j++)
        {
            if ( in[i][j].X < min_pt.X ) {
                min_pt.X = in[i][j].X;
            }
            if ( in[i][j].Y < min_pt.Y ) {
                min_pt.Y = in[i][j].Y;
            }

            if ( in[i][j].X > max_pt.X ) {
                max_pt.X = in[i][j].X;
            }
            if ( in[i][j].Y > max_pt.Y ) {
                max_pt.Y = in[i][j].Y;
            }
        }
    }

    min = MakeSGGeod( min_pt );
    max = MakeSGGeod( max_pt );
}

void clipper_to_shapefile( ClipperLib::Polygons polys, char* ds )
{
    ClipperLib::Polygons contour;
    TGPolygon tgcontour;
    char layer[32];

    void*       ds_id = tgShapefileOpenDatasource( ds );

    for (unsigned int i = 0; i < polys.size(); ++i) {
        if  ( Orientation( polys[i] ) ) {
            sprintf( layer, "%04d_boundary", i );
        } else {
            sprintf( layer, "%04d_hole", i );
        }

        void* l_id  = tgShapefileOpenLayer( ds_id, layer );
        contour.clear();
        contour.push_back( polys[i] );

        tgcontour.erase();
        make_tg_poly_from_clipper( contour, &tgcontour );

        tgShapefileCreateFeature( ds_id, l_id, tgcontour, "contour" );
    }

    // close after each write
    ds_id = tgShapefileCloseDatasource( ds_id );
}

TGPolygon polygon_clip_clipper( clip_op poly_op, const TGPolygon& subject, const TGPolygon& clip )
{
    TGPolygon result;

    ClipperLib::Polygons clipper_subject;
    make_clipper_poly( subject, &clipper_subject );

    ClipperLib::Polygons clipper_clip;
    make_clipper_poly( clip, &clipper_clip );

    ClipperLib::Polygons clipper_result;

    ClipperLib::ClipType op;
    if ( poly_op == POLY_DIFF ) {
        op = ClipperLib::ctDifference;
    } else if ( poly_op == POLY_INT ) {
        op = ClipperLib::ctIntersection;
    } else if ( poly_op == POLY_XOR ) {
        op = ClipperLib::ctXor;
    } else if ( poly_op == POLY_UNION ) {
        op = ClipperLib::ctUnion;
    } else {
        throw sg_exception("Unknown polygon op, exiting.");
    }

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);

    c.Execute(op, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    make_tg_poly_from_clipper( clipper_result, &result );

    return result;
}


// Difference
TGPolygon tgPolygonDiff( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip_clipper( POLY_DIFF, subject, clip );
}

// Intersection
TGPolygon tgPolygonInt( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip_clipper( POLY_INT, subject, clip );
}

// Exclusive or
TGPolygon tgPolygonXor( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip_clipper( POLY_XOR, subject, clip );
}

// Union
TGPolygon tgPolygonUnion( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip_clipper( POLY_UNION, subject, clip );
}

TGPolygon tgPolygonUnion( const poly_list& clips )
{
    ClipperLib::Polygons clipper_result;
    ClipperLib::Clipper c;
    TGPolygon result;

    c.Clear();
    for (unsigned int i=0; i<clips.size(); i++) {
        ClipperLib::Polygons clipper_clip;
        make_clipper_poly( clips[i], &clipper_clip );
        c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    }
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    make_tg_poly_from_clipper( clipper_result, &result );

    return result;
}



// Accumulator optimization ( to keep from massive data copies and format changes
// Start out the accumulator as a list of Polygons - so we can bounding box check
// new additions
typedef std::vector < ClipperLib::Polygons > ClipperPolysList;
ClipperPolysList clipper_accumulator;

void tgPolygonInitClipperAccumulator( void )
{
    clipper_accumulator.clear();
}

void tgPolygonFreeClipperAccumulator( void )
{
    clipper_accumulator.clear();
}

void tgPolygonDumpAccumulator( char* ds, char* layer, char* name )
{
    void* ds_id = tgShapefileOpenDatasource( ds );
    void* l_id  = tgShapefileOpenLayer( ds_id, layer );
    TGPolygon accum;

    for (unsigned int i=0; i<clipper_accumulator.size(); i++) {
        make_tg_poly_from_clipper( clipper_accumulator[i], &accum );
        tgShapefileCreateFeature( ds_id, l_id, accum, name );
    }

    // close after each write
    ds_id = tgShapefileCloseDatasource( ds_id );
}

void tgPolygonAddToClipperAccumulator( const TGPolygon& subject, bool dump )
{
    ClipperLib::Polygons clipper_subject;
    make_clipper_poly( subject, &clipper_subject );

    clipper_accumulator.push_back( clipper_subject );
}

TGPolygon tgPolygonDiffClipperWithAccumulator( const TGPolygon& subject )
{
    TGPolygon result;
    SGGeod min, max, minp, maxp;
    unsigned int num_hits = 0;

    ClipperLib::Polygons clipper_subject;
    make_clipper_poly( subject, &clipper_subject );

    // Start with full poly
    result = subject;
    result.get_bounding_box(minp, maxp);
    tg::Rectangle box1(minp, maxp);

    ClipperLib::Clipper c;
    c.Clear();

    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);

    // clip result against all polygons in the accum that intersect our bb
    for (unsigned int i=0; i < clipper_accumulator.size(); i++) {
        get_Clipper_bounding_box( clipper_accumulator[i], min, max);
        tg::Rectangle box2(min, max);

        if ( box2.intersects(box1) )
        {
            c.AddPolygons(clipper_accumulator[i], ClipperLib::ptClip);
            num_hits++;
        }
    }

    if (num_hits) {
        ClipperLib::Polygons clipper_result;
        
        if ( !c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE" );
            exit(-1);
        }

        make_tg_poly_from_clipper( clipper_result, &result );
    }

    return result;
}

// CLIPPER
TGPolygon tgPolygonDiffClipper( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip_clipper( POLY_DIFF, subject, clip );
}

TGPolygon tgPolygonIntClipper( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip_clipper( POLY_INT, subject, clip );
}

TGPolygon tgPolygonUnionClipper( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip_clipper( POLY_UNION, subject, clip );
}

void tgPolygonDumpClipper(const TGPolygon &poly, char* file)
{
    ClipperLib::Polygons clipper_subject;
    make_clipper_poly( poly, &clipper_subject );

    SG_LOG(SG_GENERAL, SG_ALERT, "DUMP POLY" );
    SG_LOG(SG_GENERAL, SG_ALERT, clipper_subject );
    SG_LOG(SG_GENERAL, SG_ALERT, "\n" );
}

// canonify the polygon winding, outer contour must be anti-clockwise,
// all inner contours must be clockwise.
TGPolygon polygon_canonify( const TGPolygon& in_poly ) {
    TGPolygon result;
    result.erase();

    // Negative areas indicate counter clockwise winding.  Postitive
    // areas indicate clockwise winding.

    int non_hole_count = 0;

    for ( int i = 0; i < in_poly.contours(); ++i ) {
	point_list contour = in_poly.get_contour( i );
	int hole_flag = in_poly.get_hole_flag( i );
	if ( !hole_flag ) {
	    non_hole_count++;
	    if ( non_hole_count > 1 )
	      throw sg_exception("ERROR: polygon with more than one enclosing contour");
	}
	double area = in_poly.area_contour( i );
	if ( hole_flag && (area < 0) ) {
	    // reverse contour
	    point_list rcontour;
	    rcontour.clear();
	    for ( int j = (int)contour.size() - 1; j >= 0; --j ) {
		rcontour.push_back( contour[j] );
	    }
	    result.add_contour( rcontour, hole_flag );
	} else if ( !hole_flag && (area > 0) ) {
	    // reverse contour
	    point_list rcontour;
	    rcontour.clear();
	    for ( int j = (int)contour.size() - 1; j >= 0; --j ) {
		rcontour.push_back( contour[j] );
	    }
	    result.add_contour( rcontour, hole_flag );
	} else {
	    result.add_contour( contour, hole_flag );
	}
    }

    return result;
}


// Traverse a polygon and split edges until they are less than max_len
// (specified in meters)
TGPolygon tgPolygonSplitLongEdges( const TGPolygon &poly, double max_len ) {
    TGPolygon result;
    Point3D p0, p1;
    int i, j, k;

    SG_LOG(SG_GENERAL, SG_DEBUG, "split_long_edges()");

    for ( i = 0; i < poly.contours(); ++i ) {
	SG_LOG(SG_GENERAL, SG_DEBUG, "contour = " << i);
	for ( j = 0; j < poly.contour_size(i) - 1; ++j ) {
	    SG_LOG(SG_GENERAL, SG_DEBUG, "point = " << j);
	    p0 = poly.get_pt( i, j );
	    p1 = poly.get_pt( i, j + 1 );
	    SG_LOG(SG_GENERAL, SG_DEBUG, " " << p0 << "  -  " << p1);

	    if ( fabs(p0.y()) < (90.0 - SG_EPSILON) 
		 || fabs(p1.y()) < (90.0 - SG_EPSILON) )
	    {
	      double az1, az2, s;
	      geo_inverse_wgs_84( 0.0,
				  p0.y(), p0.x(), p1.y(), p1.x(),
				  &az1, &az2, &s );
	      SG_LOG(SG_GENERAL, SG_DEBUG, "distance = " << s);

	      if ( s > max_len ) {
		int segments = (int)(s / max_len) + 1;
		SG_LOG(SG_GENERAL, SG_DEBUG, "segments = " << segments);

		double dx = (p1.x() - p0.x()) / segments;
		double dy = (p1.y() - p0.y()) / segments;

		for ( k = 0; k < segments; ++k ) {
		    Point3D tmp( p0.x() + dx * k, p0.y() + dy * k, 0.0 );
		    SG_LOG(SG_GENERAL, SG_DEBUG, tmp);
		    result.add_node( i, tmp );
		}
	      } else {
		SG_LOG(SG_GENERAL, SG_DEBUG, p0);
		result.add_node( i, p0 );
	      }
	    } else {
	      SG_LOG(SG_GENERAL, SG_DEBUG, p0);
	      result.add_node( i, p0 );
	    }
		
	    // end of segment is beginning of next segment
	}
	p0 = poly.get_pt( i, poly.contour_size(i) - 1 );
	p1 = poly.get_pt( i, 0 );

	double az1, az2, s;
	geo_inverse_wgs_84( 0.0,
			    p0.y(), p0.x(), p1.y(), p1.x(),
			    &az1, &az2, &s );
	SG_LOG(SG_GENERAL, SG_DEBUG, "distance = " << s);

	if ( s > max_len ) {
	    int segments = (int)(s / max_len) + 1;
	    SG_LOG(SG_GENERAL, SG_DEBUG, "segments = " << segments);
	    
	    double dx = (p1.x() - p0.x()) / segments;
	    double dy = (p1.y() - p0.y()) / segments;

	    for ( k = 0; k < segments; ++k ) {
		Point3D tmp( p0.x() + dx * k, p0.y() + dy * k, 0.0 );
		SG_LOG(SG_GENERAL, SG_DEBUG, tmp);
		result.add_node( i, tmp );
	    }
	} else {
	    SG_LOG(SG_GENERAL, SG_DEBUG, p0);
	    result.add_node( i, p0 );
	}

	// maintain original hole flag setting
	result.set_hole_flag( i, poly.get_hole_flag( i ) );
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "split_long_edges() complete");

    return result;
}


// Traverse a polygon and return the union of all the non-hole contours
TGPolygon tgPolygonStripHoles( const TGPolygon &poly ) {
    TGPolygon result; result.erase();

    SG_LOG(SG_GENERAL, SG_DEBUG, "strip_out_holes()");

    for ( int i = 0; i < poly.contours(); ++i ) {
	// SG_LOG(SG_GENERAL, SG_DEBUG, "contour = " << i);
        point_list contour = poly.get_contour( i );
        if ( ! poly.get_hole_flag(i) ) {
            TGPolygon tmp;
            tmp.add_contour( contour, poly.get_hole_flag(i) );
            result = tgPolygonUnion( tmp, result );
        }
    }

    return result;
}

void PrintClipperPoly( ClipperLib::Polygons polys )
{
    int nContours = polys.size();

    SG_LOG(SG_GENERAL, SG_INFO, "CLIPPER POLY : contours " << nContours );

    for (int i = 0; i < nContours; i++) {
    	int nPoints = polys[i].size();
        SG_LOG(SG_GENERAL, SG_INFO, nPoints );
    	
    	for (int j = 0; j < nPoints; j++) {
            SG_LOG(SG_GENERAL, SG_INFO, "(" << polys[i][j].X << "," << polys[i][j].Y << ")" );
	    }
    }
}

TGPolygon tgPolygonExpand(const TGPolygon &poly, double delta)
{
    TGPolygon result;

    ClipperLib::Polygons clipper_src, clipper_dst;
    
    make_clipper_poly( poly, &clipper_src );

    //SG_LOG(SG_GENERAL, SG_INFO, "Clipper Source" );
    //PrintClipperPoly( clipper_src );

    // convert delta from meters to clipper units
    OffsetPolygons( clipper_src, clipper_dst, MakeClipperDelta(delta) );

    //SG_LOG(SG_GENERAL, SG_INFO, "Clipper Dest" );
    //PrintClipperPoly( clipper_dst );

	make_tg_poly_from_clipper( clipper_dst, &result );

    return result;
}

TGPolygon tgPolygonSimplify(const TGPolygon &poly)
{
    TGPolygon result;
    ClipperLib::Polygons clipper_poly;
    
    make_clipper_poly( poly, &clipper_poly );

    SimplifyPolygons(clipper_poly);

	make_tg_poly_from_clipper( clipper_poly, &result );

    return result;
}


// Send a polygon to standard output.
std::ostream& operator << (std::ostream &output, const TGPolygon &poly)
{
    int  nContours = poly.contours();

    // Save the number of contours
    output << nContours << "\n";
    for (int i = 0; i < nContours; i++) {
        int nPoints = poly.contour_size(i);

        // Save number of points in the contour
        output << nPoints << "\n";

        // Then save the points
        for ( int j = 0; j < nPoints; j++ ) {
            output << poly.get_pt(i, j).x() << " ";
            output << poly.get_pt(i, j).y() << " ";
            output << poly.get_pt(i, j).z() << "\n";
        }

        // Then save contour hole flag
        output << poly.get_hole_flag(i) << "\n";
    }

    return output;
}

void TGPolygon::SaveToGzFile(gzFile& fp)
{
    int  nContours = poly.size();

    // Save the number of contours
    sgWriteInt( fp, nContours );
    for (int i = 0; i < nContours; i++) {
        int nPoints = poly[i].size();

        // Save number of points in the contour
        sgWriteInt( fp, nPoints );

        // Then save the points
        for ( int j = 0; j < nPoints; j++ ) {
            sgWritePoint3D( fp, poly[i][j] );
        }

        sgWriteInt( fp, hole_list[i] );
    }
}
// Read a polygon from input buffer.
std::istream& operator >> (std::istream &input, TGPolygon &poly)
{
    int    nContours;
    double x, y, z;

    // Read the number of contours
    input >> nContours;
    for (int i = 0; i < nContours; i++) {
        int nPoints;
        int hole;

        // Read number of points in the contour
        input >> nPoints;

        // Then read the points
        for ( int j = 0; j < nPoints; j++ ) {
            input >> x;
            input >> y;
            input >> z;

            poly.add_node(i, Point3D(x,y,z));
        }

        // Then read contour hole flag
        input >> hole;
        poly.set_hole_flag(i, hole);
    }

    return input;
}

void TGPolygon::LoadFromGzFile(gzFile& fp)
{
    int nContours;
    int nPoints;
    int hole;
    Point3D pt;

    // Save the number of contours
    sgReadInt( fp, &nContours );
    for (int i = 0; i < nContours; i++) {
        sgReadInt( fp, &nPoints );

        // Then read the points
        for ( int j = 0; j < nPoints; j++ ) {
            sgReadPoint3D( fp, pt );
            add_node( i, pt );
        }

        sgReadInt( fp, &hole );
        set_hole_flag( i, hole );
    }
}







/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NEW IMPLEMENTATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
                           const point_list& nodes, SGGeod& result,
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
            SGGeod current = nodes[i].toSGGeod();

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
            SGGeod current = nodes[i].toSGGeod();

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

void AddIntermediateNodes( const SGGeod& p0, const SGGeod& p1, point_list& tmp_nodes, tgContour& result, double bbEpsilon, double errEpsilon )
{
    SGGeod new_pt;

    SG_LOG(SG_GENERAL, SG_BULK, "   " << p0 << " <==> " << p1 );

    bool found_extra = FindIntermediateNode( p0, p1, tmp_nodes, new_pt, bbEpsilon, errEpsilon );

    if ( found_extra ) {
        AddIntermediateNodes( p0, new_pt, tmp_nodes, result, bbEpsilon, errEpsilon  );

        result.AddNode( new_pt );
        SG_LOG(SG_GENERAL, SG_BULK, "    adding = " << new_pt);

        AddIntermediateNodes( new_pt, p1, tmp_nodes, result, bbEpsilon, errEpsilon  );
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

static tg::Rectangle BoundingBox_FromClipper( const ClipperLib::Polygons& subject )
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

    return tg::Rectangle( min, max );
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

tg::Rectangle tgContour::GetBoundingBox( void ) const
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

    return tg::Rectangle( min, max );
}

tgPolygon tgContour::DiffWithAccumulator( const tgContour& subject )
{
    tgPolygon result;

    unsigned int  num_hits = 0;
    tg::Rectangle box1 = subject.GetBoundingBox();

    ClipperLib::Polygon clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();

    c.AddPolygon(clipper_subject, ClipperLib::ptSubject);

    // clip result against all polygons in the accum that intersect our bb
    for (unsigned int i=0; i < clipper_accumulator.size(); i++) {
        tg::Rectangle box2 = BoundingBox_FromClipper( clipper_accumulator[i] );

        if ( box2.intersects(box1) )
        {
            c.AddPolygons(clipper_accumulator[i], ClipperLib::ptClip);
            num_hits++;
        }
    }

    if (num_hits) {
        if ( !c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE" );
            exit(-1);
        }
        result = tgPolygon::FromClipper( clipper_result );
    } else {
        result.AddContour( subject );
    }

    return result;
}

void tgContour::AddToAccumulator( const tgContour& subject )
{
    tgPolygon poly;
    poly.AddContour( subject );

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( poly );
    clipper_accumulator.push_back( clipper_subject );
}

tgPolygon tgPolygon::Union( const tgContour& subject, tgPolygon& clip )
{
    tgPolygon result;

    ClipperLib::Polygon  clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygon(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    return tgPolygon::FromClipper( clipper_result );
}

tgContour tgContour::AddColinearNodes( const tgContour& subject, TGTriNodes nodes )
{
    SGGeod p0, p1;
    tgContour result;
    point_list tmp_nodes = nodes.get_node_list();

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
    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

tgPolygon tgPolygon::Simplify( const tgPolygon& subject )
{
    tgPolygon result;

    ClipperLib::Polygons clipper_poly = tgPolygon::ToClipper( subject );
    SimplifyPolygons( clipper_poly );

    result = tgPolygon::FromClipper( clipper_poly );
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
            SG_LOG(SG_GENERAL, SG_INFO, "remove_tiny_contours " << c << " area is " << area << ": removing");
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

    return result;
}

tgPolygon tgPolygon::Union( const tgPolygon& subject, tgPolygon& clip )
{
    tgPolygon result;

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    return tgPolygon::FromClipper( clipper_result );
}

tgPolygon tgPolygon::Diff( const tgPolygon& subject, tgPolygon& clip )
{
    tgPolygon result;

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    return tgPolygon::FromClipper( clipper_result );
}

tgPolygon tgPolygon::Intersect( const tgPolygon& subject, const tgPolygon& clip )
{
    tgPolygon result;

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctIntersection, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

tg::Rectangle tgPolygon::GetBoundingBox( void ) const
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

    return tg::Rectangle( min, max );
}

void clipperToShapefile( ClipperLib::Polygons polys, const std::string& path, const std::string&layer, const std::string& name )
{
    tgPolygon poly = tgPolygon::FromClipper(polys);
    tgPolygon::ToShapefile( poly, path, layer, name);
}

void tgPolygon::AccumulatorToShapefiles( const std::string& path, const std::string& layer_prefix )
{
    char shapefile[16];
    char layer[16];

    for (unsigned int i=0; i < clipper_accumulator.size(); i++) {
        sprintf( layer, "%s_%d", layer_prefix.c_str(), i );
        sprintf( shapefile, "accum_%d", i );
        clipperToShapefile( clipper_accumulator[i], path, layer, std::string(shapefile) );
    }
}

tgPolygon tgPolygon::DiffWithAccumulator( const tgPolygon& subject )
{
    tgPolygon result;

    unsigned int  num_hits = 0;
    tg::Rectangle box1 = subject.GetBoundingBox();

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();

    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);

    // clip result against all polygons in the accum that intersect our bb
    for (unsigned int i=0; i < clipper_accumulator.size(); i++) {
        tg::Rectangle box2 = BoundingBox_FromClipper( clipper_accumulator[i] );

        if ( box2.intersects(box1) )
        {
            c.AddPolygons(clipper_accumulator[i], ClipperLib::ptClip);
            num_hits++;
        }
    }

    if (num_hits) {
        if ( !c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE" );
            exit(-1);
        }
        SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator had " << num_hits << " hits " );
        SG_LOG(SG_GENERAL, SG_ALERT, "  cklipper_result has " << clipper_result.size() << " contours " );
        
        result = tgPolygon::FromClipper( clipper_result );

        // Make sure we keep texturing info
        result.SetMaterial( subject.GetMaterial() );
        result.SetTexParams( subject.GetTexParams() );
    } else {
        result = subject;
    }

    return result;
}

void tgPolygon::AddToAccumulator( const tgPolygon& subject )
{
    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    clipper_accumulator.push_back( clipper_subject );
}

// Move slivers from in polygon to out polygon.
void tgPolygon::RemoveSlivers( tgPolygon& subject, tgcontour_list& slivers )
{
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
                SG_LOG(SG_GENERAL, SG_INFO, "      Found SLIVER!");

                slivers.push_back( contour );
            }
        }
    }
}

void tgPolygon::MergeSlivers( tgpolygon_list& polys, tgcontour_list& sliver_list ) {
    tgPolygon poly, result;
    tgContour sliver;
    tgContour contour;
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
        }
    }
}

tgPolygon tgPolygon::AddColinearNodes( const tgPolygon& subject, TGTriNodes& nodes )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::AddColinearNodes( subject.GetContour(c), nodes ) );
    }

    return result;
}

void tgPolygon::InheritElevations( const tgPolygon& source )
{
    TGTriNodes nodes;
    nodes.clear();

    // build a list of points from the source and dest polygons
    for ( unsigned int i = 0; i < source.Contours(); ++i ) {
        for ( unsigned int j = 0; j < source.ContourSize(i); ++j ) {
            Point3D p = Point3D::fromSGGeod( source.GetNode( i, j ) );
            nodes.unique_add( p );
        }
    }

    // traverse the dest polygon and build a mirror image but with
    // elevations from the source polygon
    for ( unsigned int i = 0; i < contours.size(); ++i ) {
        for ( unsigned int j = 0; j < contours[i].GetSize(); ++j ) {
            Point3D p = Point3D::fromSGGeod( GetNode(i,j) );
            int index = nodes.find( p );
            if ( index >= 0 ) {
                Point3D ref = nodes.get_node( index );
                SetNode( i, j, SGGeod::fromDegM( p.x(), p.y(), ref.z() ) );
            }
        }
    }

    // now post process result to catch any nodes that weren't updated
    // (because the clipping process may have added points which
    // weren't in the original.)
    double last = -9999.0;
    for ( unsigned int i = 0; i < contours.size(); ++i ) {
        // go front ways
        last = -9999.0;
        for ( unsigned int j = 0; j < contours[i].GetSize(); ++j ) {
            Point3D p = Point3D::fromSGGeod( GetNode(i,j) );
            if ( p.z() > -9000 ) {
                last = p.z();
            } else {
                if ( last > -9000 ) {
                    GetContour(i).SetNode( j, SGGeod::fromDegM( p.x(), p.y(), last ) );
                }
            }
        }

        // go back ways
        last = -9999.0;
        for ( unsigned int j = contours[i].GetSize()-1; j > 0; --j ) {
            Point3D p = Point3D::fromSGGeod( GetNode(i,j) );
            if ( p.z() > -9000 ) {
                last = p.z();
            } else {
                if ( last > -9000 ) {
                    GetContour(i).SetNode( j, SGGeod::fromDegM( p.x(), p.y(), last ) );
                }
            }
        }
    }
}

void tgPolygon::Texture( void )
{
    SGGeod  p;
    SGVec2d t;
    double  x, y;
    double  tx, ty;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Texture Poly with material " << material << " method " << tp.method << " tpref " << tp.ref << " heading " << tp.heading );

    switch( tp.method ) {
        case TG_TEX_BY_GEODE:
            break;

        case TG_TEX_BY_TPS_NOCLIP:
        case TG_TEX_BY_TPS_CLIPU:
        case TG_TEX_BY_TPS_CLIPV:
        case TG_TEX_BY_TPS_CLIPUV:
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
                    double tmp;

                    tmp = x / tp.width;
                    tx = tmp * (tp.maxu - tp.minu) + tp.minu;
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ")");

                    // clip u?
                    if ( (tp.method == TG_TEX_BY_TPS_CLIPU) || (tp.method == TG_TEX_BY_TPS_CLIPUV) ) {
                        if ( tx < tp.min_clipu ) { tx = tp.min_clipu; }
                        if ( tx > tp.max_clipu ) { tx = tp.max_clipu; }
                    }

                    tmp = y / tp.length;
                    ty = tmp * (tp.maxv - tp.minv) + tp.minv;
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << ty << ")");

                    // clip v?
                    if ( (tp.method == TG_TEX_BY_TPS_CLIPV) || (tp.method == TG_TEX_BY_TPS_CLIPUV) ) {
                        if ( ty < tp.min_clipv ) { ty = tp.min_clipv; }
                        if ( ty > tp.max_clipv ) { ty = tp.max_clipv; }
                    }

                    t = SGVec2d( tx, ty );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ", " << ty << ")");

                    triangles[i].SetTexCoord( j, t );
                }
            }

            break;
    }
}

#include <ogrsf_frmts.h>
void tgPolygon::ToShapefile( const tgPolygon& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void*          ds_id = tgShapefileOpenDatasource( datasource.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefileOpenDatasource returned " << (unsigned long)ds_id);
    
    OGRLayer*      l_id  = (OGRLayer *)tgShapefileOpenLayer( ds_id, layer.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefileOpenLayer returned " << (unsigned long)l_id);

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
    ds_id = tgShapefileCloseDatasource( ds_id );
}


////////////////////////////// CHOP ////////////////////////////////
#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>

// initialize the unique polygon index counter stored in path
static long int poly_index = 0;
static std::string poly_path;

bool tgPolygon_index_init( const std::string& path )
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

static long int tgPolygon_index_next()
{
    ++poly_index;

    FILE *fp = fopen( poly_path.c_str(), "w" );

    if ( fp == NULL ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Error cannot open BLAH BLAG BLAH " << poly_path << " for writing");
    }

    fprintf( fp, "%ld\n", poly_index );
    fclose( fp );

    return poly_index;
}

static void ClipToFile( const tgPolygon& subject, std::string root,
                        long int p_index,
                        const std::string& type,
                        SGBucket& b,
                        bool withTexparams,
                        bool preserve3d )
{
    Point3D p;

    SGGeod min, max;
    SGGeod c    = b.get_center();
    double span = b.get_width();

    tgPolygon base, result;
    char tile_name[256];
    char poly_index[256];

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

    result = tgPolygon::Intersect( base, subject );

    SG_LOG(SG_GENERAL, SG_DEBUG, "result contours = " << result.Contours() );
    for ( unsigned int ii = 0; ii < result.Contours(); ii++ ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  hole = " << result.GetContour(ii).GetHole() );
    }

    if ( preserve3d ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "preserve3d" );
        result.InheritElevations( subject );
        SG_LOG(SG_GENERAL, SG_DEBUG, "preserve3d  - done" );

        SG_LOG(SG_GENERAL, SG_DEBUG, subject );
        SG_LOG(SG_GENERAL, SG_DEBUG, result );
    }

    tgTexParams tp = subject.GetTexParams();

    if ( result.Contours() > 0 ) {
        long int t_index = b.gen_index();
        std::string path = root + "/" + b.gen_base_path();

        SGPath sgp( path );
        sgp.append( "dummy" );
        sgp.create_dir( 0755 );

        sprintf( tile_name, "%ld", t_index );
        std::string polyfile = path + "/" + tile_name;

        sprintf( poly_index, "%ld", p_index );
        polyfile += ".";
        polyfile += poly_index;

        FILE *rfp = fopen( polyfile.c_str(), "w" );
        if ( preserve3d && withTexparams ) {
            fprintf( rfp, "#3D_TP\n" );
        } else if ( withTexparams ) {
            fprintf( rfp, "#2D_TP\n" );
        } else if ( preserve3d ) {
            fprintf( rfp, "#3D\n" );
        } else {
            fprintf( rfp, "#2D\n" );
        }

        fprintf( rfp, "%s\n", type.c_str() );

        if ( withTexparams ) {
            fprintf( rfp, "%.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f\n",
                     tp.ref.getLongitudeDeg(), tp.ref.getLatitudeDeg(),
                     tp.width, tp.length,
                     tp.heading,
                     tp.minu, tp.maxu, tp.minu, tp.maxu);
        }

        fprintf( rfp, "%d\n", result.Contours() );
        for ( unsigned int i = 0; i < result.Contours(); ++i ) {
            fprintf( rfp, "%d\n", result.ContourSize(i) );
            fprintf( rfp, "%d\n", result.GetContour(i).GetHole() );
            for ( unsigned int j = 0; j < result.ContourSize(i); ++j ) {
                p = Point3D::fromSGGeod( result.GetNode( i, j ) );
                if ( preserve3d )
                    fprintf( rfp, "%.15f  %.15f %.15f\n", p.x(), p.y(), p.z() );
                else
                    fprintf( rfp, "%.15f  %.15f\n", p.x(), p.y() );
            }
        }
        fclose( rfp );
    }
}

void tgPolygon::Chop( const tgPolygon& subject, const std::string& path, const std::string& type, bool withTexparams, bool preserve3d )
{
    tg::Rectangle bb;
    SGGeod p;
    long int index;

    // bail out immediately if polygon is empty
    if ( subject.Contours() == 0 )
        return;

    bb = subject.GetBoundingBox();

    // get next polygon index
    index = tgPolygon_index_next();

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
        ClipToFile( subject, path, index, type, b_min, withTexparams, preserve3d );
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
                ClipToFile( subject, path, index, type, b_cur, withTexparams, preserve3d );
            }
        }
        return;
    }

    // we have two or more rows left, split in half (along a
    // horizontal dividing line) and recurse with each half

    // find mid point (integer math)
    int mid = (dy + 1) / 2 - 1;

    // determine horizontal clip line
    SGBucket b_clip = sgBucketOffset( bb.getMin().getLongitudeDeg(), bb.getMin().getLongitudeDeg(), 0, mid);
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

        bottom_clip = tgPolygon::Intersect( bottom, subject );

        // the texparam should be constant over each clipped poly.
        // when they are reassembled, we want the texture map to
        // be seamless
        tgPolygon::Chop( bottom_clip, path, type, withTexparams, preserve3d );
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

        top_clip = tgPolygon::Intersect( top, subject );

        tgPolygon::Chop( top_clip, path, type, withTexparams, preserve3d );
    }
}

/************************ TESSELATION ***********************************/
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangle_2.h>
#include <iostream>

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

void tgPolygon::Tesselate( std::vector<SGGeod> extra )
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
