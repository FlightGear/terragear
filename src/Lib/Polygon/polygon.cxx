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
// $Id: polygon.cxx,v 1.30 2007-11-05 14:02:21 curt Exp $


// include Generic Polygon Clipping Library
//
//    http://www.cs.man.ac.uk/aig/staff/alan/software/
//

#include <iostream>
#include <fstream>

#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <Geometry/point3d.hxx>
#include <Geometry/poly_support.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/structure/exception.hxx>

#include <Geometry/trinodes.hxx>
#include <poly2tri/interface.h>

#include "polygon.hxx"
#include "point2d.hxx"

using std::endl;
using std::cout;

// Constructor 
TGPolygon::TGPolygon( void )
{
}


// Destructor
TGPolygon::~TGPolygon( void ) {
}

void TGPolygon::get_bounding_box( Point3D& min, Point3D& max ) const
{
    double minx = std::numeric_limits<double>::infinity();
    double miny = std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();

    for ( int i = 0; i < contours(); i++ ) {
        for (unsigned int j = 0; j < poly[i].size(); j++) {
            Point3D pt = poly[i][j];
            if ( pt.x() < minx ) { minx = pt.x(); }
            if ( pt.x() > maxx ) { maxx = pt.x(); }
            if ( pt.y() < miny ) { miny = pt.y(); }
            if ( pt.y() > maxy ) { maxy = pt.y(); }
        }
    }

    min = Point3D( minx, miny, 0.0 );
    max = Point3D( maxx, maxy, 0.0 );
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
double tgPolygonCalcAngle(point2d a, point2d b, point2d c) {
    point2d u, v;
    double udist, vdist, uv_dot, tmp;

    // u . v = ||u|| * ||v|| * cos(theta)

    u.x = b.x - a.x;
    u.y = b.y - a.y;
    udist = sqrt( u.x * u.x + u.y * u.y );
    // printf("udist = %.6f\n", udist);

    v.x = b.x - c.x;
    v.y = b.y - c.y;
    vdist = sqrt( v.x * v.x + v.y * v.y );
    // printf("vdist = %.6f\n", vdist);

    uv_dot = u.x * v.x + u.y * v.y;
    // printf("uv_dot = %.6f\n", uv_dot);

    tmp = uv_dot / (udist * vdist);
    // printf("tmp = %.6f\n", tmp);

    return acos(tmp);
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
    point2d p1, p2, p3;
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

	p1.x = c[p1_index].x();
	p1.y = c[p1_index].y();

	p2.x = c[p2_index].x();
	p2.y = c[p2_index].y();

	p3.x = c[p3_index].x();
	p3.y = c[p3_index].y();

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

double MakeClipperDelta( double mDelta )
{
    double cDelta = mDelta * ( FIXEDPT / FIXED1M );

    // SG_LOG(SG_GENERAL, SG_INFO, "mdelta:" << mDelta << " is " << cDelta );
    
    return( cDelta );
}

void make_clipper_poly( const TGPolygon& in, ClipperLib::Polygons *out ) 
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

#if 0
void make_tg_poly_from_clipper_ex( const ClipperLib::ExPolygons& in, TGPolygon *out )
{
	int res_contour = 0;
	out->erase();

  	for (unsigned int i=0; i<in.size(); i++)
    {
      	const struct ClipperLib::ExPolygon* pg = &in[i];
		ClipperLib::IntPoint ip;

		// Get the boundary contour
        for (unsigned int j = 0; j < pg->outer.size(); j++)
        {
			ip = ClipperLib::IntPoint( pg->outer[j].X, pg->outer[j].Y );
       	    out->add_node(res_contour, MakeTGPoint(ip));
        }
        out->set_hole_flag(res_contour, 0);
        res_contour++;

        // then the holes
        for (unsigned int j = 0; j < pg->holes.size(); j++)
        {
            for (unsigned int k = 0; k < pg->holes[j].size(); k++)
            {
				ip = ClipperLib::IntPoint( pg->holes[j].at(k).X, pg->holes[j].at(k).Y );
               	out->add_node(res_contour, MakeTGPoint(ip));
            }
            out->set_hole_flag(res_contour, 1);
            res_contour++;
        }
    }
}
#endif

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

#if 0
ClipperLib::Polygons clipper_simplify( ClipperLib::ExPolygons &in )
{
    ClipperLib::Polygons out;
    ClipperLib::Polygon  contour;

  	for (unsigned int i=0; i<in.size(); i++)
    {
      	const struct ClipperLib::ExPolygon* pg = &in[i];

        // first the boundary
        contour = pg->outer;
        if ( !Orientation( contour ) ) {
            ReversePolygon( contour );
        }
        out.push_back( contour );

        // then the holes
        for (unsigned int j = 0; j < pg->holes.size(); j++)
        {
            contour = pg->holes[j];
            if ( Orientation( contour ) ) {
                ReversePolygon( contour );
            }
            out.push_back( contour );
        }
    }

    // Now simplify
    SimplifyPolygons(out);

    return out;
}
#endif

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




// Accumulator optimization ( to keep from massive data copies and format changes
ClipperLib::Polygons clipper_accumulator;

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

    make_tg_poly_from_clipper( clipper_accumulator, &accum );
    tgShapefileCreateFeature( ds_id, l_id, accum, name );

    // close after each write
    ds_id = tgShapefileCloseDatasource( ds_id );
}

void tgPolygonAddToClipperAccumulator( const TGPolygon& subject, bool dump )
{
    std::ofstream subjectFile, clipFile, resultFile;

    ClipperLib::Polygons clipper_subject;
    make_clipper_poly( subject, &clipper_subject );

    if (dump) {
        subjectFile.open ("subject.txt");
        subjectFile << clipper_subject;
        subjectFile.close();

        clipFile.open ("clip.txt");
        clipFile << clipper_accumulator;
        clipFile.close();
    }

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_accumulator, ClipperLib::ptClip);

    if ( !c.Execute(ClipperLib::ctUnion, clipper_accumulator, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd) ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Add to Accumulator returned FALSE" );
        exit(-1);
    }

    if (dump) {
        resultFile.open ("result.txt");
        resultFile << clipper_accumulator;
        resultFile.close();
    }
}

void clipper_to_shapefile( ClipperLib::Polygons polys, char* ds )
{
    ClipperLib::Polygons contour;
    TGPolygon tgcontour;
    char layer[32];

    void*       ds_id = tgShapefileOpenDatasource( ds );

    for (unsigned int i = 0; i < polys.size(); ++i) {
        if  ( Orientation( polys[i] ) ) {
            sprintf( layer, "%04d_hole", i );
        } else {
            sprintf( layer, "%04d_boundary", i );
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

TGPolygon tgPolygonDiffClipperWithAccumulator( const TGPolygon& subject )
{
    TGPolygon result;

    ClipperLib::Polygons clipper_subject;
    make_clipper_poly( subject, &clipper_subject );
    
    ClipperLib::Polygons clipper_result;
    
    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_accumulator, ClipperLib::ptClip);

    if ( !c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE" );
        exit(-1);
    }

    make_tg_poly_from_clipper( clipper_result, &result );

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

// Read a polygon from input buffer.
std::istream& operator >> (std::istream &input, TGPolygon &poly)
{
    int    nContours = poly.contours();
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
