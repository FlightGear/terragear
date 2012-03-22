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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: polygon.cxx,v 1.30 2007-11-05 14:02:21 curt Exp $


// include Generic Polygon Clipping Library
//
//    http://www.cs.man.ac.uk/aig/staff/alan/software/
//


#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <Geometry/point3d.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/structure/exception.hxx>

#include <Geometry/trinodes.hxx>
#include <poly2tri/interface.h>

#include "polygon.hxx"
#include "point2d.hxx"

using std::endl;

// Constructor 
TGPolygon::TGPolygon( void )
{
}


// Destructor
TGPolygon::~TGPolygon( void ) {
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
    // area = 1/2 * sum[i = 0 to k-1][x(i)*y(i+1) - x(i+1)*y(i)]
    // where i=k is defined as i=0

    point_list c = poly[contour];
    int size = c.size();
    double sum = 0.0;

    for ( int i = 0; i < size; ++i ) {
	sum += c[(i+1)%size].x() * c[i].y() - c[i].x() * c[(i+1)%size].y();
    }

    // area can be negative or positive depending on the polygon
    // winding order
    return fabs(sum / 2.0);
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
void TGPolygon::write( const string& file ) const {
    FILE *fp = fopen( file.c_str(), "w" );

    fprintf(fp, "%d\n", poly.size());
    for ( int i = 0; i < (int)poly.size(); ++i ) {
        fprintf(fp, "%d\n", poly[i].size());
	for ( int j = 0; j < (int)poly[i].size(); ++j ) {
	    fprintf(fp, "%.6f %.6f\n", poly[i][j].x(), poly[i][j].y());
	}
	fprintf(fp, "%.6f %.6f\n", poly[i][0].x(), poly[i][0].y());
    }

    fclose(fp);
}


// output
void TGPolygon::write_contour( const int contour, const string& file ) const {
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

#ifdef CLIP_GPC
//
// wrapper functions for gpc polygon clip routines
//

// Make a gpc_poly from an TGPolygon
void make_gpc_poly( const TGPolygon& in, gpc_polygon *out ) {
    gpc_vertex_list v_list;
    v_list.num_vertices = 0;
    v_list.vertex = new gpc_vertex[FG_MAX_VERTICES];

    Point3D p;
    // build the gpc_polygon structures
    for ( int i = 0; i < in.contours(); ++i ) {
	// SG_LOG(SG_GENERAL, SG_DEBUG, "    contour " << i << " = " << in.contour_size( i ));
	if ( in.contour_size( i ) > FG_MAX_VERTICES ) {
	  char message[128];
	  sprintf(message, "Polygon too large, need to increase FG_MAX_VERTICES to a least %d", in.contour_size(i));
	  throw sg_exception(message);;
	}

#if 0
    SG_LOG(SG_GENERAL, SG_ALERT, 
        "  make_clipper_poly : processing contour " << i << ", nodes = " 
        << in.contour_size(i) << ", hole = "
        << in.get_hole_flag(i) 
    );
#endif

	for ( int j = 0; j < in.contour_size( i ); ++j ) {
	    p = in.get_pt( i, j );
	    v_list.vertex[j].x = p.x();
	    v_list.vertex[j].y = p.y();
	}
	v_list.num_vertices = in.contour_size( i );
	gpc_add_contour( out, &v_list, in.get_hole_flag( i ) );
    }

    // free alocated memory
    delete [] v_list.vertex;
}

void make_tg_poly( const gpc_polygon* in, TGPolygon *out )
{
    for ( int i = 0; i < in->num_contours; ++i ) {
    	for ( int j = 0; j < in->contour[i].num_vertices; j++ ) {
	        Point3D p( in->contour[i].vertex[j].x, in->contour[i].vertex[j].y, -9999.0 );
    	    out->add_node(i, p);
    	}

    	out->set_hole_flag( i, in->hole[i] );
    }
}

// Generic clipping routine
TGPolygon polygon_clip( clip_op poly_op, const TGPolygon& subject, 
			const TGPolygon& clip )
{
    TGPolygon result;

    gpc_polygon *gpc_subject = new gpc_polygon;
    gpc_subject->num_contours = 0;
    gpc_subject->contour = NULL;
    gpc_subject->hole = NULL;
    make_gpc_poly( subject, gpc_subject );

    gpc_polygon *gpc_clip = new gpc_polygon;
    gpc_clip->num_contours = 0;
    gpc_clip->contour = NULL;
    gpc_clip->hole = NULL;
    make_gpc_poly( clip, gpc_clip );

    gpc_polygon *gpc_result = new gpc_polygon;
    gpc_result->num_contours = 0;
    gpc_result->contour = NULL;
    gpc_result->hole = NULL;

    gpc_op op;
    if ( poly_op == POLY_DIFF ) {
    	op = GPC_DIFF;
    } else if ( poly_op == POLY_INT ) {
	    op = GPC_INT;
    } else if ( poly_op == POLY_XOR ) {
    	op = GPC_XOR;
    } else if ( poly_op == POLY_UNION ) {
	    op = GPC_UNION;
    } else {
        throw sg_exception("Unknown polygon op, exiting.");
    }

    gpc_polygon_clip( op, gpc_subject, gpc_clip, gpc_result );

	make_tg_poly( gpc_result, &result );

    // free allocated memory
    gpc_free_polygon( gpc_subject );
    gpc_free_polygon( gpc_clip );
    gpc_free_polygon( gpc_result );

    return result;
}

// Generic clipping routine
TGPolygon polygon_clip( clip_op poly_op, const TGPolygon& subject, const gpc_polygon& clip )
{
    TGPolygon result;

    gpc_polygon *gpc_subject = new gpc_polygon;
    gpc_subject->num_contours = 0;
    gpc_subject->contour = NULL;
    gpc_subject->hole = NULL;
    make_gpc_poly( subject, gpc_subject );

	gpc_polygon *gpc_clip = (gpc_polygon*)&clip;

    gpc_polygon *gpc_result = new gpc_polygon;
    gpc_result->num_contours = 0;
    gpc_result->contour = NULL;
    gpc_result->hole = NULL;

    gpc_op op;
    if ( poly_op == POLY_DIFF ) {
    	op = GPC_DIFF;
    } else if ( poly_op == POLY_INT ) {
	    op = GPC_INT;
    } else if ( poly_op == POLY_XOR ) {
    	op = GPC_XOR;
    } else if ( poly_op == POLY_UNION ) {
	    op = GPC_UNION;
    } else {
        throw sg_exception("Unknown polygon op, exiting.");
    }

    gpc_polygon_clip( op, gpc_subject, gpc_clip, gpc_result );

	make_tg_poly( gpc_result, &result );

    // free allocated memory
    gpc_free_polygon( gpc_subject );
    gpc_free_polygon( gpc_result );

    return result;
}

gpc_polygon polygon_clip_keep_native_fmt( clip_op poly_op, const TGPolygon& subject, const gpc_polygon& clip )
{
    gpc_polygon *gpc_subject = new gpc_polygon;
    gpc_subject->num_contours = 0;
    gpc_subject->contour = NULL;
    gpc_subject->hole = NULL;
    make_gpc_poly( subject, gpc_subject );

	gpc_polygon *gpc_clip = (gpc_polygon*)&clip;

    gpc_polygon result;
    result.num_contours = 0;
    result.contour = NULL;
    result.hole = NULL;

    gpc_op op;
    if ( poly_op == POLY_DIFF ) {
    	op = GPC_DIFF;
    } else if ( poly_op == POLY_INT ) {
	    op = GPC_INT;
    } else if ( poly_op == POLY_XOR ) {
    	op = GPC_XOR;
    } else if ( poly_op == POLY_UNION ) {
	    op = GPC_UNION;
    } else {
        throw sg_exception("Unknown polygon op, exiting.");
    }

    gpc_polygon_clip( op, gpc_subject, gpc_clip, &result );

    // free allocated memory
    gpc_free_polygon( gpc_subject );

	return result;
}
#endif

#ifdef CLIP_CLIPPER

//#define FIXEDPT (10000000000000)
#define FIXEDPT (10000000000000000)

IntPoint MakeClipperPoint( Point3D pt )
{
	long64 x, y;

	x = (long64)( pt.x() * FIXEDPT );
	y = (long64)( pt.y() * FIXEDPT );

	return IntPoint( x, y );
}

Point3D MakeTGPoint( IntPoint pt )
{
	double x, y;

	x = (double)( ((double)pt.X) / (double)FIXEDPT );
	y = (double)( ((double)pt.Y) / (double)FIXEDPT );

	return Point3D( x, y, -9999.0f);
}

void make_clipper_poly( const TGPolygon& in, Polygons *out ) 
{
    Polygon  contour;
    Point3D  p;
    int      i, j;	

    if (in.contours())
    {
        // assume contour 0 is boundary, 1..x are holes
        // create the boundary
        for (j=0; j<in.contour_size(0); ++j)
        {
	        p = in.get_pt( 0, j );
            contour.push_back(MakeClipperPoint(p));
        }
        out->push_back(contour);

        // create the holes
        for (i=1; i<in.contours(); ++i ) 
        {        
            contour.clear();
	        for (j=0; j<in.contour_size(i); ++j) 
            {
        	    p = in.get_pt( i, j );
                contour.push_back(MakeClipperPoint(p));
            }
            out->push_back(contour);
	    }
    }
}

void make_tg_poly( const ExPolygons& in, TGPolygon *out )
{
	int res_contour = 0;
	out->erase();

  	for (int i=0; i<in.size(); i++)
    {
      	const struct ExPolygon* pg = &in[i];
		IntPoint ip;

		// Get the boundary contour
        for (int j = 0; j < pg->outer.size(); j++)
        {
			ip = IntPoint( pg->outer[j].X, pg->outer[j].Y );
       	    out->add_node(res_contour, MakeTGPoint(ip));
        }
        out->set_hole_flag(res_contour, 0);
        res_contour++;

        // then the holes
        for (int j = 0; j < pg->holes.size(); j++)
        {
            for (int k = 0; k < pg->holes[j].size(); k++)
            {
				ip = IntPoint( pg->holes[j].at(k).X, pg->holes[j].at(k).Y );
               	out->add_node(res_contour, MakeTGPoint(ip));
            }
            out->set_hole_flag(res_contour, 1);
            res_contour++;
        }
    }
}

TGPolygon polygon_clip( clip_op poly_op, const TGPolygon& subject, const TGPolygon& clip )
{
    TGPolygon result;

    Polygons clipper_subject;
    make_clipper_poly( subject, &clipper_subject );

    Polygons clipper_clip;
    make_clipper_poly( clip, &clipper_clip );

    ExPolygons clipper_result;

    ClipType op;
    if ( poly_op == POLY_DIFF ) {
	    op = ctDifference;
    } else if ( poly_op == POLY_INT ) {
	    op = ctIntersection;
    } else if ( poly_op == POLY_XOR ) {
	    op = ctXor;
    } else if ( poly_op == POLY_UNION ) {
    	op = ctUnion;
    } else {
        throw sg_exception("Unknown polygon op, exiting.");
    }

    Clipper c;
	c.Clear();
	c.AddPolygons(clipper_subject, ptSubject);
	c.AddPolygons(clipper_clip, ptClip);

	c.Execute(op, clipper_result, pftEvenOdd, pftEvenOdd);
	make_tg_poly( clipper_result, &result );

	return result;
}

TGPolygon polygon_clip( clip_op poly_op, const TGPolygon& subject, const Polygons& clipper_clip )
{
    TGPolygon result;

    Polygons clipper_subject;
    make_clipper_poly( subject, &clipper_subject );

    ExPolygons clipper_result;

    ClipType op;
    if ( poly_op == POLY_DIFF ) {
	    op = ctDifference;
    } else if ( poly_op == POLY_INT ) {
	    op = ctIntersection;
    } else if ( poly_op == POLY_XOR ) {
	    op = ctXor;
    } else if ( poly_op == POLY_UNION ) {
    	op = ctUnion;
    } else {
        throw sg_exception("Unknown polygon op, exiting.");
    }

    Clipper c;
	c.Clear();
	c.AddPolygons(clipper_subject, ptSubject);
	c.AddPolygons(clipper_clip, ptClip);

	c.Execute(op, clipper_result, pftEvenOdd, pftEvenOdd);
	make_tg_poly( clipper_result, &result );

	return result;
}

Polygons polygon_clip_keep_native_fmt( clip_op poly_op, const TGPolygon& subject, const Polygons& clipper_clip )
{
    Polygons clipper_subject;
    make_clipper_poly( subject, &clipper_subject );

    ExPolygons clipper_result;
	Polygons result;

    ClipType op;
    if ( poly_op == POLY_DIFF ) {
	    op = ctDifference;
    } else if ( poly_op == POLY_INT ) {
	    op = ctIntersection;
    } else if ( poly_op == POLY_XOR ) {
	    op = ctXor;
    } else if ( poly_op == POLY_UNION ) {
    	op = ctUnion;
    } else {
        throw sg_exception("Unknown polygon op, exiting.");
    }

    Clipper c;
	c.Clear();
	c.AddPolygons(clipper_subject, ptSubject);
	c.AddPolygons(clipper_clip, ptClip);

	c.Execute(op, clipper_result, pftEvenOdd, pftEvenOdd);
	
	// copy contours to polygons structure
	for (int i=0; i<clipper_result.size(); i++)
	{
		result.push_back( clipper_result[i].outer );
		for (int j=0; j<clipper_result[i].holes.size(); j++)
		{
			result.push_back( clipper_result[i].holes[j] );
		}
	}

	return result;
}
#endif

// Difference
TGPolygon tgPolygonDiff( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip( POLY_DIFF, subject, clip );
}

#if CLIP_NATIVE
TGPolygon tgPolygonDiff( const TGPolygon& subject, const ClipPolyType& clip ) {
    return polygon_clip( POLY_DIFF, subject, clip );
}
#endif

// Intersection
TGPolygon tgPolygonInt( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip( POLY_INT, subject, clip );
}


// Exclusive or
TGPolygon tgPolygonXor( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip( POLY_XOR, subject, clip );
}


// Union
TGPolygon tgPolygonUnion( const TGPolygon& subject, const TGPolygon& clip ) {
    return polygon_clip( POLY_UNION, subject, clip );
}

#if CLIP_NATIVE
ClipPolyType tgPolygonUnion( const TGPolygon& subject, const ClipPolyType& clip ) {
    return polygon_clip_keep_native_fmt( POLY_UNION, subject, clip );
}
#endif

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


#if 0
// Wrapper for the fast Polygon Triangulation based on Seidel's
// Algorithm by Atul Narkhede and Dinesh Manocha
// http://www.cs.unc.edu/~dm/CODE/GEM/chapter.html

// I make this oversize because given an n sided concave polygon with
// m, n, o, ... sided holes, I have no idea how many triangles would
// result and I don't have time right now to see if an upper bound can
// be determined easily.
#define FG_MAX_TRIANGLES 100000

TGPolygon polygon_to_tristrip( const TGPolygon& in_poly ) {
    int i, j;

    // canonify the polygon winding, outer contour must be
    // anti-clockwise, all inner contours must be clockwise.
    TGPolygon canon_poly = polygon_canonify( in_poly );

    // create and fill in the required structures
    int ncontours = canon_poly.contours();
    int cntr[ncontours];
    int vsize = 1;
    for ( i = 0; i < canon_poly.contours(); ++i ) {
	cntr[i] = canon_poly.contour_size( i );
	vsize += cntr[i];
    }
    double vertices[vsize][2];
    int counter = 1;
    Point3D p;
    for ( i = 0; i < canon_poly.contours(); ++i ) {
	for ( j = 0; j < canon_poly.contour_size( i ); ++j ) {
	    p = canon_poly.get_pt( i, j );
	    vertices[counter][0] = p.x();
	    vertices[counter][1] = p.y();
	    counter++;
	}
    }
    int triangles[FG_MAX_TRIANGLES][3];

    // do the triangulation
    int ntriangles = triangulate_polygon(ncontours, cntr, vertices, triangles);

    /*
    gpc_polygon *tmp_poly = new gpc_polygon;
    tmp_poly->num_contours = 0;
    tmp_poly->contour = NULL;
    tmp_poly->hole = NULL;
    make_gpc_poly( in_poly, tmp_poly );

    gpc_tristrip *tmp_tristrip = new gpc_tristrip;
    tmp_tristrip->num_strips = 0;
    tmp_tristrip->strip = NULL;
    
    gpc_polygon_to_tristrip( tmp_poly, tmp_tristrip );

    TGPolygon result;

    for ( int i = 0; i < tmp_tristrip->num_strips; ++i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  processing strip = "
	       << i << ", nodes = " 
	       << tmp_tristrip->strip[i].num_vertices);
	
	// sprintf(junkn, "g.%d", junkc++);
	// junkfp = fopen(junkn, "w");

	for ( int j = 0; j < tmp_tristrip->strip[i].num_vertices; j++ ) {
	    Point3D p( tmp_tristrip->strip[i].vertex[j].x,
		       tmp_tristrip->strip[i].vertex[j].y,
		       0 );
	    // junkp = in_nodes.get_node( index );
	    // fprintf(junkfp, "%.4f %.4f\n", junkp.x(), junkp.y());
	    result.add_node(i, p);
	    // SG_LOG(SG_GENERAL, SG_DEBUG, "  - " << index);
	}
	// fprintf(junkfp, "%.4f %.4f\n", 
	//    gpc_result->contour[i].vertex[0].x, 
	//    gpc_result->contour[i].vertex[0].y);
	// fclose(junkfp);
    }

    // free allocated memory
    gpc_free_polygon( tmp_poly );
    gpc_free_tristrip( tmp_tristrip );
    */

    // return result;
}
#endif


#if 0
//
// wrapper functions for gpc polygon to tristrip routine
//

TGPolygon polygon_to_tristrip_old( const TGPolygon& in_poly ) {
    gpc_polygon *tmp_poly = new gpc_polygon;
    tmp_poly->num_contours = 0;
    tmp_poly->contour = NULL;
    tmp_poly->hole = NULL;
    make_gpc_poly( in_poly, tmp_poly );

    gpc_tristrip *tmp_tristrip = new gpc_tristrip;
    tmp_tristrip->num_strips = 0;
    tmp_tristrip->strip = NULL;
    
    gpc_polygon_to_tristrip( tmp_poly, tmp_tristrip );

    TGPolygon result;

    for ( int i = 0; i < tmp_tristrip->num_strips; ++i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  processing strip = "
	       << i << ", nodes = " 
	       << tmp_tristrip->strip[i].num_vertices);
	
	// sprintf(junkn, "g.%d", junkc++);
	// junkfp = fopen(junkn, "w");

	for ( int j = 0; j < tmp_tristrip->strip[i].num_vertices; j++ ) {
	    Point3D p( tmp_tristrip->strip[i].vertex[j].x,
		       tmp_tristrip->strip[i].vertex[j].y,
		       0 );
	    // junkp = in_nodes.get_node( index );
	    // fprintf(junkfp, "%.4f %.4f\n", junkp.x(), junkp.y());
	    result.add_node(i, p);
	    // SG_LOG(SG_GENERAL, SG_DEBUG, "  - " << index);
	}
	// fprintf(junkfp, "%.4f %.4f\n", 
	//    gpc_result->contour[i].vertex[0].x, 
	//    gpc_result->contour[i].vertex[0].y);
	// fclose(junkfp);
    }

    // free allocated memory
    gpc_free_polygon( tmp_poly );
    gpc_free_tristrip( tmp_tristrip );

    return result;
}
#endif


// Send a polygon to standard output.
ostream &
operator<< (ostream &output, const TGPolygon &poly)
{
    int nContours = poly.contours();
    output << nContours << endl;
    for (int i = 0; i < nContours; i++) {
	int nPoints = poly.contour_size(i);
	output << nPoints << endl;
	output << poly.get_hole_flag(i) << endl;
	for (int j = 0; j < nPoints; j++) {
	    output << poly.get_pt(i, j) << endl;
	}
    }

    return output;  // MSVC
}
