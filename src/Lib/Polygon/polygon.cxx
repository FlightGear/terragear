// polygon.cxx -- polygon (with holes) management class
//
// Written by Curtis Olson, started March 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
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
// $Id$


// include Generic Polygon Clipping Library
//
//    http://www.cs.man.ac.uk/aig/staff/alan/software/
//
extern "C" {
#include <gpc.h>
}

#include <simgear/constants.h>
#include <simgear/math/point3d.hxx>

#include <poly2tri/interface.h>

#include "polygon.hxx"

FG_USING_STD(cout);
FG_USING_STD(endl);


// Constructor 
FGPolygon::FGPolygon( void ) {
}


// Destructor
FGPolygon::~FGPolygon( void ) {
}


// Calculate theta of angle (a, b, c)
static double calc_angle(point2d a, point2d b, point2d c) {
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

double FGPolygon::area_contour( const int contour ) const {
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
double FGPolygon::minangle_contour( const int contour ) {
    point_list c = poly[contour];
    int size = c.size();
    int p1_index, p2_index, p3_index;
    point2d p1, p2, p3;
    double angle;
    double min_angle = 2.0 * FG_PI;

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

	angle = calc_angle( p1, p2, p3 );

	if ( angle < min_angle ) {
	    min_angle = angle;
	}
    }

    return min_angle;
}


// return true if contour B is inside countour A
bool FGPolygon::is_inside( int a, int b ) const {
    // make polygons from each specified contour
    FGPolygon A, B;
    point_list pl;
    A.erase();
    B.erase();

    pl = get_contour( a );
    A.add_contour( pl, 0 );

    pl = get_contour( b );
    B.add_contour( pl, 0 );

    // B is "inside" A if the polygon_diff( B, A ) is not null.
    FGPolygon result = polygon_diff( B, A );
    if ( result.contours() == 0 ) {
	// cout << "    is_inside() result = true" << endl;
	return true;
    }

    // cout << "    is_inside() result = false" << endl;
    return false;
}


// shift every point in the polygon by lon, lat
void FGPolygon::shift( double lon, double lat ) {
    for ( int i = 0; i < (int)poly.size(); ++i ) {
	for ( int j = 0; j < (int)poly[i].size(); ++j ) {
	    poly[i][j].setx( poly[i][j].x() + lon );
	    poly[i][j].sety( poly[i][j].y() + lat );
	}
    }
}


// output
void FGPolygon::write( const string& file ) const {
    FILE *fp = fopen( file.c_str(), "w" );
    
    for ( int i = 0; i < (int)poly.size(); ++i ) {
	for ( int j = 0; j < (int)poly[i].size(); ++j ) {
	    fprintf(fp, "%.6f %.6f\n", poly[i][j].x(), poly[i][j].y());
	}
	fprintf(fp, "%.6f %.6f\n", poly[i][0].x(), poly[i][0].y());
    }

    fclose(fp);
}


// output
void FGPolygon::write_contour( const int contour, const string& file ) const {
    FILE *fp = fopen( file.c_str(), "w" );
    
    for ( int j = 0; j < (int)poly[contour].size(); ++j ) {
	fprintf(fp, "%.6f %.6f\n", poly[contour][j].x(), poly[contour][j].y());
    }

    fclose(fp);
}


//
// wrapper functions for gpc polygon clip routines
//

// Make a gpc_poly from an FGPolygon
void make_gpc_poly( const FGPolygon& in, gpc_polygon *out ) {
    gpc_vertex_list v_list;
    v_list.num_vertices = 0;
    v_list.vertex = new gpc_vertex[FG_MAX_VERTICES];

    // cout << "making a gpc_poly" << endl;
    // cout << "  input contours = " << in.contours() << endl;

    Point3D p;
    // build the gpc_polygon structures
    for ( int i = 0; i < in.contours(); ++i ) {
	// cout << "    contour " << i << " = " << in.contour_size( i ) << endl;
	if ( in.contour_size( i ) > FG_MAX_VERTICES ) {
	    cout << "Polygon too large, need to increase FG_MAX_VERTICES to at "
		 << "least " << in.contour_size( i ) << endl;
	    exit(-1);
	}

	for ( int j = 0; j < in.contour_size( i ); ++j ) {
	    p = in.get_pt( i, j );
	    v_list.vertex[j].x = p.x();
	    v_list.vertex[j].y = p.y();
	}
	v_list.num_vertices = in.contour_size( i );
	gpc_add_contour( out, &v_list, in.get_hole_flag( i ) );
    }

    // free alocated memory
    delete v_list.vertex;
}


// Set operation type
typedef enum {
    POLY_DIFF,			// Difference
    POLY_INT,			// Intersection
    POLY_XOR,			// Exclusive or
    POLY_UNION			// Union
} clip_op;


// Generic clipping routine
FGPolygon polygon_clip( clip_op poly_op, const FGPolygon& subject, 
			const FGPolygon& clip )
{
    FGPolygon result;

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
	cout << "Unknown polygon op, exiting." << endl;
	exit(-1);
    }

    gpc_polygon_clip( op, gpc_subject, gpc_clip, gpc_result );

    for ( int i = 0; i < gpc_result->num_contours; ++i ) {
	// cout << "  processing contour = " << i << ", nodes = " 
	//      << gpc_result->contour[i].num_vertices << ", hole = "
	//      << gpc_result->hole[i] << endl;
	
	// sprintf(junkn, "g.%d", junkc++);
	// junkfp = fopen(junkn, "w");

	for ( int j = 0; j < gpc_result->contour[i].num_vertices; j++ ) {
	    Point3D p( gpc_result->contour[i].vertex[j].x,
		       gpc_result->contour[i].vertex[j].y,
		       0 );
	    // junkp = in_nodes.get_node( index );
	    // fprintf(junkfp, "%.4f %.4f\n", junkp.x(), junkp.y());
	    result.add_node(i, p);
	    // cout << "  - " << index << endl;
	}
	// fprintf(junkfp, "%.4f %.4f\n", 
	//    gpc_result->contour[i].vertex[0].x, 
	//    gpc_result->contour[i].vertex[0].y);
	// fclose(junkfp);

	result.set_hole_flag( i, gpc_result->hole[i] );
    }

    // free allocated memory
    gpc_free_polygon( gpc_subject );
    gpc_free_polygon( gpc_clip );
    gpc_free_polygon( gpc_result );

    return result;
}


// Difference
FGPolygon polygon_diff(	const FGPolygon& subject, const FGPolygon& clip ) {
    return polygon_clip( POLY_DIFF, subject, clip );
}

// Intersection
FGPolygon polygon_int( const FGPolygon& subject, const FGPolygon& clip ) {
    return polygon_clip( POLY_INT, subject, clip );
}


// Exclusive or
FGPolygon polygon_xor( const FGPolygon& subject, const FGPolygon& clip ) {
    return polygon_clip( POLY_XOR, subject, clip );
}


// Union
FGPolygon polygon_union( const FGPolygon& subject, const FGPolygon& clip ) {
    return polygon_clip( POLY_UNION, subject, clip );
}


// canonify the polygon winding, outer contour must be anti-clockwise,
// all inner contours must be clockwise.
FGPolygon polygon_canonify( const FGPolygon& in_poly ) {
    FGPolygon result;
    result.erase();

    // Negative areas indicate counter clockwise winding.  Postitive
    // areas indicate clockwise winding.

    int non_hole_count = 0;

    for ( int i = 0; i < in_poly.contours(); ++i ) {
	point_list contour = in_poly.get_contour( i );
	int hole_flag = in_poly.get_hole_flag( i );
	if ( !hole_flag ) {
	    non_hole_count++;
	    if ( non_hole_count > 1 ) {
		cout << "ERROR: polygon with more than one enclosing" << endl;
		cout << "  contour.  I bet you don't handle that!" << endl;
		cout << "  dying!!!" << endl;
	    }
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


#if 0
// Wrapper for the fast Polygon Triangulation based on Seidel's
// Algorithm by Atul Narkhede and Dinesh Manocha
// http://www.cs.unc.edu/~dm/CODE/GEM/chapter.html

// I make this oversize because given an n sided concave polygon with
// m, n, o, ... sided holes, I have no idea how many triangles would
// result and I don't have time right now to see if an upper bound can
// be determined easily.
#define FG_MAX_TRIANGLES 100000

FGPolygon polygon_to_tristrip( const FGPolygon& in_poly ) {
    int i, j;

    // canonify the polygon winding, outer contour must be
    // anti-clockwise, all inner contours must be clockwise.
    FGPolygon canon_poly = polygon_canonify( in_poly );

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

    FGPolygon result;

    for ( int i = 0; i < tmp_tristrip->num_strips; ++i ) {
	cout << "  processing strip = " << i << ", nodes = " 
	     << tmp_tristrip->strip[i].num_vertices << endl;
	
	// sprintf(junkn, "g.%d", junkc++);
	// junkfp = fopen(junkn, "w");

	for ( int j = 0; j < tmp_tristrip->strip[i].num_vertices; j++ ) {
	    Point3D p( tmp_tristrip->strip[i].vertex[j].x,
		       tmp_tristrip->strip[i].vertex[j].y,
		       0 );
	    // junkp = in_nodes.get_node( index );
	    // fprintf(junkfp, "%.4f %.4f\n", junkp.x(), junkp.y());
	    result.add_node(i, p);
	    // cout << "  - " << index << endl;
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

FGPolygon polygon_to_tristrip_old( const FGPolygon& in_poly ) {
    gpc_polygon *tmp_poly = new gpc_polygon;
    tmp_poly->num_contours = 0;
    tmp_poly->contour = NULL;
    tmp_poly->hole = NULL;
    make_gpc_poly( in_poly, tmp_poly );

    gpc_tristrip *tmp_tristrip = new gpc_tristrip;
    tmp_tristrip->num_strips = 0;
    tmp_tristrip->strip = NULL;
    
    gpc_polygon_to_tristrip( tmp_poly, tmp_tristrip );

    FGPolygon result;

    for ( int i = 0; i < tmp_tristrip->num_strips; ++i ) {
	cout << "  processing strip = " << i << ", nodes = " 
	     << tmp_tristrip->strip[i].num_vertices << endl;
	
	// sprintf(junkn, "g.%d", junkc++);
	// junkfp = fopen(junkn, "w");

	for ( int j = 0; j < tmp_tristrip->strip[i].num_vertices; j++ ) {
	    Point3D p( tmp_tristrip->strip[i].vertex[j].x,
		       tmp_tristrip->strip[i].vertex[j].y,
		       0 );
	    // junkp = in_nodes.get_node( index );
	    // fprintf(junkfp, "%.4f %.4f\n", junkp.x(), junkp.y());
	    result.add_node(i, p);
	    // cout << "  - " << index << endl;
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
operator<< (ostream &output, const FGPolygon &poly)
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
