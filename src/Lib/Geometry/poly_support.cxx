// poly_support.cxx -- additional supporting routines for the TGPolygon class
//                     specific to the object building process.
//
// Written by Curtis Olson, started October 1999.
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
// $Id: poly_support.cxx,v 1.39 2005-12-31 19:29:26 curt Exp $


#include <float.h>
#include <stdio.h>

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <Geometry/point3d.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/structure/exception.hxx>

#include <Polygon/polygon.hxx>
#include <Triangulate/trieles.hxx>

#include <algorithm>
#include <iterator>

#define REAL double
extern "C" {
#include <TriangleJRS/triangle.h>
}
#include <TriangleJRS/tri_support.h>

#include "contour_tree.hxx"
#include "poly_support.hxx"
#include "trinodes.hxx"
#include "trisegs.hxx"

using std::copy;
using std::cout;
using std::endl;
using std::ostream_iterator;
using std::sort;
using std::vector;

// Given a line segment specified by two endpoints p1 and p2, return
// the slope of the line.
static double slope( const Point3D& p0, const Point3D& p1 ) {
    if ( fabs(p0.x() - p1.x()) > SG_EPSILON ) {
	return (p0.y() - p1.y()) / (p0.x() - p1.x());
    } else {
	// return 1.0e+999; // really big number
	return DBL_MAX;
    }
}


// Given a line segment specified by two endpoints p1 and p2, return
// the y value of a point on the line that intersects with the
// verticle line through x.  Return true if an intersection is found,
// false otherwise.
static bool intersects( Point3D p0, Point3D p1, double x, Point3D *result ) {
    // sort the end points
    if ( p0.x() > p1.x() ) {
	Point3D tmp = p0;
	p0 = p1;
	p1 = tmp;
    }
    
    if ( (x < p0.x()) || (x > p1.x()) ) {
	// out of range of line segment, bail right away
	return false;
    }

    // equation of a line through (x0,y0) and (x1,y1):
    // 
    //     y = y1 + (x - x1) * (y0 - y1) / (x0 - x1)

    double y;

    if ( fabs(p0.x() - p1.x()) > SG_EPSILON ) {
	y = p1.y() + (x - p1.x()) * (p0.y() - p1.y()) / (p0.x() - p1.x());
    } else {
	return false;
    }
    result->setx(x);
    result->sety(y);

    if ( p0.y() <= p1.y() ) {
	if ( (p0.y() <= y) && (y <= p1.y()) ) {
	    return true;
	}
    } else {
 	if ( (p0.y() >= y) && (y >= p1.y()) ) {
	    return true;
	}
    }

    return false;
}

// basic triangulation of a polygon with out adding points or
// splitting edges, this should triangulate around interior holes.
void polygon_tesselate( const TGPolygon &p,
			triele_list &elelist,
			point_list &out_pts )
{
    struct triangulateio in, out, vorout;
    int i;

    // make sure all elements of these structs point to "NULL"
    zero_triangulateio( &in );
    zero_triangulateio( &out );
    zero_triangulateio( &vorout );
    
    int counter, start, end;

    // list of points
    double max_x = p.get_pt(0,0).x();

    int total_pts = 0;
    for ( i = 0; i < p.contours(); ++i ) {
	total_pts += p.contour_size( i );
    }

    in.numberofpoints = total_pts;
    in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));

    counter = 0;
    for ( i = 0; i < p.contours(); ++i ) {
	point_list contour = p.get_contour( i );
	for ( int j = 0; j < (int)contour.size(); ++j ) {
	    in.pointlist[2*counter] = contour[j].x();
	    in.pointlist[2*counter + 1] = contour[j].y();
	    if ( contour[j].x() > max_x ) {
		    max_x = contour[j].x();
	    }
	    ++counter;
	}
    }

    in.numberofpointattributes = 1;
    in.pointattributelist = (REAL *) malloc(in.numberofpoints *
					    in.numberofpointattributes *
					    sizeof(REAL));
    counter = 0;
    for ( i = 0; i < p.contours(); ++i ) {
	point_list contour = p.get_contour( i );
	for ( int j = 0; j < (int)contour.size(); ++j ) {
	    in.pointattributelist[counter] = contour[j].z();
	    ++counter;
	}
    }

    // in.pointmarkerlist = (int *) malloc(in.numberofpoints * sizeof(int));
    // for ( i = 0; i < in.numberofpoints; ++i) {
    //    in.pointmarkerlist[i] = 0;
    // }
    in.pointmarkerlist = NULL;

    // segment list
    in.numberofsegments = in.numberofpoints;
    in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
    counter = 0;
    start = 0;
    end = -1;

    for ( i = 0; i < p.contours(); ++i ) {
	point_list contour = p.get_contour( i );
	start = end + 1;
	end = start + contour.size() - 1;
	for ( int j = 0; j < (int)contour.size() - 1; ++j ) {
	    in.segmentlist[counter++] = j + start;
	    in.segmentlist[counter++] = j + start + 1;
	}
	in.segmentlist[counter++] = end;
	in.segmentlist[counter++] = start;
    }

    in.segmentmarkerlist = (int *) malloc(in.numberofsegments * sizeof(int));
    for ( i = 0; i < in.numberofsegments; ++i ) {
       in.segmentmarkerlist[i] = 0;
    }

    // hole list
    in.numberofholes = 1;
    for ( i = 0; i < p.contours(); ++i ) {
	if ( p.get_hole_flag( i ) ) {
	    ++in.numberofholes;
	}
    }
    in.holelist = (REAL *) malloc(in.numberofholes * 2 * sizeof(REAL));
    // outside of polygon
    counter = 0;
    in.holelist[counter++] = max_x + 1.0;
    in.holelist[counter++] = 0.0;

    for ( i = 0; i < (int)p.contours(); ++i ) {
	if ( p.get_hole_flag( i ) ) {
	    in.holelist[counter++] = p.get_point_inside(i).x();
	    in.holelist[counter++] = p.get_point_inside(i).y();
	}
    }

    // region list
    in.numberofregions = 0;
    in.regionlist = NULL;

    // no triangle list
    in.numberoftriangles = 0;
    in.numberofcorners = 0;
    in.numberoftriangleattributes = 0;
    in.trianglelist = NULL;
    in.triangleattributelist = NULL;
    in.trianglearealist = NULL;
    in.neighborlist = NULL;

    // no edge list
    in.numberofedges = 0;
    in.edgelist = NULL;
    in.edgemarkerlist = NULL;
    in.normlist = NULL;

    // dump the results to screen
    // print_tri_data( &in );

    // TEMPORARY
    // write_tri_data(&in);
    /* cout << "Press return to continue:";
    char junk;
    cin >> junk; */

    // Triangulate the points.  Switches are chosen to read and write
    // a PSLG (p), number everything from zero (z), and produce an
    // edge list (e), and a triangle neighbor list (n).
    // no new points on boundary (Y), no internal segment
    // splitting (YY), no quality refinement (q)
    // Quite (Q)

    string tri_options;
    tri_options = "pzYYenQ";	// add multiple "V" entries for verbosity
    //cout << "Triangulation with options = " << tri_options << endl;

    triangulate( (char *)tri_options.c_str(), &in, &out, &vorout );

    // TEMPORARY
    // write_tri_data(&out);

    // now copy the results back into the corresponding TGTriangle
    // structures

    // triangles
    elelist.clear();
    int n1, n2, n3;
    double attribute;
    for ( i = 0; i < out.numberoftriangles; ++i ) {
	n1 = out.trianglelist[i * 3];
	n2 = out.trianglelist[i * 3 + 1];
	n3 = out.trianglelist[i * 3 + 2];
	if ( out.numberoftriangleattributes > 0 ) {
	    attribute = out.triangleattributelist[i];
	} else {
	    attribute = 0.0;
	}
	// cout << "triangle = " << n1 << " " << n2 << " " << n3 << endl;

	elelist.push_back( TGTriEle( n1, n2, n3, attribute ) );
    }

    // output points
    out_pts.clear();
    double x, y, z;
    for ( i = 0; i < out.numberofpoints; ++i ) {
	x = out.pointlist[i * 2    ];
	y = out.pointlist[i * 2 + 1];
	z = out.pointattributelist[i];
	out_pts.push_back( Point3D(x, y, z) );
    }
   
    // free mem allocated to the "Triangle" structures
    free(in.pointlist);
    free(in.pointattributelist);
    free(in.pointmarkerlist);
    free(in.regionlist);
    free(out.pointlist);
    free(out.pointattributelist);
    free(out.pointmarkerlist);
    free(out.trianglelist);
    free(out.triangleattributelist);
    // free(out.trianglearealist);
    free(out.neighborlist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);
    free(out.edgelist);
    free(out.edgemarkerlist);
    free(vorout.pointlist);
    free(vorout.pointattributelist);
    free(vorout.edgelist);
    free(vorout.normlist);
}


// Alternate basic triangulation of a polygon with out adding points
// or splitting edges and without regard for holes.  Returns a polygon
// with one contour per tesselated triangle.  This is mostly just a
// wrapper for the polygon_tesselate() function.  Note, this routine
// will modify the points_inside list for your polygon.

TGPolygon polygon_tesselate_alt( TGPolygon &p ) {
    TGPolygon result;
    result.erase();
    int i;

    // Bail right away if polygon is empty
    if ( p.contours() == 0 ) {
	return result;
    }

    // 1.  Robustly find a point inside each contour that is not
    //     inside any other contour
    calc_points_inside( p );
    for ( i = 0; i < p.contours(); ++i ) {
	//cout << "final point inside =" << p.get_point_inside( i )
	//     << endl;
    }

    // 2.  Do a final triangulation of the entire polygon
    triele_list trieles;
    point_list nodes;
    polygon_tesselate( p, trieles, nodes );

    // 3.  Convert the tesselated output to a list of tringles.
    //     basically a polygon with a contour for every triangle
    for ( i = 0; i < (int)trieles.size(); ++i ) {
	TGTriEle t = trieles[i];
	Point3D p1 = nodes[ t.get_n1() ];
	Point3D p2 = nodes[ t.get_n2() ];
	Point3D p3 = nodes[ t.get_n3() ];
	result.add_node( i, p1 );
	result.add_node( i, p2 );
	result.add_node( i, p3 );
    }

    return result;
}

/*
 * Find all intersections of the given contour with the x-parallel line at
 * y=yline. Assume that no points are on the line (callers take care of this!).
 */
static void intersect_yline_with_contour( double yline, TGContourNode *node, TGPolygon &p, vector < double > &xcuts ) {
        int contour_num = node->get_contour_num();
        const point_list& pts=p.get_contour(contour_num);
        
        point_list::size_type count = pts.size();
        
        // cout << "intersect_yline_with_contour() yline=" << yline << endl;
        
        for ( int i = 0; i < count; ++i ) {
                const Point3D &p0 = pts[ i ];
                const Point3D &p1 = pts[ ( i + 1 ) % count ];
                
                // cout << "intersect_yline_with_contour() i=" << i << " p0=(" << p0.x() << ", " << p0.y() << ") p1=(" << p1.x() << ", " << p1.y() << ")  yline=" << yline << endl;
                
                double t=(yline-p0.y())/(p1.y()-p0.y());
                
                if (t<0.0 || 1.0<t) {
                        // cout << "intersect_yline_with_contour() does not intersect t=" << t << endl;
                        continue;
                }
                
                double x=p0.x()+t*(p1.x()-p0.x());
                
                // cout << "intersect_yline_with_contour() intersection at x=" << x << endl;
                xcuts.push_back(x);
        }
}

/*
 * Collect the points of the given contour.
 */
static void collect_contour_points( TGContourNode* node, TGPolygon &p, point_list& pts ) {
        int contour_num = node->get_contour_num();
        const point_list& contourpts=p.get_contour(contour_num);
        
        pts.insert(pts.end(),contourpts.begin(),contourpts.end());
}

static void write_tree_element( TGContourNode *node, TGPolygon& p, int hole=0) {
    int contour_num = node->get_contour_num();
    
    if (contour_num != -1) {
            char buf[256];
            sprintf(buf, "failed/element%ld.poly",contour_num);
            FILE *treefp = fopen(buf,"w");
            
            fprintf(treefp,"#2D\n");
            fprintf(treefp,"Airport\n");
            
            fprintf(treefp,"1\n");
            
            fprintf(treefp,"%ld\n",p.contour_size(contour_num));
            fprintf(treefp,"%d\n",hole);
            
            for (int i=0;i<p.contour_size(contour_num);i++) {
                    Point3D pt=p.get_pt(contour_num,i);
                    fprintf(treefp,"%.15f %.15f\n",pt.x(),pt.y());
            }
            
            fclose(treefp);
    }
    
    for ( int i = 0; i < node->get_num_kids(); ++i ) {
            if ( node->get_kid( i ) != NULL ) {
                    write_tree_element( node->get_kid( i ), p, 1-hole);
            }
    }
}

class Point3DYOrdering {
public:
    bool operator()(const Point3D& a, const Point3D& b) const {
        return a.y()<b.y();
    }
};

// recurse the contour tree and build up the point inside list for
// each contour/hole
static void calc_point_inside( TGContourNode *node, TGPolygon &p ) {
    int contour_num = node->get_contour_num();
    // cout << "starting calc_point_inside() with contour = " << contour_num << endl;

    for ( int i = 0; i < node->get_num_kids(); ++i ) {
	if ( node->get_kid( i ) != NULL ) {
	    calc_point_inside( node->get_kid( i ), p );
	}
    }
    
    if ( contour_num < 0 )
            return;
    
    /*
     * Find a line intersecting the contour and intersect it with the segments
     * of our children. Sort the intersection points along the line. They then
     * partition the line in IN/OUT parts. Find the longest segment and take
     * its midpoint as point inside the contour.
     */
    
    /*
     * Try to find a line on which none of the contour points lie. For that,
     * sort all contour points (also those of our direct children) by
     * y-coordinate, find the two with the largest distance and take their
     * center y-coordinate.
     */
    point_list allpoints;
    
    collect_contour_points( node, p, allpoints );
    
    for ( int i = 0; i < node->get_num_kids(); ++i ) {
            if ( node->get_kid( i ) != NULL ) {
                    collect_contour_points( node->get_kid( i ), p, allpoints );
            }
    }
    
    if ( allpoints.size() < 2 ) {
            throw sg_exception("Polygon must have at least 2 contour points");
    }
    
    sort(allpoints.begin(), allpoints.end(), Point3DYOrdering());

    point_list::iterator point_it;
    
    point_it=allpoints.begin();
    Point3D lastpt=*point_it;

    double maxdiff=0.0;
    double yline; // the y-location of the intersection line
    
    while ((++point_it) != allpoints.end()) {
            double diff=point_it->y()-lastpt.y();
            if (diff>maxdiff) {
                    maxdiff=diff;
                    yline=(point_it->y()+lastpt.y())/2.0;
            }
            lastpt=*point_it;
    }
    
    // cout << "calc_point_inside() " << allpoints.size() << " points ";
    // copy(allpoints.begin(), allpoints.end(), ostream_iterator<Point3D>(cout, " "));
    // cout << endl;

    // cout << "calc_point_inside() maxdiff=" << maxdiff << " yline=" << yline << endl;
    
    vector < double > xcuts;
    
    intersect_yline_with_contour( yline, node, p, xcuts );
    
    for ( int i = 0; i < node->get_num_kids(); ++i ) {
            if ( node->get_kid( i ) != NULL ) {
                    intersect_yline_with_contour( yline, node->get_kid( i ), p, xcuts );
            }
    }
    
    sort( xcuts.begin(), xcuts.end() );
    
    // cout << "calc_point_inside() " << xcuts.size() << " intersections ";
    // copy(xcuts.begin(), xcuts.end(), ostream_iterator<double>(cout, " "));
    // cout << endl;
    
    if ( xcuts.size() < 2 || (xcuts.size() % 2) != 0 ) {
            throw sg_exception("Geometric inconsistency in calc_point_inside()");
    }
    
    double maxlen=0.0;
    int longest=0;
    for ( int i = 0; i < xcuts.size(); i+=2 ) {
            double x0 = xcuts[ i ];
            double x1 = xcuts[ i + 1 ];
            
            if (x1-x0 > maxlen) {
                    maxlen=x1-x0;
                    longest=i;
            }
    }
    
    double x = (xcuts[ longest ] + xcuts[ longest + 1 ])/2.0;
    
    // cout << "calc_point_inside() found point inside x=" << x << " y=" << yline << endl;
    
    p.set_point_inside( contour_num, Point3D(x, yline, -9999.0) );
}

static void print_contour_tree( TGContourNode *node, string indent ) {
    cout << indent << node->get_contour_num() << endl;

    indent += "  ";
    for ( int i = 0; i < node->get_num_kids(); ++i ) {
	if ( node->get_kid( i ) != NULL ) {
	    print_contour_tree( node->get_kid( i ), indent );
	}
    }
}


// Build the contour "is inside of" tree
static void build_contour_tree( TGContourNode *node,
				const TGPolygon &p,
				int_list &avail )
{
    //cout << "working on contour = " << node->get_contour_num() << endl;
    //cout << "  total contours = " << p.contours() << endl;
    TGContourNode *tmp;
    int i;

    // see if we are building on a hole or not
    bool flag;
    if ( node->get_contour_num() >= 0 ) {
	flag = (bool)p.get_hole_flag( node->get_contour_num() );
    } else {
	flag = true;
    }
    //cout << "  hole flag = " << flag << endl;

    // add all remaining hole/non-hole contours as children of the
    // current node if they are inside of it.
    for ( i = 0; i < p.contours(); ++i ) {
	//cout << "  testing contour = " << i << endl;
	if ( p.get_hole_flag( i ) != flag ) {
	    // only holes can be children of non-holes and visa versa
	    if ( avail[i] ) {
		// must still be an available contour
		int cur_contour = node->get_contour_num();
		if ( (cur_contour < 0 ) || p.is_inside( i, cur_contour ) ) {
		    // must be inside the parent (or if the parent is
		    // the root, add all available non-holes.
		    //cout << "  adding contour = " << i << endl;
		    avail[i] = 0;
		    tmp = new TGContourNode( i );
		    node->add_kid( tmp );
		} else {
		    //cout << "  not inside" << endl;
		}
	    } else {
		//cout << "  not available" << endl;
	    }
	} else {
	    //cout << "  wrong hole/non-hole type" << endl;
	}
    }

    // if any of the children are inside of another child, remove the
    // inside one
    //cout << "node now has num kids = " << node->get_num_kids() << endl;

    for ( i = 0; i < node->get_num_kids(); ++i ) {
	for ( int j = 0; j < node->get_num_kids(); ++j ) {
	    // cout << "working on kid " << i << ", " << j << endl;
	    if ( i != j ) {
		if ( (node->get_kid(i) != NULL)&&(node->get_kid(j) != NULL) ) {
		    int A = node->get_kid( i )->get_contour_num();
		    int B = node->get_kid( j )->get_contour_num();
		    if ( p.is_inside( A, B ) ) {
			// p.write_contour( i, "a" );
			// p.write_contour( j, "b" );
			// exit(-1);
		        // need to remove contour j from the kid list
		        avail[ node->get_kid( i ) -> get_contour_num() ] = 1;
		        node->remove_kid( i );
		        //cout << "removing contour " << A 
                        //     << " which is inside of contour " << B << endl;
			continue;
		    }
		} else {
		    // one of these kids is already NULL, skip
		}
	    } else {
		// doesn't make sense to check if a contour is inside itself
	    }
	}
    }

    // for each child, extend the contour tree
    for ( i = 0; i < node->get_num_kids(); ++i ) {
	tmp = node->get_kid( i );
	if ( tmp != NULL ) {
	    build_contour_tree( tmp, p, avail );
	}
    }
}


// calculate some "arbitrary" point inside each of the polygons contours
void calc_points_inside( TGPolygon& p ) {
    // first build the contour tree

    // make a list of all still available contours (all of the for
    // starters)
    int_list avail;
    for ( int i = 0; i < p.contours(); ++i ) {
	avail.push_back( 1 );
    }

    // create and initialize the root node
    TGContourNode *ct = new TGContourNode( -1 );

    // recursively build the tree
    // cout << "building contour tree" << endl;
    build_contour_tree( ct, p, avail );
    // print_contour_tree( ct, "" );

    // recurse the tree and build up the point inside list for each
    // contour/hole
    // cout << " calc_point_inside()\n";
    calc_point_inside( ct, p );
}


// remove duplicate nodes in a polygon should they exist.  Returns the
// fixed polygon
TGPolygon remove_dups( const TGPolygon &poly ) {
    TGPolygon result;
    point_list contour, new_contour;
    result.erase();

    TGPolygon tmp = poly;
    for ( int i = 0; i < tmp.contours(); ++i ) {
	contour = poly.get_contour( i );
	// cout << "testing contour " << i << "  size = " << contour.size() 
	//      << "  hole = " << poly.get_hole_flag( i ) << endl;
	bool have_dups = true;
	while ( have_dups && contour.size() ) {
	    have_dups = false;
	    new_contour.clear();
	    Point3D last = contour[ contour.size() - 1 ];
	    for ( int j = 0; j < (int)contour.size(); ++j ) {
		// cout << "  " << i << " " << j << endl;
		Point3D cur = contour[j];
		if ( cur == last ) {
		    have_dups = true;
		    // cout << "skipping a duplicate point" << endl;
		} else {
		    new_contour.push_back( cur );
		    last = cur;
		}
	    }
	    contour = new_contour;
	}

	// cout << "  final size = " << contour.size() << endl;

	if ( contour.size() ) {
	    int flag = poly.get_hole_flag( i );
	    result.add_contour( contour, flag );
	} else {
	    // too small an area ... add a token point to the contour
	    // to keep other things happy, but this "bad" contour will
	    // get nuked later
	    result.add_node( i, poly.get_pt( i, 0 ) );
	}
    }

    return result;
}


static inline double
snap (double value, double grid_size)
{
				// I have no idea if this really works.
  double factor = 1.0 / grid_size;
  return double(int(value * factor)) / factor;
}

static inline Point3D
snap (const Point3D &p, double grid_size)
{
  Point3D result;
  result.setx(snap(p.x(), grid_size));
  result.sety(snap(p.y(), grid_size));
  result.setz(snap(p.z(), grid_size));
  //cout << result << endl;
  return result;
}

// snap all points in a polygon to the given grid size.
TGPolygon snap (const TGPolygon &poly, double grid_size)
{
  TGPolygon result;
  for (int contour = 0; contour < poly.contours(); contour++) {
    for (int i = 0; i < poly.contour_size(contour); i++) {
      result.add_node(contour, snap(poly.get_pt(contour, i), grid_size));
    }
    result.set_hole_flag(contour, poly.get_hole_flag(contour));
  }
  return result;
}


// static const double tgAirportEpsilon = SG_EPSILON / 10.0;
static const double tgAirportEpsilon = SG_EPSILON;


// Find a point in the given node list that lies between start and
// end, return true if something found, false if nothing found.
bool find_intermediate_node( const Point3D& start, const Point3D& end,
			     const point_list& nodes, Point3D *result )
{
    bool found_node = false;
    double m, m1, b, b1, y_err, x_err, y_err_min, x_err_min;

    Point3D p0 = start;
    Point3D p1 = end;

    // cout << "  find_intermediate_nodes() " << p0 << " <=> " << p1 << endl;

    double xdist = fabs(p0.x() - p1.x());
    double ydist = fabs(p0.y() - p1.y());
    // cout << "xdist = " << xdist << "  ydist = " << ydist << endl;
    x_err_min = xdist + 1.0;
    y_err_min = ydist + 1.0;

    if ( xdist > ydist ) {
	// cout << "use y = mx + b" << endl;

	// sort these in a sensible order
	Point3D p_min, p_max;
	if ( p0.x() < p1.x() ) {
	    p_min = p0;
	    p_max = p1;
	} else {
	    p_min = p1;
	    p_max = p0;
	}

	m = (p_min.y() - p_max.y()) / (p_min.x() - p_max.x());
	b = p_max.y() - m * p_max.x();

	// cout << "m = " << m << " b = " << b << endl;

	for ( int i = 0; i < (int)nodes.size(); ++i ) {
	    // cout << i << endl;
	    Point3D current = nodes[i];

	    if ( (current.x() > (p_min.x() + SG_EPSILON)) 
		 && (current.x() < (p_max.x() - SG_EPSILON)) ) {

		// printf( "found a potential candidate %.7f %.7f %.7f\n",
		//         current.x(), current.y(), current.z() );

		y_err = fabs(current.y() - (m * current.x() + b));
		// cout << "y_err = " << y_err << endl;

		if ( y_err < tgAirportEpsilon ) {
		    // cout << "FOUND EXTRA SEGMENT NODE (Y)" << endl;
		    // cout << p_min << " < " << current << " < "
		    //      << p_max << endl;
		    found_node = true;
		    if ( y_err < y_err_min ) {
			*result = current;
			y_err_min = y_err;
		    }
		}
	    }
	}
    } else {
	// cout << "use x = m1 * y + b1" << endl;

	// sort these in a sensible order
	Point3D p_min, p_max;
	if ( p0.y() < p1.y() ) {
	    p_min = p0;
	    p_max = p1;
	} else {
	    p_min = p1;
	    p_max = p0;
	}

	m1 = (p_min.x() - p_max.x()) / (p_min.y() - p_max.y());
	b1 = p_max.x() - m1 * p_max.y();

	// cout << "  m1 = " << m1 << " b1 = " << b1 << endl;
	// printf( "  m = %.8f  b = %.8f\n", 1/m1, -b1/m1);

	// cout << "  should = 0 = "
	//      << fabs(p_min.x() - (m1 * p_min.y() + b1)) << endl;
	// cout << "  should = 0 = "
	//      << fabs(p_max.x() - (m1 * p_max.y() + b1)) << endl;

	for ( int i = 0; i < (int)nodes.size(); ++i ) {
	    Point3D current = nodes[i];

	    if ( (current.y() > (p_min.y() + SG_EPSILON)) 
		 && (current.y() < (p_max.y() - SG_EPSILON)) ) {
		
		// printf( "found a potential candidate %.7f %.7f %.7f\n",
		//         current.x(), current.y(), current.z() );

		x_err = fabs(current.x() - (m1 * current.y() + b1));
		// cout << "x_err = " << x_err << endl;

		// if ( temp ) {
		// cout << "  (" << counter << ") x_err = " << x_err << endl;
		// }

		if ( x_err < tgAirportEpsilon ) {
		    // cout << "FOUND EXTRA SEGMENT NODE (X)" << endl;
		    // cout << p_min << " < " << current << " < "
		    //      << p_max << endl;
		    found_node = true;
		    if ( x_err < x_err_min ) {
			*result = current;
			x_err_min = x_err;
		    }
		}
	    }
	}
    }

    return found_node;
}


// Attempt to reduce degeneracies where a subsequent point of a
// polygon lies *on* a previous line segment.  These artifacts are
// occasionally introduced by the gpc polygon clipper.
static point_list reduce_contour_degeneracy( const point_list& contour ) {
    point_list result = contour;

    Point3D p0, p1, bad_node;
    bool done = false;

    while ( !done ) {
	// traverse the contour until we find the first bad node or
	// hit the end of the contour
        // cout << "   ... reduce_degeneracy(): not done ... " << endl;
	bool bad = false;

	int i = 0;
	int j = 0;

	// look for first stray intermediate nodes
	while ( i < (int)result.size() - 1 && !bad ) {
	    p0 = result[i];
	    p1 = result[i+1];
	
	    bad = find_intermediate_node( p0, p1, result, &bad_node );
	    // if ( bad ) { cout << "bad in n-1 nodes" << endl; }
	    ++i;
	}
	if ( !bad ) {
	    // do the end/start connecting segment
	    p0 = result[result.size() - 1];
	    p1 = result[0];

	    bad = find_intermediate_node( p0, p1, result, &bad_node );
	    // if ( bad ) { cout << "bad in 0 to n segment" << endl; }
	}

	// CLO: look for later nodes that match earlier segment end points
	// (i.e. a big cycle.)  There's no good automatic fix to this
	// so just drop the problematic point and live with our
	// contour changing shape.
	//
	// WARNING: FIXME??? By changing the shape of a polygon at
	// this point we are introducing some overlap which could show
	// up as flickering/depth-buffer fighting at run time.  The
	// better fix would be to split up the two regions into
	// separate contours, but I just don't have the energy to
	// think though that right now.
	if ( !bad ) {
	    i = 0;
	    while ( i < (int)result.size() - 1 && !bad ) {
	        j = i + 1;
		while ( j < (int)result.size() && !bad ) {
		    if ( result[i] == result[j] ) {
		        bad = true;
			bad_node = result[j];
			// cout << "size = " << result.size() << " i = "
			//      << i << " j = " << j << " result[i] = "
			//      << result[i] << " result[j] = " << result[j]
			//      << endl;
		    }
		    j++;
		}
		i++;
	    }
	}

	if ( bad ) {
	    // remove bad node from contour.  But only remove one node.  If
	    // the 'badness' is caused by coincident adjacent nodes, we don't
	    // want to remove both of them, just one (either will do.)
  	    cout << "found a bad node = " << bad_node << endl;
	    point_list tmp; tmp.clear();
	    bool found_one = false;
	    for ( int j = 0; j < (int)result.size(); ++j ) {
		if ( result[j] == bad_node && !found_one) {
		    // skip
		    found_one = true;
		} else {
		    tmp.push_back( result[j] );
		}
	    }
	    result = tmp;
	} else { 
	    done = true;
	}
    }

    return result;
}

// Search each segment of each contour for degenerate points (i.e. out
// of order points that lie coincident on other segments
TGPolygon reduce_degeneracy( const TGPolygon& poly ) {
    TGPolygon result;

    for ( int i = 0; i < poly.contours(); ++i ) {
        // cout << "reduce_degeneracy() contour = " << i << endl;
	point_list contour = poly.get_contour(i);
	contour = reduce_contour_degeneracy( contour );
	result.add_contour( contour, poly.get_hole_flag(i) );

	// maintain original hole flag setting
	// result.set_hole_flag( i, poly.get_hole_flag( i ) );
    }

    return result;
}

// find short cycles in a contour and snip them out.
static point_list remove_small_cycles( const point_list& contour ) {
    point_list result;
    result.clear();

    unsigned int i = 0;
    while ( i < contour.size() ) {
        result.push_back( contour[i] );
        for ( unsigned int j = i + 1; j < contour.size(); ++j ) {
            if ( contour[i] == contour[j] && i + 4 > j ) {
                cout << "detected a small cycle: i = "
                     << i << " j = " << j << endl;
                for ( unsigned int k = i; k <= j; ++k ) {
                    cout << "  " << contour[k] << endl;
                }
                i = j;
            }
        }
        ++i;
    }

    return result;
}


// Occasionally the outline of the clipped polygon can take a side
// track, then double back on return to the start of the side track
// branch and continue normally.  Attempt to detect and clear this
// extraneous nonsense.
TGPolygon remove_cycles( const TGPolygon& poly ) {
    TGPolygon result;
    // cout << "remove cycles: " << poly << endl;
    for ( int i = 0; i < poly.contours(); ++i ) {
	point_list contour = poly.get_contour(i);
	contour = remove_small_cycles( contour );
	result.add_contour( contour, poly.get_hole_flag(i) );
    }

    return result;
}


// remove any degenerate contours
TGPolygon remove_bad_contours( const TGPolygon &poly ) {
    TGPolygon result;
    result.erase();

    for ( int i = 0; i < poly.contours(); ++i ) {
	point_list contour = poly.get_contour( i );
	if ( contour.size() < 3) {
	    //cout << "tossing a bad contour" << endl;
	    continue;
        }
        
	/* keeping the contour */
	int flag = poly.get_hole_flag( i );
	result.add_contour( contour, flag );
    }

    return result;
}

// remove any too small contours
TGPolygon remove_tiny_contours( const TGPolygon &poly ) {
    TGPolygon result;
    result.erase();

    for ( int i = 0; i < poly.contours(); ++i ) {
	point_list contour = poly.get_contour( i );
        
        double area=poly.area_contour(i);
        
        if (-SG_EPSILON*SG_EPSILON<area && area<SG_EPSILON*SG_EPSILON) {
            // cout << "tossing a bad contour " << i << " (too small)" << endl;
            continue;
        }
        
	/* keeping the contour */
	int flag = poly.get_hole_flag( i );
	result.add_contour( contour, flag );
    }

    return result;
}


