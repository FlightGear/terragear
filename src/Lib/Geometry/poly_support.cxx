// poly_support.cxx -- additional supporting routines for the FGPolygon class
//                     specific to the object building process.
//
// Written by Curtis Olson, started October 1999.
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


#include <simgear/compiler.h>
#include <simgear/fg_types.hxx>

#include <Polygon/polygon.hxx>

#include "trinodes.hxx"


// calculate some "arbitrary" point inside the specified contour for
// assigning attribute areas.  This requires data structures outside
// of "FGPolygon" which is why it is living over here in "Lib/Build"

void calc_point_inside( const int contour, const FGTriNodes& trinodes );


#include <simgear/constants.h>
#include <simgear/point3d.hxx>

#include <Build/trisegs.hxx>


// Given a line segment specified by two endpoints p1 and p2, return
// the slope of the line.
static double slope( const Point3D& p0, const Point3D& p1 ) {
    if ( fabs(p0.x() - p1.x()) > FG_EPSILON ) {
	return (p0.y() - p1.y()) / (p0.x() - p1.x());
    } else {
	return 1.0e+999; // really big number
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

    if ( fabs(p0.x() - p1.x()) > FG_EPSILON ) {
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


// calculate some "arbitrary" point inside the specified contour for
// assigning attribute areas
Point3D calc_point_inside( const FGPolygon& p, const int contour, 
			   const FGTriNodes& trinodes ) {
    Point3D tmp, min, ln, p1, p2, p3, m, result, inside_pt;
    int min_node_index = 0;
    int min_index = 0;
    int p1_index = 0;
    int p2_index = 0;
    int ln_index = 0;

    // 1. find a point on the specified contour, min, with smallest y

    // min.y() starts greater than the biggest possible lat (degrees)
    min.sety( 100.0 );

    point_list c = p.get_contour(contour);
    point_list_iterator current, last;
    current = c.begin();
    last = c.end();

    for ( int i = 0; i < p.contour_size( contour ); ++i ) {
	tmp = p.get_pt( contour, i );
	if ( tmp.y() < min.y() ) {
	    min = tmp;
	    min_index = trinodes.find( min );
	    min_node_index = i;

	    // cout << "min index = " << *current 
	    //      << " value = " << min_y << endl;
	} else {
	    // cout << "  index = " << *current << endl;
	}
    }

    cout << "min node index = " << min_node_index << endl;
    cout << "min index = " << min_index
	 << " value = " << trinodes.get_node( min_index ) 
	 << " == " << min << endl;

    // 2. take midpoint, m, of min with neighbor having lowest
    // fabs(slope)

    if ( min_node_index == 0 ) {
	p1 = c[1];
	p2 = c[c.size() - 1];
    } else if ( min_node_index == (int)(c.size()) - 1 ) {
	p1 = c[0];
	p2 = c[c.size() - 2];
    } else {
	p1 = c[min_node_index - 1];
	p2 = c[min_node_index + 1];
    }
    p1_index = trinodes.find( p1 );
    p2_index = trinodes.find( p2 );

    double s1 = fabs( slope(min, p1) );
    double s2 = fabs( slope(min, p2) );
    if ( s1 < s2  ) {
	ln_index = p1_index;
	ln = p1;
    } else {
	ln_index = p2_index;
	ln = p2;
    }

    FGTriSeg base_leg( min_index, ln_index, 0 );

    m.setx( (min.x() + ln.x()) / 2.0 );
    m.sety( (min.y() + ln.y()) / 2.0 );
    cout << "low mid point = " << m << endl;

    // 3. intersect vertical line through m and all other segments of
    // all other contours of this polygon.  save point, p3, with
    // smallest y > m.y

    p3.sety(100);
    
    for ( int i = 0; i < (int)p.contours(); ++i ) {
	cout << "contour = " << i << " size = " << p.contour_size( i ) << endl;
	for ( int j = 0; j < (int)(p.contour_size( i ) - 1); ++j ) {
	    // cout << "  p1 = " << poly[i][j] << " p2 = " 
	    //      << poly[i][j+1] << endl;
	    p1 = p.get_pt( i, j );
	    p2 = p.get_pt( i, j+1 );
	    p1_index = trinodes.find( p1 );
	    p2_index = trinodes.find( p2 );
	
	    if ( intersects(p1, p2, m.x(), &result) ) {
		cout << "intersection = " << result << endl;
		if ( ( result.y() < p3.y() ) &&
		     ( result.y() > m.y() ) &&
		     ( base_leg != FGTriSeg(p1_index, p2_index, 0) ) ) {
		    p3 = result;
		}
	    }
	}
	// cout << "  p1 = " << poly[i][0] << " p2 = " 
	//      << poly[i][poly[i].size() - 1] << endl;
	p1 = p.get_pt( i, 0 );
	p2 = p.get_pt( i, p.contour_size( i ) - 1 );
	p1_index = trinodes.find( p1 );
	p2_index = trinodes.find( p2 );
	if ( intersects(p1, p2, m.x(), &result) ) {
	    cout << "intersection = " << result << endl;
	    if ( ( result.y() < p3.y() ) &&
		 ( result.y() > m.y() ) &&
		 ( base_leg != FGTriSeg(p1_index, p2_index, 0) ) ) {
		p3 = result;
	    }
	}
    }
    if ( p3.y() < 100 ) {
	cout << "low intersection of other segment = " << p3 << endl;
	inside_pt = Point3D( (m.x() + p3.x()) / 2.0,
			     (m.y() + p3.y()) / 2.0,
			     0.0 );
    } else {
	cout << "Error:  Failed to find a point inside :-(" << endl;
	inside_pt = p3;
    }

    // 4. take midpoint of p2 && m as an arbitrary point inside polygon

    cout << "inside point = " << inside_pt << endl;

    return inside_pt;
}
