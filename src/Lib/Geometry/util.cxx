// util.cxx - a collection of simple geometry utility functions.
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.

#include "util.hxx"

#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/exception.hxx>

#include <Polygon/polygon.hxx>

namespace tg {


bool
getIntersection (const Point3D &p0, const Point3D &p1,
		 const Point3D &p2, const Point3D &p3,
		 Point3D &intersection)
{
  double u_num =
    ((p3.x()-p2.x())*(p0.y()-p2.y()))-((p3.y()-p2.y())*(p0.x()-p2.x()));
  double u_den =
    ((p3.y()-p2.y())*(p1.x()-p0.x()))-((p3.x()-p2.x())*(p1.y()-p0.y()));

  if (u_den == 0) {
    if (u_num == 0) {
      SG_LOG(SG_GENERAL, SG_ALERT, "Intersection: coincident lines");
    } else {
      SG_LOG(SG_GENERAL, SG_ALERT, "Intersection: parallel lines");
    }
    return false;
  } else {
    double u = u_num/u_den;
    intersection = Point3D((p0.x()+u*(p1.x()-p0.x())),
			   (p0.y()+u*(p1.y()-p0.y())),
			   0);
    return true;
  }
}


/**
 * Create a polygon out of a point.
 *
 * Note that simple geometry doesn't work here, because the scale is
 * not even -- the points on the x-axis (longitude) become closer and
 * closer as the y-axis (latitude) approaches the poles, meeting in
 * a single point at y=90 and y=-90.  As a result, this function
 * uses the WGS80 functions, rather than simple Pythagorean stuff.
 */
void
makePolygon (const Point3D &p, int width, TGPolygon &polygon)
{
  double x, y, az;
  double lon = p.x();
  double lat = p.y();

  polygon.erase();
      
  geo_direct_wgs_84(0, lat, lon, 90, width/2, &y, &x, &az);
  double dlon = x - lon;

  geo_direct_wgs_84(0, lat, lon, 0, width/2, &y, &x, &az);
  double dlat = y - lat;

  polygon.add_node(0, Point3D(lon - dlon, lat - dlat, 0));
  polygon.add_node(0, Point3D(lon + dlon, lat - dlat, 0));
  polygon.add_node(0, Point3D(lon + dlon, lat + dlat, 0));
  polygon.add_node(0, Point3D(lon - dlon, lat + dlat, 0));
}


// Alternate makePolygon from a line routine
void
makePolygon (const Line &line, int width, TGPolygon &polygon)
{
    point_list left_side, right_side;
    left_side.clear();
    right_side.clear();
 
    Point3D p0, p1, p2;
    double a1, a2, dist;
    double angle0, angle1;
    double x, y, az1;
    int i;
 
    // prime the pump
    p0 = line.getPoint(0);
    p1 = line.getPoint(1);

    // calculate heading of first line
    geo_inverse_wgs_84(0, p0.y(), p0.x(), p1.y(), p1.x(),
                       &a1, &a2, &dist);
    angle0 = (a1 + a2) / 2.0;
    
    // calculate right offset edge for line
    geo_direct_wgs_84(0, p0.y(), p0.x(),
                      CLAMP_ANGLE(angle0+90), width/2, &y, &x, &az1);
    right_side.push_back( Point3D( x, y, 0 ) );

    // calculate left offset edge for line
    geo_direct_wgs_84(0, p0.y(), p0.x(),
                      CLAMP_ANGLE(angle0-90), width/2, &y, &x, &az1);
    left_side.push_back( Point3D( x, y, 0 ) );

    int nPoints = line.getPointCount();
    for ( i = 0; i < nPoints - 2; i++ ) {
        p0 = line.getPoint(i);
        p1 = line.getPoint(i+1);
        p2 = line.getPoint(i+2);

        // calculate heading of line0
        geo_inverse_wgs_84(0, p0.y(), p0.x(), p1.y(), p1.x(),
                           &a1, &a2, &dist);
        angle0 = (a1 + a2) / 2.0;

        // calculate heading of line1
        geo_inverse_wgs_84(0, p1.y(), p1.x(), p2.y(), p2.x(),
                           &a1, &a2, &dist);
        angle1 = (a1 + a2) / 2.0;

        // calculate right offset edge for line0
        geo_direct_wgs_84(0, p0.y(), p0.x(),
                          CLAMP_ANGLE(angle0+90), width/2, &y, &x, &az1);
        Point3D roff0( x, y, 0 );
        geo_direct_wgs_84(0, p1.y(), p1.x(),
                          CLAMP_ANGLE(angle0+90), width/2, &y, &x, &az1);
        Point3D roff1( x, y, 0 );

        // calculate left offset edge for line0
        geo_direct_wgs_84(0, p0.y(), p0.x(),
                          CLAMP_ANGLE(angle0-90), width/2, &y, &x, &az1);
        Point3D loff0( x, y, 0 );
        geo_direct_wgs_84(0, p1.y(), p1.x(),
                          CLAMP_ANGLE(angle0-90), width/2, &y, &x, &az1);
        Point3D loff1( x, y, 0 );

        // calculate right offset edge for line1
        geo_direct_wgs_84(0, p1.y(), p1.x(),
                          CLAMP_ANGLE(angle1+90), width/2, &y, &x, &az1);
        Point3D roff2( x, y, 0 );
        geo_direct_wgs_84(0, p2.y(), p2.x(),
                          CLAMP_ANGLE(angle1+90), width/2, &y, &x, &az1);
        Point3D roff3( x, y, 0 );

        // calculate left offset edge for line1
        geo_direct_wgs_84(0, p1.y(), p1.x(),
                          CLAMP_ANGLE(angle1-90), width/2, &y, &x, &az1);
        Point3D loff2( x, y, 0 );
        geo_direct_wgs_84(0, p2.y(), p2.x(),
                          CLAMP_ANGLE(angle1-90), width/2, &y, &x, &az1);
        Point3D loff3( x, y, 0 );

        // calculate intersection of roffset line1 and right offset line2
        Point3D rint, lint;
        if ( !getIntersection( roff0, roff1, roff2, roff3, rint ) ) {
            // intersection failed, use the average of roff1 and roff2
            rint = ( roff1 + roff2 ) / 2.0;
        }
        right_side.push_back( rint );
        if ( !getIntersection( loff0, loff1, loff2, loff3, lint ) ) {
            // intersection failed, use the average of roff1 and roff2
            lint = ( loff1 + loff2 ) / 2.0;
        }
        left_side.push_back( lint );
    }

                         // finish the end of the polyline
    p0 = line.getPoint(nPoints - 2);
    p1 = line.getPoint(nPoints - 1);

    // calculate heading of last line
    geo_inverse_wgs_84(0, p0.y(), p0.x(), p1.y(), p1.x(),
                       &a1, &a2, &dist);
    angle0 = (a1 + a2) / 2.0;
    
    // calculate right offset edge for line
    geo_direct_wgs_84(0, p1.y(), p1.x(),
                      CLAMP_ANGLE(angle0+90), width/2, &y, &x, &az1);
    right_side.push_back( Point3D(x, y, 0) );

    // calculate left offset edge for line
    geo_direct_wgs_84(0, p1.y(), p1.x(),
                      CLAMP_ANGLE(angle0-90), width/2, &y, &x, &az1);
    left_side.push_back( Point3D(x, y, 0) );

    // Finally assemble the polygon with counter clockwise winding
    polygon.erase();
    for ( i = 0; i < (int)right_side.size(); ++i ) {
        polygon.add_node( 0, right_side[i] );
    }
    for ( i = left_side.size() - 1; i >= 0; --i ) {
        polygon.add_node( 0, left_side[i] );
    }
}


Rectangle
makeBounds (const TGPolygon &polygon)
{
    double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
    for (int i = 0; i < polygon.contours(); i++) {
        for (int j = 0; j < polygon.contour_size(i); j++) {
            Point3D p = polygon.get_pt(i, j);
            if (i == 0 && j == 0) {
                min_x = max_x = p.x();
                min_y = max_y = p.y();
            } else {
                if (min_x > p.x())
                    min_x = p.x();
                if (max_x < p.x())
                    max_x = p.x();
                if (min_y > p.y())
                    min_y = p.y();
                if (max_y < p.y())
                    max_y = p.y();
            }
        }
    }
    return Rectangle(Point3D(min_x, min_y, 0), Point3D(max_x, max_y, 0));
}


Rectangle
parseChunk (const string &s)
{
  Rectangle bounds;
  int x_factor;
  int y_factor;

  if (s.size() != 7)
    throw sg_exception(string("Bad length for chunk specifier: ") + s);

  if (s[0] == 'w')
    x_factor = -1;
  else if (s[0] == 'e')
    x_factor = 1;
  else
    throw sg_exception(string("Chunk specifier must begin with 'e' or 'w': "
			      + s));

  if (s[4] == 's')
    y_factor = -1;
  else if (s[4] == 'n')
    y_factor = 1;
  else
    throw sg_exception("Second part of chunk specifier must begin with 's' or 'n': " + s);

  
  double x = atoi(s.substr(1,3).c_str()) * x_factor;
  double y = atoi(s.substr(5).c_str()) * y_factor;
  bounds.setMin(Point3D(x, y, 0));
  bounds.setMax(Point3D(x + 10, y + 10, 0));

  return bounds;
}

};

// end of util.cxx
