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
    if (u_num == 0)
      SG_LOG(SG_GENERAL, SG_ALERT, "Intersection: coincident lines");
    else
      SG_LOG(SG_GENERAL, SG_ALERT, "Intersection: parallel lines");
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


void
makePolygon (const Line &line, int width, TGPolygon &polygon)
{
  vector<TGPolygon> segment_list;

  int nPoints = line.getPointCount();
  int i;
  for (i = 0; i < nPoints - 1; i++) {
    const Point3D p1 = line.getPoint(i);
    const Point3D p2 = line.getPoint(i+1);

    double angle1, angle2, dist, x, y, az;
      
    geo_inverse_wgs_84(0, p1.y(), p1.x(), p2.y(), p2.x(), &angle1, &angle2, &dist);
    polygon.erase();

				// Wind each rectangle counterclockwise

				// Corner 1
    geo_direct_wgs_84(0, p1.y(), p1.x(), CLAMP_ANGLE(angle1+90), width/2, &y, &x, &az);
    polygon.add_node(0, Point3D(x, y, 0));

				// Corner 2
    geo_direct_wgs_84(0, p2.y(), p2.x(), CLAMP_ANGLE(angle1+90), width/2, &y, &x, &az);
    polygon.add_node(0, Point3D(x, y, 0));

				// Corner 3
    geo_direct_wgs_84(0, p2.y(), p2.x(), CLAMP_ANGLE(angle1-90), width/2, &y, &x, &az);
    polygon.add_node(0, Point3D(x, y, 0));

				// Corner 4
    geo_direct_wgs_84(0, p1.y(), p1.x(), CLAMP_ANGLE(angle1-90), width/2, &y, &x, &az);
    polygon.add_node(0, Point3D(x, y, 0));

				// Save this rectangle
    segment_list.push_back(polygon);
  }

  // Build one big polygon out of all the rectangles by intersecting
  // the lines running through the bottom and top sides

  polygon.erase();

				// Connect the bottom part.
  int nSegments = segment_list.size();
  Point3D intersection;
  polygon.add_node(0, segment_list[0].get_pt(0, 0));
  for (i = 0; i < nSegments - 1; i++) {
    if (getIntersection(segment_list[i].get_pt(0, 0),
			segment_list[i].get_pt(0, 1),
			segment_list[i+1].get_pt(0, 0),
			segment_list[i+1].get_pt(0, 1),
			intersection))
      polygon.add_node(0, intersection);
    else
      polygon.add_node(0, segment_list[i].get_pt(0, 1));
  }
  polygon.add_node(0, segment_list[nSegments-1].get_pt(0, 1));

				// Connect the top part
  polygon.add_node(0, segment_list[nSegments-1].get_pt(0, 2));
  for (i = nSegments - 1; i > 0; i--) {
    if (getIntersection(segment_list[i].get_pt(0, 2),
			segment_list[i].get_pt(0, 3),
			segment_list[i-1].get_pt(0, 2),
			segment_list[i-1].get_pt(0, 3),
			intersection))
      polygon.add_node(0, intersection);
    else
      polygon.add_node(0, segment_list[i].get_pt(0, 3));
  }
  polygon.add_node(0, segment_list[0].get_pt(0, 3));
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
