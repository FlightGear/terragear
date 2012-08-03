// util.cxx - a collection of simple geometry utility functions.
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.

#include "util.hxx"

#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/structure/exception.hxx>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>

#include <stdlib.h>

#define MP_STRETCH  (0.000001)

using std::string;
using std::vector;

namespace tg {


/*
 * Computer the intersection point of two lines.  Reference:
 * http://astronomy.swin.edu.au/~pbourke/geometry/lineline2d/
 */
bool
getIntersection (const Point3D &p0, const Point3D &p1,
		 const Point3D &p2, const Point3D &p3,
		 Point3D &intersection)
{
    const double my_eps = 1E-12;
    
    double u_num =
        ((p3.x()-p2.x())*(p0.y()-p2.y()))-((p3.y()-p2.y())*(p0.x()-p2.x()));
    double u_den =
        ((p3.y()-p2.y())*(p1.x()-p0.x()))-((p3.x()-p2.x())*(p1.y()-p0.y()));

    if ( fabs(u_den) < my_eps ) {
        if ( fabs(u_num) < my_eps ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Intersection: coincident lines");
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "Intersection: parallel lines");
        }
        return false;
    } else {
        double u = u_num/u_den;
        // cout << "u = " << u << " u_num = " << u_num << " u_den = " << u_den << endl;
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
makePolygon (const Point3D &p, double width, TGPolygon &polygon)
{
  double x = 0.0f, y = 0.0f, az = 0.0f;
  double lon = p.x();
  double lat = p.y();

  polygon.erase();
      
  geo_direct_wgs_84(0, lat, lon, 90, width/2.0, &y, &x, &az);
  double dlon = x - lon;

  geo_direct_wgs_84(0, lat, lon, 0, width/2.0, &y, &x, &az);
  double dlat = y - lat;

  polygon.add_node(0, Point3D(lon - dlon, lat - dlat, 0));
  polygon.add_node(0, Point3D(lon + dlon, lat - dlat, 0));
  polygon.add_node(0, Point3D(lon + dlon, lat + dlat, 0));
  polygon.add_node(0, Point3D(lon - dlon, lat + dlat, 0));

  polygon.set_hole_flag(0, 0);  // mark as solid
}


void
makePolygon (const Line &line, double width, TGPolygon &polygon)
{
  vector<TGPolygon> segment_list;

  int nPoints = line.getPointCount();
  int i;
  for (i = 0; i < nPoints - 1; i++) {
    const Point3D p1 = line.getPoint(i);
    const Point3D p2 = line.getPoint(i+1);

    double angle1 = 0.0f;
    double angle2 = 0.0f;
    double dist = 0.0f;
    double x = 0.0f;
    double y = 0.0f;
    double az = 0.0f;
      
    geo_inverse_wgs_84(0, p1.y(), p1.x(), p2.y(), p2.x(), &angle1, &angle2, &dist);
    polygon.erase();

				// Wind each rectangle counterclockwise

				// Corner 1
    // cout << "point = " << i << endl;
    geo_direct_wgs_84(0, p1.y(), p1.x(), CLAMP_ANGLE(angle1+90), width/2.0, &y, &x, &az);
    // Sometimes creating the new point can wrap the date line, even if the
    // original line does not.  This can cause problems later, so lets just
    // clip the result to date line and hope that the typical error is very
    // small.  FIXME: we should handle this more robustly in case larger
    // structures cross the date line, but that implies a much broader swath
    // of changes.
    if ( x - p1.x() > 180.0 ) x = -180.0;
    else if ( x - p1.x() < -180.0 ) x = 180.0;
    polygon.add_node(0, Point3D(x, y, 0));
    // cout << "corner 1 = " << Point3D(x, y, 0) << endl;

				// Corner 2
    geo_direct_wgs_84(0, p2.y(), p2.x(), CLAMP_ANGLE(angle1+90), width/2.0, &y, &x, &az);
    // Sometimes creating the new point can wrap the date line, even if the
    // original line does not.  This can cause problems later, so lets just
    // clip the result to date line and hope that the typical error is very
    // small.  FIXME: we should handle this more robustly in case larger
    // structures cross the date line, but that implies a much broader swath
    // of changes.
    if ( x - p2.x() > 180.0 ) x = -180.0;
    else if ( x - p2.x() < -180.0 ) x = 180.0;
    polygon.add_node(0, Point3D(x, y, 0));
    // cout << "corner 2 = " << Point3D(x, y, 0) << endl;

				// Corner 3
    geo_direct_wgs_84(0, p2.y(), p2.x(), CLAMP_ANGLE(angle1-90), width/2.0, &y, &x, &az);
    // Sometimes creating the new point can wrap the date line, even if the
    // original line does not.  This can cause problems later, so lets just
    // clip the result to date line and hope that the typical error is very
    // small.  FIXME: we should handle this more robustly in case larger
    // structures cross the date line, but that implies a much broader swath
    // of changes.
    if ( x - p2.x() > 180.0 ) x = -180.0;
    else if ( x - p2.x() < -180.0 ) x = 180.0;
    polygon.add_node(0, Point3D(x, y, 0));
    // cout << "corner 3 = " << Point3D(x, y, 0) << endl;

				// Corner 4
    geo_direct_wgs_84(0, p1.y(), p1.x(), CLAMP_ANGLE(angle1-90), width/2.0, &y, &x, &az);
    // Sometimes creating the new point can wrap the date line, even if the
    // original line does not.  This can cause problems later, so lets just
    // clip the result to date line and hope that the typical error is very
    // small.  FIXME: we should handle this more robustly in case larger
    // structures cross the date line, but that implies a much broader swath
    // of changes.
    if ( x - p1.x() > 180.0 ) x = -180.0;
    else if ( x - p1.x() < -180.0 ) x = 180.0;
    polygon.add_node(0, Point3D(x, y, 0));
    // cout << "corner 4 = " << Point3D(x, y, 0) << endl;

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
  // cout << "bnode = " << segment_list[0].get_pt(0, 0) << endl;
  for (i = 0; i < nSegments - 1; i++) {
      // cout << "point = " << i << endl;
      if (getIntersection(segment_list[i].get_pt(0, 0),
                          segment_list[i].get_pt(0, 1),
                          segment_list[i+1].get_pt(0, 0),
                          segment_list[i+1].get_pt(0, 1),
                          intersection)) {
          polygon.add_node(0, intersection);
          // cout << "bnode (int) = " << intersection << endl;
      } else {
          polygon.add_node(0, segment_list[i].get_pt(0, 1));
          // cout << "bnode = " << segment_list[i].get_pt(0, 1) << endl;
      }
  }
  polygon.add_node(0, segment_list[nSegments-1].get_pt(0, 1));
  // cout << "bnode = " << segment_list[nSegments-1].get_pt(0, 1) << endl;

				// Connect the top part
  polygon.add_node(0, segment_list[nSegments-1].get_pt(0, 2));
  // cout << "tnode = " << segment_list[nSegments-1].get_pt(0, 2) << endl;
  for (i = nSegments - 1; i > 0; i--) {
      // cout << "point = " << i << endl;
      if (getIntersection(segment_list[i].get_pt(0, 2),
                          segment_list[i].get_pt(0, 3),
                          segment_list[i-1].get_pt(0, 2),
                          segment_list[i-1].get_pt(0, 3),
                          intersection)) {
          polygon.add_node(0, intersection);
          // cout << "tnode (int) = " << intersection << endl;
      } else {
          polygon.add_node(0, segment_list[i].get_pt(0, 3));
          // cout << "tnode = " << segment_list[i].get_pt(0, 3) << endl;
      }
  }
  polygon.add_node(0, segment_list[0].get_pt(0, 3));
  // cout << "tnode = " << segment_list[0].get_pt(0, 3) << endl;

  polygon.set_hole_flag(0, 0);  // mark as solid
}

inline double CalculateTheta( Point3D p0, Point3D p1, Point3D p2 )
{
    Point3D u, v;
    double  udist, vdist, uv_dot, tmp;

    // u . v = ||u|| * ||v|| * cos(theta)

    u = p1 - p0;
    udist = sqrt( u.x() * u.x() + u.y() * u.y() );
    // printf("udist = %.6f\n", udist);

    v = p1 - p2;
    vdist = sqrt( v.x() * v.x() + v.y() * v.y() );
    // printf("vdist = %.6f\n", vdist);

    uv_dot = u.x() * v.x() + u.y() * v.y();
    // printf("uv_dot = %.6f\n", uv_dot);

    tmp = uv_dot / (udist * vdist);
    // printf("tmp = %.6f\n", tmp);

    return acos(tmp);
}

Point3D OffsetPointMiddle( Point3D prev, Point3D cur, Point3D next, double offset_by, int& turn_dir )
{
    double offset_dir;
    double next_dir;
    double az2;
    double dist;
    double theta;
    double pt_x = 0, pt_y = 0;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find average angle for contour: prev (" << prev << "), "
                                                                  "cur (" << cur  << "), "
                                                                 "next (" << next << ")" );

 
    // first, find if the line turns left or right ar src
    // for this, take the cross product of the vectors from prev to src, and src to next.
    // if the cross product is negetive, we've turned to the left
    // if the cross product is positive, we've turned to the right
    // if the cross product is 0, then we need to use the direction passed in
    SGVec3d dir1 = prev.toSGVec3d() - cur.toSGVec3d();
    dir1 = normalize(dir1);

    SGVec3d dir2 = next.toSGVec3d() - cur.toSGVec3d();
    dir2 = normalize(dir2);

    // Now find the average
    SGVec3d avg = dir1 + dir2;
    avg = normalize(avg);

    // check the turn direction
    SGVec3d cp = cross( dir1, dir2 );
    SG_LOG(SG_GENERAL, SG_DEBUG, "\tcross product of dir1: " << dir1 << " and dir2: " << dir2 << " is " << cp );

    // calculate the angle between cur->prev and cur->next
    theta = SGMiscd::rad2deg(CalculateTheta(prev, cur, next));

    if ( abs(theta - 180.0) < 0.1 )
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: (theta close to 180) theta is " << theta );

        // find the direction to the next point
        geo_inverse_wgs_84( cur.y(), cur.x(), next.y(), next.x(), &next_dir, &az2, &dist);

        offset_dir = next_dir - 90.0;
        while (offset_dir < 0.0)
        {
            offset_dir += 360.0;
        }

        // straight line blows up math - dist should be exactly as given
        dist = offset_by;        
    }
    else if ( abs(theta) < 0.1 )
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: (theta close to 0) : theta is " << theta );

        // find the direction to the next point
        geo_inverse_wgs_84( cur.y(), cur.x(), next.y(), next.x(), &next_dir, &az2, &dist);

        offset_dir = next_dir - 90;
        while (offset_dir < 0.0)
        {
            offset_dir += 360.0;
        }

        // straight line blows up math - dist should be exactly as given
        dist = offset_by;        
    }
    else
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: (theta NOT close to 180) : theta is " << theta );

        // find the offset angle
        geo_inverse_wgs_84( avg.y(), avg.x(), 0.0f, 0.0f, &offset_dir, &az2, &dist);

        // if we turned right, reverse the heading 
        if (cp.z() < 0.0f)
        {
            turn_dir = 0;
            offset_dir += 180.0;
        }
        else
        {
            turn_dir = 1;
        }

        while (offset_dir >= 360.0)
        {
            offset_dir -= 360.0;
        }

        // find the direction to the next point
        geo_inverse_wgs_84( cur.y(), cur.x(), next.y(), next.x(), &next_dir, &az2, &dist);

        // calculate correct distance for the offset point
        dist = (offset_by)/sin(SGMiscd::deg2rad(next_dir-offset_dir));
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << offset_dir << " distance is " << dist );

    // calculate the point from cur
    geo_direct_wgs_84( cur.y(), cur.x(), offset_dir, dist, &pt_y, &pt_x, &az2 );

    SG_LOG(SG_GENERAL, SG_DEBUG, "\tpoint is (" << pt_x << "," << pt_y << ")" );

    return Point3D(pt_x, pt_y, 0.0f);
}

Point3D OffsetPointFirst( Point3D cur, Point3D next, double offset_by )
{
    double offset_dir;
    double az2;
    double dist;
    double pt_x = 0, pt_y = 0;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at Start : cur (" << cur  << "), "
                                                            "next (" << next << ")" );

    // find the offset angle
    geo_inverse_wgs_84( cur.y(), cur.x(), next.y(), next.x(), &offset_dir, &az2, &dist);
    offset_dir -= 90;
    if (offset_dir < 0)
    {
        offset_dir += 360;
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << offset_dir << " distance is " << offset_by );

    // calculate the point from cur
    geo_direct_wgs_84( cur.y(), cur.x(), offset_dir, offset_by, &pt_y, &pt_x, &az2 );

    SG_LOG(SG_GENERAL, SG_DEBUG, "\tpoint is (" << pt_x << "," << pt_y << ")" );

    return Point3D(pt_x, pt_y, 0.0f);
}

Point3D OffsetPointLast( Point3D prev, Point3D cur, double offset_by )
{
    double offset_dir;
    double az2;
    double dist;
    double pt_x = 0, pt_y = 0;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at End   : prev (" << prev  << "), "
                                                              "cur (" << cur << ")" );

    // find the offset angle
    geo_inverse_wgs_84( prev.y(), prev.x(), cur.y(), cur.x(), &offset_dir, &az2, &dist);
    offset_dir -= 90;
    if (offset_dir < 0)
    {
        offset_dir += 360;
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << offset_dir << " distance is " << offset_by );

    // calculate the point from cur
    geo_direct_wgs_84( cur.y(), cur.x(), offset_dir, offset_by, &pt_y, &pt_x, &az2 );

    SG_LOG(SG_GENERAL, SG_DEBUG, "\tpoint is (" << pt_x << "," << pt_y << ")" );

    return Point3D(pt_x, pt_y, 0.0f);
}

Point3D midpoint( Point3D p0, Point3D p1 )
{
    return Point3D( (p0.x() + p1.x()) / 2, (p0.y() + p1.y()) / 2, (p0.z() + p1.z()) / 2 );
}

void
makePolygons (const Line &line, double width, poly_list& polys)
{
    int nPoints = line.getPointCount();
    int i;
    int turn_dir;

    Point3D cur_inner;
    Point3D cur_outer;
    Point3D prev_inner = Point3D(0.0f, 0.0f, 0.0f);
    Point3D prev_outer = Point3D(0.0f, 0.0f, 0.0f);

    double heading = 0.0f;
    double az2 = 0.0f;
    double dist = 0.0f;
    double pt_x = 0.0f;
    double pt_y = 0.0f;

    TGPolygon   poly;

    // generate poly and texparam lists for each line segment
    for (i=0; i<nPoints; i++)
    {
        turn_dir = 0;

        SG_LOG(SG_GENERAL, SG_DEBUG, "makePolygonsTP: calculating offsets for segment " << i);

        // for each point on the PointsList, generate a quad from
        // start to next, offset by 1/2 width from the edge
        if (i == 0)
        {
            // first point on the list - offset heading is 90deg 
            cur_outer = OffsetPointFirst( line.getPoint(i), line.getPoint(i+1), -width/2.0f );
            cur_inner = OffsetPointFirst( line.getPoint(i), line.getPoint(i+1),  width/2.0f );
        }
        else if (i == nPoints-1)
        {
            // last point on the list - offset heading is 90deg 
            cur_outer = OffsetPointLast( line.getPoint(i-1), line.getPoint(i), -width/2.0f );
            cur_inner = OffsetPointLast( line.getPoint(i-1), line.getPoint(i),  width/2.0f );
        }
        else
        {
            // middle section
            cur_outer = OffsetPointMiddle( line.getPoint(i-1), line.getPoint(i), line.getPoint(i+1), -width/2.0f, turn_dir );
            cur_inner = OffsetPointMiddle( line.getPoint(i-1), line.getPoint(i), line.getPoint(i+1),  width/2.0f, turn_dir );
        }

        if ( (prev_inner.x() != 0.0f) && (prev_inner.y() != 0.0f) )
        {
            Point3D prev_mp = midpoint( prev_outer, prev_inner );
            Point3D cur_mp  = midpoint( cur_outer,  cur_inner  );
            geo_inverse_wgs_84( prev_mp.y(), prev_mp.x(), cur_mp.y(), cur_mp.x(), &heading, &az2, &dist);

            poly.erase();

            poly.add_node( 0, prev_inner );
            poly.add_node( 0, prev_outer );

            // we need to extend one of the points so we're sure we don't create adjacent edges
            if (turn_dir == 0)
            {
                // turned right - offset outer
                geo_inverse_wgs_84( prev_outer.y(), prev_outer.x(), cur_outer.y(), cur_outer.x(), &heading, &az2, &dist);
                geo_direct_wgs_84( cur_outer.y(), cur_outer.x(), heading, MP_STRETCH, &pt_y, &pt_x, &az2 );

                poly.add_node( 0, Point3D( pt_x, pt_y, 0.0f) );
                //poly.add_node( 0, cur_outer );
                poly.add_node( 0, cur_inner );
            }
            else
            {
                // turned left - offset inner
                geo_inverse_wgs_84( prev_inner.y(), prev_inner.x(), cur_inner.y(), cur_inner.x(), &heading, &az2, &dist);
                geo_direct_wgs_84( cur_inner.y(), cur_inner.x(), heading, MP_STRETCH, &pt_y, &pt_x, &az2 );

                poly.add_node( 0, cur_outer );
                poly.add_node( 0, Point3D( pt_x, pt_y, 0.0f) );
                //poly.add_node( 0, cur_inner );
            }

            polys.push_back(poly);
        }

        prev_outer = cur_outer;
        prev_inner = cur_inner;
    }
}

void
makePolygonsTP (const Line &line, double width, poly_list& polys, texparams_list &tps)
{
    int nPoints = line.getPointCount();
    int i;
    int turn_dir;

    Point3D cur_inner;
    Point3D cur_outer;
    Point3D prev_inner = Point3D(0.0f, 0.0f, 0.0f);
    Point3D prev_outer = Point3D(0.0f, 0.0f, 0.0f);

    double last_end_v = 0.0f;
    double heading = 0.0f;
    double az2 = 0.0f;
    double dist = 0.0f;
    double pt_x = 0.0f;
    double pt_y = 0.0f;

    TGPolygon   poly;
    TGTexParams tp;

    // generate poly and texparam lists for each line segment
    for (i=0; i<nPoints; i++)
    {
        last_end_v   = 0.0f;
        turn_dir = 0;

        SG_LOG(SG_GENERAL, SG_DEBUG, "makePolygonsTP: calculating offsets for segment " << i);

        // for each point on the PointsList, generate a quad from
        // start to next, offset by 1/2 width from the edge
        if (i == 0)
        {
            // first point on the list - offset heading is 90deg 
            cur_outer = OffsetPointFirst( line.getPoint(i), line.getPoint(i+1), -width/2.0f );
            cur_inner = OffsetPointFirst( line.getPoint(i), line.getPoint(i+1),  width/2.0f );
        }
        else if (i == nPoints-1)
        {
            // last point on the list - offset heading is 90deg 
            cur_outer = OffsetPointLast( line.getPoint(i-1), line.getPoint(i), -width/2.0f );
            cur_inner = OffsetPointLast( line.getPoint(i-1), line.getPoint(i),  width/2.0f );
        }
        else
        {
            // middle section
            cur_outer = OffsetPointMiddle( line.getPoint(i-1), line.getPoint(i), line.getPoint(i+1), -width/2.0f, turn_dir );
            cur_inner = OffsetPointMiddle( line.getPoint(i-1), line.getPoint(i), line.getPoint(i+1),  width/2.0f, turn_dir );
        }

        if ( (prev_inner.x() != 0.0f) && (prev_inner.y() != 0.0f) )
        {
            Point3D prev_mp = midpoint( prev_outer, prev_inner );
            Point3D cur_mp  = midpoint( cur_outer,  cur_inner  );
            geo_inverse_wgs_84( prev_mp.y(), prev_mp.x(), cur_mp.y(), cur_mp.x(), &heading, &az2, &dist);

            poly.erase();

            poly.add_node( 0, prev_inner );
            poly.add_node( 0, prev_outer );

            // we need to extend one of the points so we're sure we don't create adjacent edges
            if (turn_dir == 0)
            {
                // turned right - offset outer
                geo_inverse_wgs_84( prev_outer.y(), prev_outer.x(), cur_outer.y(), cur_outer.x(), &heading, &az2, &dist);
                geo_direct_wgs_84( cur_outer.y(), cur_outer.x(), heading, MP_STRETCH, &pt_y, &pt_x, &az2 );

                poly.add_node( 0, Point3D( pt_x, pt_y, 0.0f) );
                //poly.add_node( 0, cur_outer );
                poly.add_node( 0, cur_inner );
            }
            else
            {
                // turned left - offset inner
                geo_inverse_wgs_84( prev_inner.y(), prev_inner.x(), cur_inner.y(), cur_inner.x(), &heading, &az2, &dist);
                geo_direct_wgs_84( cur_inner.y(), cur_inner.x(), heading, MP_STRETCH, &pt_y, &pt_x, &az2 );

                poly.add_node( 0, cur_outer );
                poly.add_node( 0, Point3D( pt_x, pt_y, 0.0f) );
                //poly.add_node( 0, cur_inner );
            }

            polys.push_back(poly);

            tp = TGTexParams( prev_inner, width, 20.0f, heading );
            tp.set_minv(last_end_v);
            tps.push_back(tp);

            last_end_v = 1.0f - (fmod( (double)(dist - last_end_v), (double)1.0f ));
        }

        prev_outer = cur_outer;
        prev_inner = cur_inner;
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
parseChunk (const string &s, double delta)
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
  bounds.setMax(Point3D(x + delta, y + delta, 0));

  return bounds;
}

Rectangle
parseTile (const string &s)
{
  return( parseChunk(s, 1.0) );
}

};

// end of util.cxx
