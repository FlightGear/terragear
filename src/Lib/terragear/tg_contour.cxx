#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/SGGeometry.hxx>
#include <simgear/io/lowlevel.hxx>
#include <simgear/debug/logstream.hxx>

#include "tg_misc.hxx"
#include "tg_accumulator.hxx"
#include "tg_contour.hxx"
#include "tg_polygon.hxx"
#include "tg_unique_tgnode.hxx"

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
    unsigned int p1_index, p3_index;
    double       min_angle = 2.0 * SGD_PI;
    unsigned int size = node_list.size();

    SG_LOG(SG_GENERAL, SG_DEBUG, "  tgContour::GetMinimumAngle() : contour size is " << size );

    for ( unsigned int i = 0; i < size; i++ ) {
        if ( i == 0) {
            p1_index = size -1;
        } else {
            p1_index = i - 1;
        }

        unsigned p2_index = i;

        if ( i == size - 1 ) {
            p3_index = 0;
        } else {
            p3_index = i + 1;
        }

        double angle = SGGeod_CalculateTheta( node_list[p1_index], node_list[p2_index], node_list[p3_index] );
        if ( angle < min_angle ) {
            min_angle = angle;
        }
    }

    return min_angle;
}

double tgContour::GetArea( void ) const
{
    double area = 0.0;

    if ( node_list.size() ) {
        unsigned j = node_list.size() - 1;
        for (unsigned i = 0; i < node_list.size(); ++i) {
            SGVec2d a = SGGeod_ToSGVec2d( node_list[i] );
            SGVec2d b = SGGeod_ToSGVec2d( node_list[j] );
            area += (b.x() + a.x()) * (b.y() - a.y());
            j = i;
        }
    }

    return fabs(area * 0.5);
}

// Check that the two supplied points are on the same side of the contour
bool tgContour::AreSameSide( const SGGeod& firstpt, const SGGeod& secondpt) const
{
    //Find equation of line segment joining the points
    double x1 = firstpt.getLatitudeDeg();
    double x2 = secondpt.getLatitudeDeg();
    double y1 = firstpt.getLongitudeDeg();
    double y2 = secondpt.getLongitudeDeg();
    
    //Store differences for later
    double xdif = x2-x1;
    double ydif = y2-y1;
    
    /*We describe a line parametrically:

      x1        (x2-x1)
      L =        + t 
      y1        (y2-y1)

      with u the parametric coefficient for the second line.
      Then the line segments intersect if 0 <= t,u <= 1.

      To determine t and u we use the approach of Goldman ("Graphics
      Gems" as described in Stack Overflow question 563198).

      if r x s = r_x * s_y - r_y * s_x, then

      t = (q - p) x s / (r x s)
      and 
      u = (q - p) x r / (r x s)

      for line 1 = p + t r, line 2 = q + u s
    */
    
    //Now cycle over all nodes and count how many times we intersect
    int intersect_ct = 0;
    if (node_list.size()) {
        for (int i = 0; i < static_cast<int>(node_list.size()) - 1; ++i) {
            double nx1 = node_list[i].getLatitudeDeg();
            double ny1 = node_list[i].getLongitudeDeg();
            double nx2 = node_list[i+1].getLatitudeDeg();
            double ny2 = node_list[i+1].getLongitudeDeg();
            double nydif = ny2-ny1;
            double nxdif = nx2-nx1;
            double denom = xdif*nydif - ydif*nxdif;
            
            if (denom != 0) {     //Not parallel
                double crossx = nx1-x1; double crossy = ny1-y1;
                double t = (crossx*nydif - crossy*nxdif)/denom;
                double u = -1*(xdif*crossy - ydif*crossx)/denom;
                // We consider that an intersection at the edge of the line has
                // crossed
                // over, that is, they lie on opposite sides. This way we capture
                // places where the chopper has clipped a cliff on the tile edge
                if (t > -0.0001 && t < 1.0001 && u > -0.0001 && u < 1.0001) intersect_ct++;
            }
        }
    }
    
    bool isinter = (intersect_ct%2 == 0);
    return isinter;
}

double tgContour::MinDist(const SGGeod& probe) const {
    SGVec3d probexyz;
    SGGeodesy::SGGeodToCart( probe, probexyz );
    double mindist = 100000.0;
    
    if ( node_list.size() ) {
        int j = node_list.size() - 1;
        
        for (int i = 0; i < j; ++i) {
            SGVec3d start,end;
            SGGeodesy::SGGeodToCart( node_list[i],start );
            SGGeodesy::SGGeodToCart( node_list[i+1],end );
            SGLineSegment<double> piece = SGLineSegment<double>(start,end);
            double dist = distSqr( piece,probexyz );
            if (dist < mindist) mindist = dist;
        }
    }
    
    return sqrt(mindist);
}

bool tgContour::IsInside( const tgContour& inside, const tgContour& outside )
{
    // first contour is inside second if the intersection of first with second is == first
    // Intersection returns a polygon...
    tgPolygon result;
    bool isInside = false;
    result = Intersect( inside, outside );

    if ( result.Contours() == 1 ) {
        if ( result.GetContour(0) == inside ) {
            isInside = true;
        }
    }
    
    return isInside;
}

bool tgContour::RemoveCycles( const tgContour& subject, tgcontour_list& result )
{
    SG_LOG(SG_GENERAL, SG_DEBUG, "remove cycles : contour has " << subject.GetSize() << " points" );
    bool split = false;

    if ( subject.GetSize() > 2) {
        if ( subject.GetArea() > SG_EPSILON*SG_EPSILON ) {
            // Step 1 - find the first duplicate point
            for ( unsigned int i = 0; i < subject.GetSize() && !split; i++ ) {
                // first check until the end of the vector
                for ( unsigned int j = i + 1; j < subject.GetSize() && !split; j++ ) {
                    if ( SGGeod_isEqual2D( subject.GetNode(i), subject.GetNode(j) ) ) {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "detected a dupe: i = " << i << " j = " << j );
                        split = true;

                        tgContour first;
                        if ( j < subject.GetSize()-1 ) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "first contour is " << 0 << ".." << i << "," << j+1 << ".." <<  subject.GetSize()-1);

                            // first contour is (0 .. i) + (j+1..size()-1)
                            for ( unsigned int n=0; n<=i; n++) {
                                first.AddNode( subject.GetNode(n) );
                            }
                            for ( unsigned int n=j+1; n<=subject.GetSize()-1; n++) {
                                first.AddNode( subject.GetNode(n) );
                            }
                        } else {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "first contour is " << 0 << ".." << i );

                            // first contour is (0 .. i)
                            for ( unsigned int n=0; n<=i; n++) {
                                first.AddNode( subject.GetNode(n) );
                            }
                        }

                        tgContour second;
                        SG_LOG(SG_GENERAL, SG_DEBUG, "second contour is " << i << ".." << j-1 );

                        // second contour is (i..j-1)
                        for ( unsigned int n=i; n<j; n++) {
                            second.AddNode( subject.GetNode(n) );
                        }

                        // determine hole vs boundary
                        if ( IsInside( first, second ) ) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "first contur is within second contour " );

                            // first contour is inside second : mark first contour as opposite of subject
                            first.SetHole( !subject.GetHole() );
                            second.SetHole( subject.GetHole() );
                        } else if ( IsInside( second, first ) ) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "second contur is within first contour " );

                            // second contour is inside first : mark second contour as opposite of subject
                            first.SetHole(   subject.GetHole() );
                            second.SetHole( !subject.GetHole() );
                        } else {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "conturs are (mostly) disjoint " );

                            // neither contour is inside - bots are the same as subject
                            first.SetHole(   subject.GetHole() );
                            second.SetHole(  subject.GetHole() );
                        }

                        SG_LOG(SG_GENERAL, SG_DEBUG, "remove first: size " << first.GetSize() );
                        first.SetHole( subject.GetHole() );
                        RemoveCycles( first, result );

                        SG_LOG(SG_GENERAL, SG_DEBUG, "remove second: size " << second.GetSize() );
                        second.SetHole( subject.GetHole() );
                        RemoveCycles( second, result );
                    }
                }
            }

            if (!split) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "no dupes - complete" );
                result.push_back( subject );
            }
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, "degenerate contour: area is " << subject.GetArea() << " discard." );
        }
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, "degenerate contour: size is " << subject.GetSize() << " discard." );
    }

    return split;
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

ClipperLib::Path tgContour::ToClipper( const tgContour& subject )
{
    ClipperLib::Path  contour;

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
            ReversePath( contour );
        }
    } else {
        // boundaries need to be orientation: true
        if ( !Orientation( contour ) ) {
            //SG_LOG(SG_GENERAL, SG_INFO, "Building clipper contour - boundary contour needs to be reversed" );
            ReversePath( contour );
        }
    }

    return contour;
}

tgContour tgContour::FromClipper( const ClipperLib::Path& subject )
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

    for ( unsigned int i = 0; i < clip.Contours(); ++i ) {
        for ( unsigned int j = 0; j < clip.ContourSize( i ); ++j ) {
            all_nodes.add( clip.GetNode(i, j) );
        }
    }

    ClipperLib::Path  clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Paths clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Paths clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPath(clipper_subject, ClipperLib::PolyType::Subject, true);
    c.AddPaths(clipper_clip, ClipperLib::PolyType::Clip, true);
    c.Execute(ClipperLib::ClipType::Union, clipper_result, ClipperLib::PolyFillType::EvenOdd, ClipperLib::PolyFillType::EvenOdd);

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
    
    for ( unsigned int i = 0; i < clip.Contours(); ++i ) {
        for ( unsigned int j = 0; j < clip.ContourSize( i ); ++j ) {
            all_nodes.add( clip.GetNode(i, j) );
        }
    }

    ClipperLib::Path clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Paths clipper_clip   = tgPolygon::ToClipper( clip );
    ClipperLib::Paths clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPath(clipper_subject, ClipperLib::PolyType::Subject, true);
    c.AddPaths(clipper_clip, ClipperLib::PolyType::Clip, true);
    c.Execute(ClipperLib::ClipType::Difference, clipper_result, ClipperLib::PolyFillType::EvenOdd, ClipperLib::PolyFillType::EvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    return result;
}

tgPolygon tgContour::Intersect( const tgContour& subject, const tgContour& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.GetSize(); ++i ) {
        all_nodes.add( subject.GetNode(i) );
    }

    for ( unsigned int i = 0; i < clip.GetSize(); ++i ) {
        all_nodes.add( clip.GetNode(i) );
    }

    ClipperLib::Path  clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Path  clipper_clip    = tgContour::ToClipper( clip );
    ClipperLib::Paths clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPath(clipper_subject, ClipperLib::PolyType::Subject, true);
    c.AddPath(clipper_clip, ClipperLib::PolyType::Clip, true);
    c.Execute(ClipperLib::ClipType::Intersection, clipper_result, ClipperLib::PolyFillType::EvenOdd, ClipperLib::PolyFillType::EvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    return result;
}

static bool FindIntermediateNode( const SGGeod& start, const SGGeod& end,
                                  const std::vector<SGGeod>& nodes, SGGeod& result,
                                  double bbEpsilon, double errEpsilon )
{
    bool found_node = false;

    SGGeod p0 = start;
    SGGeod p1 = end;

    double xdist = fabs(p0.getLongitudeDeg() - p1.getLongitudeDeg());
    double ydist = fabs(p0.getLatitudeDeg()  - p1.getLatitudeDeg());

    double x_err_min = xdist + 1.0;
    double y_err_min = ydist + 1.0;

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

        double m = (p_min.getLatitudeDeg() - p_max.getLatitudeDeg()) / (p_min.getLongitudeDeg() - p_max.getLongitudeDeg());
        double b = p_max.getLatitudeDeg() - m * p_max.getLongitudeDeg();

        for ( int i = 0; i < (int)nodes.size(); ++i ) {
            // cout << i << endl;
            SGGeod current = nodes[i];

            if ( (current.getLongitudeDeg() > (p_min.getLongitudeDeg() + (bbEpsilon))) && (current.getLongitudeDeg() < (p_max.getLongitudeDeg() - (bbEpsilon))) ) {
                double y_err = fabs(current.getLatitudeDeg() - (m * current.getLongitudeDeg() + b));

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

        double m1 = (p_min.getLongitudeDeg() - p_max.getLongitudeDeg()) / (p_min.getLatitudeDeg() - p_max.getLatitudeDeg());
        double b1 = p_max.getLongitudeDeg() - m1 * p_max.getLatitudeDeg();

        for ( int i = 0; i < (int)nodes.size(); ++i ) {
            SGGeod current = nodes[i];

            if ( (current.getLatitudeDeg() > (p_min.getLatitudeDeg() + (bbEpsilon))) && (current.getLatitudeDeg() < (p_max.getLatitudeDeg() - (bbEpsilon))) ) {

                double x_err = fabs(current.getLongitudeDeg() - (m1 * current.getLatitudeDeg() + b1));

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

static bool FindIntermediateNode( const SGGeod& start, const SGGeod& end,
                                  const std::vector<TGNode*>& nodes, TGNode*& result,
                                  double bbEpsilon, double errEpsilon )
{
    bool found_node = false;
    
    SGGeod p0 = start;
    SGGeod p1 = end;
    
    double xdist = fabs(p0.getLongitudeDeg() - p1.getLongitudeDeg());
    double ydist = fabs(p0.getLatitudeDeg()  - p1.getLatitudeDeg());
    
    double x_err_min = xdist + 1.0;
    double y_err_min = ydist + 1.0;
    
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
        
        double m = (p_min.getLatitudeDeg() - p_max.getLatitudeDeg()) / (p_min.getLongitudeDeg() - p_max.getLongitudeDeg());
        double b = p_max.getLatitudeDeg() - m * p_max.getLongitudeDeg();
        
        for ( int i = 0; i < (int)nodes.size(); ++i ) {
            // cout << i << endl;
            SGGeod current = nodes[i]->GetPosition();
            
            if ( (current.getLongitudeDeg() > (p_min.getLongitudeDeg() + (bbEpsilon))) && (current.getLongitudeDeg() < (p_max.getLongitudeDeg() - (bbEpsilon))) ) {
                double y_err = fabs(current.getLatitudeDeg() - (m * current.getLongitudeDeg() + b));
                
                if ( y_err < errEpsilon ) {
                    found_node = true;
                    if ( y_err < y_err_min ) {
                        result = nodes[i];
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
        
        double m1 = (p_min.getLongitudeDeg() - p_max.getLongitudeDeg()) / (p_min.getLatitudeDeg() - p_max.getLatitudeDeg());
        double b1 = p_max.getLongitudeDeg() - m1 * p_max.getLatitudeDeg();
        
        for ( int i = 0; i < (int)nodes.size(); ++i ) {
            SGGeod current = nodes[i]->GetPosition();
            
            if ( (current.getLatitudeDeg() > (p_min.getLatitudeDeg() + (bbEpsilon))) && (current.getLatitudeDeg() < (p_max.getLatitudeDeg() - (bbEpsilon))) ) {
                
                double x_err = fabs(current.getLongitudeDeg() - (m1 * current.getLatitudeDeg() + b1));
                
                if ( x_err < errEpsilon ) {
                    found_node = true;
                    if ( x_err < x_err_min ) {
                        result = nodes[i];
                        x_err_min = x_err;
                    }
                }
            }
        }
    }
    
    return found_node;
}

static void AddIntermediateNodes( const SGGeod& p0, const SGGeod& p1, std::vector<SGGeod>& nodes, tgContour& result, double bbEpsilon, double errEpsilon )
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

extern SGGeod InterpolateElevation( const SGGeod& dst_node, const SGGeod& start, const SGGeod& end );

static void AddIntermediateNodes( const SGGeod& p0, const SGGeod& p1, bool preserve3d, std::vector<TGNode*>& nodes, tgContour& result, double bbEpsilon, double errEpsilon )
{
    TGNode* new_pt = nullptr;
    SGGeod new_geode;
    
    SG_LOG(SG_GENERAL, SG_BULK, "   " << p0 << " <==> " << p1 );
    
    bool found_extra = FindIntermediateNode( p0, p1, nodes, new_pt, bbEpsilon, errEpsilon );
    
    if ( found_extra ) {
        if ( preserve3d ) {
            // interpolate the new nodes elevation based on p0, p1            
            new_geode = InterpolateElevation( new_pt->GetPosition(), p0, p1 );
            
            SG_LOG(SG_GENERAL, SG_ALERT, "INTERPOLATE ELVATION between " << p0 << " and " << p1 << " returned elvation " << new_geode.getElevationM() );
            
            new_pt->SetElevation( new_geode.getElevationM() );
            new_pt->SetFixedPosition( true );
        }

        AddIntermediateNodes( p0, new_pt->GetPosition(), preserve3d, nodes, result, bbEpsilon, errEpsilon  );
        
        result.AddNode( new_pt->GetPosition() );
        SG_LOG(SG_GENERAL, SG_BULK, "    adding = " << new_pt->GetPosition() );
        
        AddIntermediateNodes( new_pt->GetPosition(), p1, preserve3d, nodes, result, bbEpsilon, errEpsilon  );
    }
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

    if (!subject.GetOpen()) {
      // add start of segment
      result.AddNode( p0 );
      
      // add intermediate points
      AddIntermediateNodes( p0, p1, nodes, result, SG_EPSILON*10, SG_EPSILON*4 );
    }
    // maintain original hole and openness flag setting
    result.SetHole( subject.GetHole() );
    result.SetOpen( subject.GetOpen() );

    return result;
}

tgContour tgContour::AddColinearNodes( const tgContour& subject, bool preserve3d, std::vector<TGNode*>& nodes )
{
    SGGeod p0, p1;
    tgContour result;
    
    for ( unsigned int n = 0; n < subject.GetSize()-1; n++ ) {
        p0 = subject.GetNode( n );
        p1 = subject.GetNode( n+1 );
        
        // add start of segment
        result.AddNode( p0 );
        
        // add intermediate points
        AddIntermediateNodes( p0, p1, preserve3d, nodes, result, SG_EPSILON*10, SG_EPSILON*4 );
    }
    
    p0 = subject.GetNode( subject.GetSize() - 1 );
    p1 = subject.GetNode( 0 );

    if(!subject.GetOpen()) {
      // add start of segment
      result.AddNode( p0 );
      
      // add intermediate points
      AddIntermediateNodes( p0, p1, preserve3d, nodes, result, SG_EPSILON*10, SG_EPSILON*4 );
    }
    // maintain original hole and open flag settings
    result.SetHole( subject.GetHole() );
    result.SetOpen( subject.GetOpen() );
    
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

tgContour tgContour::Expand( const tgContour& subject, double offset )
{
    tgPolygon poly;
    tgContour result;

    poly.AddContour( subject );
    ClipperLib::Paths clipper_src, clipper_dst;

    clipper_src = tgPolygon::ToClipper( poly );

    ClipperLib::ClipperOffset co(2.0, 2.0);
    co.AddPaths(clipper_src, ClipperLib::JoinType::Square, ClipperLib::EndType::ClosedPolygon);
    co.Execute(clipper_dst, Dist_ToClipper(offset) );
  
    poly = tgPolygon::FromClipper( clipper_dst );

    if ( poly.Contours() == 1 ) {
        result = poly.GetContour( 0 );
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "Expanding contour resulted in more than 1 contour ! ");
        exit(0);
    }

    return result;
}

tgpolygon_list tgContour::ExpandToPolygons( const tgContour& subject, double width )
{
    return ExpandToPolygons(subject,width,TG_TEX_BY_TPS_CLIPU);
}

tgpolygon_list tgContour::ExpandToPolygons( const tgContour& subject, double width, int texturing )
{
    SGGeod cur_inner;
    SGGeod cur_outer;
    SGGeod prev_inner;
    SGGeod prev_outer;
    SGGeod calc_inner;
    SGGeod calc_outer;

    tgContour      expanded;
    tgPolygon      segment;
    tgAccumulator  accum("ExpandToPolygons");
    tgpolygon_list result;
    double total_length = 0;    //Will become texture cooordinate

    // generate poly and texparam lists for each line segment
    for (unsigned int i = 0; i < subject.GetSize(); i++)
    {
        int turn_dir = 0;

        SG_LOG(SG_GENERAL, SG_DEBUG, "makePolygonsTP: calculating offsets for segment " << i);

        // for each point on the PointsList, generate a quad from
        // start to next, offset by 1/2 width from the edge
        if (i == 0)
        {
            // first point on the list - offset heading is 90deg
            cur_outer = OffsetPointFirst( subject.GetNode(i), subject.GetNode(1), -width/2.0f );
            cur_inner = OffsetPointFirst( subject.GetNode(i), subject.GetNode(1),  width/2.0f );
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
		if ( intersection( prev_inner, prev_outer, cur_inner, cur_outer, intersect ) )
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
		if ( intersection( prev_inner, prev_outer, cur_inner, cur_outer, intersect ) )
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

            double last_end_v = 0.0;

            expanded.SetHole(false);
            segment.AddContour(expanded);
            segment.SetTexParams( prev_inner, width, 20.0f, heading );
	    if( texturing == TG_TEX_BY_TPS_CLIPU )
	    {
		segment.SetTexLimits( 0, last_end_v, 1, 1 );
		segment.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1.0, 0.0, 1.0, 0.0 );
	    }
	    else
	    {
		segment.SetTexMethod( TG_TEX_BY_HORIZ_REF );
		segment.SetTexReference( prev_mp, total_length ); //Ref is midpoint
                SG_LOG(SG_GENERAL, SG_DEBUG, "HORIZ_REF " << total_length);
	    }
            result.push_back( segment );

	    total_length = total_length + dist; //Total length of line

            // BUG??: value will never be utilized
            last_end_v = 1.0f - (fmod( (double)(dist - last_end_v), (double)1.0f ));
        }

        prev_outer = cur_outer;
        prev_inner = cur_inner;
    }

    return result;
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
    // Sanity check
    if ( count > 1000000 ) {
      SG_LOG(SG_GENERAL,SG_ALERT, "Got bad contour count " << count);
      exit(1);
    }
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
