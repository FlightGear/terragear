// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#include <simgear/compiler.h>
#include "apt_math.hxx"

tgContour gen_wgs84_area( SGGeod origin,
                          double length_m,
                          double displ1, double displ2,
                          double width_m,
                          double heading_deg,
                          bool   add_mid )
{
    tgContour result;
    double length_hdg = heading_deg;
    double left_hdg   = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);

    // move to the +l end/center of the runway
    SGGeod ref = SGGeodesy::direct( origin, length_hdg, length_m / 2.0 - displ2 );

    // move to the l,-w corner (then we add points in a clockwise direction)
    result.AddNode( SGGeodesy::direct( ref, left_hdg, -width_m / 2.0) );

    // move to the l,w corner
    result.AddNode( SGGeodesy::direct( ref, left_hdg, width_m / 2.0 ) );

    if ( add_mid ) {
        // move to the 0,w point (then we add points in a clockwise direction)
        result.AddNode( SGGeodesy::direct( origin, left_hdg, width_m / 2.0 ) );
    }

    // move to the -l end/center of the runway
    ref = SGGeodesy::direct( origin, length_hdg, displ1 - length_m/2.0);

    // move to the -l,w corner (then we add points in a clockwise direction)
    result.AddNode( SGGeodesy::direct( ref, left_hdg, width_m / 2.0 ) );

    // move to the -l,-w corner
    result.AddNode( SGGeodesy::direct( ref, left_hdg, -width_m / 2.0 ) );

    if ( add_mid ) {
        // move to the 0,-w point (then we add points in a clockwise direction)
        result.AddNode( SGGeodesy::direct( origin, left_hdg, -width_m / 2.0 ) );
    }

    result.SetHole( false );
    return result;
}

tgContour gen_wgs84_area( SGGeod end1, SGGeod end2,
                          double length_ext,
                          double displ1, double displ2,
                          double width_m,
                          double heading_deg,
                          bool   add_mid )
{
    tgContour result;
    double left_hdg = SGMiscd::normalizePeriodic( 0, 360, heading_deg - 90.0 );
    double course1, course2, distance;

    SGGeodesy::inverse(end1, end2, course1, course2, distance);

    // Calc the center
    SGGeod center = SGGeodesy::direct(end1, course1, distance/2 );

    // move from end2 to the displaced threshold
    SGGeod ref = SGGeodesy::direct( end2, heading_deg, length_ext / 2.0 - displ2);


    // move to the l,-w corner
    result.AddNode( SGGeodesy::direct(ref, left_hdg, -width_m / 2.0) );

    // move to the l,w corner
    result.AddNode( SGGeodesy::direct(ref, left_hdg, width_m / 2.0) );

    if ( add_mid ) {
        // move to the 0,w point
        result.AddNode( SGGeodesy::direct( center, left_hdg, width_m / 2.0 ) );
    }

    // move to the end1 center to the displ. threshold
    ref = SGGeodesy::direct( end1, heading_deg, displ1 - length_ext / 2.0 );

    // move to the -l,w corner
    result.AddNode( SGGeodesy::direct( ref, left_hdg, width_m / 2.0 ) );

    // move to the -l,-w corner
    result.AddNode( SGGeodesy::direct( ref, left_hdg, -width_m / 2.0 ) );

    if ( add_mid ) {
        // move to the 0,-w point
        result.AddNode( SGGeodesy::direct( center, left_hdg, -width_m / 2.0 ) );
    }

    return result;
}

tgContour gen_wgs84_rect( SGGeod origin, double heading, double length, double width )
{
    tgContour result;
    SGGeod    p;

    // starting point is in the middle of the rectangle width, at the beginning - stretch to heading
    // Point 1 is -90deg, 1/2 width away
    double left_hdg = SGMiscd::normalizePeriodic(0, 360, heading -90 );
    p = SGGeodesy::direct( origin, left_hdg, width / 2.0 );
    result.AddNode( p );

    // Point 2 is heading, length away from point 1
    p = SGGeodesy::direct( p, heading, length );
    result.AddNode( p );

    // Point 3 is -90deg, -width away from point 2
    p = SGGeodesy::direct( p, left_hdg, -width );
    result.AddNode( p );

    // last point is heading, -length from point 3
    p = SGGeodesy::direct( p, heading, -length );
    result.AddNode( p );

    result.SetHole( false );
    return result;
}
