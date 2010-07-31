// runway.hxx -- class to store runway info
//
// Written by Curtis Olson, started November 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
//
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: runway.hxx,v 1.16 2005-04-20 18:20:15 curt Exp $
//


#ifndef _RUNWAY_HXX
#define _RUNWAY_HXX


#include <string>
#include <vector>

#include <Geometry/point3d.hxx>

#include <Polygon/polygon.hxx>


struct TGRunway {
    std::string rwy_no;

    double lon;
    double lat;
    double heading;
    double length;
    double width;
    double disp_thresh1;
    double disp_thresh2;
    double stopway1;
    double stopway2;

    std::string lighting_flags;
    int surface_code;
    std::string shoulder_code;
    int marking_code;
    double smoothness;
    bool   dist_remaining;

    double gs_angle1;
    double gs_angle2;

    TGPolygon threshold;
    TGPolygon tens, tens_margin, ones, ones_margin;
    TGPolygon letter, letter_margin_left, letter_margin_right;
    TGPolygon pre_td_zone;
    TGPolygon td3_zone, td2_zone, td1a_zone, td1b_zone;
    TGPolygon aim_point;

    bool really_taxiway;
    bool generated;
};


typedef std::vector < TGRunway > runway_list;
typedef runway_list::iterator runway_list_iterator;
typedef runway_list::const_iterator const_runway_list_iterator;


// given a runway center point, length, width, and heading, and
// altitude (meters) generate the lon and lat 4 corners using wgs84
// math.
TGPolygon gen_wgs84_area( Point3D origin,
                          double length_m,
                          double displ1, double displ2,
                          double width_m,
                          double heading_deg,
                          double alt_m,
                          bool add_mid );

// generate an area for a runway with expantion specified as a scale
// factor (return result points in degrees)
TGPolygon gen_runway_area_w_scale( const TGRunway& runway, 
                                   double alt_m,
				   double length_scale = 1.0,
				   double width_scale = 1.0 );

// generate an area for a runway with expansion specified in meters
// (return result points in degrees)
TGPolygon gen_runway_area_w_extend( const TGRunway& runway, 
                                    double alt_m,
				    double length_extend,
                                    double displ1, double displ2,
				    double width_extend );


// generate an area for half a runway
TGPolygon gen_runway_w_mid( const TGRunway& runway,
                            double alt_m,
			    double length_extend_m,
			    double width_extend_m );


#endif // _RUNWAY_HXX
