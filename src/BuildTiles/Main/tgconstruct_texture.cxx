
// tgconstruct_texture.cxx --Handle texture coordinate generation in tgconstruct
//
// Written by Curtis Olson, started May 1999.
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: construct.cxx,v 1.4 2004-11-19 22:25:49 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/texcoord.hxx>
#include <simgear/debug/logstream.hxx>

#include "tgconstruct.hxx"

TGPolygon TGConstruct::area_tex_coords( const TGPolygon& tri )
{
    TGPolygon result;
    result.erase();

    // lots of conversion needed to use simgear API - perhaps we need a new simgear API?
    for (int c=0; c<tri.contours(); c++)
    {
        // get the points, and calculate the elevations
        point_list nodes = tri.get_contour(c);
        std::vector< SGGeod > conv_geods;
        point_list tex_coords;

        for (int i = 0; i < (int)nodes.size(); ++i )
        {
            SGGeod conv_geod = SGGeod::fromDegM( nodes[i].x(), nodes[i].y(), nodes[i].z() );
            SG_LOG(SG_GENERAL, SG_DEBUG, "geod pt = " << nodes[i] );
            conv_geods.push_back( conv_geod );
        }

        // now calculate texture coordinates
        // generate identity interger list...
        std::vector< int > node_idx;
        for (int i = 0; i < (int)conv_geods.size(); i++) {
            node_idx.push_back(i);
        }

        std::vector< SGVec2f > tp_list = sgCalcTexCoords( bucket, conv_geods, node_idx );
        // generate a contour of texture coordinates from the tp list
        for (int i = 0; i < (int)tp_list.size(); i++)
        {
            tex_coords.push_back( Point3D::fromSGVec2( tp_list[i] ) );
        }
        result.add_contour( tex_coords, 0 );
    }

    return result;
}

TGPolygon TGConstruct::linear_tex_coords( const TGPolygon& tri, const TGTexParams& tp )
{
    TGPolygon result;
    int i, j;

    result.erase();

    SGGeod ref = tp.get_ref();
    double width = tp.get_width();
    double length = tp.get_length();
    double heading = tp.get_heading();
    double minu = tp.get_minu();
    double maxu = tp.get_maxu();
    double minv = tp.get_minv();
    double maxv = tp.get_maxv();
    SG_LOG( SG_GENERAL, SG_DEBUG, "section ref = " << ref );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  width   = " << width );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  length  = " << length );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  heading = " << heading );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  minv    = " << minv );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  maxv    = " << maxv );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  heading = " << heading );

    Point3D p, t;
    double x, y, tx, ty;

    for ( i = 0; i < tri.contours(); ++i )
    {
        for ( j = 0; j < tri.contour_size( i ); ++j )
        {
            p = tri.get_pt( i, j );
            SG_LOG(SG_GENERAL, SG_DEBUG, "tex coords for contour " << i << " point " << j << ": " << p );

            //
            // 1. Calculate distance and bearing from the center of
            // the feature
            //

            // given alt, lat1, lon1, lat2, lon2, calculate starting
            // and ending az1, az2 and distance (s).  Lat, lon, and
            // azimuth are in degrees.  distance in meters
            double az1, az2, dist;
            geo_inverse_wgs_84( 0, ref.getLatitudeDeg(), ref.getLongitudeDeg(), p.y(), p.x(),
                    &az1, &az2, &dist );
            SG_LOG(SG_GENERAL, SG_DEBUG, "basic course from ref = " << az2);

            //
            // 2. Rotate this back into a coordinate system where Y
            // runs the length of the runway and X runs crossways.
            //

            double course = az2 - heading;
            while ( course < -360 ) { course += 360; }
            while ( course > 360 ) { course -= 360; }
            SG_LOG( SG_GENERAL, SG_DEBUG,
                        "  course = " << course << "  dist = " << dist );

            //
            // 3. Convert from polar to cartesian coordinates
            //

            x = sin( course * SGD_DEGREES_TO_RADIANS ) * dist;
            y = cos( course * SGD_DEGREES_TO_RADIANS ) * dist;
            SG_LOG(SG_GENERAL, SG_DEBUG, "  x = " << x << " y = " << y);

            //
            // 4. Map x, y point into texture coordinates
            //
            double tmp;

            tmp = x / width;
            tx = tmp * (maxu - minu) + minu;

            if ( tx < -1.0 )  { tx = -1.0; }
            if ( tx > 1.0 ) { tx = 1.0; }

            SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ")");

            ty = (y/length) + minv;
            SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << ty << ")");

            t = Point3D( tx, ty, 0 );
            SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ", " << ty << ")");

            result.add_node( i, t );
        }
    }

    return result;
}

void TGConstruct::CalcTextureCoordinates( void )
{
    for ( unsigned int area = 0; area < TG_MAX_AREA_TYPES; area++ ) {
        for( unsigned int shape = 0; shape < polys_clipped.area_size(area); shape++ ) {
            for ( unsigned int segment = 0; segment < polys_clipped.shape_size(area, shape); segment++ ) {
                tgPolygon poly = polys_clipped.get_poly(area, shape, segment);
                SG_LOG( SG_CLIPPER, SG_INFO, "Texturing " << get_area_name( (AreaType)area ) << "(" << area << "): " <<
                        shape+1 << "-" << segment << " of " << polys_clipped.area_size(area) << " with " << poly.GetMaterial() );

                if ( polys_clipped.get_textured( area, shape ) ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "USE TEXTURE PARAMS for tex coord calculations" );
                    poly.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, -1, -1, 1, 1 );
                } else {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "USE SIMGEAR for tex coord calculations" );
                    poly.SetTexMethod( TG_TEX_BY_GEODE, bucket.get_center_lat() );
                }
                poly.Texture( );
                polys_clipped.set_poly(area, shape, segment, poly);
            }
        }
    }
}
