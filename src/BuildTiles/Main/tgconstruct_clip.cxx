// tgconstruct_clip.cxx -- handle polygon clipping
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

#include <simgear/debug/logstream.hxx>

#include "tgconstruct.hxx"

using std::string;

bool TGConstruct::ClipLandclassPolys( void ) {
    TGPolygon clipped, tmp;
    TGPolygon remains;
    TGPolygon safety_base;
    poly_list slivers;
    int i, j;
    Point3D p;
    point2d min, max;
    bool debug_area, debug_shape;
    static int accum_idx = 0;

#if !USE_ACCUMULATOR
    TGPolygon accum;
#endif

    // Get clip bounds
    min.x = bucket.get_center_lon() - 0.5 * bucket.get_width();
    min.y = bucket.get_center_lat() - 0.5 * bucket.get_height();
    max.x = bucket.get_center_lon() + 0.5 * bucket.get_width();
    max.y = bucket.get_center_lat() + 0.5 * bucket.get_height();

#if USE_ACCUMULATOR

    tgPolygonInitClipperAccumulator();

#else
    accum.erase();
#endif

    // set up clipping tile : and remember to add the nodes!
    safety_base.erase();

    p = Point3D(min.x, min.y, -9999.0);
    safety_base.add_node( 0, p );
    nodes.unique_add( p );

    p = Point3D(max.x, min.y, -9999.0);
    safety_base.add_node( 0, p );
    nodes.unique_add( p );

    p = Point3D(max.x, max.y, -9999.0);
    safety_base.add_node( 0, p );
    nodes.unique_add( p );

    p = Point3D(min.x, max.y, -9999.0);
    safety_base.add_node( 0, p );
    nodes.unique_add( p );

    // set up land mask, we clip most things to this since it is our
    // best representation of land vs. ocean.  If we have other less
    // accurate data that spills out into the ocean, we want to just
    // clip it.
    // also set up a mask for all water and islands
    TGPolygon land_mask, water_mask, island_mask;
    land_mask.erase();
    water_mask.erase();
    island_mask.erase();

    for ( i = 0; i < TG_MAX_AREA_TYPES; i++ ) {
        if ( is_landmass_area( i ) && !ignoreLandmass ) {
            for ( unsigned int j = 0; j < polys_in.area_size(i); ++j ) {
                land_mask = tgPolygonUnion( polys_in.get_mask(i, j), land_mask );
            }

        } else if ( is_water_area( i ) ) {
            for (unsigned int j = 0; j < polys_in.area_size(i); j++) {
                water_mask = tgPolygonUnion( polys_in.get_mask(i, j), water_mask );
            }
        } else if ( is_island_area( i ) ) {
            for (unsigned int j = 0; j < polys_in.area_size(i); j++) {
                island_mask = tgPolygonUnion( polys_in.get_mask(i, j), island_mask );
            }
        }
    }

    // Dump the masks
    if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
        WriteDebugPoly( "land_mask", "", land_mask );
        WriteDebugPoly( "water_mask", "", water_mask );
        WriteDebugPoly( "island_mask", "", island_mask );
    }

    // process polygons in priority order
    for ( i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
        debug_area = IsDebugArea( i );
        for( j = 0; j < (int)polys_in.area_size(i); ++j ) {
            TGPolygon current = polys_in.get_mask(i, j);
            debug_shape = IsDebugShape( polys_in.get_shape( i, j ).id );

            SG_LOG( SG_CLIPPER, SG_INFO, "Clipping " << get_area_name( (AreaType)i ) << "(" << i << "):" << j+1 << " of " << polys_in.area_size(i) << " id " << polys_in.get_shape( i, j ).id );

            tmp = current;


            // if not a hole, clip the area to the land_mask
            if ( !ignoreLandmass && !is_hole_area( i ) ) {
                tmp = tgPolygonInt( tmp, land_mask );
            }

            // if a water area, cut out potential islands
            if ( is_water_area( i ) ) {
                // clip against island mask
                tmp = tgPolygonDiff( tmp, island_mask );

            }

            if ( debug_area || debug_shape ) {
                char layer[32];
                char name[32];
                sprintf(layer, "pre_clip_%d", polys_in.get_shape( i, j ).id );
                sprintf(name, "shape %d,%d", i,j);
                WriteDebugPoly( layer, name, tmp );

                sprintf(layer, "pre_clip_accum_%d_%d", accum_idx, polys_in.get_shape( i, j ).id );
                sprintf(name, "shape_accum %d,%d", i,j);

#if USE_ACCUMULATOR
                tgPolygonDumpAccumulator( ds_name, layer, name );
#else
                WriteDebugPoly( layer, name, accum );
#endif
            }


#if USE_ACCUMULATOR
            clipped = tgPolygonDiffClipperWithAccumulator( tmp );
#else            
            clipped = tgPolygonDiff( tmp, accum );
#endif


            if ( debug_area || debug_shape ) {
                char layer[32];
                char name[32];
                sprintf(layer, "post_clip_%d", polys_in.get_shape( i, j ).id );
                sprintf(name, "shape %d,%d", i,j);
                WriteDebugPoly( layer, name, clipped );
            }
            
            // only add to output list if the clip left us with a polygon
            if ( clipped.contours() > 0 ) {

#if FIND_SLIVERS
                // move slivers from clipped polygon to slivers polygon
                tgPolygonFindSlivers( clipped, slivers );
#endif

                // add the sliverless result polygon to the clipped polys list
                if ( clipped.contours() > 0  ) {
                    TGShape shape;

                    // copy all of the superpolys and texparams
                    shape.SetMask( clipped );
                    shape.textured = polys_in.get_textured( i, j );
                    shape.id  = polys_in.get_shape( i, j ).id;

                    shape.area= polys_in.get_shape( i, j ).area;
                    shape.sps = polys_in.get_shape( i, j ).sps;
                    shape.tps = polys_in.get_shape( i, j ).tps;

                    // shape.sps.push_back( sp );
                    polys_clipped.add_shape( i, shape );

                    if ( debug_area || debug_shape ) {
                        WriteDebugShape( "clipped", shape );
                    }
                }
            }

#if USE_ACCUMULATOR
            if ( debug_shape ) {
                tgPolygonAddToClipperAccumulator( tmp, true );
            } else {
            tgPolygonAddToClipperAccumulator( tmp, false );
            }
#else
            accum   = tgPolygonUnion( tmp, accum );
#endif

            if ( debug_area || debug_shape ) {
                char layer[32];
                char name[32];
                sprintf(layer, "post_clip_accum_%d_%d", accum_idx, polys_in.get_shape( i, j ).id );
                sprintf(name, "shape_accum %d,%d", i,j);
#if USE_ACCUMULATOR
                tgPolygonDumpAccumulator( ds_name, layer, name );
#else
                WriteDebugPoly( layer, name, accum );
#endif
            }
            accum_idx++;
        }
    }

    if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
        // Dump the sliver list
        WriteDebugPolys( "poly_slivers", slivers );
    }

#if FIND_SLIVERS
    // Now, merge any slivers with clipped polys
    merge_slivers(polys_clipped, slivers);
#endif

    slivers.clear();

    // finally, what ever is left over goes to ocean
#if USE_ACCUMULATOR
    remains = tgPolygonDiffClipperWithAccumulator( safety_base );
#else
    remains = tgPolygonDiff( safety_base, accum );
#endif

    if ( remains.contours() > 0 ) {
        // cout << "remains contours = " << remains.contours() << endl;
        // move slivers from remains polygon to slivers polygon

#if FIND_SLIVERS
        tgPolygonFindSlivers( remains, slivers );
#endif
        // cout << "  After sliver move:" << endl;
        // cout << "    remains = " << remains.contours() << endl;
        // cout << "    slivers = " << slivers.contours() << endl;

#if FIND_SLIVERS
        // merge any slivers with previously clipped
        // neighboring polygons
        if ( slivers.size() > 0 ) {

            if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
                // Dump the sliver list
                WriteDebugPolys( "remains_slivers", slivers );
            }

            merge_slivers(polys_clipped, slivers);
        }
#endif

        if ( remains.contours() > 0 ) {
            TGSuperPoly sp;
            TGShape shape;

            string material = get_area_name(get_sliver_target_area_type());

            sp.set_material( material );
            sp.set_poly( remains );
            shape.SetMask( remains );
            shape.textured = false;
            shape.sps.push_back( sp );

            polys_clipped.add_shape( (int)get_sliver_target_area_type(), shape );
        }
    }

#if USE_ACCUMULATOR

    tgPolygonFreeClipperAccumulator();

#endif

    // Once clipping is complete, intersect the individual segments with their clip masks
    for (unsigned int area = 0; area < TG_MAX_AREA_TYPES; area++) {
        for (unsigned int shape = 0; shape < polys_clipped.area_size(area); shape++ ) {
            SG_LOG( SG_CLIPPER, SG_INFO, "Seperating segments from clip mask for " << get_area_name( (AreaType)area ) << ":" << shape+1 << " of " << polys_clipped.area_size(area) );
            polys_clipped.get_shape(area, shape).IntersectPolys();
        }
    }

    // Now make sure any newly added intersection nodes are added to the tgnodes
    for (unsigned int area = 0; area < TG_MAX_AREA_TYPES; area++) {
        for (unsigned int shape = 0; shape < polys_clipped.area_size(area); shape++ ) {
            for (unsigned int segment = 0; segment < polys_clipped.shape_size(area, shape); segment++) {
                TGPolygon poly = polys_clipped.get_poly( area, shape, segment );

                SG_LOG( SG_CLIPPER, SG_INFO, "Collecting nodes for " << get_area_name( (AreaType)area ) << ":" << shape+1 << "-" << segment << " of " << polys_clipped.area_size(area) );

                for (int con=0; con < poly.contours(); con++) {
                    for (int node = 0; node < poly.contour_size( con ); node++) {
                        // ensure we have all nodes...
                        nodes.unique_add( poly.get_pt( con, node ) );
                    }
                }
            }
        }
    }

    return true;
}
