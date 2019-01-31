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

#include <terragear/tg_accumulator.hxx>
#include <terragear/tg_shapefile.hxx>
#include <terragear/tg_misc.hxx>

#include "tgconstruct.hxx"

using std::string;

bool TGConstruct::ClipLandclassPolys( void ) {
    tgPolygon clipped, tmp;
    tgPolygon remains;
    tgPolygon safety_base;
    tgcontour_list slivers;
    bool debug_shape;
    tgAccumulator accum(bucket.gen_index_str());
    unsigned int accum_idx = 0;

    // set up clipping tile : and remember to add the nodes!
    SGGeod p = bucket.get_corner( SG_BUCKET_SW );
    p.setElevationM( -9999.0 );
    safety_base.AddNode( 0, p );
    nodes.unique_add( p );

    p = bucket.get_corner( SG_BUCKET_SE );
    p.setElevationM( -9999.0 );
    safety_base.AddNode( 0, p );
    nodes.unique_add( p );

    p = bucket.get_corner( SG_BUCKET_NE );
    p.setElevationM( -9999.0 );
    safety_base.AddNode( 0, p );
    nodes.unique_add( p );

    p = bucket.get_corner( SG_BUCKET_NW );
    p.setElevationM( -9999.0 );
    safety_base.AddNode( 0, p );
    nodes.unique_add( p );

    // set up land mask, we clip most things to this since it is our
    // best representation of land vs. ocean.  If we have other less
    // accurate data that spills out into the ocean, we want to just
    // clip it.
    // also set up a mask for all water and islands
    tgPolygon land_mask, water_mask, island_mask;
    tgpolygon_list land_list, water_list, island_list;

    for ( unsigned int i = 0; i < area_defs.size(); i++ ) {
        if ( area_defs.is_landmass_area(i) && !ignoreLandmass ) {
            for ( unsigned int j = 0; j < polys_in.area_size(i); ++j ) {
                land_list.push_back( polys_in.get_poly(i, j) );
            }
        } else if ( area_defs.is_water_area(i) ) {
            for (unsigned int j = 0; j < polys_in.area_size(i); j++) {
                water_list.push_back( polys_in.get_poly(i, j) );
            }
        } else if ( area_defs.is_island_area(i) ) {
            for (unsigned int j = 0; j < polys_in.area_size(i); j++) {
                island_list.push_back( polys_in.get_poly(i, j) );
            }
        }
    }

    land_mask   = tgPolygon::Union( land_list );
    water_mask  = tgPolygon::Union( water_list );
    island_mask = tgPolygon::Union( island_list );

    // Dump the masks
    if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
        tgShapefile::FromPolygon( land_mask, ds_name, "land_mask", "" );
        tgShapefile::FromPolygon( water_mask, ds_name, "water_mask", "" );
        tgShapefile::FromPolygon( island_mask, ds_name, "island_mask", "" );
    }

    // process polygons in priority order
    for ( unsigned int i = 0; i < area_defs.size(); i++ ) {
        bool debug_area = IsDebugArea( i );
        for( unsigned int j = 0; j < polys_in.area_size(i); ++j ) {
            tgPolygon& current = polys_in.get_poly(i, j);
            debug_shape = IsDebugShape( polys_in.get_poly( i, j ).GetId() );

            SG_LOG( SG_CLIPPER, SG_DEBUG, "Clipping " << area_defs.get_area_name( i ) << "(" << i << "):" << j+1 << " of " << polys_in.area_size(i) << " id " << polys_in.get_poly( i, j ).GetId() );

            tmp = current;

            // if not a hole, clip the area to the land_mask
            if ( !ignoreLandmass && !area_defs.is_hole_area(i) ) {
                tmp = tgPolygon::Intersect( tmp, land_mask );
            }

            // if a water area, cut out potential islands
            if ( area_defs.is_water_area(i) ) {
                // clip against island mask
                tmp = tgPolygon::Diff( tmp, island_mask );
            }

            if ( debug_area || debug_shape ) {
                char layer[32];
                char name[32];

                sprintf(layer, "pre_clip_%d", polys_in.get_poly( i, j ).GetId() );
                sprintf(name, "shape %u,%u", i,j);
                tgShapefile::FromPolygon( tmp, ds_name, layer, name );
                tgPolygon::ToClipperFile( tmp, ds_name, layer );
                
                sprintf(layer, "pre_clip_accum_%d", polys_in.get_poly( i, j ).GetId() );
                accum.ToShapefiles( ds_name, layer, false );
                //accum.ToClipperFile( ds_name, layer, false );
            }

            clipped = accum.Diff( tmp );

            // only add to output list if the clip left us with a polygon
            if ( clipped.Contours() > 0 ) {

#if FIND_SLIVERS
                // move slivers from clipped polygon to slivers polygon
                tgPolygon::RemoveSlivers( clipped, slivers );
#endif

                // add the sliverless result polygon to the clipped polys list
                if ( clipped.Contours() > 0  ) {
                    // copy all of the superpolys and texparams
                    clipped.SetId( polys_in.get_poly( i, j ).GetId() );

                    // shape.sps.push_back( sp );
                    polys_clipped.add_poly( i, clipped );

                    if ( debug_area || debug_shape ) {
                        char layer[32];
                        char name[32];

                        sprintf(layer, "post_clip_%d", polys_in.get_poly( i, j ).GetId() );
                        sprintf(name, "shape %u,%u", i,j);

                        tgShapefile::FromPolygon( clipped, ds_name, layer, name );
                    }
                }
            }
            if ( debug_area || debug_shape ) {
                char layer[32];
                sprintf(layer, "pre2_clip_accum_%d", polys_in.get_poly( i, j ).GetId() );
                
                accum.ToShapefiles( ds_name, layer, false );
                accum.ToClipperfiles( ds_name, layer, false );
            }
            accum.Add( tmp );
            if ( debug_area || debug_shape ) {
                char layer[32];
                sprintf(layer, "post_clip_accum_%u_%u", accum_idx++, polys_in.get_poly( i, j ).GetId() );
                
                accum.ToShapefiles( ds_name, layer, false );
                accum.ToClipperfiles( ds_name, layer, false );
            }
        }
    }

#if 0
    if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
        // Dump the sliver list
        char name[32];
        for ( unsigned int i=0; i<slivers.size(); i++ ) {
            sprintf( name, "sliver %4d", i );
            tgShapefile::FromContour( slivers[i], ds_name, "slivers", name );
        }
    }
#endif

#if FIND_SLIVERS
    // Now, merge any slivers with clipped polys
    // merge_slivers(polys_clipped, slivers);
    for ( unsigned int i = 0; i < area_defs.size(); i++ ) {
        tgPolygon::MergeSlivers( polys_clipped.get_polys(i), slivers );
    }
#endif

    slivers.clear();

    if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
        char layer[32];
        char name[32];
        
        sprintf(layer, "tile_rect" );
        sprintf(name, "shape");
        
        tgShapefile::FromPolygon( safety_base, ds_name, layer, name );
        //tgPolygon::ToClipperFile( safety_base, ds_name, layer );
    }
    
    // finally, what ever is left over goes to ocean
    remains = accum.Diff( safety_base );
    
    // tgShapefile::FromPolygon( remains, ds_name, "remains", "poly" );
    
    if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
        char layer[32];
	    char name[32];

	    sprintf(layer, "remains_sb" );
	    sprintf(name, "shape");

	    tgShapefile::FromPolygon( remains, ds_name, layer, name );      
    }


    remains = tgPolygon::RemoveDups( remains );
    remains = tgPolygon::RemoveCycles( remains );

    if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
        char layer[32];
	    char name[32];

	    sprintf(layer, "remains_postclean" );
	    sprintf(name, "shape");

	    tgShapefile::FromPolygon( remains, ds_name, layer, name );      
    }
    
    if ( remains.Contours() > 0 ) {
        // cout << "remains contours = " << remains.contours() << endl;
        // move slivers from remains polygon to slivers polygon

#if FIND_SLIVERS
        tgPolygon::RemoveSlivers( remains, slivers );

        if ( slivers.size() > 0 ) {

            if ( debug_all || debug_shapes.size() || debug_areas.size() ) {
                // Dump the sliver list
                char name[32];
                for ( unsigned int i=0; i<slivers.size(); i++ ) {
                    sprintf( name, "sliver %4d", i );
                    tgShapefile::FromContour( slivers[i], ds_name, "remains slivers", name );
                }
            }

            for ( unsigned int i = 0; i < area_defs.size(); i++ ) {
                tgPolygon::MergeSlivers( polys_clipped.get_polys(i), slivers );
            }
        }
#endif

        if ( remains.Contours() > 0 ) {
            remains.SetMaterial( area_defs.get_sliver_area_name() );
            remains.SetTexMethod( TG_TEX_BY_GEODE, bucket.get_center_lat() );
            remains.SetId(9999);

            SG_LOG( SG_CLIPPER, SG_DEBUG, "Adding remains to area " << area_defs.get_sliver_area_priority() );
            polys_clipped.add_poly( area_defs.get_sliver_area_priority(), remains );
        }
    }

    // Now make sure any newly added intersection nodes are added to the tgnodes
    for (unsigned int area = 0; area < area_defs.size(); area++) {
        bool isRoad = area_defs.is_road_area( area );
        for (unsigned int idxPoly = 0; idxPoly < polys_clipped.area_size(area); ++idxPoly ) {
            tgPolygon& poly = polys_clipped.get_poly( area, idxPoly );

            SG_LOG( SG_CLIPPER, SG_DEBUG, "Collecting nodes for " << area_defs.get_area_name(area) << ":" << idxPoly + 1 << " of " << polys_clipped.area_size(area) );

            for (unsigned int con=0; con < poly.Contours(); con++) {
                for (unsigned int n = 0; n < poly.ContourSize( con ); n++) {
                    // ensure we have all nodes...
                    SGGeod node = poly.GetNode( con, n );
                    if ( CheckMatchingNode( node, isRoad, false ) ) {
                        poly.SetNode( con, n, node );
                    } else {
                        poly.DelNode( con, n );
                    }
                }
            }
        }
    }

    return true;
}
