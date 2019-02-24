// tgconstruct_poly.cxx -- load and handle polygon data
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

#include <simgear/misc/sg_dir.hxx>
#include <simgear/debug/logstream.hxx>

#include <terragear/tg_shapefile.hxx>

#include "tgconstruct.hxx"

using std::string;

static unsigned int cur_poly_id = 0;

// load all 2d polygons from the specified load disk directories and
// clip against each other to resolve any overlaps
int TGConstruct::LoadLandclassPolys( void ) {
    string base = bucket.gen_base_path();
    string poly_path;
    int    total_polys_read = 0;
    tgPolygon poly;

    // load 2D polygons from all directories provided
    for ( int i = 0; i < (int)load_dirs.size(); ++i ) {
        poly_path = work_base + "/" + load_dirs[i] + '/' + base;

        string tile_str = bucket.gen_index_str();
        simgear::Dir d(poly_path);
        if (!d.exists()) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "directory not found: " << poly_path);
            continue;
        }

        simgear::PathList files = d.children(simgear::Dir::TYPE_FILE);
        SG_LOG( SG_CLIPPER, SG_DEBUG, files.size() << " Polys in " << d.path() );

        for (const SGPath& p : files) {
            if (p.file_base() != tile_str) {
                continue;
            }

            string lext = p.complete_lower_extension();
            string lastext = p.extension();
            if ((lext == "arr") || (lext == "arr.gz") || (lext == "btg.gz") ||
                (lext == "fit") || (lext == "fit.gz") || (lext == "ind") ||
                (lastext == "cliffs") || (lext == "arr.rectified.gz") || (lext == "arr.new.gz"))
            {
                // skipped!
            } else {
                std::string material;
                gzFile fp = gzopen( p.c_str(), "rb" );
                unsigned int count;

                sgReadUInt( fp, &count );
                if ( count > 1000000 ) {
                    SG_LOG( SG_GENERAL, SG_ALERT, " Too many polys in " << p.realpath() << ":" << count );
                    exit( EXIT_FAILURE );
                }
                SG_LOG( SG_GENERAL, SG_DEBUG, " Load " << count << " polys from " << p.realpath() );

                for ( unsigned int idx = 0; idx < count; ++idx ) {
                    if ( poly.LoadFromGzFile( fp ) == EXIT_FAILURE ) {
                        SG_LOG( SG_GENERAL, SG_DEBUG, "Failed to load from " << p.realpath() );
                        return EXIT_FAILURE;
                    }
                    int area = area_defs.get_area_priority( poly.GetFlag() );
                    material = area_defs.get_area_name( area );
                    bool isRoad = area_defs.is_road_area( area );

                    poly.SetMaterial( material );
                    poly.SetId( cur_poly_id++ );

                    if ( poly.Contours() ) {
                        // add the nodes
                        for (unsigned int j=0; j<poly.Contours(); j++) {
                            for (int k=poly.ContourSize(j)-1; k>= 0; k--) {
                                SGGeod node  = poly.GetNode( j, k );
                                bool isFixed = poly.GetPreserve3D();
                                
                                if ( CheckMatchingNode( node, isRoad, isFixed ) ) {
                                    poly.SetNode( j, k, node );
                                } else {
                                    poly.DelNode( j, k );
                                }
                            }
                        }

                        if (IsDebugShape( poly.GetId() )) {
                            char layer[32];
                            sprintf(layer, "loaded_%d", poly.GetId() );

                            tgShapefile::FromPolygon( poly, ds_name, layer, material.c_str() );
                        }

                        /* make sure we loaded a valid poly */
                        poly = tgPolygon::RemoveDups(poly);
                        poly = tgPolygon::RemoveCycles(poly);
                        poly = tgPolygon::RemoveDups(poly);
                        poly = tgPolygon::RemoveBadContours(poly);
                        if ( poly.Contours() ) {
                            polys_in.add_poly( area, poly );
                            total_polys_read++;
                        }
                    }
                }

                gzclose( fp );
                SG_LOG(SG_GENERAL, SG_DEBUG, " Loaded " << p.file());
            }
        } // of directory file children
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, " Total polys read in this tile: " <<  total_polys_read );

    return total_polys_read;
}

bool TGConstruct::CheckMatchingNode( SGGeod& node, bool road, bool fixed )
{
    bool   matched = false;
    bool   added = false;
    
    if ( fixed ) {
        node = nodes.unique_add_fixed_elevation( node );
        added = true;
    } else {
        // check mutable edge
        if ( !nm_north.empty() ) {
            double north_compare = bucket.get_center_lat() + 0.5 * bucket.get_height();
            if ( fabs(node.getLatitudeDeg() - north_compare) < SG_EPSILON) {
                if ( !road ) {
                    // node is on the non_mutable northern border - get closest pt
                    node = GetNearestNodeLongitude( node, nm_north );
                    SG_LOG(SG_GENERAL, SG_DEBUG, " AddNode: constrained on north border from " <<  node << " to " << node );
                    added = true;
                } else {
                    matched = true;
                }
            } 
        } 
        
        if ( !added && !matched && !nm_south.empty() ) {
            double south_compare = bucket.get_center_lat() - 0.5 * bucket.get_height();
            if ( fabs(node.getLatitudeDeg() - south_compare) < SG_EPSILON) {
                if ( !road ) {
                    // node is on the non_mutable southern border - get closest pt
                    node = GetNearestNodeLongitude( node, nm_south );
                    SG_LOG(SG_GENERAL, SG_DEBUG, " AddNode: constrained on south border from " <<  node << " to " << node );
                    added = true;
                } else {
                    matched = true;
                }
            }
        } 
        
        if ( !added && !matched && !nm_east.empty() ) {
            double east_compare  = bucket.get_center_lon() + 0.5 * bucket.get_width();
            if ( fabs(node.getLongitudeDeg() - east_compare) < SG_EPSILON) {
                if ( !road ) {
                    // node is on the non_mutable eastern border - get closest pt
                    node = GetNearestNodeLatitude( node, nm_east );
                    SG_LOG(SG_GENERAL, SG_DEBUG, " AddNode: constrained on east border from " <<  node << " to " << node );
                    added = true;
                } else {
                    matched = true;
                }
            }
        } 
        
        if ( !added && !matched && !nm_west.empty() ) {
            double west_compare  = bucket.get_center_lon() - 0.5 * bucket.get_width();
            if ( fabs(node.getLongitudeDeg() - west_compare) < SG_EPSILON) {
                if ( !road ) {
                    // node is on the non_mutable western border - get closest pt
                    node = GetNearestNodeLatitude( node, nm_west );
                    SG_LOG(SG_GENERAL, SG_DEBUG, " AddNode: constrained on west border from " <<  node << " to " << node );                
                    added = true;
                } else {
                    matched = true;
                }
            }
        } 
    }
    
    if (!added && !matched) {
        node = nodes.unique_add( node );
        added = true;
    }
    
    return added;
}

SGGeod TGConstruct::GetNearestNodeLongitude( const SGGeod& node, const std::vector<SGGeod>& selection )
{
    double       min_dist = std::numeric_limits<double>::infinity();
    unsigned int min_idx = 0;
    
    for ( unsigned int i = 0; i < selection.size(); ++i ) {
        double cur_dist = fabs( node.getLongitudeDeg() - selection[i].getLongitudeDeg() );
        if ( cur_dist < min_dist ) {
            min_dist = cur_dist;
            min_idx = i;
        }
    }
    
    return selection[min_idx];
}

SGGeod TGConstruct::GetNearestNodeLatitude( const SGGeod& node, const std::vector<SGGeod>& selection )
{
    double       min_dist = std::numeric_limits<double>::infinity();
    unsigned int min_idx = 0;
    
    for ( unsigned int i = 0; i < selection.size(); ++i ) {
        double cur_dist = fabs( node.getLatitudeDeg() - selection[i].getLatitudeDeg() );
        if ( cur_dist < min_dist ) {
            min_dist = cur_dist;
            min_idx = i;
        }
    }
    
    return selection[min_idx];
}
