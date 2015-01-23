// tg_btg_mesh_simplify.cxx -- BTG mesh simplification
//
// Written by Peter Sadrozinski, started Dec 2014.
//
// Copyright (C) 2014  Curtis L. Olson  - http://www.flightgear.org/~curt
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
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifdef _MSC_VER
#  include <windows.h>
#endif

#include <cstdio>

#include "tg_btg_mesh.hxx"

// CGAL edge collapse API
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
// Non-default cost and placement policies
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk.h> 
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
// Visitor base
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>

#include <simgear/misc/texcoord.hxx>
#include <simgear/debug/logstream.hxx>
#include <terragear/tg_shapefile.hxx>

typedef CGAL::Line_3<tgBtgKernel>                             tgBtg_Line_3;

#define DEBUG_SIMPLIFY  (1)

//
// BGL property map which indicates whether an edge is marked as non-removable

// Currently, just makes the tile edges unremovable.
// I need to experiment to allo tile edge to be collapsed as long as the 
// placement of the new vertex is ON the tile edge.
struct Border_is_constrained_edge_map {
    const   tgBtgMesh* sm_ptr;
    typedef boost::graph_traits<tgBtgMesh>::edge_descriptor   key_type;
    typedef bool                                              value_type;
    typedef value_type                                        reference;
    typedef boost::readable_property_map_tag                  category;
    
    Border_is_constrained_edge_map() {}
    Border_is_constrained_edge_map(const tgBtgMesh& sm) : sm_ptr(&sm) {}
    
#if 0
    friend bool get(Border_is_constrained_edge_map m, const key_type& edge) {
        return CGAL::is_border(edge, *m.sm_ptr);
#else
    friend bool get(Border_is_constrained_edge_map m, const key_type& edge) {
        return false;
    }
#endif    
};

// Placement class
namespace SMS = CGAL::Surface_mesh_simplification;
typedef SMS::Constrained_placement<SMS::LindstromTurk_placement<tgBtgMesh>, Border_is_constrained_edge_map > ConstrainedPlacement;

// mesh simplification visitor ( called during edge collapse )
struct CollapseInfo
{
    CollapseInfo() 
        : collected(0)
        , processed(0)
        , collapsing(0)
        , collapsed(0)
        , non_collapsable(0)
        , cost_uncomputable(0) 
        , placement_uncomputable(0) {} 
    
    std::size_t collected ;
    std::size_t processed ;
    std::size_t collapsing ;
    std::size_t collapsed ;
    std::size_t non_collapsable ;
    std::size_t cost_uncomputable  ;
    std::size_t placement_uncomputable ; 
};

struct CollapseVisitor : SMS::Edge_collapse_visitor_base<tgBtgMesh>
{
    CollapseVisitor( CollapseInfo* ci, const std::string& n, double cl ) : cinfo(ci), center_lat(cl), name(n) {}
    char datasource[64];
    char layer[64];
    
    // Called during the collecting phase for each edge collected.
    void OnCollected( Profile const&, boost::optional<double> const& )
    {
        ++cinfo->collected;
    }                
    
    // Called during the processing phase for each edge selected.
    // If cost is absent the edge won't be collapsed.
    void OnSelected(Profile const&, boost::optional<double> cost, std::size_t initial, std::size_t current)
    {
        ++cinfo->processed;
        if ( !cost ) {
            ++cinfo->cost_uncomputable;
        }
    }                
    
    // Called during the processing phase for each edge being collapsed.
    // If placement is absent the edge is left uncollapsed.
    void OnCollapsing(Profile const& profile, boost::optional<Point> placement)
    {
        if ( !placement ) {
            ++cinfo->placement_uncomputable;
        } else {
            ++cinfo->collapsing;            
            
#if 0
            // DEBUG_SIMPLIFY            
            sprintf( datasource, "./simp_dbg/%s", name.c_str() );
            sprintf( layer, "collapsing_verts_%04lu", cinfo->collapsing );
            // for now, recalculate texture coordinates based on new placement
            // convert cartesian point to Geod, and use simgear's tc function
            //
            // note this doesnt work for roads - we need to store texparams in the face 
            // to do that 
            //
            // this is possible if we still have the tgconstruct data
            // perhaps LOD should be performed during tgconstruct ( 4th stage )
            SGGeod gplace = SGGeod::fromCart( SGVec3d( placement.get().x(), placement.get().y(), placement.get().z() ) );
            tgShapefile::FromGeod( gplace, datasource, layer, "Placement" );
            
            //SG_LOG( SG_GENERAL, SG_ALERT, "New node will be at " << gplace );
            
            SGGeod gv0 = SGGeod::fromCart( SGVec3d( profile.p0().x(), profile.p0().y(), profile.p0().z() ) );
            tgShapefile::FromGeod( gv0, datasource, layer, "V0" );
            SGGeod gv1 = SGGeod::fromCart( SGVec3d( profile.p1().x(), profile.p1().y(), profile.p1().z() ) );
            tgShapefile::FromGeod( gv1, datasource, layer, "V1" );
            
            if ( profile.left_face_exists() ) {   
                SGGeod gL  = SGGeod::fromCart( SGVec3d( profile.vL()->point().x(), profile.vL()->point().y(), profile.vL()->point().z() ) );
                tgShapefile::FromGeod( gL,  datasource, layer, "VL" );
            }
            
            if ( profile.right_face_exists() ) {
                SGGeod gR  = SGGeod::fromCart( SGVec3d( profile.vR()->point().x(), profile.vR()->point().y(), profile.vR()->point().z() ) );
                tgShapefile::FromGeod( gR,  datasource, layer, "VR" );
            }
            
            // which face is the point in?
            // traverse the triangles
            Profile::Triangle_vector triangles = profile.triangles();
            sprintf( layer, "collapsing_tris_%04lu", cinfo->collapsing );
                     
            for ( unsigned int i=0; i< triangles.size(); i++ ) {
                std::vector<tgSegment> segs;
                
                SGGeod g0 = SGGeod::fromCart( SGVec3d(triangles[i].v0->point().x(),
                                                      triangles[i].v0->point().y(),
                                                      triangles[i].v0->point().z() ));

                SGGeod g1 = SGGeod::fromCart( SGVec3d(triangles[i].v1->point().x(),
                                                      triangles[i].v1->point().y(),
                                                      triangles[i].v1->point().z() ));

                SGGeod g2 = SGGeod::fromCart( SGVec3d(triangles[i].v2->point().x(),
                                                      triangles[i].v2->point().y(),
                                                      triangles[i].v2->point().z() ));
                
                segs.push_back( tgSegment( g0, g1 ) );
                segs.push_back( tgSegment( g1, g2 ) );
                segs.push_back( tgSegment( g2, g0 ) );
                
                tgShapefile::FromSegmentList( segs, false, datasource, layer, "tris" );
            }
#endif
        }
    }            
    
    // Called for each edge which failed the so called link-condition,
    // that is, which cannot be collapsed because doing so would
    // turn the surface mesh into a non-manifold.
    void OnNonCollapsable( Profile const& )
    {
        ++cinfo->non_collapsable;
    }                
    
    // Called AFTER each edge has been collapsed
    void OnCollapsed( Profile const& profile, tgBtgVertex_handle new_node )
    {
        ++cinfo->collapsed;
        
#if 0        
        // calculate the tex coords of the collapsed vertex in Geodetic coordinates
        SGGeod g = SGGeod::fromCart( SGVec3d( new_node->point().x(),
                                              new_node->point().y(),
                                              new_node->point().z() ));
        
        // ToDo : Create a new SimGear texcoordinate generator that just takes a list of geods, or a single geod
        // The Simgear General texture coordinate routine currently takes a fan.
        std::vector< int >      node_idxs;
        std::vector< SGGeod >   nodes;
        std::vector< SGVec2f >  tc_list;
        
        node_idxs.push_back(0);
        nodes.push_back(g);

        tc_list = sgCalcTexCoords( center_lat, nodes, node_idxs );
        
        // how set this tc on all half edges incident to this vertex
        tgBtgHalfedge_vertex_circulator hv_cur = new_node->vertex_begin();
        tgBtgHalfedge_vertex_circulator hv_end = hv_cur;
                    
        int num_he = 1;
        
        do { 
            // set primary texture coordinate : all normals need to be recomputed
            // when we are done, as all the faces change
            hv_cur->SetTexCoord( tc_list[0] );
            hv_cur++;
            num_he++;
        } while(hv_cur != hv_end);

        SG_LOG( SG_GENERAL, SG_ALERT, "Set TC incident to vertex " << g << " : " << num_he << " halfedges to " << tc_list[0] );
#endif

    }                
    
    CollapseInfo* cinfo;
    double        center_lat;
    std::string   name;    
};

int tgBtgSimplify( tgBtgMesh& mesh, float stop_percentage, float volume_wgt, float boundary_wgt, float shape_wgt, double cl, const std::string& name )
{
    CollapseInfo    ci;
    CollapseVisitor vis(&ci, name, cl );
    
    // first write the whole mesh as triangles
#if DEBUG_SIMPLIFY
    char mesh_name[1024];
    //sprintf( mesh_name, "%s_%s", name.c_str(), "before" );
    //tgMeshToShapefile( mesh, mesh_name );
#endif
    
    // In this example, the simplification stops when the number of undirected edges
    // drops below 25% of the initial count
    SMS::Count_ratio_stop_predicate<tgBtgMesh>  stop(stop_percentage);
    SMS::LindstromTurk_params                   params(volume_wgt, boundary_wgt, shape_wgt);
    SMS::LindstromTurk_placement<tgBtgMesh>     base_placement(params);
    SMS::LindstromTurk_cost<tgBtgMesh>          cost;
    Border_is_constrained_edge_map              constrain_map(mesh);
    ConstrainedPlacement                        placement(constrain_map, base_placement);

    // crazy boost named parameters overriding '.' character
    int r = SMS::edge_collapse(mesh, stop 
        ,CGAL::edge_is_constrained_map(constrain_map)
        .get_cost(cost)
        .get_placement(placement)
        .visitor(vis)
    );

    
    SG_LOG( SG_GENERAL, SG_ALERT, "           SUCCESS Simplifying obj : " << r << " edges removed " << mesh.size_of_halfedges()/2 << " edges left " );

#if DEBUG_SIMPLIFY
    sprintf( mesh_name, "%s_%s", name.c_str(), "after" );
    tgMeshToShapefile( mesh, mesh_name );
 #endif
    
    return r;
}