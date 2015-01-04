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

#include <simgear/debug/logstream.hxx>
#include <terragear/tg_shapefile.hxx>

typedef CGAL::Line_3<tgBtgKernel>                             tgBtg_Line_3;

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
    
    friend bool get(Border_is_constrained_edge_map m, const key_type& edge) {
        return CGAL::is_border(edge, *m.sm_ptr);
    }
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
    CollapseVisitor( CollapseInfo* ci, const std::string& n ) : cinfo(ci), name(n) {}
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
            
            sprintf( layer, "%s_collapsing_%04lu", name.c_str(), cinfo->collapsing );
            // for now, recalculate texture coordinates based on new placement
            // convert cartesian point to Geod, and use simgear's tc function
            //
            // note this doesnt work for roads - we need to store texparams in the face 
            // to do that 
            //
            // this is possible if we still have the tgconstruct data
            // perhaps LOD should be performed during tgconstruct ( 4th stage )
            if ( profile.left_face_exists() && profile.right_face_exists() ) {
                
                SGGeod gv0 = SGGeod::fromCart( SGVec3d( profile.p0().x(), profile.p0().y(), profile.p0().z() ) );
                SGGeod gv1 = SGGeod::fromCart( SGVec3d( profile.p1().x(), profile.p1().y(), profile.p1().z() ) );
                SGGeod gL  = SGGeod::fromCart( SGVec3d( profile.vL()->point().x(), profile.vL()->point().y(), profile.vL()->point().z() ) );
                SGGeod gR  = SGGeod::fromCart( SGVec3d( profile.vR()->point().x(), profile.vR()->point().y(), profile.vR()->point().z() ) );

                tgShapefile::FromGeod( gv0, "./simp_dbg", layer, "V0" );
                tgShapefile::FromGeod( gv1, "./simp_dbg", layer, "V1" );
                tgShapefile::FromGeod( gL,  "./simp_dbg", layer, "VL" );
                tgShapefile::FromGeod( gR,  "./simp_dbg", layer, "VR" );
            }
            
            // which face is the point in?
            
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
    void OnCollapsed( Profile const&, tgBtgVertex_handle )
    {
        ++cinfo->collapsed;
    }                
    
    CollapseInfo* cinfo;
    std::string   name;
};

int tgBtgSimplify( tgBtgMesh& mesh, float stop_percentage, float volume_wgt, float boundary_wgt, float shape_wgt, const std::string& name )
{
    CollapseInfo    ci;
    CollapseVisitor vis(&ci, name);

    // first write the whole mesh as triangles
    
    
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
    return r;
}