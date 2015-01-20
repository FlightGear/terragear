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

//#include <cstdio>
#include <simgear/math/SGMath.hxx>
#include <simgear/misc/texcoord.hxx>
#include <simgear/debug/logstream.hxx>

#include <terragear/tg_shapefile.hxx>

#include "tg_btg_mesh.hxx"

template <class HDS>
class tgBuildBtgMesh : public CGAL::Modifier_base<HDS> {
public:
    tgBuildBtgMesh(const SGBinObject& o) { obj = o; }
    
    void operator()( HDS& hds ) {
        int num_vertices = obj.get_wgs84_nodes().size();
        int num_indices  = 0;
        
        // just read in triangles. ignoring fans and strips
        int num_groups = obj.get_tris_v().size();
        for ( int g=0; g<num_groups; g++ ) {
            num_indices += obj.get_tris_v()[g].size();
        }
        
        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( num_vertices, num_indices/3, 0);
        typedef typename HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;
        
        const std::vector<SGVec3d>& wgs84_nodes = obj.get_wgs84_nodes();
        SGVec3d gbs_center = obj.get_gbs_center();

        for ( int v=0; v<num_vertices; v++ ) {
            SGVec3d sgn = obj.get_wgs84_nodes()[v] + gbs_center;
            B.add_vertex( Point( sgn.x(), sgn.y(), sgn.z() ) );
        }
        
        // read texture coordinates
        int num_tc_groups = obj.get_tris_tcs().size();
        if ( num_tc_groups != num_groups ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "number of groups != num_tc_groups: num_groups " << num_groups << ", tc_groups " << num_tc_groups );
        }
                
        for ( int grp=0; grp<num_groups; grp++ ) {
            const int_list& tris_v(obj.get_tris_v()[grp]);
            const int_list& tris_n(obj.get_tris_n()[grp]);
            const tci_list& tris_tc(obj.get_tris_tcs()[grp]);
            
            // just worry abount primary num_vertices
            if ( tris_v.size() != tris_tc[0].size() ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "number of vertex != number of tcs.  verticies: " << tris_v.size() << ", tcs: " << tris_tc[0].size() );
            }
            
            for (unsigned i = 2; i < tris_v.size(); i += 3) {
                std::vector< std::size_t> indices;
                std::vector< Point > points_inserted;
                std::vector< Point > points_traversed;
                
                indices.push_back( tris_v[i-2] );
                indices.push_back( tris_v[i-1] );
                indices.push_back( tris_v[i-0] );

                // for verification we added the triangles clockwise.
                SGVec3d sgn;
                sgn = obj.get_wgs84_nodes()[tris_v[i-2]];
                points_inserted.push_back( Point( sgn.x(), sgn.y(), sgn.z() ) );
                sgn = obj.get_wgs84_nodes()[tris_v[i-1]];
                points_inserted.push_back( Point( sgn.x(), sgn.y(), sgn.z() ) );
                sgn = obj.get_wgs84_nodes()[tris_v[i-0]];
                points_inserted.push_back( Point( sgn.x(), sgn.y(), sgn.z() ) );
                    
                int vidx = i-2;
                if ( B.test_facet( indices.begin(), indices.end() ) ) {
                    tgBtgHalfedge_handle hh = B.add_facet( indices.begin(), indices.end() );
                    
                    // add the per face stuff (material)
                    hh->facet()->SetMaterial( obj.get_tri_materials()[grp] );
                    
                    // now add the per vertex stuff
                    tgBtgHalfedge_facet_circulator hfc_end = (tgBtgHalfedge_facet_circulator)hh;
                    tgBtgHalfedge_facet_circulator hfc_cur = hfc_end;
                    do { 
                        // to verify, read back the points
                        points_traversed.push_back( hfc_cur->vertex()->point() );
                        
                        // set normal and primary texture coordinate
                        hfc_cur->SetTexCoord( obj.get_texcoords()[tris_tc[0][vidx]] );
                        hfc_cur->SetNormal( obj.get_normals()[tris_n[vidx]] );
                        
                        vidx++;
                        hfc_cur++;
                    } while(hfc_cur != hfc_end);
                    
                    for ( int i=0; i<3; i++ ) {
                        if ( points_inserted[i] != points_inserted[i] ) {
                            SG_LOG(SG_GENERAL, SG_ALERT, "Added triangle vertex at " << i << ":" << points_inserted[i] << ", and read back " << points_traversed[i] );
                        }
                    }
                    
                } else {
                    SG_LOG(SG_GENERAL, SG_ALERT, "Couldn't add triangle w/indices " << indices[0] << ", " << indices[1] << ", " <<  indices[2] );
                    
                    // TODO: keep a triangle list of errors as well in Build_BTG_Mesh object
                    // - so new BTG can get bright red error triangles...
                }
            }
        }
        B.end_surface();        
    }
    
private:
    SGBinObject obj;
};

template <class HDS>
class tgBuildArrayMesh : public CGAL::Modifier_base<HDS> {
public:
    tgBuildArrayMesh(const Arrays& a) { arr = a; }
    
    void operator()( HDS& hds ) {
        int num_vertices = arr.vertices.get_list().size();
        
        // just read in triangles. ignoring fans and strips
        int num_triangles = arr.getTriangleCount();
        
        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( num_vertices, num_triangles, 0);
        typedef typename HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;
        
        const std::vector<SGVec3d>& vertices  = arr.vertices.get_list();
        const std::vector<SGVec3f>& normals   = arr.normals.get_list();
        const std::vector<SGVec2f>& texcoords = arr.texcoords.get_list();
        
        for ( int v=0; v<num_vertices; v++ ) {
            SGVec3d sgn = vertices[v];
            B.add_vertex( Point( sgn.x(), sgn.y(), sgn.z() ) );
        }
                                
        // loop through all the materials, and get the list of triangle indicies        
        for ( matTris::iterator mti = arr.tris.begin(); mti != arr.tris.end(); mti++ ) {
            PointList& pl = mti->second;
            for ( unsigned int i = 2; i < pl.vertexIndex.size(); i += 3 ) {
                std::vector< std::size_t> indices;
                std::vector< Point > points_inserted;
                std::vector< Point > points_traversed;
                
                indices.push_back( pl.vertexIndex[i-2] );
                indices.push_back( pl.vertexIndex[i-1] );
                indices.push_back( pl.vertexIndex[i-0] );

                // for verification we added the triangles clockwise.
                SGVec3d sgn;
                sgn = vertices[pl.vertexIndex[i-2]];
                points_inserted.push_back( Point( sgn.x(), sgn.y(), sgn.z() ) );
                sgn = vertices[pl.vertexIndex[i-1]];
                points_inserted.push_back( Point( sgn.x(), sgn.y(), sgn.z() ) );
                sgn = vertices[pl.vertexIndex[i-0]];
                points_inserted.push_back( Point( sgn.x(), sgn.y(), sgn.z() ) );
                    
                int vidx = i-2;
                if ( B.test_facet( indices.begin(), indices.end() ) ) {
                    tgBtgHalfedge_handle hh = B.add_facet( indices.begin(), indices.end() );
                    
                    // add the per face stuff (material)
                    hh->facet()->SetMaterial( mti->first );
                    
                    // now add the per vertex stuff
                    tgBtgHalfedge_facet_circulator hfc_end = (tgBtgHalfedge_facet_circulator)hh;
                    tgBtgHalfedge_facet_circulator hfc_cur = hfc_end;
                    do { 
                        // to verify, read back the points
                        points_traversed.push_back( hfc_cur->vertex()->point() );
                        
                        // set normal and primary texture coordinate
                        hfc_cur->SetTexCoord( texcoords[pl.texcoordIndex[vidx]] );
                        hfc_cur->SetNormal( normals[pl.normalIndex[vidx]] );
                        
                        vidx++;
                        hfc_cur++;
                    } while(hfc_cur != hfc_end);
                    
                    for ( int i=0; i<3; i++ ) {
                        if ( points_inserted[i] != points_inserted[i] ) {
                            SG_LOG(SG_GENERAL, SG_ALERT, "Added triangle vertex at " << i << ":" << points_inserted[i] << ", and read back " << points_traversed[i] );
                        }
                    }
                    
                } else {
                    SG_LOG(SG_GENERAL, SG_ALERT, "Couldn't add triangle w/indices " << indices[0] << ", " << indices[1] << ", " <<  indices[2] );
                    
                    // TODO: keep a triangle list of errors as well in Build_BTG_Mesh object
                    // - so new BTG can get bright red error triangles...
                }
            }
        }
        B.end_surface();        
    }
    
private:
    Arrays arr;
};


void tgReadBtgAsMesh(const SGBinObject& inobj, tgBtgMesh& mesh)
{
    tgBuildBtgMesh<tgBtgHalfedgeDS> m(inobj);
    mesh.delegate( m );
    
    // now that the mesh has been created - set the IDs
    // This just makes the edge_collapse call easier to follow :)
    std::size_t vertex_id   = 0 ;
    std::size_t halfedge_id = 0 ;
    std::size_t face_id     = 0 ;
    
    for ( tgBtgVertex_iterator vit = mesh.vertices_begin(), evit = mesh.vertices_end(); vit != evit; ++vit) {
        vit->id() = vertex_id++;
    }
    for ( tgBtgHalfedge_iterator hit = mesh.halfedges_begin(), ehit = mesh.halfedges_end(); hit != ehit; ++hit) {
        hit->id() = halfedge_id++;
    }
    for ( tgBtgFacet_iterator fit = mesh.facets_begin(), efit = mesh.facets_end(); fit != efit; ++fit ) {
        fit->id() = face_id++;
    }
}

void tgReadArraysAsMesh( const Arrays& arrays, tgBtgMesh& mesh )
{
    tgBuildArrayMesh<tgBtgHalfedgeDS> m(arrays);
    mesh.delegate( m );
    
    // now that the mesh has been created - set the IDs
    // This just makes the edge_collapse call easier to follow :)
    std::size_t vertex_id   = 0 ;
    std::size_t halfedge_id = 0 ;
    std::size_t face_id     = 0 ;
    
    for ( tgBtgVertex_iterator vit = mesh.vertices_begin(), evit = mesh.vertices_end(); vit != evit; ++vit) {
        vit->id() = vertex_id++;
    }
    for ( tgBtgHalfedge_iterator hit = mesh.halfedges_begin(), ehit = mesh.halfedges_end(); hit != ehit; ++hit) {
        hit->id() = halfedge_id++;
    }
    for ( tgBtgFacet_iterator fit = mesh.facets_begin(), efit = mesh.facets_end(); fit != efit; ++fit ) {
        fit->id() = face_id++;
    }   
}

bool tgWriteMeshAsBtg( tgBtgMesh& p, const SGGeod& center, SGPath& outfile) 
{
    typedef std::vector<tgBtgFacet_handle>          FacetList_t;
    typedef FacetList_t::iterator                   FacetList_iterator;
    
    typedef std::map<std::string, FacetList_t >     MaterialFacetMap_t;
    typedef MaterialFacetMap_t::iterator            MaterialFacetMap_iterator;
    
    MaterialFacetMap_t          MatFacetMap;
    UniqueSGVec3dSet            vertices;
    UniqueSGVec3fSet            normals;
    UniqueSGVec2fSet            texcoords;
    SGBinObject                 outobj;
    SGBinObjectTriangle         sgboTri;
    
    // grab nodes, normals, and texture coordinates from the triangle list
    
    // first, order the facets by material - sgbinobj expects sorted triangles
    // we will just create a map group to a list of facets.
    // note - we jst need the incident he to get group - it will be the same all 
    // the way around.    
    for ( tgBtgFacet_iterator fit = p.facets_begin(); fit != p.facets_end(); fit++ ) {
        MatFacetMap[fit->GetMaterial()].push_back((tgBtgFacet_handle)fit);
    }
    
    // now traverse all the facets to add the nodes, normals, and tcs
    for ( MaterialFacetMap_iterator mit=MatFacetMap.begin(); mit != MatFacetMap.end(); mit++ ) {
        FacetList_t& facets = mit->second;
        for ( FacetList_iterator fit = facets.begin(); fit != facets.end(); fit++ ) {
            sgboTri.clear();
            sgboTri.material = mit->first;
        
            tgBtgHalfedge_handle hh = (*fit)->halfedge();
            
            tgBtgHalfedge_facet_circulator hfc_end = (tgBtgHalfedge_facet_circulator)hh;
            tgBtgHalfedge_facet_circulator hfc_cur = hfc_end;

            // need to send a list of geods for the tcs
            std::vector< int >      node_idxs;
            for (int i = 0; i < 3; i++) {
                node_idxs.push_back(i);
            }
            std::vector< SGGeod >   nodes;
            int index;
            
            do {                
                SGVec3d node = SGVec3d( hfc_cur->vertex()->point().x(),
                                        hfc_cur->vertex()->point().y(),
                                        hfc_cur->vertex()->point().z() );

                index = vertices.add( node );
                sgboTri.v_list.push_back( index );

                index = normals.add( hfc_cur->GetNormal() );
                sgboTri.n_list.push_back( index );
                
                // calc tc 
                nodes.push_back(SGGeod::fromCart( node ));
                                        
                hfc_cur++;
            } while(hfc_cur != hfc_end);            
        
            if ( sgboTri.v_list.size() != 3 ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Facet had more than 3 vertices " );
            }
            
            std::vector<SGVec2f> tc_list = sgCalcTexCoords( center, nodes, node_idxs );            
            for ( unsigned int i=0; i<tc_list.size(); i++ ) {
                index = texcoords.add( tc_list[i] );
                sgboTri.tc_list[0].push_back( index );
            }
            
            outobj.add_triangle( sgboTri );
        }
    }
    
    SGVec3d gbs_center = SGVec3d::fromGeod( center );
    double dist_squared, radius_squared = 0;
    std::vector<SGVec3d> wgs84_nodes = vertices.get_list();
    
    for (int i = 0; i < (int)wgs84_nodes.size(); ++i)
    {
        dist_squared = distSqr(gbs_center, wgs84_nodes[i]);
        if ( dist_squared > radius_squared ) {
            radius_squared = dist_squared;
        }
    }
    double gbs_radius = sqrt(radius_squared);
    
    outobj.set_gbs_center( gbs_center );
    outobj.set_gbs_radius( gbs_radius );
    outobj.set_wgs84_nodes( wgs84_nodes );
    outobj.set_normals( normals.get_list() );
    outobj.set_texcoords( texcoords.get_list() );
    
    return outobj.write_bin_file( outfile );
}

void tgMeshToShapefile(tgBtgMesh& mesh, const std::string& name)
{
    std::vector<SGGeod>    nodes;
    std::vector<tgSegment> segs;

    for (tgBtgFacet_iterator fit = mesh.facets_begin(); fit != mesh.facets_end(); fit++ ) {
        // create a tgSegment list for the face    
        nodes.clear();
        segs.clear();
        
        tgBtgHalfedge_handle hh = fit->halfedge();
        
        tgBtgHalfedge_facet_circulator hfc_end = (tgBtgHalfedge_facet_circulator)hh;
        tgBtgHalfedge_facet_circulator hfc_cur = hfc_end;
        do {
            // create a list of geods
            SGGeod gnode = SGGeod::fromCart( SGVec3d( hfc_cur->vertex()->point().x(),
                                                    hfc_cur->vertex()->point().y(),
                                                    hfc_cur->vertex()->point().z() ) );
            nodes.push_back( gnode );
            
            hfc_cur++;
        } while(hfc_cur != hfc_end);
        
        for ( unsigned int i=0; i<nodes.size(); i++ ) {
            if ( i != nodes.size()-1 ) {
                tgSegment seg(nodes[i], nodes[i+1]);
                segs.push_back(seg);
            } else {
                tgSegment seg(nodes[i], nodes[0]);                
                segs.push_back(seg);
            }            
        }
        
      char datasource[64];
      sprintf( datasource, "./simp_dbg/%s", name.c_str() );
        
      tgShapefile::FromSegmentList( segs, false, datasource, "original_mesh", "mesh" );
    }
}