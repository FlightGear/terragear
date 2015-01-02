// main.cxx -- top level construction routines
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


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/HalfedgeDS_vertex_base.h>

#include <CGAL/Polyhedron_items_with_id_3.h>

#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>

// Visitor base
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>

// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>

// Non-default cost and placement policies
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_and_length.h> 
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk.h> 

#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/debug/logstream.hxx>

#include <terragear/BucketBox.hxx>
#include <terragear/tg_shapefile.hxx>
#include <terragear/tg_unique_vec3d.hxx>
#include <terragear/tg_unique_vec3f.hxx>
#include <terragear/tg_unique_vec2f.hxx>

#include <Include/version.h>


// Very preliminary BTG -> CGAL Mesh converted
// to be used to simplify meshes from High detail to low detail.
// as a side benefit - I see it spots mesh degeneracy in current BTG files...
// need to be able to save the transformed mesh back to a BTG - 
// and maybe highlight the rejected triangles ( with a bright red material )
// to debug...



// For BTG files, store most of the point information in mesh halfedges (incident vertex)
// We do this, as the .btg format stores its information local to a particular
// face.
// for example, each triangle stores 3 indicies to normals, even though the normal
// to vertex ratio should be 1:1.
// texture coordinates, on the other hand can be different at a particular vertex,
// and do depend on the triangle we are referencing.

template <class Refs>
struct BTG_Face : public CGAL::HalfedgeDS_face_base<Refs> {    
public:
    std::size_t&       id()       { return mID; }
    std::size_t const& id() const { return mID; }
    
    void SetMaterial( const std::string& mat ) {
        material = mat;
    }
    
    std::string GetMaterial( void ) const {
        return material;
    }
    
private:
    std::size_t mID;
    std::string material;
};

template <class Refs>
struct BTG_Halfedge : public CGAL::HalfedgeDS_halfedge_base<Refs> {    
public:
    std::size_t&       id()       { return mID; }
    std::size_t const& id() const { return mID; }

    void SetNormal( const SGVec3f& n ) {
        normal = n;
    }
    
    SGVec3f GetNormal( void ) const {
        return normal;
    }
    
    void SetTexCoord( const SGVec2f& tc ) {
        texCoord = tc;
    }
        
    SGVec2f GetTexCoord( void ) const {
        return texCoord;
    }
    
private:
    std::size_t     mID;
    SGVec3f         normal;         // incident vertex normal
    SGVec2f         texCoord;       // incident vertex texture coordinate
};


template < class Refs, class Point >
class BTG_Vertex : public CGAL::HalfedgeDS_vertex_base< Refs, CGAL::Tag_true, Point>
{
public:
    typedef CGAL::HalfedgeDS_vertex_base< Refs, CGAL::Tag_true, Point> Base;    
    
private:
    std::size_t mID;
    
public:
    BTG_Vertex() : mID ( std::size_t(-1) )  {}
    BTG_Vertex( Point const& p) : Base(p), mID( std::size_t(-1) ) {}
    BTG_Vertex( Point const& p, std::size_t i ) : Base(p), mID(i) {}
    
    std::size_t&       id()       { return mID; }
    std::size_t const& id() const { return mID; }
};



// An items type using btg style halfedges
struct BTG_items : public CGAL::Polyhedron_items_3 {
    template <class Refs, class Traits>
    struct Face_wrapper {
        typedef BTG_Face<Refs>              Face;
    };
    
    template <class Refs, class Traits>
    struct Halfedge_wrapper {
        typedef BTG_Halfedge<Refs>          Halfedge;
    };
    
    template <class Refs, class Traits>
    struct Vertex_wrapper {
        typedef typename Traits::Point_3    Point;
        typedef BTG_Vertex<Refs, Point>     Vertex;
    };
};


typedef CGAL::Simple_cartesian<double>                  Kernel;
typedef CGAL::Polyhedron_3<Kernel, BTG_items>           Polyhedron;
typedef Polyhedron::HalfedgeDS                          HalfedgeDS;
typedef Polyhedron::Facet_iterator                      Facet_iterator;
typedef Polyhedron::Facet_handle                        Facet_handle;
typedef Polyhedron::Halfedge_iterator                   Halfedge_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator    Halfedge_facet_circulator;
typedef Polyhedron::Halfedge_handle                     Halfedge_handle;
typedef Polyhedron::Vertex_iterator                     Vertex_iterator;
typedef Polyhedron::Vertex_handle                       Vertex_handle;

namespace SMS = CGAL::Surface_mesh_simplification;


//
// BGL property map which indicates whether an edge is marked as non-removable
//
struct Border_is_constrained_edge_map {
  const Polyhedron* sm_ptr;
  typedef boost::graph_traits<Polyhedron>::edge_descriptor  key_type;
  typedef bool                                              value_type;
  typedef value_type                                        reference;
  typedef boost::readable_property_map_tag                  category;
  
  Border_is_constrained_edge_map() {}
  Border_is_constrained_edge_map(const Polyhedron& sm) : sm_ptr(&sm) {}
  
  friend bool get(Border_is_constrained_edge_map m, const key_type& edge) {
    return CGAL::is_border(edge, *m.sm_ptr);
  }
};

//
// Placement class
//
typedef SMS::Constrained_placement<SMS::LindstromTurk_placement<Polyhedron>, Border_is_constrained_edge_map > ConstrainedPlacement;


template <class HDS>
class BuildBTGMesh : public CGAL::Modifier_base<HDS> {
public:
    BuildBTGMesh(const SGBinObject& o) { obj = o; }
    
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
        
        for ( int v=0; v<num_vertices; v++ ) {
            SGVec3d sgn = obj.get_wgs84_nodes()[v];
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
                    Halfedge_handle hh = B.add_facet( indices.begin(), indices.end() );
                    
                    // add the per face stuff (material)
                    hh->facet()->SetMaterial( obj.get_tri_materials()[grp] );
                    
                    // now add the per vertex stuff
                    Halfedge_facet_circulator hfc_end = (Halfedge_facet_circulator)hh;
                    Halfedge_facet_circulator hfc_cur = hfc_end;
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
                }
            }
        }
        B.end_surface();        
    }
    
private:
    SGBinObject obj;
};

// display usage and exit
static void usage( const std::string name ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Usage: " << name);
    exit(-1);
}

void WriteBtg( Polyhedron& p, const SGGeod& center, SGPath& outfile) 
{
    typedef std::vector<Facet_handle>               FacetList_t;
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
    for ( Facet_iterator fit = p.facets_begin(); fit != p.facets_end(); fit++ ) {
        MatFacetMap[fit->GetMaterial()].push_back((Facet_handle)fit);
    }
    
    // now traverse all the facets to add the nodes, normals, and tcs
    for ( MaterialFacetMap_iterator mit=MatFacetMap.begin(); mit != MatFacetMap.end(); mit++ ) {
        FacetList_t& facets = mit->second;
        for ( FacetList_iterator fit = facets.begin(); fit != facets.end(); fit++ ) {
            sgboTri.clear();
            sgboTri.material = mit->first;
        
            Halfedge_handle hh = (*fit)->halfedge();
            
            // now add the per vertex stuff
            Halfedge_facet_circulator hfc_end = (Halfedge_facet_circulator)hh;
            Halfedge_facet_circulator hfc_cur = hfc_end;
            do { 
                int index;
                
                // to verify, read back the points
                SGVec3d node = SGVec3d( hfc_cur->vertex()->point().x(),
                                        hfc_cur->vertex()->point().y(),
                                        hfc_cur->vertex()->point().z() );
                index = vertices.add( node );
                sgboTri.v_list.push_back( index );

                index = normals.add( hfc_cur->GetNormal() );
                sgboTri.n_list.push_back( index );
                
                index = texcoords.add( hfc_cur->GetTexCoord() );
                sgboTri.tc_list[0].push_back( index );
                        
                hfc_cur++;
            } while(hfc_cur != hfc_end);            
        
            if ( sgboTri.v_list.size() != 3 ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Facet had more than 3 vertices " );
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
    
    bool result;
    result = outobj.write_bin_file( outfile );
}

// usage tglod minx, miny, maxx, maxy, level input_dir output_dir
// 

// first test : malta - generate 2 level 8 ( 0.25 x 0.25 ) tiles
//              14.00,35.75 - 14.25,36.00
//              14.25,35.75 - 14.50,36.00


int main(int argc, char **argv) 
{
    std::string output_dir = ".";
    std::string work_dir = ".";
    
    // default = whole earth - hehe
    double min_lon = -180;
    double min_lat =  -90;
    double max_lon =  180;
    double max_lat =   90;
    int    level = 0;
    
    sglog().setLogLevels( SG_ALL, SG_DEBUG );

    //
    // Parse the command-line arguments.
    //
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        std::string arg = argv[arg_pos];

        if (arg.find("--output-dir=") == 0) {
            output_dir = arg.substr(13);
        } else if (arg.find("--work-dir=") == 0) {
            work_dir = arg.substr(11);
        } else if ( arg.find("--min-lon=") == 0 ) {
            min_lon = atof( arg.substr(10).c_str() );
        } else if ( arg.find("--max-lon=") == 0 ) {
            max_lon = atof( arg.substr(10).c_str() );
        } else if ( arg.find("--min-lat=") == 0 ) {
            min_lat = atof( arg.substr(10).c_str() );
        } else if ( arg.find("--max-lat=") == 0 ) {
            max_lat = atof( arg.substr(10).c_str() );
        } else if (arg.find("--level=") == 0) {
            level = atoi( arg.substr(8).c_str() );
        } else {
            SG_LOG( SG_GENERAL, SG_ALERT, "unknown param " << arg );
        }
    }
    
    
    double width  = max_lon - min_lon;
    double height = max_lat - min_lat;

    // generate a bucketbox covering the area
    BucketBox box( min_lon, min_lat, width, height );
    SG_LOG( SG_GENERAL, SG_ALERT, "Box is: " << box << " level is " << box.getStartLevel() );
    
    if ( box.getStartLevel() < 8 ) {
        // we are not in the bucket stage        
        BucketBox subdivide[32];    // Pretty sure the maximum number of sub tiles is 25 
                                    // when we traverse between level 1 and 2
                                    // level 2 is  36.000 x  12.000     
                                    // level 1 is 180.000 x  60.000 ( 25 level 2 tiles = 1 level 1 tile )
    
        // subdivide the box to get higher res mesh
        unsigned numTiles = box.getSubDivision(subdivide, 32);
    
        if ( numTiles ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "subdivided box " << box << " into " << numTiles << " tiles " );
    
            for( unsigned int i=0; i<numTiles; i++ ) {
                // TODO : after subdividing - see if the mesh (.SPT format) is available 
                
                // if not - we need to generate it ( for now - recursively - I'd like to not respawn, however )
                char cmdline[256];
        
                SG_LOG( SG_GENERAL, SG_ALERT, "           box " << i << " : " << subdivide[i] << " level " << subdivide[i].getStartLevel() << " height is " << subdivide[i].getHeightDeg() );
                sprintf( cmdline, "/home/psadro/Development/terragear/release/src/BuildTiles/tglod/tg-lod --work-dir=%s --min-lon=%f --min-lat=%f --max-lon=%f --max-lat=%f", 
                        work_dir.c_str(),
                        subdivide[i].getLongitudeDeg(), 
                        subdivide[i].getLatitudeDeg(), 
                        subdivide[i].getLongitudeDeg() + subdivide[i].getWidthDeg(), 
                        subdivide[i].getLatitudeDeg()  + subdivide[i].getHeightDeg() );
                
                system( cmdline );
            }
        }
    } 
    else 
    {
        // we've reached the bucket stage - read in all of the bucket triangles to generate a simplified mesh
        BucketBox subdivide[32];
    
        // subdivide the box to get higher res mesh
        unsigned numTiles = box.getSubDivision(subdivide, 32);
    
        if ( numTiles ) {
            for( unsigned int i=0; i<numTiles; i++ ) {
                SGBucket    b = subdivide[i].getBucket();
                SGPath      infile = work_dir + "/" + b.gen_base_path() + "/" + b.gen_index_str() + ".btg.gz";
                SGBinObject inobj;
                
                SGPath      outfile = output_dir + "/" + b.gen_base_path() + "/" + b.gen_index_str() + ".btg.gz";
                
                SG_LOG( SG_GENERAL, SG_ALERT, "           read SG bucket " << infile.str() );
                if ( inobj.read_bin( infile.str() ) ) {
                    SG_LOG( SG_GENERAL, SG_ALERT, "           SUCCESS READING obj" );

                    Polyhedron P;
                    BuildBTGMesh<HalfedgeDS> mesh(inobj);
                    P.delegate( mesh );

                    // now that the mesh has been created - set the IDs
                    // This just makes the edge_collapse call easier to follow :)
                    std::size_t vertex_id   = 0 ;
                    std::size_t halfedge_id = 0 ;
                    std::size_t face_id     = 0 ;
  
                    for ( Vertex_iterator vit = P.vertices_begin(), evit = P.vertices_end(); vit != evit; ++vit) {
                        vit->id() = vertex_id++;
                    }
                    for ( Halfedge_iterator hit = P.halfedges_begin(), ehit = P.halfedges_end(); hit != ehit; ++hit) {
                        hit->id() = halfedge_id++;
                    }
                    for ( Facet_iterator fit = P.facets_begin(), efit = P.facets_end(); fit != efit; ++fit ) {
                        fit->id() = face_id++;
                    }
                    
                    // TODO: keep a triangle list of errors as well in Build_BTG_Mesh object
                    // - so new BTG can get bright red error triangles...
                    
                    // In this example, the simplification stops when the number of undirected edges
                    // drops below 25% of the initial count
                    SMS::Count_ratio_stop_predicate<Polyhedron> stop(0.25);                    
                    SMS::LindstromTurk_params                   params(0.5, 0.5, 0.0);
                    SMS::LindstromTurk_placement<Polyhedron>    base_placement(params);
                    SMS::LindstromTurk_cost<Polyhedron>         cost;
                    Border_is_constrained_edge_map              constrain_map(P);
                    ConstrainedPlacement                        placement(constrain_map, base_placement);

                    // crazy boost named parameters overriding '.' character
                    int r = SMS::edge_collapse(P, stop 
                                               ,CGAL::edge_is_constrained_map(constrain_map)
                                               .get_cost(cost)
                                               .get_placement(placement)
                                 );

                    SG_LOG( SG_GENERAL, SG_ALERT, "           SUCCESS Simplifying obj : " << r << " edges removed " << P.size_of_halfedges()/2 << " edges left " );                    
                    WriteBtg( P, b.get_center(), outfile );
                }
            }
        }
    }
        
    return 0;
}