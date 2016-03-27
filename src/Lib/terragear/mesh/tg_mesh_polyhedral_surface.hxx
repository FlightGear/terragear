// tg_mesh_polyhedral_surface.hxx -- 
//
// Written by Peter Sadrozinski, started March 2016.
//
// Copyright (C) 2016  Peter Sadrozinski  - http://www.flightgear.org
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
#ifndef __TG_MESH_POLYHEDRAL_SURFACE_HXX__
#define __TG_MESH_POLYHEDRAL_SURFACE_HXX__

// Define the CGAL Polyhedral Mesh 
// simple cartesian (double) kernel and an incremental builder
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/property_map.h>
#include <boost/graph/properties.hpp>

// for writing mesh to Simgear BTG
#include <simgear/math/SGMath.hxx>
#include <simgear/io/sg_binobj.hxx>

#include "tg_mesh.hxx"

// forward declarations
class tgMesh;

// CGAL mesh consists of three data structures.
// Points, directed haldedges, and faces.
// We can add extra data to these structures.
// For simpiler use of the Simplification API, we also add
// an ID field to each structure.

// A vertex : just add ID field

// most custom terragear info is added to the halfedge structure.
// The reason is, is that there may be multiple values for a particular item 
// associated with a vertex.
// In the case of texture coordinates, the face being textured may use different
// schemes.  Consider an edge ending at a vertex.  The face to the right may be
// a road w/textureing in relation to the direction of the road.  The face to the left
// may be forest, with geo referenced texture coordinates.
// In this case, the texture coordinates are assigned to the halfedge, and applied to the 
// target vertex, as every face has just one halfedge incident to the vertex. 

// The halfedge also has a length to assist with monge jet firring

// The face gets the normal vector ( instead of default plane equation )
// it also has a map to ring tags for jet fitting assistance

typedef CGAL::Simple_cartesian<double> tgMeshSurfaceKernel;

template < class Refs, class Point >
class tgMeshSurfaceVertex : public CGAL::HalfedgeDS_vertex_base< Refs, CGAL::Tag_true, Point>
{
public:
    typedef CGAL::HalfedgeDS_vertex_base< Refs, CGAL::Tag_true, Point> Base;    
    
private:
    std::size_t mID;
    
public:
    tgMeshSurfaceVertex() : mID ( std::size_t(-1) )  {}
    tgMeshSurfaceVertex( Point const& p) : Base(p), mID( std::size_t(-1) ) {}
    tgMeshSurfaceVertex( Point const& p, std::size_t i ) : Base(p), mID(i) {}
    
    std::size_t&       id()       { return mID; }
    std::size_t const& id() const { return mID; }
};

// The halfedge : twin halfedges make an edge
// v0->v1 and v1->v0
// extra data on this structure applies to the target vertex
template <class Refs>
struct tgMeshSurfaceHalfedge : public CGAL::HalfedgeDS_halfedge_base<Refs> {    
public:
    tgMeshSurfaceHalfedge() : mID ( std::size_t(-1) ), len(-1)  {}
    
    std::size_t&       id()       { return mID; }
    std::size_t const& id() const { return mID; }
    
    double getLength( void ) const { return len; }
    
    void setTexCoord( const SGVec2f& tc ) {
        texCoord = tc;
    }
    
    SGVec2f getTexCoord( void ) const {
        return texCoord;
    }
    
private:
    std::size_t     mID;
    double          len;
    
    SGVec2f         texCoord;       // incident vertex texture coordinate    
};

// The Face : for our purposes, the face should always be a triangle.
// we store the material name here as it applies to the face.
template <class Refs>
struct tgMeshSurfaceFace : public CGAL::HalfedgeDS_face_base<Refs> {    
public:
    std::size_t&       id()       { return mID; }
    std::size_t const& id() const { return mID; }
    
    typedef typename tgMeshSurfaceKernel::Vector_3  Vector_3;
    
    void SetMaterial( const std::string& mat ) {
        material = mat;
    }
    
    std::string GetMaterial( void ) const {
        return material;
    }
    
private:
    std::size_t mID;
    std::string material;
    Vector_3    normal;
};

// create a halfedge property map
template <class TPoly>
class HEdge_PM : public boost::put_get_helper<typename TPoly::Traits::FT&, HEdge_PM<TPoly> >
{
public:
    //read_write or lvalue
    typedef boost::lvalue_property_map_tag  category;
    typedef typename TPoly::Halfedge        key_type;
    typedef typename TPoly::Traits::FT      value_type;
    typedef typename TPoly::Traits::FT&     reference;
    
    HEdge_PM() {}
    reference operator[](key_type h) const { return h.len; }
};

//use the std edge_weight_t tag...
template <class TPoly>
HEdge_PM<TPoly> get_hepm(boost::edge_weight_t, TPoly& )
{
    return HEdge_PM<TPoly>();
}

// Here's where we tell CGAL what our data strutures are
// It's the three typedefs Vertex, Point, and Face that matter
struct tgMeshSurfaceItems : public CGAL::Polyhedron_items_3 {
    template <class Refs, class Traits>
    struct Vertex_wrapper {
        typedef typename Traits::Point_3            Point;
        typedef tgMeshSurfaceVertex<Refs, Point>    Vertex;
    };
    
    template <class Refs, class Traits>
    struct Halfedge_wrapper {
        typedef tgMeshSurfaceHalfedge<Refs>         Halfedge;
    };

    template <class Refs, class Traits>
    struct Face_wrapper {
        typedef tgMeshSurfaceFace<Refs>             Face;
    };
};

typedef CGAL::Polyhedron_3<tgMeshSurfaceKernel, tgMeshSurfaceItems> tgMeshSurfacePolyhedron;
typedef tgMeshSurfacePolyhedron::HalfedgeDS                         tgMeshSurfaceHalfedgeDS;
typedef tgMeshSurfacePolyhedron::Vertex_iterator                    tgMeshSurfaceVertex_iterator;
typedef tgMeshSurfacePolyhedron::Vertex_handle                      tgMeshSurfaceVertex_handle;
typedef tgMeshSurfacePolyhedron::Halfedge_iterator                  tgMeshSurfaceHalfedge_iterator;
typedef tgMeshSurfacePolyhedron::Halfedge_around_facet_circulator   tgMeshSurfaceHalfedge_facet_circulator;
typedef tgMeshSurfacePolyhedron::Halfedge_around_vertex_circulator  tgMeshSurfaceHalfedge_vertex_circulator;
typedef tgMeshSurfacePolyhedron::Halfedge_handle                    tgMeshSurfaceHalfedge_handle;
typedef tgMeshSurfacePolyhedron::Facet_iterator                     tgMeshSurfaceFacet_iterator;
typedef tgMeshSurfacePolyhedron::Facet_handle                       tgMeshSurfaceFacet_handle;

class tgMeshPolyhedralSurface
{
public:
    tgMeshPolyhedralSurface( tgMesh* m ) { mesh = m; }
    
    void loadTriangulation( const std::string& basePath, const SGBucket& b );
    
private:
    tgMesh*                     mesh;
    tgMeshSurfacePolyhedron     meshSurface;
};

#endif /* __TG_MESH_POLYHEDRAL_SURFACE_HXX__ */
