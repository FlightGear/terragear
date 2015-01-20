// tgBtgMeshBuilder -- Responsible for converting BTG to CGAL Surface Mesh
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
#ifndef __TG_BTG_MESH_HXX__
#define __TG_BTG_MESH_HXX__

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifdef _MSC_VER
#  include <windows.h>
#endif

// Define the CGAL Surface Mesh 
// simple cartesian (double) kernel and an incremental builder
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include <simgear/math/SGMath.hxx>
#include <simgear/io/sg_binobj.hxx>

#include "tg_geometry_arrays.hxx"

// CGAL mesh consists of three data structures.
// Points, directed haldedges, and faces.
// We can add extra data to these structures.
// For simpiler use of the Simplification API, we also add
// an ID field to each structure.


// A vertex : just add ID field
// most custom BTG info is added to the halfedge structure.
// The reason is, is that there may be multiple values for a particular item 
// associated with a vertex.
// In the case of texture coordinates, the face being textured may use different
// schemes.  Consider an edge ending at a vertex.  The face to the right may be
// a road w/textureing in relation to the direction of the road.  The face to the left
// may be forest, with geo referenced texture coordinates.
// In this case, the texture coordinates are assigned to the halfedge, and applied to the 
// target vertex, as every face has just one halfedge incident to the vertex. 

template < class Refs, class Point >
class tgBtgVertex : public CGAL::HalfedgeDS_vertex_base< Refs, CGAL::Tag_true, Point>
{
public:
    typedef CGAL::HalfedgeDS_vertex_base< Refs, CGAL::Tag_true, Point> Base;    
    
private:
    std::size_t mID;
    
public:
    tgBtgVertex() : mID ( std::size_t(-1) )  {}
    tgBtgVertex( Point const& p) : Base(p), mID( std::size_t(-1) ) {}
    tgBtgVertex( Point const& p, std::size_t i ) : Base(p), mID(i) {}
    
    std::size_t&       id()       { return mID; }
    std::size_t const& id() const { return mID; }
};

// The halfedge : twin halfedges make an edge
// v0->v1 and v1->v0
// extra data on this structure applies to the target vertex
template <class Refs>
struct tgBtgHalfedge : public CGAL::HalfedgeDS_halfedge_base<Refs> {    
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

// The Face : for our purposes, the face should always be a triangle.
// we store the material name here as it applies to the face.
template <class Refs>
struct tgBtgFace : public CGAL::HalfedgeDS_face_base<Refs> {    
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

// Here's where we tell CGAL what our data strutures are
// It's the three typedefs Vertex, Point, and Face that matter
struct tgBtgItems : public CGAL::Polyhedron_items_3 {
    template <class Refs, class Traits>
    struct Vertex_wrapper {
        typedef typename Traits::Point_3    Point;
        typedef tgBtgVertex<Refs, Point>    Vertex;
    };
    
    template <class Refs, class Traits>
    struct Halfedge_wrapper {
        typedef tgBtgHalfedge<Refs>         Halfedge;
    };

    template <class Refs, class Traits>
    struct Face_wrapper {
        typedef tgBtgFace<Refs>             Face;
    };
};

typedef CGAL::Simple_cartesian<double>                  tgBtgKernel;
typedef CGAL::Polyhedron_3<tgBtgKernel, tgBtgItems>     tgBtgMesh;
typedef tgBtgMesh::HalfedgeDS                           tgBtgHalfedgeDS;
typedef tgBtgMesh::Vertex_iterator                      tgBtgVertex_iterator;
typedef tgBtgMesh::Vertex_handle                        tgBtgVertex_handle;
typedef tgBtgMesh::Halfedge_iterator                    tgBtgHalfedge_iterator;
typedef tgBtgMesh::Halfedge_around_facet_circulator     tgBtgHalfedge_facet_circulator;
typedef tgBtgMesh::Halfedge_around_vertex_circulator    tgBtgHalfedge_vertex_circulator;
typedef tgBtgMesh::Halfedge_handle                      tgBtgHalfedge_handle;
typedef tgBtgMesh::Facet_iterator                       tgBtgFacet_iterator;
typedef tgBtgMesh::Facet_handle                         tgBtgFacet_handle;

                                                        

void tgReadBtgAsMesh( const SGBinObject& inobj, tgBtgMesh& mesh );
void tgReadArraysAsMesh( const Arrays& arrays, tgBtgMesh& mesh );
bool tgWriteMeshAsBtg( tgBtgMesh& p, const SGGeod& center, SGPath& outfile );
int  tgBtgSimplify( tgBtgMesh& mesh, float stop_percentage, float volume_wgt, float boundary_wgt, float shape_wgt, double cl, const std::string& name );
void tgMeshToShapefile( tgBtgMesh& mesh, const std::string& name );

#endif /* __TG_BTG_MESH_HXX__ */
