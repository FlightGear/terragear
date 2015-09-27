// tgconstruct_mesh -- Responsible for generating CGAL Surface Mesh
//   from an array of clipped polygons
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
#ifndef __TGCONSTRUCT_MESH_HXX__
#define __TGCONSTRUCT_MESH_HXX__

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifdef _MSC_VER
#  include <windows.h>
#endif

// To generate our mesh, we start with an arrangement of all tile polygons
// boundaries.  We don't add the interior holes, as these will be filled
// with other polygon boundaries.
// we use an observer to identify the inserted countour with face handles.
// when done, we can triangulate with a CDT.  For each triangle midpoint, 
// we look up the face the triangle belongs to, so we can find the original 
// poly info for texture information

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_landmarks_point_location.h>
#include <CGAL/Arr_observer.h>

#include "priorities.hxx"
#include <terragear/tg_areas.hxx>
#include <terragear/tg_contour.hxx>


typedef CGAL::Exact_predicates_exact_constructions_kernel               meshKernel;
typedef CGAL::Arr_segment_traits_2<meshKernel>                          meshTraits;

typedef meshTraits::Point_2                                             meshPoint;
typedef meshTraits::Segment_2                                           meshSegment;
typedef CGAL::Arrangement_2<meshTraits>                                 meshArrangement;
typedef meshArrangement::Face_handle                                    meshFaceHandle;
typedef CGAL::Arr_landmarks_point_location<meshArrangement>             meshLandmarks_pl;

struct face_area_poly 
{
public:
    face_area_poly( meshFaceHandle h, unsigned int a, unsigned int p ) : face(h), area(a), poly(p) {}
    
    meshFaceHandle  face;
    unsigned int    area;
    unsigned int    poly;
};

struct face_material
{
public:
    face_material( meshArrangement::Face_const_handle h, std::string m ) : face(h), material(m) {}
    
    meshArrangement::Face_const_handle  face;
    std::string                         material;
};

// typedef std::map<meshFaceHandle, area_poly*> facePolyMap;

class tgMesh  
{
public:
    tgMesh( const TGAreaDefinitions& ad, const tgAreas& p );
    
    void AddFaceHandle( meshFaceHandle oldFace, meshFaceHandle newFace, bool isHole );
    void ToShapefile( const char* path );
    
private:
    void Insert( const tgContour& c );
    std::string GetMaterial( meshFaceHandle fh );
    
    void SaveFace( meshArrangement::Face_const_handle fh, const char* path, const char* prefix );
    
    unsigned int current_area;
    unsigned int current_poly;
    
    meshArrangement mesh;
    TGAreaDefinitions const& area_defs;
    tgAreas           const& polys;
    
    std::vector<face_area_poly> polyLookup;
    std::vector<face_material>  materialLookup;
};

// An arrangement observer, used to receive notifications of face splits and
// face mergers.
class meshObserver : public CGAL::Arr_observer<meshArrangement>
{
public:
    meshObserver (meshArrangement& arr, tgMesh* mesh) : CGAL::Arr_observer<meshArrangement>(arr) 
    {
        pMesh = mesh;
    }
    
    virtual void after_split_face(meshFaceHandle oldFace, meshFaceHandle newFace, bool isHole );
    
private:
    tgMesh* pMesh;
};

#endif /* __TGCONSTRUCT_MESH_HXX__ */
