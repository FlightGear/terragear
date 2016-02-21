// main.cxx -- CGAL tiangulation test utility
//
// Written by Peter Sadrozinski, started July 2015.
//
// Copyright (C) 2015  Peter Sadrozinski
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

#include <fstream>

// source data ( an epec arrangement is used to 'clean' the input )
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>

// triangulation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

// mesh refinement
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>


typedef CGAL::Exact_predicates_exact_constructions_kernel                                   meshArrKernel;
typedef CGAL::Arr_segment_traits_2<meshArrKernel>                                           meshArrTraits;
typedef meshArrTraits::Point_2                                                              meshArrPoint;
typedef meshArrTraits::Curve_2                                                              meshArrSegment;

typedef CGAL::Exact_predicates_inexact_constructions_kernel                                 meshTriKernel;
typedef meshTriKernel::Point_2                                                              meshTriPoint;
typedef meshTriKernel::Segment_2                                                            meshTriSegment;

typedef CGAL::Triangulation_vertex_base_2<meshTriKernel>                                    meshTriVertexBase;

typedef CGAL::Constrained_triangulation_face_base_2<meshTriKernel>                          Fbb;
typedef CGAL::Delaunay_mesh_face_base_2<meshTriKernel,Fbb>                                  meshTriFaceBase;

typedef CGAL::Triangulation_data_structure_2<meshTriVertexBase,meshTriFaceBase>             meshTriTDS;
typedef CGAL::Exact_intersections_tag                                                       meshTriItag;
typedef CGAL::Constrained_Delaunay_triangulation_2<meshTriKernel, meshTriTDS, meshTriItag>  meshTriCDT;
typedef CGAL::Constrained_triangulation_plus_2<meshTriCDT>                                  meshTriCDTPlus;

typedef meshTriCDTPlus::Edge                                                                meshTriEdge;
typedef meshTriCDTPlus::Face_handle                                                         meshTriFaceHandle;
typedef meshTriCDTPlus::Finite_faces_iterator                                               meshTriFaceIterator;
typedef CGAL::Triangle_2<meshTriKernel>                                                     meshTriangle;

typedef CGAL::Delaunay_mesh_size_criteria_2<meshTriCDTPlus>                                 meshCriteria;
typedef CGAL::Delaunay_mesher_2<meshTriCDTPlus, meshCriteria>                               meshRefiner;

meshTriPoint toMeshTriPoint( const meshArrPoint& aPoint ) {
    return meshTriPoint ( CGAL::to_double( aPoint.x() ), CGAL::to_double(aPoint.y()) );
}

int main(int argc, char* argv[])
{
    const char* filename = (argc > 1) ? argv[1] : "tri_test.txt";
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
        std::cerr << "Failed to open the " << filename <<std::endl;
        return -1;
    }

    meshTriCDTPlus                              cdt;
    size_t                                      num_points;
    size_t                                      num_constraints;
  
    input_file >> num_points;
    for ( unsigned int i=0; i<num_points; i++ ) {
        meshArrPoint pt;
        input_file >> pt;
      
        cdt.insert( toMeshTriPoint(pt) );
    }
  
    input_file >> num_constraints;
    for ( unsigned int i=0; i<num_constraints; i++ ) {
        meshArrPoint s, t;
        input_file >> s >> t;
      
        cdt.insert_constraint( toMeshTriPoint(s), toMeshTriPoint(t) );      
    }    
    input_file.close();
    
    meshRefiner mesher(cdt);
    mesher.set_criteria(meshCriteria(0.125));
    
    std::cout << "refine mesh" << std::endl;    
    mesher.refine_mesh();
    std::cout << "complete" << std::endl;    
    
    return 0;
}