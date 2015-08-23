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

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

/* determining if a face is within the reulting poly */
struct FaceInfo2
{
    FaceInfo2() {}
    int nesting_level;
    
    bool in_domain(){
        return nesting_level%2 == 1;
    }
};

typedef CGAL::Exact_predicates_exact_constructions_kernel         K;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_intersections_tag                             Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT>               CDTPlus;
typedef CDTPlus::Point                                            Point;

int main(int argc, char* argv[])
{
    const char* filename = (argc > 1) ? argv[1] : "cgalpolys.txt";
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
        std::cerr << "Failed to open the " << filename <<std::endl;
        return -1;
    }

    CDTPlus                               cdt;
    std::vector<CDTPlus::Vertex_handle>   handles;
    CDTPlus::Vertex_handle                h;
    size_t                                num_points;
    size_t                                num_constraints;
  
    input_file >> num_points;
    for ( unsigned int i=0; i<num_points; i++ ) {
        Point pt;
        input_file >> pt;
      
        h = cdt.insert( pt );
        handles.push_back(h);
    }
  
    input_file >> num_constraints;
    for ( unsigned int i=0; i<num_constraints; i++ ) {
        int s, t;
        input_file >> s >> t;
      
        std::cout << " inserting constraint " << i << std::endl;
        cdt.insert_constraint( handles[s], handles[t] );      
    }

    input_file.close();
  
    return 0;
}