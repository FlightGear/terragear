// main.cxx -- CGAL clip test utility
//
// Written by Peter Sadrozinski, started Dec 2014.
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
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/connect_holes.h>
#include <CGAL/Polygon_set_2.h>
#include <list>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef CGAL::Polygon_set_2<Kernel>                       Polygon_set_2;


int main(int argc, char* argv[])
{
  const char* filename = (argc > 1) ? argv[1] : "cgalpolys.txt";
  std::ifstream input_file(filename);
  if (!input_file.is_open()) {
    std::cerr << "Failed to open the " << filename <<std::endl;
    return -1;
  }

  std::ofstream output_file("./output_polys.txt");
  if (!output_file.is_open()) {
      std::cerr << "Failed to open the " << "./output_polys.txt" << std::endl;
      exit(0);
  }
  
  Polygon_2 outer_p;
  size_t    num_holes;
  
  input_file >> outer_p;
  input_file >> num_holes;
  
  std::cout << "boundary has " << outer_p.size() << " vertices " << std::endl;
  std::cout << "Read " << num_holes << " holes" << std::endl;
  
  std::vector<Polygon_2> holes(num_holes);
  for (size_t k = 0; k < num_holes; k++) input_file >> holes[k];
  
  Polygon_with_holes_2 P(outer_p, holes.begin(), holes.end());
  output_file << std::setprecision(16) << P;

  Polygon_set_2 gps;

  gps.insert(P);
  
  std::cout << "# pwhs: " << gps.number_of_polygons_with_holes() <<
  std::endl;
  
  return 0;
}