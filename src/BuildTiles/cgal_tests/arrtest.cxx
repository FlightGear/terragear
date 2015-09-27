// main.cxx -- CGAL arrangement test utility
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
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel   arrKernel;
typedef CGAL::Arr_segment_traits_2<arrKernel>               arrTraits;
typedef arrTraits::Point_2                                  arrPoint;
typedef arrTraits::Curve_2                                  arrSegment;
typedef CGAL::Arrangement_2<arrTraits>                      arrArrangement;

int main(int argc, char* argv[])
{
  const char* filename = (argc > 1) ? argv[1] : "arr_test.txt";
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
    
  size_t num_segs;
  input_file >> num_segs;
  
  std::cout << "Read " << num_segs << " segments" << std::endl;
  
  std::vector<arrSegment> segs(num_segs);
  
  arrArrangement arr;
  
  for (size_t k = 0; k < num_segs; k++) {
      arrPoint source, target;
      input_file >> source;
      input_file >> target;
      segs[k] = arrSegment( source, target);
  }

  std::cout << "Inserting " << segs.size() << " segments " << std::endl;  
  insert( arr, segs.begin(), segs.end() );
  
  return 0;
}