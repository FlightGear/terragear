// main.cxx -- CGAL join test utility
//
// Written by Peter Sadrozinski, started Jan 2016.
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
  
  Polygon_set_2                   ps;
  std::list<Polygon_with_holes_2> pwhs;  
  size_t                          num_pwhs;
  
  input_file >> num_pwhs;
  std::cout << "Need to Read " << num_pwhs << " PolygonWithHoles " << std::endl;

  for ( unsigned int i=0; i<num_pwhs; i++ ) {
      Polygon_with_holes_2 pwh;
      input_file >> pwh;
      pwhs.push_back( pwh );
  }
  input_file.close();
  
  std::cout << "Read " << pwhs.size() << " PolygonWithHoles " << std::endl;
  
  std::list<Polygon_with_holes_2>::const_iterator it = pwhs.begin();
  while ( it != pwhs.end() ) {
      ps.join( (*it) );
      it++;
  }
  
  return 0;
}