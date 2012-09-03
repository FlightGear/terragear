#include <float.h>
#include <stdio.h>

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <Geometry/point3d.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/structure/exception.hxx>

#include <Polygon/polygon.hxx>
#include "poly_support.hxx"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangle_2.h>
#include <iostream>

/* determining if a face is within the reulting poly */
struct FaceInfo2
{
  FaceInfo2() {}
  int nesting_level;

  bool in_domain(){
    return nesting_level%2 == 1;
  }
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Point                                                Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;
typedef CGAL::Triangle_2<K>                                       Triangle_2;

void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border )
{
    if(start->info().nesting_level != -1) {
        return;
    }

    std::list<CDT::Face_handle> queue;
    queue.push_back(start);

    while( !queue.empty() ){
        CDT::Face_handle fh = queue.front();
        queue.pop_front();
        if(fh->info().nesting_level == -1) {
            fh->info().nesting_level = index;
            for(int i = 0; i < 3; i++) {
                CDT::Edge e(fh,i);
                CDT::Face_handle n = fh->neighbor(i);
                if(n->info().nesting_level == -1) {
                    if(ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void mark_domains(CDT& cdt)
{
    for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
        it->info().nesting_level = -1;
    }

    int index = 0;
    std::list<CDT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), index++, border);
    while(! border.empty()) {
        CDT::Edge e = border.front();
        border.pop_front();
        CDT::Face_handle n = e.first->neighbor(e.second);
        if(n->info().nesting_level == -1) {
            mark_domains(cdt, n, e.first->info().nesting_level+1, border);
        }
    }
}

void insert_polygon(CDT& cdt,const Polygon_2& polygon)
{
    if ( polygon.is_empty() ) return;

    CDT::Vertex_handle v_prev=cdt.insert(*CGAL::cpp0x::prev(polygon.vertices_end()));
    for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin(); vit!=polygon.vertices_end();++vit) {
        CDT::Vertex_handle vh=cdt.insert(*vit);
        cdt.insert_constraint(vh,v_prev);
        v_prev=vh;
    }
}

TGPolygon polygon_tesselate_alt_with_extra_cgal( TGPolygon &p, const point_list& extra_nodes, bool verbose ) {
    TGPolygon result;
    CDT       cdt;

    result.erase();

    // Bail right away if polygon is empty
    if ( p.contours() == 0 ) {
        return result;
    }

    // First, insert the extra points
    std::vector<Point> points;
    points.reserve(extra_nodes.size());
    for (unsigned int n = 0; n < extra_nodes.size(); n++) {
        points.push_back( Point(extra_nodes[n].x(), extra_nodes[n].y()) );     
    }
    cdt.insert(points.begin(), points.end());
    
    // then insert each polygon as a constraint into the triangulation
    for (int c = 0; c < p.contours(); c++) {
        point_list contour = p.get_contour( c );
        Polygon_2  poly;

        for (unsigned int n = 0; n < contour.size(); n++ ) {
            Point3D node = contour[n];
            poly.push_back( Point( node.x(), node.y()) );
        }

        insert_polygon(cdt, poly);
    }

    mark_domains( cdt );

    int count=0;
    for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end();++fit) {
        if ( fit->info().in_domain() ) {
            Triangle_2 tri = cdt.triangle(fit);

            Point3D p0 = Point3D( tri.vertex(0).x(), tri.vertex(0).y(), 0.0f );
            Point3D p1 = Point3D( tri.vertex(1).x(), tri.vertex(1).y(), 0.0f );
            Point3D p2 = Point3D( tri.vertex(2).x(), tri.vertex(2).y(), 0.0f );

            result.add_node( count, p0 );
            result.add_node( count, p1 );
            result.add_node( count, p2 );

            ++count;
            // create a contour in result with this face
        }
    }

    return result;
}

TGPolygon polygon_tesselate_alt_cgal( TGPolygon &p, bool verbose ) {

    point_list pl; pl.clear();
    return ( polygon_tesselate_alt_with_extra_cgal(p, pl, verbose) );
}
