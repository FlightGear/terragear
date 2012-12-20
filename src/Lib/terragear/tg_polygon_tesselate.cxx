#include <iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangle_2.h>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon.hxx"

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

static void tg_mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border )
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
static void tg_mark_domains(CDT& cdt)
{
    for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
        it->info().nesting_level = -1;
    }

    int index = 0;
    std::list<CDT::Edge> border;
    tg_mark_domains(cdt, cdt.infinite_face(), index++, border);
    while(! border.empty()) {
        CDT::Edge e = border.front();
        border.pop_front();
        CDT::Face_handle n = e.first->neighbor(e.second);
        if(n->info().nesting_level == -1) {
            tg_mark_domains(cdt, n, e.first->info().nesting_level+1, border);
        }
    }
}

static void tg_insert_polygon(CDT& cdt,const Polygon_2& polygon)
{
    if ( polygon.is_empty() ) return;

    CDT::Vertex_handle v_prev=cdt.insert(*CGAL::cpp0x::prev(polygon.vertices_end()));
    for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin(); vit!=polygon.vertices_end();++vit) {
        CDT::Vertex_handle vh=cdt.insert(*vit);
        cdt.insert_constraint(vh,v_prev);
        v_prev=vh;
    }
}

void tgPolygon::Tesselate( const std::vector<SGGeod>& extra )
{
    CDT       cdt;

    SG_LOG( SG_GENERAL, SG_DEBUG, "Tess with extra" );

    // Bail right away if polygon is empty
    if ( contours.size() != 0 ) {
        // First, convert the extra points to cgal Points
        std::vector<Point> points;
        points.reserve(extra.size());
        for (unsigned int n = 0; n < extra.size(); n++) {
            points.push_back( Point(extra[n].getLongitudeDeg(), extra[n].getLatitudeDeg() ) );
        }

        cdt.insert(points.begin(), points.end());

        // then insert each polygon as a constraint into the triangulation
        for ( unsigned int c = 0; c < contours.size(); c++ ) {
            tgContour contour = contours[c];
            Polygon_2 poly;

            for (unsigned int n = 0; n < contour.GetSize(); n++ ) {
                SGGeod node = contour.GetNode(n);
                poly.push_back( Point( node.getLongitudeDeg(), node.getLatitudeDeg() ) );
            }

            tg_insert_polygon(cdt, poly);
        }

        tg_mark_domains( cdt );

        int count=0;
        for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end(); ++fit) {
            if ( fit->info().in_domain() ) {
                Triangle_2 tri = cdt.triangle(fit);

                SGGeod p0 = SGGeod::fromDeg( tri.vertex(0).x(), tri.vertex(0).y() );
                SGGeod p1 = SGGeod::fromDeg( tri.vertex(1).x(), tri.vertex(1).y() );
                SGGeod p2 = SGGeod::fromDeg( tri.vertex(2).x(), tri.vertex(2).y() );

                AddTriangle( p0, p1, p2 );

                ++count;
            }
        }
    }
}

void tgPolygon::Tesselate()
{
    CDT       cdt;

    SG_LOG( SG_GENERAL, SG_DEBUG, "Tess" );

    // Bail right away if polygon is empty
    if ( contours.size() != 0 ) {
        // insert each polygon as a constraint into the triangulation
        for ( unsigned int c = 0; c < contours.size(); c++ ) {
            tgContour contour = contours[c];
            Polygon_2 poly;

            for (unsigned int n = 0; n < contour.GetSize(); n++ ) {
                SGGeod node = contour.GetNode(n);
                SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : Adding GEOD " << node);
                poly.push_back( Point( node.getLongitudeDeg(), node.getLatitudeDeg() ) );
            }

            tg_insert_polygon(cdt, poly);
        }

        tg_mark_domains( cdt );

        int count=0;
        for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end(); ++fit) {
            if ( fit->info().in_domain() ) {
                SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : face   in domain");

                Triangle_2 tri = cdt.triangle(fit);

                SGGeod p0 = SGGeod::fromDeg( tri.vertex(0).x(), tri.vertex(0).y() );
                SGGeod p1 = SGGeod::fromDeg( tri.vertex(1).x(), tri.vertex(1).y() );
                SGGeod p2 = SGGeod::fromDeg( tri.vertex(2).x(), tri.vertex(2).y() );

                AddTriangle( p0, p1, p2 );

                ++count;
            } else {
                SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : face not in domain");
            }
        }
    }
    else
    {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : no contours" );
    }
}