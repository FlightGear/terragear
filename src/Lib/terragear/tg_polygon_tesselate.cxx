#include <iostream>
#include <cassert>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangle_2.h>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon.hxx"
#include "tg_misc.hxx"

/* determining if a face is within the resulting poly */
struct FaceInfo2
{
    FaceInfo2() {
        nesting_level = 0;
    }

    int nesting_level;

    bool in_domain() {
        return nesting_level % 2 == 1;
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
typedef CGAL::Polygon_2<K>                                        Polygon_2;
typedef CGAL::Triangle_2<K>                                       Triangle_2;

static void tg_mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border )
{
    if(start->info().nesting_level != -1) {
        return;
    }

    std::list<CDTPlus::Face_handle> queue;
    queue.push_back(start);

    while( !queue.empty() ){
        CDTPlus::Face_handle fh = queue.front();
        queue.pop_front();
        if(fh->info().nesting_level == -1) {
            fh->info().nesting_level = index;
            for(int i = 0; i < 3; i++) {
                CDTPlus::Edge e(fh,i);
                CDTPlus::Face_handle n = fh->neighbor(i);
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
static void tg_mark_domains(CDTPlus& cdt)
{
    for(CDTPlus::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
        it->info().nesting_level = -1;
    }

    int index = 0;
    std::list<CDTPlus::Edge> border;
    tg_mark_domains(cdt, cdt.infinite_face(), index++, border);
    while(! border.empty()) {
        CDTPlus::Edge e = border.front();
        border.pop_front();
        CDTPlus::Face_handle n = e.first->neighbor(e.second);
        if(n->info().nesting_level == -1) {
            tg_mark_domains(cdt, n, e.first->info().nesting_level+1, border);
        }
    }
}

static void tg_insert_polygon(CDTPlus& cdt,const Polygon_2& polygon)
{
    if ( polygon.is_empty() ) return;

    CDTPlus::Vertex_handle v_prev=cdt.insert(*CGAL::cpp0x::prev(polygon.vertices_end()));
    for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin(); vit!=polygon.vertices_end();++vit) {
        CDTPlus::Vertex_handle vh=cdt.insert(*vit);
        cdt.insert_constraint(vh,v_prev);
        v_prev=vh;
    }
}

void tgPolygon::Tesselate( const std::vector<SGGeod>& extra )
{
    CDTPlus cdt;

    SG_LOG( SG_GENERAL, SG_DEBUG, "Tess with extra" );

    // clear any triangles from previous tesselation
    triangles.clear();
    
    // Bail right away if polygon is empty
    if ( contours.size() != 0 ) {
        // First, convert the extra points to cgal Points
        std::vector<Point> points;
        points.reserve(extra.size());
        for (unsigned int n = 0; n < extra.size(); n++) {
            points.push_back( Point(extra[n].getLongitudeDeg(), extra[n].getLatitudeDeg() ) );
        }

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

        if ( !points.empty() ) {
            cdt.insert(points.begin(), points.end());
        }
        
        /* make conforming - still has an issue, and can't be compiled with exact_construction kernel */
        // CGAL::make_conforming_Delaunay_2( cdt );

        tg_mark_domains( cdt );

        int count=0;
        for (CDTPlus::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end(); ++fit) {
            if ( fit->info().in_domain() ) {
                Triangle_2 tri = cdt.triangle(fit);

                SGGeod p0 = SGGeod::fromDeg( to_double(tri.vertex(0).x()), to_double(tri.vertex(0).y()) );
                SGGeod p1 = SGGeod::fromDeg( to_double(tri.vertex(1).x()), to_double(tri.vertex(1).y()) );
                SGGeod p2 = SGGeod::fromDeg( to_double(tri.vertex(2).x()), to_double(tri.vertex(2).y()) );

                /* Check for Zero Area before inserting */
                if ( !SGGeod_isEqual2D( p0, p1 ) && !SGGeod_isEqual2D( p1, p2 ) && !SGGeod_isEqual2D( p0, p2 ) ) {
                    AddTriangle( p0, p1, p2 );
                }

                ++count;
            }
        }
    }
}

void tgPolygon::Tesselate()
{
    CDTPlus cdt;

    SG_LOG( SG_GENERAL, SG_DEBUG, "Tess" );

    // clear any triangles from previous tesselation
    triangles.clear();

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

        assert(cdt.is_valid());
        tg_mark_domains( cdt );

        int count=0;
        for (CDTPlus::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end(); ++fit) {
            if ( fit->info().in_domain() ) {
                SG_LOG( SG_GENERAL, SG_DEBUG, "Tess : face   in domain");

                Triangle_2 tri = cdt.triangle(fit);

                SGGeod p0 = SGGeod::fromDeg( to_double(tri.vertex(0).x()), to_double(tri.vertex(0).y()) );
                SGGeod p1 = SGGeod::fromDeg( to_double(tri.vertex(1).x()), to_double(tri.vertex(1).y()) );
                SGGeod p2 = SGGeod::fromDeg( to_double(tri.vertex(2).x()), to_double(tri.vertex(2).y()) );

                /* Check for Zero Area before inserting */
                if ( !SGGeod_isEqual2D( p0, p1 ) && !SGGeod_isEqual2D( p1, p2 ) && !SGGeod_isEqual2D( p0, p2 ) ) {
                    AddTriangle( p0, p1, p2 );
                }
                
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