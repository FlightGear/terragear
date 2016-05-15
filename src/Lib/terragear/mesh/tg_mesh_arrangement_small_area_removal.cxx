// using minkowski sum for inset
// This generates polygon with linear segments,
// and circle arcs

#include <CGAL/basic.h>
#include <CGAL/approximated_offset_2.h>
#include <CGAL/Gps_circle_segment_traits_2.h>

#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"
#include "../polygon_set/tg_polygon_set.hxx"

typedef CGAL::Gps_circle_segment_traits_2<meshArrKernel>  InsetTraits;
typedef InsetTraits::Polygon_2                            InsetPolygon;
typedef InsetTraits::Curve_2                              InsetCurve;

#define DEBUG_SMALLAREAS    (0)
#define LOG_SMALLAREAS      (SG_INFO)

#define MIN_AREA_THRESHOLD (0.002 * 0.002)

meshArrPolygon tgMeshArrangement::toPolygon( meshArrFaceHandle f )
{
    meshArrPolygon p;

    if ( f->has_outer_ccb() ) {
        meshArrHalfedgeCirculator ccb = f->outer_ccb();
        meshArrHalfedgeCirculator cur = ccb;
        meshArrHalfedgeHandle     curHe;

        do {
            curHe = cur;

            p.push_back( curHe->source()->point() );
            cur++;
        } while ( cur != ccb );
    }

    return p;
}

bool tgMeshArrangement::insetFaceEmpty( meshArrPolygon& p )
{
    std::list<InsetPolygon> inset_polygons;
    approximated_inset_2(p, 0.00001, 0.0000000001, std::back_inserter(inset_polygons));

    std::list<InsetPolygon>::iterator it;
    SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "The inset comprises " << inset_polygons.size() << " polygon(s)." );
    unsigned int total_verts = 0;
    for (it = inset_polygons.begin(); it != inset_polygons.end(); ++it) {
        SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "    Polygon with " << it->size() << " vertices." );
        total_verts += it->size();

#if DEBUG_SMALLAREAS
        GDALDataset* poDs = mesh->openDatasource( mesh->getDebugPath() );
        OGRLayer*    poInsetLayer = mesh->openLayer( poDs, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, "insetPolys" );

        InsetPolygon::Curve_const_iterator cit;
        for ( cit = it->curves_begin(); cit != it->curves_end(); cit++ ) {
            if ( cit->is_linear() ) {
                meshArrPoint src( CGAL::to_double( cit->source().x() ),
                                  CGAL::to_double( cit->source().y() ) );
                meshArrPoint trg( CGAL::to_double( cit->target().x() ),
                                  CGAL::to_double( cit->target().y() ) );
                toShapefile( poInsetLayer, meshArrSegment( src, trg ), "seg" );
            }
        }
        GDALClose( poDs );
#endif

    }

    return total_verts == 0 ? true : false;
}

// todo - if face is on the tile edge - don't touch
bool tgMeshArrangement::removeFace( meshArrFaceHandle f )
{
    bool faceRemoved = false;

    // to remove the face, remove all edges connected to the vertices with degree == 2
    if ( f->has_outer_ccb() ) {
        meshArrHalfedgeCirculator ccb = f->outer_ccb();
        meshArrHalfedgeCirculator cur = ccb;
        meshArrHalfedgeHandle     curHe;

        std::vector<meshArrHalfedgeHandle> delEdges;
        do {
            curHe = cur;

            // first, make sure this is not on tile edge
            if ( curHe->face()->is_unbounded() ||
                 curHe->twin()->face()->is_unbounded() ) {
                SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::removeFace - edge is part of tile edge - don't remove" );
            } else {
                // also make sure one of the edges vertices is degree 2
                meshArrVertexHandle srcv = curHe->source();
                meshArrVertexHandle trgv = curHe->target();
                if ( (srcv->degree() == 2) || (trgv->degree() == 2) ) {
                    bool added = false;

                    // make sure the edge hasn't been added
                    for ( unsigned int i=0; i<delEdges.size(); i++ ) {
                        if ( curHe == delEdges[i] || curHe->twin() == delEdges[i] ) {
                            added = true;
                            break;
                        }
                    }

                    if ( !added ) {
                        delEdges.push_back( curHe );
                    } else {
                        SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::removeFace - edge already in delEdges array" );
                    }
                } else {
                    SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::removeFace - edge is part of other " );
                }
            }

            cur++;
        } while ( cur != ccb );

        if ( delEdges.empty() ) {
            SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::removeFace - uhoh- no edges can be removed" );
        } else {
            for ( unsigned int i=0; i<delEdges.size(); i++ ) {

#if DEBUG_SMALLAREAS
                // write layer on every seg
                GDALDataset* poDs        = mesh->openDatasource( mesh->getDebugPath() );
                OGRLayer* poDelEdgeLayer = mesh->openLayer( poDs, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, "DeletedEdges" );

                meshArrPoint src( CGAL::to_double( delEdges[i]->source()->point().x() ),
                                  CGAL::to_double( delEdges[i]->source()->point().y() ) );
                meshArrPoint trg( CGAL::to_double( delEdges[i]->target()->point().x() ),
                                  CGAL::to_double( delEdges[i]->target()->point().y() ) );
                char desc[32];
                sprintf(desc, "deledge_%d", i );
                toShapefile( poDelEdgeLayer, meshArrSegment( src, trg ), desc );
                GDALClose( poDs );
#endif

                SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::removeFace - remove edge " << i << " of " << delEdges.size() );
                meshArr.remove_edge( delEdges[i], true, true );
                SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::removeFace - removed edge " << i << " of " << delEdges.size() );
            }

            faceRemoved = true;
        }
    }

    return faceRemoved;
}

void tgMeshArrangement::doRemoveSmallAreas( void )
{
    bool faceRemoved;

    do {
        meshArrFaceIterator fit;
        faceRemoved = false;
        fit = meshArr.faces_begin();
        do {
            // first, generate a polygon from the face, while calculating the area.
            meshArrPolygon poly = toPolygon( fit );

            if ( poly.area() < MIN_AREA_THRESHOLD && poly.area() > 0 ) {

#if DEBUG_SMALLAREAS
                char desc[64];

                GDALDataset* poDs = mesh->openDatasource( mesh->getDebugPath() );
                OGRLayer*    poSmallLayer = mesh->openLayer( poDs, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, "SmallAreas" );
                sprintf( desc, "%lf", CGAL::to_double(poly.area() ) );
                tgPolygonSet::toDebugShapefile( poSmallLayer, poly, desc );
                GDALClose( poDs );
#endif

                SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::doRemoveSmallAreas poly area is " << poly.area() << " which is less than " << MIN_AREA_THRESHOLD );
                if ( insetFaceEmpty( poly ) ) {
                    SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::doRemoveSmallAreas Remove Face" );
                    faceRemoved = removeFace( fit );;
                    SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::doRemoveSmallAreas Remove Face returned " << faceRemoved );

#if DEBUG_SMALLAREAS
                    if (!faceRemoved) {
                        GDALDataset* poDs = mesh->openDatasource( mesh->getDebugPath() );
                        OGRLayer*    poIssueLayer = mesh->openLayer( poDs, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, "IssueAreas" );
                        sprintf( desc, "%lf", CGAL::to_double(poly.area() ) );
                        tgPolygonSet::toDebugShapefile( poIssueLayer, poly, "cant remove" );
                        GDALClose( poDs );
                    }
#endif

                }
            } else {

#if DEBUG_SMALLAREAS
                char desc[64];

                GDALDataset* poDs = mesh->openDatasource( mesh->getDebugPath() );
                OGRLayer*    poLargeLayer = mesh->openLayer( poDs, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, "LargeAreas" );
                sprintf( desc, "%lf", CGAL::to_double(poly.area() ) );
                tgPolygonSet::toDebugShapefile( poLargeLayer, poly, desc );
                GDALClose( poDs );

                SG_LOG( SG_GENERAL, LOG_SMALLAREAS, "tgMeshArrangement::doRemoveSmallAreas poly area is " << poly.area() << " which is greater than " << MIN_AREA_THRESHOLD );
#endif

            }

            // only increment iterator if face hasn't been removed.
            // if face removed - iterator is invalid.
            if ( !faceRemoved ) {
                fit++;
            } else {
                break;
            }

        } while ( fit != meshArr.faces_end() );

    } while (faceRemoved);
}