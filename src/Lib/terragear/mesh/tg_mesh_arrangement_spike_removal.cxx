#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

#define DEBUG_SPIKES    (1)
#define LOG_SPIKES      (SG_INFO)

void tgMeshArrangement::addSharpAngle( std::vector<tgSharpAngle>& angles, meshArrVertexHandle v1, meshArrVertexHandle v2, meshArrVertexHandle v3, double angle )
{
    bool exists = false;

    for ( unsigned int i=0; i<angles.size(); i++ ) {
        if ( ( angles[i].v2 == v2 ) && 
             ( angles[i].v1 == v3 ) && 
             ( angles[i].v3 == v1 ) ) {
            exists = true;
        } else if ( ( angles[i].v2 == v2 ) && 
             ( angles[i].v1 == v1 ) && 
             ( angles[i].v3 == v3 ) ) {
            exists = true;
        }
    }

    if (!exists) {
        angles.push_back( tgSharpAngle( v1, v2, v3, angle ) );
    }
}

void tgMeshArrangement::findSpikes( meshArrFaceHandle f, std::vector<tgSharpAngle>& angles, std::vector<meshArrHalfedgeHandle>& dups )
{
    if ( f->has_outer_ccb() ) {
        meshArrHalfedgeCirculator ccb = f->outer_ccb();
        meshArrHalfedgeCirculator cur = ccb;
        meshArrHalfedgeHandle     curHe;

        do {
            curHe = cur;

            if ( curHe != curHe->next()->twin() ) {
                meshArrVertexHandle v1 = curHe->source();
                meshArrVertexHandle v2 = curHe->target();
                meshArrVertexHandle v3 = curHe->next()->target();

                meshTriVector v21 = meshTriVector(toMeshTriPoint(v2->point()), toMeshTriPoint(v1->point()));
                v21 = v21 / std::sqrt(v21 * v21);

                meshTriVector v23 = meshTriVector(toMeshTriPoint(v2->point()), toMeshTriPoint(v3->point()));
                v23 = v23 / std::sqrt(v23 * v23);

                double angle = std::acos(v21*v23)/CGAL_PI * 180;

                if (  angle == 0 ) {
                    SG_LOG( SG_GENERAL, SG_ALERT, "tgMeshArrangement::findSpikes angle is 0 !!! " );
                    // try to remove the edge, but not the vertices
                    dups.push_back( curHe );
                } else if ( angle < 1 ) {
                    // add angle if unique
                    addSharpAngle( angles, v1, v2, v3, angle );
                }
            } else {
                SG_LOG( SG_GENERAL, SG_DEBUG, "tgMeshArrangement::findSpikes he is equal to next - ignore antenna " );
            }
            cur++;
        } while ( cur != ccb );

    }
}

void tgMeshArrangement::insertAngleIntoSeries( tgSharpAngleSeriesList& saSeriesList, const tgSharpAngle& a )
{
    // look in each series endponts for center of sharp angle.
    // note the number of matches - it could be 0, 1, or 2.  
    // if 2, then we need to merge 2 series into 1.
    std::vector<unsigned int> matches;

    tgSharpAngleSeries::iterator         first;
    tgSharpAngleSeries::reverse_iterator last;
    for ( unsigned int i=0; i<saSeriesList.size(); i++ ) {
        first = saSeriesList[i].begin();
        last  = saSeriesList[i].rbegin();

        if ( a.v2 == (*first) ) {
            // find which node to add to prepend onto the series. 
            first++;
            if ( a.v1 == (*first) ) {
                // sharp angle first two vertices match - add the third
                saSeriesList[i].push_front( a.v3 );
                matches.push_back( i );
            } else if ( a.v3 == (*first) ) {
                // sharp angle last two vertices match - add the first
                saSeriesList[i].push_front( a.v1 );
                matches.push_back( i );
            } else {
                SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::insertAngleIntoSeries - Matched middle vertex in series, but not first ot second! " );
            }
        } else if ( a.v2 == (*last) ) {
            // find which node to add to append onto the series. 
            last++;
            if ( a.v1 == (*last) ) {
                // sharp angle first two vertices match - add the third
                saSeriesList[i].push_back( a.v3 );
                matches.push_back( i );
            } else if ( a.v3 == (*first) ) {
                // sharp angle last two vertices match - add the first
                saSeriesList[i].push_back( a.v2 );
                matches.push_back( i );
            } else {
                SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::insertAngleIntoSeries - Matched middle vertex in series, but not first ot second! " );
            }
        }
    }

    switch( matches.size() ) {
        case 0:
        {
            // no matching series - add a new one
            tgSharpAngleSeries series;
            series.push_back( a.v1 );
            series.push_back( a.v2 );
            series.push_back( a.v3 );

            saSeriesList.push_back( series );
            SG_LOG( SG_GENERAL, LOG_SPIKES, "tgMesh::insertAngleIntoSeries - Did not match sharp angle at a current series.  Added new, and we now have " << matches.size() << " series" );
            break;
        }

        case 1:
        {
            // we've already matched - we're done;
            SG_LOG( SG_GENERAL, LOG_SPIKES, "tgMesh::insertAngleIntoSeries - Matched sharp angle at series " << matches[0] << " DONE" );
            break;
        }

        case 2:
        {
            // we found two matches.
            SG_LOG( SG_GENERAL, LOG_SPIKES, "tgMesh::insertAngleIntoSeries - Matched sharp angle at series " << matches[0] << " and " << matches[1] );
            // todo
            break;
        }

        default:
        {
            SG_LOG( SG_GENERAL, LOG_SPIKES, "tgMesh::insertAngleIntoSeries - Matched sharp angle " << matches.size() << " times.  THIS SHOULD NOT HAPPEN!" );
            break;
        }
    }
}

void tgMeshArrangement::doRemoveSpikes( SGMutex* lock )
{
    std::vector<tgSharpAngle>           angles;
    std::vector<meshArrHalfedgeHandle>  dups;

    // instead of snap round - lets remove spikes...
    // traverse the faces - look for sharp angles with long edges...
    for ( meshArrFaceIterator fit = meshArr.faces_begin(); fit != meshArr.faces_end(); fit++ ) {
        findSpikes( fit, angles, dups );
    }

    if ( !dups.empty() ) {

#if DEBUG_SPIKES
        GDALDataset* poDs    = mesh->openDatasource( mesh->getDebugPath() );
        for ( unsigned int i=0; i<dups.size(); i++ ) {
            char layer[16];

            sprintf(layer, "dup_%05d", i);
            OGRLayer*    poLayer = mesh->openLayer( poDs, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, layer );

            toShapefile( poLayer, meshArrSegment( dups[i]->source()->point(), dups[i]->target()->point() ), "dupe" );
        }
        GDALClose( poDs );
#endif

        for ( unsigned int i=0; i<dups.size(); i++ ) {
            meshArr.remove_edge( dups[i], false, false );
        }
    }

    if ( !angles.empty() ) {

#if DEBUG_SPIKES
        GDALDataset* poDs    = mesh->openDatasource( mesh->getDebugPath() );
        for ( unsigned int i=0; i<angles.size(); i++ ) {
            char layer[32];
            char desc[32];

            sprintf(layer, "spike_%05d_segs", i);
            OGRLayer*    poLineLayer = mesh->openLayer( poDs, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, layer );

            sprintf(layer, "spike_%05d_pts", i);
            OGRLayer*    poPointLayer = mesh->openLayer( poDs, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, layer );

            sprintf(desc, "%f", angles[i].angle );
            toShapefile( poLineLayer, meshArrSegment( angles[i].v2->point(), angles[i].v1->point() ), "v2-v1" );
            toShapefile( poLineLayer, meshArrSegment( angles[i].v2->point(), angles[i].v3->point() ), "v2-v3" );

            toShapefile( poPointLayer, angles[i].v1->point(), "v1" );
            toShapefile( poPointLayer, angles[i].v2->point(), "v2" );
            toShapefile( poPointLayer, angles[i].v3->point(), "v3" );
        }
        GDALClose( poDs );
#endif

        // now we need to check if we should combine sharp angles into a series
        tgSharpAngleSeriesList saSeriesList;
        for ( unsigned int i=0; i<angles.size(); i++ ) {
            insertAngleIntoSeries( saSeriesList, angles[i] );
        }

        // now we traverse the series list, and handle each series.
        for ( unsigned int i=0; i<saSeriesList.size(); i++ ) {
            // fix each series by themselves
            tgSharpAngleSeries* curSeries = &saSeriesList[i];   // cur series is a list of handles

            SG_LOG(SG_GENERAL, LOG_SPIKES, "hadling series " << i << " size " << curSeries->size() );

#if DEBUG_SPIKES
            char name[128];
            sprintf( name, "remove_spikes_series_%d_begin", i );
            toShapefile( mesh->getDebugPath().c_str(), name );
#endif

            // traverse each interior vertex;
            tgSharpAngleSeries::iterator firstVertex = curSeries->begin();
            tgSharpAngleSeries::iterator curVertex   = firstVertex;

            meshArrVertexHandle prvHandle = (*curVertex);
            curVertex++;
            meshArrVertexHandle curHandle = (*curVertex);

            tgSharpAngleSeries::iterator lastVertex = curSeries->end();

            unsigned int curIdx = 1;
            while( curVertex != lastVertex ) {
                if ( curHandle->degree() == 2 ) {
                    // for degree 2 - just remove edge from prevHandle to curHandle
                    meshArrIncidentHalfedgeCirculator   firstCirc = curHandle->incident_halfedges();
                    meshArrIncidentHalfedgeCirculator   curCirc   = firstCirc;

                    bool foundEdge = false;
                    do {
                        meshArrHalfedgeHandle   curHe = curCirc;

                        if ( curHe->source() == prvHandle ) {
                            SG_LOG(SG_GENERAL, LOG_SPIKES, "found edge from " << curIdx-1 << " to " << curIdx );
                            meshArr.remove_edge( curHe, false, false );
                            foundEdge = true;
                            break;
                        }

                        curCirc++;
                    } while ( curCirc != firstCirc );

                    if (!foundEdge) {
                        SG_LOG(SG_GENERAL, LOG_SPIKES, "vertex with degree 2 at index " << curIdx << " couldn't find edge to remove" );
                    }
                } else {
                    SG_LOG(SG_GENERAL, LOG_SPIKES, "sharp angle vertex is degree " << curHandle->degree() << " at index " << curIdx );
                }

#if DEBUG_SPIKES
                char name[128];
                sprintf( name, "remove_spikes_series_%d_vertex_%d", i, curIdx );
                toShapefile( mesh->getDebugPath().c_str(), name );
#endif

                prvHandle = (*curVertex);
                curVertex++;

                // don't go past end -  better loop needed?
                if ( curVertex != lastVertex ) {
                    curHandle = (*curVertex);
                    curIdx++;
                }
            }

            // now connect first to last = make sure it isn't there already
            SG_LOG(SG_GENERAL, LOG_SPIKES, "add new segment from vertex 0 to vertex " << curIdx-1 );
            lastVertex--;

            meshArrIncidentHalfedgeCirculator firstCirc = (*firstVertex)->incident_halfedges();
            meshArrIncidentHalfedgeCirculator   curCirc   = firstCirc;

            bool foundEdge = false;
            do {
                meshArrHalfedgeHandle   curHe = curCirc;

                if ( curHe->source() == (*lastVertex) ) {
                    foundEdge = true;
                    SG_LOG(SG_GENERAL, LOG_SPIKES, "Edge already exists " );
                    break;
                }

                curCirc++;
            } while ( curCirc != firstCirc );

            if (!foundEdge) {
                meshArr.insert_at_vertices( meshArrSegment( (*firstVertex)->point(), (*lastVertex)->point() ), (*firstVertex), (*lastVertex) );
            }

            // and remove the isolated verticies in between
#if DEBUG_SPIKES
            sprintf( name, "remove_spikes_series_%d_complete", i );
            toShapefile( mesh->getDebugPath().c_str(), name );
#endif
        }
    }
}

// antenna are sortof like spikes - easier to find and remove, though.
void tgMeshArrangement::doRemoveAntenna( void )
{
    // remove all edges that have the same face on both sides
    bool edgeRemoved;
    do {
        meshArrEdgeIterator eit;
        edgeRemoved = false;
        for ( eit = meshArr.edges_begin(); eit != meshArr.edges_end(); ++eit ) {
            if ( eit->face() == eit->twin()->face() ) {
                SG_LOG( SG_GENERAL, LOG_SPIKES, "tgMesh::cleanArrangmentFound antenna in cleaned arrangement" );
                CGAL::remove_edge( meshArr, eit );
                edgeRemoved = true;
                break;
            }
        }
    } while (edgeRemoved);
}
