#include <iostream>
#include <fstream>
#include <cassert>
#include <iterator>
#include <CGAL/Cartesian.h>
#include <vector>
#include <math.h>
#include <algorithm>

#include <simgear/math/SGMath.hxx>

#include "tg_cluster.hxx"
#include "tg_shapefile.hxx"

// TODO Voronoi convergence is a bit slow - anything faster?
#define DEBUG_CLUSTER   (0)
#define LOG_CLUSER      SG_DEBUG


std::list<EPECSegment_2>::iterator it;
std::vector<EPECPoint_2>::iterator vfrom,vto;
std::vector<EPECPoint_2>::iterator fcit;

EPECPoint_2 tgCluster::Locate(const EPECPoint_2& point) const
{
    VDLocateResult lr = vd.locate(point);
    EPECPoint_2    q;

    if ( VDVertexHandle* v = boost::get<VDVertexHandle>(&lr) ) 
    {
        q = (*v)->site(0)->point();
    } 
    else if ( VDHalfedgeHandle* e = boost::get<VDHalfedgeHandle>(&lr) )
    {
        q = (*e)->up()->point();        
    } 
    else if ( VDFaceHandle* f = boost::get<VDFaceHandle>(&lr) ) 
    {
        q = (*f)->dual()->point();
    }

    return q;
}

void tgCluster::computenewcentroids(void)
{
    std::vector<tgVoronoiCell>::iterator oc, nc;

    tag tg;
    vd.clear();
    for(oc = oldcells.begin(); oc!= oldcells.end(); oc++)
    {
        (*oc).face = vd.insert((*oc).centroid);
        (*oc).nodes.clear();
    }
    assert( vd.is_valid() );

    for ( std::list<tgClusterNode>::iterator it = nodes.begin(); it != nodes.end(); it++ )
    {
        EPECPoint_2 q = Locate(it->point);

        for(oc = oldcells.begin(); oc!= oldcells.end(); oc++)
        {
            if ( (*oc).centroid == q ) {
                (*oc).nodes.push_back(*it);
                break;
            }
        }
    }

    // UPDATE THE CENTROIDS HERE
    oc = oldcells.begin();
    newcells.clear();

    for(oc = oldcells.begin(); oc!= oldcells.end(); oc++)
    {
        tgVoronoiCell newvc;
        if ((*oc).nodes.empty())
        {
            newvc.centroid = (*oc).centroid;
            newvc.face     = (*oc).face;
            newvc.nodes    = (*oc).nodes;
        }
        else
        {
            // if he voronoi cell has fixed nodes, set them as centroids for new cells
            std::vector<tgClusterNode>::const_iterator nit = (*oc).nodes.begin();

            // todo - split a voronoi cell if more than 1 fixed node inside
            std::vector<EPECPoint_2> positions;
            while ( nit != (*oc).nodes.end() ) {
                positions.push_back( nit->point );
                nit++;
            }

            newvc.centroid = CGAL::centroid(positions.begin(),positions.end(),tg);
            newvc.face     = (*oc).face;
            newvc.nodes    = (*oc).nodes;
        }
        newcells.push_back(newvc);
    }
}

tgCluster::tgCluster( std::list<tgClusterNode>& points, double err, const std::string& d )
{
    int  kiter = 40, iiter = 0;
    bool is_equal = false;

    squaredError = err;
    debug = d;

    std::list<tgClusterNode>::iterator lit;
    std::vector<tgClusterNode>::iterator it;

    // first step is to merge points really close to one another.
    // note: fixed nodes Should NOT move!
    // move to newcentroids
    for ( lit = points.begin(); lit != points.end(); lit++ ) {
        oldcentroids.push_back( *lit );
    }

    SG_LOG( SG_GENERAL, LOG_CLUSER,  "tgCluster: " << oldcentroids.size() << " original points" );

#if DEBUG_CLUSTER
    toShapefile( debug, "original_points", oldcentroids );
#endif

    // remove dups
    char layer[256];

    tree.clear();
    for ( it = oldcentroids.begin(); it != oldcentroids.end(); it++ ) {
        tree.insert( nodesData( it->point, it ) );
    }

    // clear new centroids
    newcentroids.clear();

    std::list<nodesData>           query_result;
    std::list<nodesData>::iterator qrit;

    unsigned int centroidIdx = 1;
    for ( it = oldcentroids.begin(); it != oldcentroids.end(); it++ ) {
        if ( !it->added ) {
            it->added = true;

            VDnodesFuzzyCir query_circle( (it->point), 0.000000001);

            // perform the query
            query_result.clear();
            tree.search(std::back_inserter( query_result ), query_circle);

            // we should always have at least one result
            if ( query_result.size() > 1 ) {
                std::vector<EPECPoint_2>             positions;
                std::vector<tgClusterNode>::iterator pos_it;

                // we create centroids for each fixed node.
                // If there are no fixed nodes, take the average
                bool fixed = false;
                for ( qrit = query_result.begin(); qrit != query_result.end(); qrit++ ) {
                    EPECPoint_2 pos = boost::get<0>(*qrit);
                    pos_it = boost::get<1>(*qrit);

                    positions.push_back(pos);
                    pos_it->added = true;
                    if ( pos_it->fixed ) {
                        fixed = true;
                    }
                }

                SG_LOG( SG_GENERAL, LOG_CLUSER,  " - found " << positions.size() << " dups" );

#if DEBUG_CLUSTER
                GDALDataset* poDs = openDatasource( debug );

                sprintf( layer, "node_%05d_dup", centroidIdx );
                OGRLayer* poDupLayer = openLayer( poDs, wkbPoint25D, layer );
                toShapefile( poDupLayer, positions );
#endif

                newcentroids.push_back( tgClusterNode( it->point, fixed ) );

#if DEBUG_CLUSTER
                GDALClose( poDs );
#endif

            } else {
                newcentroids.push_back( tgClusterNode( it->point, it->fixed ) );
            }

            centroidIdx++;
        }
    }

    // copy new centroids to old array
    oldcentroids.clear();
    for ( it = newcentroids.begin(); it != newcentroids.end(); it++ ) {
        oldcentroids.push_back( tgClusterNode(it->point, it->fixed) );
    }

    SG_LOG( SG_GENERAL, LOG_CLUSER,  "tgCluster: " << oldcentroids.size() << " unique points " );

    int tree_iter = 1;
    bool merged_centroid = false;

    do {
        // move new centroids into kd-tree
        SG_LOG( SG_GENERAL, LOG_CLUSER,  "Find centroids iteration " << tree_iter << " num_centroids is " << oldcentroids.size() );

        tree.clear();
        for ( it = oldcentroids.begin(); it != oldcentroids.end(); it++ ) {
            tree.insert( nodesData( it->point, it ) );
        }

        // clear new centroids
        newcentroids.clear();

        merged_centroid = false;
        std::list<nodesData>           query_result;
        std::list<nodesData>::iterator qrit;

        unsigned int centroidIdx = 1;
        for ( it = oldcentroids.begin(); it != oldcentroids.end(); it++ ) {
            if ( !it->added ) {
                VDnodesFuzzyCir query_circle( (it->point), squaredError);

                // perform the query
                query_result.clear();
                tree.search(std::back_inserter( query_result ), query_circle);

                // we should always have at least one result
                if ( query_result.size() > 1 ) {
                    std::vector<EPECPoint_2> fixedPos;
                    std::vector<EPECPoint_2> notFixedPos;
                    tgClusterNode            center;
                    tag                      tg;

                    // we create centroids for each fixed node.
                    // If there are no fixed nodes, take the average
                    for ( qrit = query_result.begin(); qrit != query_result.end(); qrit++ ) {
                        EPECPoint_2 pos = boost::get<0>(*qrit);
                        std::vector<tgClusterNode>::iterator pos_it = boost::get<1>(*qrit);;

                        // we only care abount non-added points
                        if ( !pos_it->added ) {
                            if ( pos_it->fixed ) {
                                fixedPos.push_back(pos);
                            } else {
                                notFixedPos.push_back(pos);
                            }

                            pos_it->added = true;
                        }
                    }

                    SG_LOG( SG_GENERAL, LOG_CLUSER,  " - found " << fixedPos.size() << " fixed nodes and " << notFixedPos.size() << " not fixed nodes" );

    #if DEBUG_CLUSTER
                    GDALDataset* poDs = openDatasource( debug );

                    sprintf( layer, "tree_%02d_node_%05d_cent", tree_iter, centroidIdx );
                    OGRLayer* poCentroidLayer = openLayer( poDs, wkbPoint25D, layer );
                    toShapefile( poCentroidLayer, (*it) );

                    if ( !fixedPos.empty() ) {
                        sprintf( layer, "tree_%02d_node_%05d_fixed", tree_iter, centroidIdx );
                        OGRLayer* poFixedLayer = openLayer( poDs, wkbPoint25D, layer );
                        toShapefile( poFixedLayer, fixedPos );
                    }

                    if ( !notFixedPos.empty() ) {
                        sprintf( layer, "tree_%02d_node_%05d_not_fixed", tree_iter, centroidIdx );
                        OGRLayer* poNotFixedLayer = openLayer( poDs, wkbPoint25D, layer );
                        toShapefile( poNotFixedLayer, notFixedPos );
                    }

                    sprintf( layer, "tree_%02d_node_%05d_new_centers", tree_iter, centroidIdx );
                    OGRLayer* poNewCentersLayer = openLayer( poDs, wkbPoint25D, layer );
    #endif

                    if ( !fixedPos.empty() ) {
                        for ( unsigned int i=0; i<fixedPos.size(); i++ ) {
                            center = tgClusterNode( fixedPos[i], true );

    #if DEBUG_CLUSTER
                            toShapefile( poNewCentersLayer, center );
    #endif

                            newcentroids.push_back( center );
                        }
                    } else if ( !notFixedPos.empty() )  {
                        center = tgClusterNode( centroid( notFixedPos.begin(), notFixedPos.end(), tg), false );

    #if DEBUG_CLUSTER
                        toShapefile( poNewCentersLayer, center );
    #endif

                        newcentroids.push_back( center );
                        merged_centroid = true;
                    } else {
                        SG_LOG( SG_GENERAL, LOG_CLUSER,  " no other nodes in cluster" );
                        newcentroids.push_back( tgClusterNode( it->point, it->fixed ) );
                    }

    #if DEBUG_CLUSTER
                    GDALClose( poDs );
    #endif

                } else {
                    qrit = query_result.begin();
                    EPECPoint_2 pos = boost::get<0>(*qrit);
                    std::vector<tgClusterNode>::iterator pos_it = boost::get<1>(*qrit);;

                    pos_it->added = true;

                    newcentroids.push_back( tgClusterNode( pos, pos_it->fixed ) );
                }
            }

            centroidIdx++;
        }

        // copy new centroids to old array
        oldcentroids.clear();
        for ( it = newcentroids.begin(); it != newcentroids.end(); it++ ) {
            oldcentroids.push_back( *it );
        }

        tree_iter++;
    } while ( merged_centroid );

    nodes = points;

    SG_LOG( SG_GENERAL, LOG_CLUSER,  " Do voronoi relaxation with " << oldcentroids.size() << " nodes" );

    // create voronoi cells for each centroid
    for ( it = oldcentroids.begin(); it != oldcentroids.end(); it++ ) {
        tgVoronoiCell newvc;
        newvc.centroid = it->point;
        newvc.fixed    = it->fixed;
        oldcells.push_back( newvc );
    }

    while( iiter < kiter )
    {
        computenewcentroids();
        iiter++;

        if(newcells.size() < oldcells.size())
        {
            is_equal = std::equal(newcells.begin(),newcells.end(),oldcells.begin());
        }
        else
        {
            is_equal = std::equal(oldcells.begin(),oldcells.end(),newcells.begin());
        }

        if( is_equal )
        {
            is_equal = false;
            break;
        }
        else 
        {
            // iterate again - move new to old
            oldcells.clear();
            std::vector<tgVoronoiCell>::iterator cit = newcells.begin();
            for(cit = newcells.begin(); cit!= newcells.end(); cit++)
            {
                tgVoronoiCell newvc;
                newvc.centroid = (*cit).centroid;
                oldcells.push_back( newvc );
            }
        }
    }
}

void tgCluster::toShapefile( const char* datasource, const char* layer_prefix )
{
    char layer[256];
    char layer2[256];
    char description[32];
    int  cell_id = 1;

    std::vector<tgVoronoiCell>::iterator cit;
    for(cit = newcells.begin(); cit!= newcells.end(); cit++, cell_id++)
    {
        std::vector<tgSegment> edges;

        // label each centroid
        sprintf( description, "voronoi_cell_%04d", cell_id );
        sprintf( layer, "%s_centroids", layer_prefix ); 

        // dump centroid ( only if there are more than one nodes in the cell )
        if ( (*cit).nodes.size() > 1 ) {
            SGGeod centroid = SGGeod::fromDeg( CGAL::to_double( (*cit).centroid.x() ),
                                               CGAL::to_double( (*cit).centroid.y() ) );

            tgShapefile::FromGeod( centroid, datasource, layer, description );
        }

        // dump edges
        VDCcbHalfedgeCirculator ec_start = (*cit).face->ccb();
        VDCcbHalfedgeCirculator ec       = ec_start;

        sprintf( layer, "%s_edges", layer_prefix ); 
        edges.clear();
        do
        {
            VDHalfedgeHandle e = ec;
            if ( e->is_segment() ) {
                SGGeod gsrc = SGGeod::fromDeg( CGAL::to_double(e->source()->point().x()),
                                               CGAL::to_double(e->source()->point().y()) );
                SGGeod gtrg = SGGeod::fromDeg( CGAL::to_double(e->target()->point().x()),
                                               CGAL::to_double(e->target()->point().y()) );

                edges.push_back( tgSegment(gsrc, gtrg) );
            }

        } while ( ++ec != ec_start );

        tgShapefile::FromSegmentList( edges, false, datasource, layer, description );

        // generate node list
        sprintf( layer, "%s_nodes", layer_prefix ); 
        sprintf( layer2, "%s_fixed_nodes", layer_prefix ); 
        for ( std::vector<tgClusterNode>::iterator nit = (*cit).nodes.begin(); nit != (*cit).nodes.end(); nit++ ) 
        {
            SGGeod node = SGGeod::fromDeg( CGAL::to_double( nit->point.x() ),
                                           CGAL::to_double( nit->point.y() ) );
            if ( nit->fixed ) {
                tgShapefile::FromGeod( node, datasource, layer2, description );
            } else {
                tgShapefile::FromGeod( node, datasource, layer, description );
            }
        }
    }
}

GDALDataset* tgCluster::openDatasource( const std::string& datasource_name ) const
{
    GDALDataset*    poDS = NULL;
    GDALDriver*     poDriver = NULL;
    const char*     format_name = "ESRI Shapefile";

    SG_LOG( SG_GENERAL, SG_DEBUG, "Open Datasource: " << datasource_name );

    GDALAllRegister();

    poDriver = GetGDALDriverManager()->GetDriverByName( format_name );
    if ( poDriver ) {
        poDS = poDriver->Create( datasource_name.c_str(), 0, 0, 0, GDT_Unknown, NULL );
    }

    return poDS;
}

OGRLayer* tgCluster::openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, const char* layer_name ) const
{
    OGRLayer*           poLayer = NULL;
 
    if ( !strlen( layer_name )) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgCluster::openLayer: layer name is NULL" );
        exit(0);
    }

    poLayer = poDS->GetLayerByName( layer_name );
    if ( !poLayer ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgCluster::openLayer: layer " << layer_name << " doesn't exist - create" );

        OGRSpatialReference srs;
        srs.SetWellKnownGeogCS("WGS84");

        poLayer = poDS->CreateLayer( layer_name, &srs, lt, NULL );

        OGRFieldDefn descriptionField( "tg_desc", OFTString );
        descriptionField.SetWidth( 128 );
        if( poLayer->CreateField( &descriptionField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_desc' failed" );
        }
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgCluster::openLayer: layer " << layer_name << " already exists - open" );        
    }

    return poLayer;
}

void tgCluster::toShapefile( const std::string& ds, const char* layer, const std::vector<tgClusterNode>& nodes )
{
    GDALDataset* poDS = openDatasource( ds );
    OGRLayer* poLayer = openLayer( poDS, wkbPoint25D, layer);

    for ( unsigned int i=0; i<nodes.size(); i++ ) {
        toShapefile( poLayer, nodes[i] );
    }

    GDALClose( poDS );
}

void tgCluster::toShapefile( OGRLayer* poLayer, const tgClusterNode& node )
{
    OGRPoint    point;
    char        desc[32];

    point.setZ( 0.0 );    
    point.setX( CGAL::to_double( node.point.x() ) );
    point.setY( CGAL::to_double( node.point.y() ) );

    sprintf( desc, "%s", node.fixed ? "fixed" : "not fixed" );
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&point);    
    poFeature->SetField("tg_desc", desc );

    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);
}

void tgCluster::toShapefile( OGRLayer* poLayer, const std::vector<EPECPoint_2>& points )
{
    for ( unsigned int i=0; i<points.size(); i++ ) {
        toShapefile( poLayer, points[i] );
    }
}
 
void tgCluster::toShapefile( OGRLayer* poLayer, const EPECPoint_2& pt )
{
    OGRPoint    point;

    point.setZ( 0.0 );    
    point.setX( CGAL::to_double( pt.x() ) );
    point.setY( CGAL::to_double( pt.y() ) );

    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&point);    
    poFeature->SetField("tg_desc", "pt" );

    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);
}