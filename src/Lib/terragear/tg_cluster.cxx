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


std::list<EPECSegment_2>::iterator it;
std::vector<EPECPoint_2>::iterator vfrom,vto;
std::vector<EPECPoint_2>::iterator fcit;

EPECPoint_2 tgCluster::Locate(const EPECPoint_2& point)
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
    std::vector<tgVoronoiCell>::iterator    oc, nc;
    
    tag tg;
    vd.clear();
    for(oc = oldcells.begin(); oc!= oldcells.end(); oc++)
    {
        (*oc).face = vd.insert((*oc).centroid);
        (*oc).nodes.clear();
    }
    assert( vd.is_valid() );

    for ( std::list<EPECPoint_2>::iterator it = nodes.begin(); it != nodes.end(); it++ )
    {
        EPECPoint_2 q = Locate(*it);

        for(oc = oldcells.begin(); oc!= oldcells.end(); oc++)
        {
            if ( (*oc).centroid == q ) {
                (*oc).nodes.push_back(*it);
                break;
            }
        }
        
#if 0
        VDLocateResult lr = vd.locate(*it);
        if ( VDVertexHandle* v = boost::get<VDVertexHandle>(&lr) ) 
        {
            EPECPoint_2 q = (*v)->site(0)->point();
            for(oc = oldcells.begin(); oc!= oldcells.end(); oc++)
            {
                if ( (*oc).centroid.x() == q.x() && (*oc).centroid.y() == q.y() ) {
                    (*oc).nodes.push_back(*it);
                    break;
                }
            }
        } 
        else if ( VDHalfedgeHandle* e = boost::get<VDHalfedgeHandle>(&lr) )
        {
            EPECPoint_2 q = (*e)->up()->point();
        
            for(oc = oldcells.begin(); oc!= oldcells.end(); oc++)
            {
                if ( (*oc).centroid.x() == q.x() && (*oc).centroid.y() == q.y() ) {
                    (*oc).nodes.push_back(*it);
                    break;
                }
            }
        } 
        else if ( VDFaceHandle* f = boost::get<VDFaceHandle>(&lr) ) 
        {
            EPECPoint_2 q = (*f)->dual()->point();
            
            for(oc = oldcells.begin(); oc!= oldcells.end(); oc++)
            {
                if ( (*oc).centroid.x() == q.x() && (*oc).centroid.y() == q.y() ) {
                    (*oc).nodes.push_back(*it);
                    break;
                }
            }
        }
#endif        
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
            newvc.centroid = CGAL::centroid((*oc).nodes.begin(),(*oc).nodes.end(),tg);
            newvc.face     = (*oc).face;
            newvc.nodes    = (*oc).nodes;
        }
        newcells.push_back(newvc);        
    }
}

tgCluster::tgCluster( std::list<EPECPoint_2>& points, double squaredError )
{
    int kiter = 40, iiter = 0;
    bool is_equal = false;
        
    std::list<EPECPoint_2>::iterator lit;
    std::vector<EPECPoint_2>::iterator it;
    
    // move to newcentroids
    for ( lit = points.begin(); lit != points.end(); lit++ ) {
        oldcentroids.push_back( (*lit) );
    }
    
    bool merged_centroid = false;
    int tree_iter = 1;
    
    do {
        // move new centroids into kd-tree
        std::cout << "Find centroids iteration " << tree_iter << " num_centroids is " << oldcentroids.size() << std::endl;
        
        tree.clear();
        for ( it = oldcentroids.begin(); it != oldcentroids.end(); it++ ) {
            tree.insert( (*it) );
        }
        
        // clear new centroids
        newcentroids.clear();

        merged_centroid = false;
        std::list<EPECPoint_2> query_result;
        
        for ( it = oldcentroids.begin(); it != oldcentroids.end(); it++ ) {
            VDnodesFuzzyCir query_circle( (*it), 0.0000025);  // approx 25 cm
            
            // perform the query
            query_result.clear();
            tree.search(std::back_inserter( query_result ), query_circle);
            
            // we should always have at least one result
            if ( query_result.size() > 0 ) 
            {                 
                if ( query_result.size() > 1 ) {
                    tag tg;
                    EPECPoint_2 cent = centroid( query_result.begin(), query_result.end(), tg);
                    
                    if ( std::find(newcentroids.begin(), newcentroids.end(), cent ) == newcentroids.end() ) {                     
                        newcentroids.push_back( cent );
                        merged_centroid = true;
                    }
                } else {
                    newcentroids.push_back( *(query_result.begin()) );
                }
            }
        }
        
        // copy new centroids to old array
        oldcentroids.clear();
        for ( it = newcentroids.begin(); it != newcentroids.end(); it++ ) {
            oldcentroids.push_back( *it );
        }
        
        tree_iter++;
    } while ( merged_centroid );      
    
    nodes = points;
    
    // create voronoi cells for each centroid
    for ( it = oldcentroids.begin(); it != oldcentroids.end(); it++ ) {
        tgVoronoiCell newvc;
        newvc.centroid = *it;
        oldcells.push_back( newvc );
    }
    
    while(1)
    {
        if(iiter != kiter)
        {
            std::cout << "Voronoi Relaxation iteration " << iiter << std::endl;
            
            computenewcentroids();
            iiter = iiter + 1;

            if(newcells.size() < oldcells.size())
            {
                is_equal = std::equal(newcells.begin(),newcells.end(),oldcells.begin());
            }
            else
            {
                is_equal = std::equal(oldcells.begin(),oldcells.end(),newcells.begin());
            }

            if(is_equal)
            {
                is_equal = false;
                std::cout << "Reached Convergence.." << std::endl;
                
                //DumpVD(newcells);
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
        else
        {
            std::cout << "Reached maximum number of iterations - Exiting..." << std::endl;
        }	
    }
    std::cout << "Finished" << std::endl;
}

void tgCluster::DumpVD(std::vector<tgVoronoiCell>& cells)
{
    char description[32];
    int  cell_id = 1;
    
    std::vector<tgVoronoiCell>::iterator cit = newcells.begin();    
    for(cit = cells.begin(); cit!= cells.end(); cit++, cell_id++)
    {
        std::vector<tgSegment>   edges;
        
        // label each centroid
        sprintf( description, "voronoi_cell_%04d", cell_id );
        
        // dump centroid ( only if there are more than one nodes in the cell )
        if ( (*cit).nodes.size() > 1 ) {
            SGGeod centroid = SGGeod::fromDeg( CGAL::to_double( (*cit).centroid.x() ),
                                               CGAL::to_double( (*cit).centroid.y() ) );
            tgShapefile::FromGeod( centroid, "./edge_dbg", "centroids", description );
        }
        
        // dump edges
        VDCcbHalfedgeCirculator ec_start = (*cit).face->ccb();
        VDCcbHalfedgeCirculator ec       = ec_start;
        
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
        tgShapefile::FromSegmentList( edges, false, "./edge_dbg", "edges", description );

        // generate node list
        for ( std::vector<EPECPoint_2>::iterator nit = (*cit).nodes.begin(); nit != (*cit).nodes.end(); nit++ ) 
        {
            SGGeod node = SGGeod::fromDeg( CGAL::to_double( (*nit).x() ),
                                           CGAL::to_double( (*nit).y() ) );
            tgShapefile::FromGeod( node, "./edge_dbg", "nodes", description );            
        }    
    }
}