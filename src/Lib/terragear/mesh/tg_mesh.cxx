#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

void tgMesh::initPriorities( const std::vector<std::string>& names )
{
    priorityNames = names;
    numPriorities = names.size();
   
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        tgPolygonSetList lc;
        sourcePolys.push_back( lc );
    }
    
    clipBucket = false;
}

void tgMesh::initDebug( const std::string& dbgRoot )
{
    datasource = dbgRoot;
}

void tgMesh::clipAgainstBucket( const SGBucket& bucket ) 
{
    clipBucket = true;
    b = bucket;
}

void tgMesh::clear( void )
{
    // clear source polys
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        sourcePolys[i].clear();
    }
    sourcePoints.clear();
    
    meshPointLocation.detach();
    meshArr.clear();
    meshTriangulation.clear();
    metaLookup.clear();    
}

bool tgMesh::empty( void )
{
    bool empty = true;
    
    // check source polys
    for ( unsigned int i=0; empty && i<numPriorities; i++ ) {
        empty = sourcePolys.empty();
    }
    
    return empty;
}

void tgMesh::addPoly( unsigned int priority, const tgPolygonSet& poly )
{
    sourcePolys[priority].push_back( poly );
}

void tgMesh::addPolys( unsigned int priority, const tgPolygonSetList& polys )
{
    sourcePolys[priority].insert( sourcePolys[priority].end(), polys.begin(), polys.end() );
}

void tgMesh::addPoints( const std::vector<cgalPoly_Point>& points )
{
    sourcePoints.insert( sourcePoints.end(), points.begin(), points.end() );    
}

tgPolygonSet tgMesh::join( unsigned int priority, const tgPolygonSetMeta& meta )
{
    return tgPolygonSet::join( sourcePolys[priority], meta );    
}

void tgMesh::generate( void )
{    
    bool havePolys = false;

    for ( unsigned int i=0; i<numPriorities && !havePolys; i++ ) {
        std::vector<tgPolygonSet>::iterator poly_it;
        if ( !sourcePolys[i].empty() ) {
            havePolys = true;
        }
    }
    
    // mesh generation from polygon soup :)
    if ( havePolys ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "have source polys" );

        // Step 1 - clip polys against one another - highest priority first ( on top )
        clipPolys();
    
        // Step 2 - insert clipped polys into an arrangement.  
        // From this point on, we don't need the individual polygons.
        arrangePolys();
    
        // step 3 - clean up the arrangement - cluster nodes that are too close - don't want
        // really small triangles blowing up the refined mesh.
        // NOTE / TODO: 
        // The cluster size MUST be smaller than the minimum distance of interiorPoints.
        // we should remember be checking the delta in interiorPoints to see if we have 
        // polys that don't meat this criteria.
        // and if it doesn't - what do we do?
        cleanArrangement();
    
        // step 4 - create constrained triangulation with arrangement edges as the constraints
        constrainedTriangulateWithEdgeModification();
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "no source polys" );        
    }
}

#include <CGAL/Fuzzy_iso_box.h>

typedef CGAL::Search_traits_2<meshTriKernel>        cgalPoly_SearchTraits;
typedef CGAL::Fuzzy_iso_box<cgalPoly_SearchTraits>  cgalPoly_FuzzyBox;
typedef CGAL::Kd_tree<cgalPoly_SearchTraits>        cgalPoly_SearchTree;

const double fgPoint3_Epsilon = 0.00000001;
void tgMesh::getEdgeNodes( std::vector<meshTriPoint>& north, std::vector<meshTriPoint>& south, std::vector<meshTriPoint>& east, std::vector<meshTriPoint>& west ) const {
    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();
    
    meshTriPoint        ll;
    meshTriPoint        ur;
    cgalPoly_FuzzyBox   exact_bb;
    
    std::list<meshTriPoint> result;
    std::list<meshTriPoint>::iterator it;
    
    north.clear();
    south.clear();
    east.clear();
    west.clear();
    
    // generate the search tree
    cgalPoly_SearchTree tree;    
    for (meshTriCDTPlus::All_vertices_iterator vit=meshTriangulation.all_vertices_begin(); vit!=meshTriangulation.all_vertices_end(); ++vit) {
        tree.insert( vit->point() );
    }
    
    // find northern points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, north_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = cgalPoly_FuzzyBox(ll, ur);

    result.clear();
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        north.push_back(*it);
    }

    // find southern points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, south_compare + fgPoint3_Epsilon );
    exact_bb = cgalPoly_FuzzyBox(ll, ur);
    result.clear();

    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        south.push_back(*it);
    }

    // find eastern points
    ll = meshTriPoint( east_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = cgalPoly_FuzzyBox(ll, ur);
    result.clear();

    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        east.push_back(*it);
    }

    // find western points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( west_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = cgalPoly_FuzzyBox(ll, ur);
    result.clear();

    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        west.push_back(*it);
    }
}


void tgMesh::save( const std::string& path ) const
{
    toShapefile( path, "stage1_arrangement", meshArr );
    toShapefile( path, "stage1_triangles", meshTriangulation, true );
    
    // generate edge node list
    if ( clipBucket ) {
        std::vector<meshTriPoint> north;
        std::vector<meshTriPoint> south;
        std::vector<meshTriPoint> east;
        std::vector<meshTriPoint> west;
        
        getEdgeNodes( north, south, east, west );
        
        // save these arrays in a point layer
        toShapefile( path, "stage1_north", north );
        toShapefile( path, "stage1_south", south );
        toShapefile( path, "stage1_east",  east );
        toShapefile( path, "stage1_west",  west );
    }
}