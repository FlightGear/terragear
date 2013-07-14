#include <list>
#include <ctime>

#include <stdio.h>

#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/SGGeometry.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/misc/texcoord.hxx>

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_surface.hxx>
#include <terragear/tg_chopper.hxx>
#include <terragear/tg_rectangle.hxx>
#include <terragear/tg_unique_geod.hxx>
#include <terragear/tg_unique_vec3f.hxx>
#include <terragear/tg_unique_vec2f.hxx>
#include <terragear/tg_shapefile.hxx>

#include "airport.hxx"
#include "beznode.hxx"
#include "debug.hxx"
#include "elevations.hxx"
#include "global.hxx"
#include "helipad.hxx"
#include "runway.hxx"
#include "output.hxx"


#define DEBUG           (0)
#define FIND_SLIVERS    (1)

#define JUST_BASE       (0)

Airport::Airport( int c, char* def)
{
    int   numParams;
    char* tok;
    int   ct = 0;

    code = c;

    // we need to tokenize airports, since we can't scanf two strings next to each other...
    numParams = 0;
    bool done = false;

    while (!done)
    {
        // trim leading whitespace
        while(isspace(*def)) def++;

        tok = strtok(def, " \t\r\n");

        if (tok)
        {
            def += strlen(tok)+1;

            switch( numParams )
            {
                case 0:
                    altitude = atoi(tok);
                    break;

                case 1:
                    ct = atoi(tok);
                    break;

                case 2:
                    // deprecated - ignore
                    break;

                case 3:
                    icao = tok;
                    description = def;
                    done = true;
                    break;
            }
        }
        numParams++;
    }

    altitude *= SG_FEET_TO_METER;

    TG_LOG( SG_GENERAL, SG_DEBUG, "Read airport with icao " << icao << ", control tower " << ct << ", and description " << description );
}

Airport::~Airport()
{
    for (unsigned int i=0; i<features.size(); i++)
    {
        delete features[i];
    }

    for (unsigned int i=0; i<helipads.size(); i++)
    {
        delete helipads[i];
    }

    for (unsigned int i=0; i<runways.size(); i++)
    {
        delete runways[i];
    }

    for (unsigned int i=0; i<waterrunways.size(); i++)
    {
        delete waterrunways[i];
    }

    for (unsigned int i=0; i<pavements.size(); i++)
    {
        delete pavements[i];
    }

    for (unsigned int i=0; i<taxiways.size(); i++)
    {
        delete taxiways[i];
    }

    for (unsigned int i=0; i<lightobjects.size(); i++)
    {
        delete lightobjects[i];
    }

    for (unsigned int i=0; i<windsocks.size(); i++)
    {
        delete windsocks[i];
    }

    for (unsigned int i=0; i<beacons.size(); i++)
    {
        delete beacons[i];
    }

    for (unsigned int i=0; i<signs.size(); i++)
    {
        delete signs[i];
    }

    for (unsigned int i=0; i<boundary.size(); i++)
    {
        delete boundary[i];
    }
}

bool Airport::isDebugRunway( int rwy )
{
    bool dbg = false;

    debug_map_const_iterator it = debug_runways.find(icao);
    if ( it != debug_runways.end() ) {
        for ( unsigned int i=0; i<it->second.size() && !dbg; i++ ) {
            if( it->second[i] == std::numeric_limits<int>::max() ) {
                dbg = true;
            } else if ( it->second[i] == rwy+1 ) {
                dbg = true;
            }
        }
    }

    return dbg;
}

bool Airport::isDebugPavement( int pvmt )
{
    bool dbg = false;

    debug_map_const_iterator it = debug_pavements.find(icao);
    if ( it != debug_pavements.end() ) {
        for ( unsigned int i=0; i<it->second.size() && !dbg; i++ ) {
            if( it->second[i] == std::numeric_limits<int>::max() ) {
                dbg = true;
            } else if ( it->second[i] == pvmt+1 ) {
                dbg = true;
            }
        }
    }

    return dbg;
}

bool Airport::isDebugTaxiway( int taxi )
{
    bool dbg = false;

    debug_map_const_iterator it = debug_taxiways.find(icao);
    if ( it != debug_taxiways.end() ) {
        for ( unsigned int i=0; i<it->second.size() && !dbg; i++ ) {
            if( it->second[i] == std::numeric_limits<int>::max() ) {
                dbg = true;
            } else if ( it->second[i] == taxi+1 ) {
                dbg = true;
            }
        }
    }

    return dbg;
}

bool Airport::isDebugFeature( int feat )
{
    bool dbg = false;

    debug_map_const_iterator it = debug_features.find(icao);
    if ( it != debug_features.end() ) {
        for ( unsigned int i=0; i<it->second.size() && !dbg; i++ ) {
            if( it->second[i] == std::numeric_limits<int>::max() ) {
                dbg = true;
            } else if ( it->second[i] == feat+1 ) {
                dbg = true;
            }
        }
    }

    return dbg;
}


// TODO : Add somewhere
// Determine node elevations of a point_list based on the provided
// TGAptSurface.  Offset is added to the final elevation
#if 0
static std::vector<SGGeod> calc_elevations( const tgSurface& surf, const std::vector<SGGeod>& geod_nodes, double offset )
{
    std::vector<SGGeod> result = geod_nodes;
    for ( unsigned int i = 0; i < result.size(); ++i ) {
        double elev = surf.query( result[i] );
        result[i].setElevationM( elev + offset );
    }

    return result;
}
#endif

#if 0
static tgContour calc_elevations( const tgSurface& surf, const tgContour& geod_nodes, double offset )
{
    tgContour result = geod_nodes;
    for ( unsigned int i = 0; i < result.GetSize(); ++i ) {
        SGGeod node = result.GetNode(i);
        double elev = surf.query( node );
        node.setElevationM( elev + offset );
        result.SetNode( i, node );
    }

    return result;
}
#endif

#if 0
static double calc_elevation( const tgSurface& surf, const SGGeod& node, double offset )
{
    double elev = surf.query( node );
    elev += offset;

    return elev;
}
#endif

// Determine node elevations of each node of a TGPolygon based on the
// provided TGAptSurface.  Offset is added to the final elevation
#if 0
static tgPolygon calc_elevations( const tgSurface& surf, const tgPolygon& poly, double offset )
{
    tgPolygon result;

    for ( unsigned int i = 0; i < poly.Contours(); ++i ) {
        tgContour contour = poly.GetContour( i );
        tgContour elevated = calc_elevations( surf, contour, offset );

        result.AddContour( elevated );
    }

    return result;
}
#endif

void Airport::BuildFeatures( void )
{
    tgpolygon_list polys;
    
    TG_LOG(SG_GENERAL, SG_INFO, "Build Lines " );

    // need a new polys_buit / Polys clipped for linear features (including priority defs )
#if 0
        TG_LOG(SG_GENERAL, SG_INFO, "Build " << features.size() << " Linear Feature Polys");
        for ( unsigned int i=0; i<features.size(); i++ )
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "Build Feature Poly " << i + 1 << " of " << features.size() << " : " << features[i]->GetDescription() );

            features[i]->BuildBtg( line_polys, rwy_lights, lf_accum, make_shapefiles );
        }

//        lf_accum.ToShapefiles( "./lf_accum", "test", false );

        log_time = time(0);
        TG_LOG( SG_GENERAL, SG_ALERT, "Finished building Linear Features for " << icao << " at " << DebugTimeToString(log_time) );
    }
#endif
}

void Airport::BuildBase( void )
{
    tgpolygon_list polys;
    bool userBoundary = false;

    /* initialize tgAreas for the number of layers we have */
    polys_built.init( AIRPORT_NUM_AREAS );
    polys_clipped.init( AIRPORT_NUM_AREAS );

    if (boundary.size()) {
        TG_LOG(SG_GENERAL, SG_INFO, "Build Base with user defined boundary" );
        userBoundary = true;
    } else {
        TG_LOG(SG_GENERAL, SG_INFO, "Build Base with approximated boundary" );
    }

#if !JUST_BASE
    // Build runways
    for ( unsigned int i=0; i<runways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Runway " << i + 1 << " of " << runways.size());
        runways[i]->GetMainPolys( polys_built.get_polys(AIRPORT_AREA_RUNWAY) );
        runways[i]->GetShoulderPolys( polys_built.get_polys(AIRPORT_AREA_RUNWAY_SHOULDER) );
        if (!userBoundary) {
            runways[i]->GetInnerBasePolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
            runways[i]->GetOuterBasePolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        }
    }
#endif

#if !JUST_BASE
    // Build helipads (use runway poly- and texture list for this)
    for ( unsigned int i=0; i<helipads.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Helipad " << i + 1 << " of " << helipads.size());
        //helipads[i]->GetMainPolys( polys_built.get_polys(AIRPORT_AREA_HELIPAD) );
        //helipads[i]->GetShoulderPolys( polys_built.get_polys(AIRPORT_AREA_HELIPAD_SHOULDER) );
        //if (!userBoundary) {
        //    helipads[i]->GetInnerBasePolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
        //    helipads[i]->GetOuterBasePolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        //}
    }
#endif

#if !JUST_BASE
    for ( unsigned int i=0; i<pavements.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Pavement " << i + 1 << " of " << pavements.size() << " : " << pavements[i]->GetDescription());
        pavements[i]->GetPolys( polys_built.get_polys(AIRPORT_AREA_PAVEMENT) );
        if (!userBoundary) {
            pavements[i]->GetInnerBasePolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
            pavements[i]->GetOuterBasePolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        }
    }
#endif

#if !JUST_BASE
    // Build the legacy taxiways
    for ( unsigned int i=0; i<taxiways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Taxiway " << i + 1 << " of " << taxiways.size());
        //taxiways[i]->GetPolys( polys_built.get_polys(AIRPORT_AREA_TAXIWAY) );
        //if (!userBoundary) {
        //    taxiways[i]->GetInnerBasePolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
        //    taxiways[i]->GetOuterBasePolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        //}
    }
#endif

    TG_LOG(SG_GENERAL, SG_INFO, "Build UserBoundary " );

    if (userBoundary)
    {
        TG_LOG(SG_GENERAL, SG_INFO, "Build " << boundary.size() << " user boundaries ");

        for ( unsigned int i=0; i<boundary.size(); i++ ) {
            TG_LOG(SG_GENERAL, SG_DEBUG, "Build Userdefined boundary " << i + 1 << " of " << boundary.size());
            boundary[i]->GetInnerBoundaryPolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
            boundary[i]->GetOuterBoundaryPolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        }
    }


    // DEBUG
#if DEBUG
    TG_LOG(SG_GENERAL, SG_INFO, "Dump " << polys_built.area_size(AIRPORT_AREA_RUNWAY) << " runway polys ");
    for (unsigned int j=0; j< polys_built.area_size(AIRPORT_AREA_RUNWAY); j++)
    {
        char layer[32];
        tgPolygon poly;
        sprintf(layer, "rwy_%d_poly", j );

        poly = polys_built.get_poly(AIRPORT_AREA_RUNWAY, j);
        tgShapefile::FromPolygon( poly, debug_path, layer, poly.GetMaterial().c_str() );
    }

    TG_LOG(SG_GENERAL, SG_INFO, "Dump " << polys_built.area_size(AIRPORT_AREA_RUNWAY_SHOULDER) << " runway shoulder polys ");
    for (unsigned int j=0; j< polys_built.area_size(AIRPORT_AREA_RUNWAY_SHOULDER); j++)
    {
        char layer[32];
        tgPolygon poly;
        sprintf(layer, "rwy_shoulder_%d_poly", j );

        poly = polys_built.get_poly(AIRPORT_AREA_RUNWAY_SHOULDER, j);
        tgShapefile::FromPolygon( poly, debug_path, layer, poly.GetMaterial().c_str() );
    }

    TG_LOG(SG_GENERAL, SG_INFO, "Dump " << polys_built.area_size(AIRPORT_AREA_PAVEMENT) << " pavement polys ");
    for (unsigned int j=0; j< polys_built.area_size(AIRPORT_AREA_PAVEMENT); j++)
    {
        char layer[32];
        tgPolygon poly;
        sprintf(layer, "pvmt_%d_poly", j );

        poly = polys_built.get_poly(AIRPORT_AREA_PAVEMENT, j);
        tgShapefile::FromPolygon( poly, debug_path, layer, poly.GetMaterial().c_str() );
    }

    TG_LOG(SG_GENERAL, SG_INFO, "Dump " << polys_built.area_size(AIRPORT_AREA_INNER_BASE) << " inner base polys ");
    for (unsigned int j=0; j< polys_built.area_size(AIRPORT_AREA_INNER_BASE); j++)
    {
        char layer[32];
        tgPolygon poly;
        sprintf(layer, "base_%d_poly", j );

        poly = polys_built.get_poly(AIRPORT_AREA_INNER_BASE, j);
        tgShapefile::FromPolygon( poly, debug_path, layer, poly.GetMaterial().c_str() );
    }

    TG_LOG(SG_GENERAL, SG_INFO, "Dump " << polys_built.area_size(AIRPORT_AREA_OUTER_BASE) << " outer base polys ");
    for (unsigned int j=0; j< polys_built.area_size(AIRPORT_AREA_OUTER_BASE); j++)
    {
        char layer[32];
        tgPolygon poly;
        sprintf(layer, "clearing_%d_poly", j );

        poly = polys_built.get_poly(AIRPORT_AREA_OUTER_BASE, j);
        tgShapefile::FromPolygon( poly, debug_path, layer, poly.GetMaterial().c_str() );
    }
#endif


    
    // build runway lights
#if 0
    for ( unsigned int i=0; i<runways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Runway light " << i + 1 << " of " << runways.size());
        runways[i]->GetRunwayLights( rwy_lights );
    }
#endif

    TG_LOG(SG_GENERAL, SG_INFO, "Build lightobjects " << lightobjects.size() );

    for ( unsigned int i=0; i<lightobjects.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build runway light " << i + 1 << " of " << lightobjects.size());
        lightobjects[i]->BuildBtg( rwy_lights );
    }
    
    TG_LOG(SG_GENERAL, SG_INFO, "done " );    
}

void Airport::ClipBase()
{
    tgAccumulator accum;
    tgPolygon clipped;

#if FIND_SLIVERS
    tgcontour_list slivers;
#endif

    // first, collect all the base nodes
    TG_LOG(SG_GENERAL, SG_INFO, "Clipping Base polys" );
    for ( unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_built.area_size(area); p++ ) {
            tgPolygon& current = polys_built.get_poly(area, p);

            clipped = accum.Diff( current );

            // only add to output list if the clip left us with a polygon
            if ( clipped.Contours() > 0 ) {

#if FIND_SLIVERS
                // move slivers from clipped polygon to slivers polygon
                tgPolygon::RemoveSlivers( clipped, slivers );
#endif

                // add the sliverless result polygon to the clipped polys list
                if ( clipped.Contours() > 0  ) {
                    // copy all of the superpolys and texparams
                    clipped.SetId( polys_built.get_poly( area, p ).GetId() );
                    polys_clipped.add_poly( area, clipped );
                }
            }

            accum.Add( current );
        }
    }

#if FIND_SLIVERS
    // Now, merge any slivers with clipped polys
    // merge_slivers(polys_clipped, slivers);
    for ( unsigned int i = 0; i <= AIRPORT_MAX_BASE; i++ ) {
        tgPolygon::MergeSlivers( polys_clipped.get_polys(i), slivers );
    }
    slivers.clear();
#endif

    // now break into max segment size
    for (unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++) {
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly( area, p );

            poly = tgPolygon::SplitLongEdges(poly, 100);

            polys_clipped.set_poly( area, p, poly );
        }
    }

    // Now, Make sure we have all the base nodes added as smoothed elevation nodes
    for (unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++) {
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly( area, p );

            for (unsigned int con=0; con < poly.Contours(); con++) {
                for (unsigned int n = 0; n < poly.ContourSize( con ); n++) {
                    // ensure
                    SGGeod const& node = poly.GetNode( con, n );
                    nodes.unique_add( node, TG_NODE_SMOOTHED );
                }
            }
        }
    }
}

void Airport::CleanBase()
{
    int before, after;
    std::vector<SGGeod> points;
    tgRectangle bb;

    // traverse each poly, and add intermediate nodes
    // for ( unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++ ) {
    for ( unsigned int area = 0; area <= AIRPORT_AREA_OUTER_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon current = polys_clipped.get_poly(area, p);
            bb = current.GetBoundingBox();
            nodes.get_geod_inside( bb.getMin(), bb.getMax(), points );

            before  = current.TotalNodes();
            current = tgPolygon::AddColinearNodes( current, points );
            current = tgPolygon::Snap(current, gSnap);
            current = tgPolygon::RemoveDups( current );
            current = tgPolygon::RemoveBadContours( current );
            after   = current.TotalNodes();

            if (before != after) {
               SG_LOG( SG_CLIPPER, SG_ALERT, "Fixed T-Junctions in " << p+1 << " of " << (int)polys_clipped.area_size(area) << " nodes increased from " << before << " to " << after );
            }

            /* Save it back */
            polys_clipped.set_poly( area, p, current );
        }
    }

    // simplify the base polys
    for ( unsigned int area = AIRPORT_AREA_INNER_BASE; area <= AIRPORT_AREA_OUTER_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& current = polys_clipped.get_poly(area, p);
            current = tgPolygon::Simplify( current );
            polys_clipped.set_poly( area, p, current );
        }
    }
}

void Airport::TesselateBase()
{
    //TG_LOG(SG_GENERAL, SG_INFO, "Tesselating Base polys" );

    // tesselate the polygons and prepair them for final output
    for (unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++) {
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            //TG_LOG(SG_GENERAL, SG_INFO, "Tesselating Base poly " << area << ", " << p );
            tgPolygon& poly = polys_clipped.get_poly(area, p );

#if DEBUG
            char layer[32];
            sprintf(layer, "tess_%d_%d", area, p );
            tgShapefile::FromPolygon( poly, debug_path, layer, poly.GetMaterial().c_str() );
#endif

            poly.Tesselate();
        }
    }

    //TG_LOG(SG_GENERAL, SG_INFO, "Done tesselating Base polys" );

    for (unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++) {
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly(area, p );

            // ensure all added nodes are accounted for
            for (unsigned int k=0; k < poly.Triangles(); k++) {
                for (int l = 0; l < 3; l++) {
                    // ensure we have all nodes...
                    nodes.unique_add( poly.GetTriNode( k, l ), TG_NODE_SMOOTHED );
                }
            }
        }
    }
}

void Airport::TexturePolys( void )
{
    for ( unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly(area, p);
            poly.Texture( );
        }
    }
}

void Airport::CalcElevations( const std::string& root, const string_list& elev_src )
{
    // first, generate the bounding rect, and average elevation
    tgRectangle bounds;

    std::vector<SGGeod> geods;
    nodes.get_geod_nodes(geods);

    if ( nodes.size() ) {
        bounds.setMin( geods[0] );
        bounds.setMax( geods[0] );
        for ( unsigned int i=1; i< geods.size(); i++ ) {
            bounds.expandBy( geods[i] );
        }
    } else {
        SG_LOG( SG_GENERAL, SG_INFO, "ERROR NO NODES" );
        return;
    }

    double average = tgAverageElevation( root, elev_src, geods );

    // then generate the surface
    tgSurface surf = tgSurface(  root, elev_src, bounds, average, slope_max, slope_eps );    
    nodes.CalcElevations( TG_NODE_SMOOTHED, surf );
}

void Airport::LookupIndexes( void )
{
    // for each node, traverse all the triangles - and create face lists
    for ( unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly( area, p );

            for (unsigned int tri=0; tri < poly.Triangles(); tri++) {
                for (unsigned int vertex = 0; vertex < 3; vertex++) {
                    int idx = nodes.find( poly.GetTriNode( tri, vertex ) );
                    if (idx >= 0) {
                        poly.SetTriIdx( tri, vertex, idx );
                    } else {
                        SG_LOG(SG_GENERAL, SG_ALERT, "didn't find vertex! " << poly.GetTriNode( tri, vertex ) );
                        exit(0);
                    }
                }
            }
        }
    }
}

void Airport::WriteOutput( const std::string& root, const SGBucket& b )
{
    if ( nodes.size() ) {
        UniqueSGVec3fSet normals;
        UniqueSGVec2fSet texcoords;

        std::string objpath = root + "/AirportObj";
        std::string name = icao + ".btg";

        SGVec3f vnt = SGVec3f::fromGeod( nodes.get_node(0).GetPosition() );
        vnt = normalize(vnt);

        group_list pts_v; pts_v.clear();
        group_list pts_n; pts_n.clear();
        string_list pt_materials; pt_materials.clear();

        group_list tris_v; tris_v.clear();
        group_list tris_n; tris_n.clear();
        group_list tris_tc; tris_tc.clear();
        string_list tri_materials; tri_materials.clear();

        group_list strips_v; strips_v.clear();
        group_list strips_n; strips_n.clear();
        group_list strips_tc; strips_tc.clear();
        string_list strip_materials; strip_materials.clear();

        int index;
        int_list pt_v, tri_v, strip_v;
        int_list pt_n, tri_n, strip_n;
        int_list tri_tc, strip_tc;

        for (unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++) {
            for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
                tgPolygon   poly      = polys_clipped.get_poly(area, p);
                std::string material  = poly.GetMaterial();

                for (unsigned int k = 0; k < poly.Triangles(); ++k) {
                    tri_v.clear();
                    tri_n.clear();
                    tri_tc.clear();
                    for (int l = 0; l < 3; ++l) {

                        // SG_LOG( SG_GENERAL, SG_INFO, "tri index is " << poly.GetTriIdx( k, l ) );

                        index = poly.GetTriIdx( k, l );
                        tri_v.push_back( index );

                        // use 'the' normal
                        index = normals.add( vnt );
                        tri_n.push_back( index );

                        index = texcoords.add( poly.GetTriTexCoord( k, l ) );
                        tri_tc.push_back( index );
                    }
                    tris_v.push_back( tri_v );
                    tris_n.push_back( tri_n );
                    tris_tc.push_back( tri_tc );

                    tri_materials.push_back( material );
                }
            }
        }

        std::vector< SGVec3d > wgs84_nodes;
        nodes.get_wgs84_nodes( wgs84_nodes );
        SGVec3d gbs_center = SGVec3d::fromGeod( b.get_center() );
        double dist_squared, radius_squared = 0;
        for (int i = 0; i < (int)wgs84_nodes.size(); ++i)
        {
            dist_squared = distSqr(gbs_center, wgs84_nodes[i]);
            if ( dist_squared > radius_squared ) {
                radius_squared = dist_squared;
            }
        }
        double gbs_radius = sqrt(radius_squared);

        SG_LOG(SG_GENERAL, SG_DEBUG, "gbs center = " << gbs_center);
        SG_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
        SG_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center << " radius = " << gbs_radius );

        // null structures
        group_list fans_v; fans_v.clear();
        group_list fans_n; fans_n.clear();
        group_list fans_tc; fans_tc.clear();
        string_list fan_materials; fan_materials.clear();

        std::string base = objpath;
        std::string binname = b.gen_index_str();
        binname += ".btg";
        std::string txtname = b.gen_index_str();
        txtname += ".txt";

        SGBinObject obj;

        obj.set_gbs_center( gbs_center );
        obj.set_gbs_radius( gbs_radius );
        obj.set_wgs84_nodes( wgs84_nodes );
        obj.set_normals( normals.get_list() );
        obj.set_texcoords( texcoords.get_list() );
        obj.set_pts_v( pts_v );
        obj.set_pts_n( pts_n );
        obj.set_pt_materials( pt_materials );
        obj.set_tris_v( tris_v );
        obj.set_tris_n( tris_n );
        obj.set_tris_tc( tris_tc );
        obj.set_tri_materials( tri_materials );
        obj.set_strips_v( strips_v );
        obj.set_strips_n( strips_n );
        obj.set_strips_tc( strips_tc );
        obj.set_strip_materials( strip_materials );
        obj.set_fans_v( fans_v );
        obj.set_fans_n( fans_n );
        obj.set_fans_tc( fans_tc );
        obj.set_fan_materials( fan_materials );

        bool result;
        result = obj.write_bin( objpath, name, b );
        if ( !result )
        {
            throw sg_exception("error writing file. :-(");
        }

        // write out airport object reference
        write_index( objpath, b, name );

    #if 0
        
    #if 0 // TODO : along with taxiway signs
        // write out tower references
        for ( i = 0; i < (int)tower_nodes.size(); ++i )
        {
            write_index_shared( objpath, b, tower_nodes[i],
                                "Models/Airport/tower.xml",
                                0.0 );
        }
    #endif

        SGGeod ref_geod;
        // calc elevations and write out windsock references
        TG_LOG(SG_GENERAL, SG_DEBUG, "Computing windsock node elevations");

        for ( unsigned int i = 0; i < windsocks.size(); ++i )
        {
            SGGeod ref_geod = windsocks[i]->GetLoc();
            ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );

            if ( windsocks[i]->IsLit() )
            {
                write_index_shared( objpath, b, ref_geod,
                                    "Models/Airport/windsock_lit.xml", 0.0 );
            }
            else
            {
                write_index_shared( objpath, b, ref_geod,
                                    "Models/Airport/windsock.xml", 0.0 );
            }
        }

        // write out beacon references
        for ( unsigned int i = 0; i < beacons.size(); ++i )
        {
            ref_geod = beacons[i]->GetLoc();
            ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );

            write_index_shared( objpath, b, ref_geod,
                                "Models/Airport/beacon.xml",
                                0.0 );
        }

        // write out taxiway signs references
        for ( unsigned int i = 0; i < signs.size(); ++i )
        {
            ref_geod = signs[i]->GetLoc();
            ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );
            write_object_sign( objpath, b, ref_geod,
                                signs[i]->GetDefinition(),
                                signs[i]->GetHeading(),
                                signs[i]->GetSize() );
        }

        // write out water buoys
        for ( unsigned int i = 0; i < waterrunways.size(); ++i )
        {
            tgContour buoys = waterrunways[i]->GetBuoys();

            for ( unsigned int j = 0; j < buoys.GetSize(); ++j )
            {
                ref_geod = buoys.GetNode(j);
                ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );
                write_index_shared( objpath, b, ref_geod,
                                    "Models/Airport/water_rw_buoy.xml",
                                    0.0 );
            }
        }
    #endif

        std::string holepath = root + "/AirportArea";
        tgChopper chopper( holepath );

        // union up all inner and outer base polys
        tgPolygon inner_base = tgPolygon::Union( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
        
        int before, after;
        std::vector<SGGeod> points;
        tgRectangle bb;
        
        bb = inner_base.GetBoundingBox();

        nodes.init_spacial_query();
        nodes.get_geod_inside( bb.getMin(), bb.getMax(), points );
                
        before  = inner_base.TotalNodes();
        inner_base = tgPolygon::AddColinearNodes( inner_base, points );
        inner_base = tgPolygon::Snap(inner_base, gSnap);
        inner_base = tgPolygon::RemoveDups( inner_base );
        inner_base = tgPolygon::RemoveBadContours( inner_base );
        after   = inner_base.TotalNodes();
        
        if (before != after) {
            SG_LOG( SG_CLIPPER, SG_ALERT, "Fixed T-Junctions in inner base: nodes increased from " << before << " to " << after );
        }

        tgPolygon outer_base = tgPolygon::Union( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );

        /* need to polulate the elevations in inner base */
        inner_base.SetElevations( nodes );

//      tgShapefile::FromPolygon( inner_base, "./hole_dbg", "hole poly", "hole" );
//      tgShapefile::FromPolygon( outer_base, "./hole_dbg", "base poly", "base" );
        
        inner_base.SetPreserve3D( true );
        chopper.Add( inner_base, "Hole" );

        outer_base.SetPreserve3D( false );
        outer_base.SetTexMethod( TG_TEX_BY_GEODE );
        chopper.Add( outer_base, "Airport" );

        chopper.Save(false);
    }
}

void Airport::BuildBtg(const std::string& root, const string_list& elev_src )
{
    TG_LOG(SG_GENERAL, SG_ALERT, "BUILDBTG");

    tgcontour_list slivers;
    tgcontour_list line_slivers;

    tgpolygon_list apt_base_polys;
    tgpolygon_list apt_clearing_polys;

    // runways
    tgpolygon_list rwy_polys;

    // pavements
    tgpolygon_list pvmt_polys;

    // linear features
    tgpolygon_list line_polys;

    // parse main airport information
    double apt_lon = 0.0, apt_lat = 0.0;

    // Find the average of all the runway and heliport long / lats
    int num_samples = 0;
    TG_LOG(SG_GENERAL, SG_ALERT, "num runways is " << runways.size() << " num heipads is " << helipads.size() );

    for (unsigned int i=0; i<runways.size(); i++)
    {
        apt_lon += runways[i]->GetMidpoint().getLongitudeDeg();
        apt_lat += runways[i]->GetMidpoint().getLatitudeDeg();
        num_samples++;
    }
    for (unsigned int i=0; i<helipads.size(); i++)
    {
        apt_lon += helipads[i]->GetLoc().getLongitudeDeg();
        apt_lat += helipads[i]->GetLoc().getLatitudeDeg();
        num_samples++;
    }
    if ( num_samples )
    {
        apt_lon = apt_lon / (double)num_samples;
        apt_lat = apt_lat / (double)num_samples;
    }
    else
    {
        TG_LOG(SG_GENERAL, SG_ALERT, "AIRPORT HAS NO RUNWAYS/HELIPADS");
        return;
    }

    TG_LOG(SG_GENERAL, SG_ALERT, " long, lat is " << apt_lon << ", " << apt_lat );

    SGBucket b( apt_lon, apt_lat );
    TG_LOG(SG_GENERAL, SG_DEBUG, b.gen_base_path() << "/" << b.gen_index_str());

#if !JUST_BASE
    // If we are cutting in the linear features, add them first
    if (pavements.size())
    {
        for ( unsigned int i=0; i<pavements.size(); i++ )
        {
            AddFeatures( pavements[i]->GetFeatures() );
        }
    }
#endif

    TG_LOG(SG_GENERAL, SG_INFO, "Parse Complete - Runways: " << runways.size() << " Pavements: " << pavements.size() << " Features: " << features.size() << " Taxiways: " << taxiways.size() );

    // Airport building Steps
    // 1: Build the base polygons
    BuildBase();

    TG_LOG(SG_GENERAL, SG_INFO, "ClipBase" );

    // 2: Clip the polys in priority order
    ClipBase();

    TG_LOG(SG_GENERAL, SG_INFO, "spacialquery" );

    // 3: Clean the polys
    nodes.init_spacial_query();

    TG_LOG(SG_GENERAL, SG_INFO, "CleanBase" );

    CleanBase();

    TG_LOG(SG_GENERAL, SG_INFO, "TesselateBase" );

    // 4: Teseelate Base polys
    TesselateBase();

    TG_LOG(SG_GENERAL, SG_INFO, "LookupIndexes" );

    LookupIndexes();

    TG_LOG(SG_GENERAL, SG_INFO, "TextureBase" );

    // 5: Texture Base polys
    TexturePolys();

    TG_LOG(SG_GENERAL, SG_INFO, "CalcElevations" );

    // 6: calculate height
    CalcElevations(root, elev_src);
    
    // 9: Build the linear feature polygons
    BuildFeatures();

    TG_LOG(SG_GENERAL, SG_INFO, "WriteOutput" );

    // save file
    WriteOutput( root, b );

#if 0
    if ( rwy_polys.size() )
    {
        TG_LOG(SG_GENERAL, SG_INFO, "Tesselating " << rwy_polys.size() << " Runway Polys " );
        for ( unsigned int i = 0; i < rwy_polys.size(); ++i )
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating runway poly = " << i + 1 << " of " << rwy_polys.size() );

            TG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << rwy_polys[i].Contours() << " total points before = " << rwy_polys[i].TotalNodes());
            rwy_polys[i].Tesselate();
            TG_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << rwy_polys[i].Triangles());
            rwy_polys[i].Texture();
        }
    }

    if ( pvmt_polys.size() )
    {
        // tesselate the polygons and prepair them for final output
        TG_LOG(SG_GENERAL, SG_INFO, "Tesselating " << pvmt_polys.size() << " Pavement Polys " );
        for ( unsigned int i = 0; i < pvmt_polys.size(); ++i )
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating pavement poly = " << i + 1 << " of " << pvmt_polys.size() );

            TG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << pvmt_polys[i].Contours() << " total points before = " << pvmt_polys[i].TotalNodes());
            pvmt_polys[i].Tesselate();
            TG_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << pvmt_polys[i].Triangles());
            pvmt_polys[i].Texture();
        }
    }
#endif
    
#if 0
    /* before tessellating the base, make sure there are no
       intersecting contours */
    // base_poly = tgPolygon::Simplify( base_poly );

    TG_LOG(SG_GENERAL, SG_INFO, "Tesselating base poly : " << base_poly.Contours() << " contours " );
    // base_poly.Tesselate();
    TG_LOG(SG_GENERAL, SG_INFO, "Tesselating base poly - done : Triangles = " << base_poly.Triangles());

    // a few airports fail here
    if ( base_poly.Triangles() == 0 )
    {
        TG_LOG(SG_GENERAL, SG_ALERT, "no base poly triangles");
        return;
    }

    triangulation_time = triangulation_end - triangulation_start;


    // now create the linear features
    // and linear features ( keep Linear feature nodes seperate)
    // Add the linear features
    if (features.size())
    {
        tgAccumulator lf_accum;

        TG_LOG(SG_GENERAL, SG_INFO, "Build " << features.size() << " Linear Feature Polys");
        for ( unsigned int i=0; i<features.size(); i++ )
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "Build Feature Poly " << i + 1 << " of " << features.size() << " : " << features[i]->GetDescription() );

            features[i]->BuildBtg( line_polys, rwy_lights, lf_accum, make_shapefiles );
        }

        log_time = time(0);
        TG_LOG( SG_GENERAL, SG_ALERT, "Finished building Linear Features for " << icao << " at " << DebugTimeToString(log_time) );
    }

    /* collect all pavement line segments */
    
    /* collect all the feature nodes */
    UniqueSGGeodSet tmp_feat_nodes;
    for ( unsigned int k = 0; k < line_polys.size(); ++k )
    {
        tgPolygon poly = line_polys[k];
        for ( unsigned int i = 0; i < poly.Contours(); ++i )
        {
            for ( unsigned int j = 0; j < poly.ContourSize( i ); ++j )
            {
                tmp_feat_nodes.add( poly.GetNode(i, j) );
            }
        }
    }

    /* then intersect each feature line segment with the underlying triangles */
    /* get all the line segments from the lf contours */
    tgsegment_list lf_segments;
    for ( unsigned int k = 0; k < line_polys.size(); ++k )
    {
        tgPolygon poly = line_polys[k];
        for ( unsigned int i = 0; i < poly.Contours(); ++i )
        {
            tgsegment_list seg = tgContour::ToSegments( poly.GetContour(i) );
            for ( unsigned int j = 0; j<seg.size(); j++ ) {
                lf_segments.push_back( seg[j] );
            }
        }
    }

    tgsegment_list pvmnt_segments;
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k )
    {
        tgPolygon poly = rwy_polys[k];
        for ( unsigned int i = 0; i < poly.Triangles(); ++i )
        {
            tgsegment_list seg = poly.GetTriangle(i).ToSegments();
            for ( unsigned int j = 0; j<seg.size(); j++ ) {
                pvmnt_segments.push_back( seg[j] );
            }
        }
    }
    for ( unsigned int k = 0; k < pvmt_polys.size(); ++k )
    {
        tgPolygon poly = pvmt_polys[k];
        for ( unsigned int i = 0; i < poly.Triangles(); ++i )
        {
            tgsegment_list seg = poly.GetTriangle(i).ToSegments();
            for ( unsigned int j = 0; j<seg.size(); j++ ) {
                pvmnt_segments.push_back( seg[j] );
            }
        }
    }

    // now, for each lf_segment - find the intersection points of the pavements, and add these to
    // tmp_feat_nodes
    for ( unsigned int k = 0; k < lf_segments.size(); ++k )
    {
        for ( unsigned int i = 0; i < pvmnt_segments.size(); ++i )
        {
            std::vector<SGGeod> ints;
            ints.clear();

            if ( FindIntersections( lf_segments[k], pvmnt_segments[i], ints ) )
            {
                for (unsigned int x = 0; x<ints.size(); x++)
                {
                    tmp_feat_nodes.add( ints[x] );
                }
            }
        }
    }

    // second pass : and lines
    for ( unsigned int k = 0; k < line_polys.size(); ++k )
    {
        tgPolygon poly = line_polys[k];
        poly = tgPolygon::AddColinearNodes( poly, tmp_feat_nodes );
        TG_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.TotalNodes());
        line_polys[k] = poly;
    }

    for ( unsigned int k = 0; k < line_polys.size(); ++k )
    {
        tgPolygon poly = line_polys[k];

        poly = tgPolygon::RemoveCycles( poly );
        poly = tgPolygon::RemoveDups( poly );
        poly = tgPolygon::RemoveBadContours( poly );

        line_polys[k] = poly;
    }

    // Teseelate the linear features
    if ( line_polys.size() )
    {
        // tesselate the polygons and prepair them for final output
        TG_LOG(SG_GENERAL, SG_INFO, "Tesselating " << line_polys.size() << " Linear Feature Polys " );
        for ( unsigned int i = 0; i < line_polys.size(); ++i )
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating line poly = " << i + 1 << " of " << line_polys.size() );

            TG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << line_polys[i].Contours() << " total points before = " << line_polys[i].TotalNodes());
            line_polys[i].Tesselate();
            TG_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << line_polys[i].Triangles());
            line_polys[i].Texture();
        }
    }


    //
    // We should now have the runway polygons all generated with their
    // corresponding triangles and texture coordinates, and the
    // surrounding base area.
    //
    // Now we need to calculate the height of each pavement node using the airport surface.
    // And drape lf nodes

    // generate surface based on airport bounds.
    tgRectangle bounds = apt_base.GetBoundingBox();

    // TODO elevation queries should be performed as member functions of surface
    // Need average of all the pavement, runway, and base nodes
    // already gathers for finding t-juntions
    double average = tgAverageElevation( root, elev_src, tmp_pvmt_nodes.get_list() );
    tgSurface apt_surf( root, elev_src, bounds, average, slope_max, slope_eps );
    TG_LOG(SG_GENERAL, SG_DEBUG, "Airport surface created");




    
    UniqueSGGeodSet  nodes;
    UniqueSGVec3fSet normals;
    UniqueSGVec2fSet texcoords;

    group_list pts_v; pts_v.clear();
    group_list pts_n; pts_n.clear();
    string_list pt_materials; pt_materials.clear();

    group_list tris_v; tris_v.clear();
    group_list tris_n; tris_n.clear();
    group_list tris_tc; tris_tc.clear();
    string_list tri_materials; tri_materials.clear();

    group_list strips_v; strips_v.clear();
    group_list strips_n; strips_n.clear();
    group_list strips_tc; strips_tc.clear();
    string_list strip_materials; strip_materials.clear();

    int index;
    int_list pt_v, tri_v, strip_v;
    int_list pt_n, tri_n, strip_n;
    int_list tri_tc, strip_tc;

    // calculate "the" normal for this airport
    SGVec3f vnt = SGVec3f::fromGeod( base_poly.GetNode(0, 0) );
    vnt = normalize(vnt);

    TG_LOG(SG_GENERAL, SG_INFO, "Adding runway nodes and normals");
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
        std::string material = rwy_polys[k].GetMaterial();
        TG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
        TG_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << rwy_polys[k].Triangles());
        for ( unsigned int i = 0; i < rwy_polys[k].Triangles(); ++i )
        {
            tri_v.clear();
            tri_n.clear();
            tri_tc.clear();
            // let's go out on a limb, and assume triangles have 3 points..
            for ( int j = 0; j < 3; ++j )
            {
                index = nodes.add( rwy_polys[k].GetTriNode( i, j ) );
                tri_v.push_back( index );

                // use 'the' normal
                index = normals.add( vnt );
                tri_n.push_back( index );

                index = texcoords.add( rwy_polys[k].GetTriTexCoord(i,j) );
                tri_tc.push_back( index );
            }
            tris_v.push_back( tri_v );
            tris_n.push_back( tri_n );
            tris_tc.push_back( tri_tc );
            tri_materials.push_back( material );
        }
    }

    TG_LOG(SG_GENERAL, SG_INFO, "Adding pavement nodes and normals");
    for ( unsigned int k = 0; k < pvmt_polys.size(); ++k )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
        std::string material = pvmt_polys[k].GetMaterial();
        TG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
        TG_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << pvmt_polys[k].Triangles());
        for ( unsigned int i = 0; i < pvmt_polys[k].Triangles(); ++i )
        {
            tri_v.clear();
            tri_n.clear();
            tri_tc.clear();
            for ( int j = 0; j < 3; ++j )
            {
                index = nodes.add( pvmt_polys[k].GetTriNode( i, j ) );
                tri_v.push_back( index );

                // use 'the' normal
                index = normals.add( vnt );
                tri_n.push_back( index );

                index = texcoords.add( pvmt_polys[k].GetTriTexCoord(i,j) );
                tri_tc.push_back( index );
            }
            tris_v.push_back( tri_v );
            tris_n.push_back( tri_n );
            tris_tc.push_back( tri_tc );
            tri_materials.push_back( material );
        }
    }

    TG_LOG(SG_GENERAL, SG_INFO, "Adding line nodes and normals");
    for ( unsigned int k = 0; k < line_polys.size(); ++k )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
        std::string material = line_polys[k].GetMaterial();
        TG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
        TG_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << line_polys[k].Triangles());
        for ( unsigned int i = 0; i < line_polys[k].Triangles(); ++i )
        {
            tri_v.clear();
            tri_n.clear();
            tri_tc.clear();
            for ( int j = 0; j < 3; ++j )
            {
                index = nodes.add( line_polys[k].GetTriNode( i, j ) );
                tri_v.push_back( index );

                // use 'the' normal
                index = normals.add( vnt );
                tri_n.push_back( index );

                index = texcoords.add( line_polys[k].GetTriTexCoord(i,j) );
                tri_tc.push_back( index );
            }
            tris_v.push_back( tri_v );
            tris_n.push_back( tri_n );
            tris_tc.push_back( tri_tc );
            tri_materials.push_back( material );
        }
    }

    // add base points
    std::vector< SGVec2f > base_txs;
    int_list base_tc;

    TG_LOG(SG_GENERAL, SG_INFO, "Adding base nodes and normals");
    for ( unsigned int i = 0; i < base_poly.Triangles(); ++i )
    {
        tri_v.clear();
        tri_n.clear();
        tri_tc.clear();
        for ( int j = 0; j < 3; ++j )
        {
            index = nodes.add( base_poly.GetTriNode( i, j ) );
            tri_v.push_back( index );

            index = normals.add( vnt );
            tri_n.push_back( index);
        }
        tris_v.push_back( tri_v );
        tris_n.push_back( tri_n );
        tri_materials.push_back( "Grass" );

        std::vector < SGGeod > geodNodes = nodes.get_list();

        base_txs.clear();
        base_txs = sgCalcTexCoords( b, geodNodes, tri_v );

        base_tc.clear();
        for ( unsigned int j = 0; j < base_txs.size(); ++j )
        {
            index = texcoords.add(  base_txs[j] );
            base_tc.push_back( index );
        }
        tris_tc.push_back( base_tc );
    }

    // on rare occasion, one or more of the divided base points can be
    // missed.  Make sure they are all in the node list so we can
    // build a proper skirt.
    for ( unsigned int i = 0; i < divided_base.Contours(); ++i )
    {
        for ( unsigned int j = 0; j < divided_base.ContourSize( i ); ++j )
        {
            index = nodes.add( divided_base.GetNode(i, j) );
            TG_LOG(SG_GENERAL, SG_DEBUG, "added base point " << divided_base.GetNode(i, j) << " at " << index );
        }
    }

    // Now that we have assembled all the airport geometry nodes into
    // a list, calculate an "average" airport elevation based on all
    // the actual airport node points.  This is more useful than
    // calculating an average over the entire airport surface because
    // it avoids biases introduced from the surrounding area if the
    // airport is located in a bowl or on a hill.
#if 0
    TG_LOG(SG_GENERAL, SG_DEBUG, " calc average elevation");
    {
        std::vector < SGGeod > dbg = nodes.get_list();

        // dump the node list
        TG_LOG(SG_GENERAL, SG_DEBUG, " node list size is " << dbg.size() );
        for (unsigned int w = 0; w<dbg.size(); w++)
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, " node " << w << " is " << dbg[w] );
        }
    }
#endif
    // Now build the fitted airport surface ...

    // calculation min/max coordinates of airport area
    TG_LOG(SG_GENERAL, SG_DEBUG, " calculation min/max coordinates of airport area");

//    SGGeod min_deg = SGGeod::fromDeg(9999.0, 9999.0);
//    SGGeod max_deg = SGGeod::fromDeg(-9999.0, -9999.0);
//    for ( unsigned int j = 0; j < nodes.get_list().size(); ++j )
//    {
//        SGGeod p = nodes.get_list()[j];
//        if ( p.getLongitudeDeg() < min_deg.getLongitudeDeg() )
//        {
//            TG_LOG(SG_GENERAL, SG_DEBUG, "new min lon from node " << j << " is " << p.getLongitudeDeg() );
//            min_deg.setLongitudeDeg( p.getLongitudeDeg() );
//        }
//        if ( p.getLongitudeDeg() > max_deg.getLongitudeDeg() )
//        {
//            TG_LOG(SG_GENERAL, SG_DEBUG, "new max lon from node " << j << " is " << p.getLongitudeDeg() );
//            max_deg.setLongitudeDeg( p.getLongitudeDeg() );
//        }
//        if ( p.getLatitudeDeg() < min_deg.getLatitudeDeg() )
//        {
//            TG_LOG(SG_GENERAL, SG_DEBUG, "new min lat from node " << j << " is " << p.getLatitudeDeg() );
//            min_deg.setLatitudeDeg( p.getLatitudeDeg() );
//        }
//        if ( p.getLatitudeDeg() > max_deg.getLatitudeDeg() )
//        {
//            TG_LOG(SG_GENERAL, SG_DEBUG, "new max lat from node " << j << " is " << p.getLatitudeDeg() );
//            max_deg.setLatitudeDeg( p.getLatitudeDeg() );
//        }
//    }
//    TG_LOG(SG_GENERAL, SG_DEBUG, "Before extending for lights: min = " << min_deg << " max = " << max_deg );

    // extend the min/max coordinates of airport area to cover all
    // lights as well

#if 0
    TG_LOG(SG_GENERAL, SG_DEBUG, " extend the min/max coordinates of airport area to cover all lights as well : num rwy lights is " << rwy_lights.size() );
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, " extend the min/max coordinates of airport area to cover all lights as well : rwy light " << i << "has " << rwy_lights[i].ContourSize() << " lights " );

        for ( unsigned int j = 0; j < rwy_lights[i].ContourSize(); ++j )
        {
            SGGeod p = rwy_lights[i].GetNode(j);
            if ( p.getLongitudeDeg() < min_deg.getLongitudeDeg() )
            {
                min_deg.setLongitudeDeg( p.getLongitudeDeg() );
            }
            if ( p.getLongitudeDeg() > max_deg.getLongitudeDeg() )
            {
                max_deg.setLongitudeDeg( p.getLongitudeDeg() );
            }
            if ( p.getLatitudeDeg() < min_deg.getLatitudeDeg() )
            {
                min_deg.setLatitudeDeg( p.getLatitudeDeg() );
            }
            if ( p.getLatitudeDeg() > max_deg.getLatitudeDeg() )
            {
                max_deg.setLatitudeDeg( p.getLatitudeDeg() );
            }
        }
    }

    // Extend the area a bit so we don't have wierd things on the edges
    double dlon = max_deg.getLongitudeDeg() - min_deg.getLongitudeDeg();
    double dlat = max_deg.getLatitudeDeg() - min_deg.getLatitudeDeg();
    min_deg.setLongitudeDeg( min_deg.getLongitudeDeg() - 0.01 * dlon );
    max_deg.setLongitudeDeg( max_deg.getLongitudeDeg() + 0.01 * dlon );
    min_deg.setLatitudeDeg( min_deg.getLatitudeDeg() - 0.01 * dlat );
    max_deg.setLatitudeDeg( max_deg.getLatitudeDeg() + 0.01 * dlat );
    TG_LOG(SG_GENERAL, SG_DEBUG, "min = " << min_deg << " max = " << max_deg );

    TG_LOG(SG_GENERAL, SG_DEBUG, "Create Apt surface:" );
    TG_LOG(SG_GENERAL, SG_DEBUG, " root: " << root );
    //TG_LOG(SG_GENERAL, SG_DEBUG, " elev: " << elev_src );
    TG_LOG(SG_GENERAL, SG_DEBUG, " min: " << min_deg );
    TG_LOG(SG_GENERAL, SG_DEBUG, " max: " << max_deg );
    TG_LOG(SG_GENERAL, SG_DEBUG, " average: " << average );


    // calculate node elevations
    TG_LOG(SG_GENERAL, SG_DEBUG, "Computing airport node elevations");
#endif

    // add light points
    // pass one, calculate raw elevations from Array
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i ) {
        for ( unsigned int j = 0; j < rwy_lights[i].ContourSize(); j++ ) {
            double light_elevation = calc_elevation( apt_surf, rwy_lights[i].GetNode(j), 0.0 );
            rwy_lights[i].SetElevation(j, light_elevation);
        }
    }
    TG_LOG(SG_GENERAL, SG_INFO, "Done with lighting calc_elevations() num light polys is " << rwy_lights.size() );

    // pass two, for each light group check if we need to lift (based
    // on flag) and do so, then output next structures.
    // for ( unsigned int i = 0; i < rwy_lights.size(); ++i )
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i )
    {
        pt_v.clear();
        pt_n.clear();
        for ( unsigned int j = 0; j < rwy_lights[i].ContourSize(); ++j )
        {
            index = nodes.add( rwy_lights[i].GetPosition(j) );
            pt_v.push_back( index );

            index = normals.add( rwy_lights[i].GetNormal(j) );
            pt_n.push_back( index );
        }
        pts_v.push_back( pt_v );
        pts_n.push_back( pt_n );
        pt_materials.push_back( rwy_lights[i].GetType() );
    }

    std::vector<SGGeod> geod_nodes = calc_elevations( apt_surf, nodes.get_list(), 0.0 );
    divided_base = calc_elevations( apt_surf, divided_base, 0.0 );

    // calculate wgs84 mapping of nodes
    std::vector<SGVec3d> wgs84_nodes;
    for ( unsigned int i = 0; i < geod_nodes.size(); ++i )
    {
        wgs84_nodes.push_back( SGVec3d::fromGeod(geod_nodes[i]) );
    }

    SGSphered d;
    for ( unsigned int i = 0; i < wgs84_nodes.size(); ++i )
    {
        d.expandBy(wgs84_nodes[ i ]);
    }

    SGVec3d gbs_center = d.getCenter();
    double gbs_radius = d.getRadius();
    TG_LOG(SG_GENERAL, SG_DEBUG, "gbs center = " << gbs_center);
    TG_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
    TG_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center << " radius = " << gbs_radius );

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_n; fans_n.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    std::string objpath = root + "/AirportObj";
    std::string name = icao + ".btg";

    SGBinObject obj;
    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals.get_list() );
    obj.set_texcoords( texcoords.get_list() );
    obj.set_pts_v( pts_v );
    obj.set_pts_n( pts_n );
    obj.set_pt_materials( pt_materials );
    obj.set_tris_v( tris_v );
    obj.set_tris_n( tris_n );
    obj.set_tris_tc( tris_tc );
    obj.set_tri_materials( tri_materials );
    obj.set_strips_v( strips_v );
    obj.set_strips_n( strips_n );
    obj.set_strips_tc( strips_tc );
    obj.set_strip_materials( strip_materials );
    obj.set_fans_v( fans_v );
    obj.set_fans_n( fans_n );
    obj.set_fans_tc( fans_tc );
    obj.set_fan_materials( fan_materials );

    bool result;
    result = obj.write_bin( objpath, name, b );
    if ( !result )
    {
        throw sg_exception("error writing file. :-(");
    }

    // write out airport object reference
    write_index( objpath, b, name );

#if 0 // TODO : along with taxiway signs
    // write out tower references
    for ( i = 0; i < (int)tower_nodes.size(); ++i )
    {
        write_index_shared( objpath, b, tower_nodes[i],
                            "Models/Airport/tower.xml",
                            0.0 );
    }
#endif

    SGGeod ref_geod;
    // calc elevations and write out windsock references
    TG_LOG(SG_GENERAL, SG_DEBUG, "Computing windsock node elevations");

    for ( unsigned int i = 0; i < windsocks.size(); ++i )
    {
        SGGeod ref_geod = windsocks[i]->GetLoc();
        ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );

        if ( windsocks[i]->IsLit() )
        {
            write_index_shared( objpath, b, ref_geod,
                                "Models/Airport/windsock_lit.xml", 0.0 );
        }
        else
        {
            write_index_shared( objpath, b, ref_geod,
                                "Models/Airport/windsock.xml", 0.0 );
        }
    }

    // write out beacon references
    for ( unsigned int i = 0; i < beacons.size(); ++i )
    {
        ref_geod = beacons[i]->GetLoc();
        ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );

        write_index_shared( objpath, b, ref_geod,
                            "Models/Airport/beacon.xml",
                            0.0 );
    }

    // write out taxiway signs references
    for ( unsigned int i = 0; i < signs.size(); ++i )
    {
        ref_geod = signs[i]->GetLoc();
        ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );
        write_object_sign( objpath, b, ref_geod,
                            signs[i]->GetDefinition(),
                            signs[i]->GetHeading(),
                            signs[i]->GetSize() );
    }

    // write out water buoys
    for ( unsigned int i = 0; i < waterrunways.size(); ++i )
    {
        tgContour buoys = waterrunways[i]->GetBuoys();

        for ( unsigned int j = 0; j < buoys.GetSize(); ++j )
        {
            ref_geod = buoys.GetNode(j);
            ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );
            write_index_shared( objpath, b, ref_geod,
                                "Models/Airport/water_rw_buoy.xml",
                                0.0 );
        }
    }

    std::string holepath = root + "/AirportArea";
    tgChopper chopper( holepath );

    divided_base.SetPreserve3D( true );
    apt_clearing.SetPreserve3D( false );
    apt_clearing.SetTexMethod( TG_TEX_BY_GEODE );

    chopper.Add( divided_base, "Hole" );
    chopper.Add( apt_clearing, "Airport" );
    chopper.Save();

#endif
}
