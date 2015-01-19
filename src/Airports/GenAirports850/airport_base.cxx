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

    // Build runways
    for ( unsigned int i=0; i<runways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Runway " << i + 1 << " of " << runways.size());
        runways[i]->GetMainPolys( this, polys_built.get_polys(AIRPORT_AREA_RUNWAY) );
        runways[i]->GetShoulderPolys( polys_built.get_polys(AIRPORT_AREA_RUNWAY_SHOULDER) );
        if (!userBoundary) {
            runways[i]->GetInnerBasePolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
            runways[i]->GetOuterBasePolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        }
    }

    // Build helipads (use runway poly- and texture list for this)
    for ( unsigned int i=0; i<helipads.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Helipad " << i + 1 << " of " << helipads.size());
        helipads[i]->GetMainPolys( polys_built.get_polys(AIRPORT_AREA_HELIPAD) );
        helipads[i]->GetShoulderPolys( polys_built.get_polys(AIRPORT_AREA_HELIPAD_SHOULDER) );
        if (!userBoundary) {
            helipads[i]->GetInnerBasePolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
            helipads[i]->GetOuterBasePolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        }
    }

    for ( unsigned int i=0; i<pavements.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Pavement " << i + 1 << " of " << pavements.size() << " : " << pavements[i]->GetDescription());
        pavements[i]->GetPolys( polys_built.get_polys(AIRPORT_AREA_PAVEMENT) );
        if (!userBoundary) {
            pavements[i]->GetInnerBasePolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
            pavements[i]->GetOuterBasePolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        }
    }

    // Build the legacy taxiways
    for ( unsigned int i=0; i<taxiways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Taxiway " << i + 1 << " of " << taxiways.size());
        taxiways[i]->GetPolys( polys_built.get_polys(AIRPORT_AREA_TAXIWAY) );
        if (!userBoundary) {
            taxiways[i]->GetInnerBasePolys( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );
            taxiways[i]->GetOuterBasePolys( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );
        }
    }

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
    
    TG_LOG(SG_GENERAL, SG_INFO, "done " );    
}

void Airport::ClipBase()
{
    tgAccumulator accum;
    tgPolygon clipped;

    // first, collect all the base nodes
    TG_LOG(SG_GENERAL, SG_INFO, "Clipping Base polys" );
    for ( unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_built.area_size(area); p++ ) {
            tgPolygon& current = polys_built.get_poly(area, p);

            clipped = accum.Diff( current );

            // only add to output list if the clip left us with a polygon
            if ( clipped.Contours() > 0 ) {
                // copy all of the superpolys and texparams
                clipped.SetId( polys_built.get_poly( area, p ).GetId() );
                polys_clipped.add_poly( area, clipped );
            }

            accum.Add( current );
        }
    }
    
    // create the inner base poly as the union of all innerbase polys
    inner_base = tgPolygon::Union( polys_built.get_polys(AIRPORT_AREA_INNER_BASE) );    

    // now break into max segment size
    for (unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++) {
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly( area, p );

            poly = tgPolygon::SplitLongEdges(poly, 100);

            polys_clipped.set_poly( area, p, poly );
        }
    }
    inner_base = tgPolygon::SplitLongEdges(inner_base, 100);
    
    
    // Now, Make sure we have all the base nodes added as smoothed elevation nodes
    for (unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++) {
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly( area, p );

            for (unsigned int con=0; con < poly.Contours(); con++) {
                for (unsigned int n = 0; n < poly.ContourSize( con ); n++) {
                    // ensure
                    SGGeod const& node = poly.GetNode( con, n );
                    base_nodes.unique_add( node, TG_NODE_SMOOTHED );
                }
            }
        }
    }
    
    // and the inner base
    for (unsigned int con=0; con < inner_base.Contours(); con++) {
        for (unsigned int n = 0; n < inner_base.ContourSize( con ); n++) {
            // ensure
            SGGeod const& node = inner_base.GetNode( con, n );
            base_nodes.unique_add( node, TG_NODE_SMOOTHED );
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
            base_nodes.get_geod_inside( bb.getMin(), bb.getMax(), points );

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

    points.clear();
    bb = inner_base.GetBoundingBox();
    base_nodes.get_geod_inside( bb.getMin(), bb.getMax(), points );
    
    before  = inner_base.TotalNodes();
    inner_base = tgPolygon::AddColinearNodes( inner_base, points );
    inner_base = tgPolygon::Snap(inner_base, gSnap);
    inner_base = tgPolygon::RemoveDups( inner_base );
    inner_base = tgPolygon::RemoveBadContours( inner_base );
    after   = inner_base.TotalNodes();
    
    if (before != after) {
        SG_LOG( SG_CLIPPER, SG_ALERT, "Fixed T-Junctions in inner_base nodes increased from " << before << " to " << after );
    }
    
#if 0    
    // simplify the base polys
    for ( unsigned int area = AIRPORT_AREA_INNER_BASE; area <= AIRPORT_AREA_OUTER_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& current = polys_clipped.get_poly(area, p);
            current = tgPolygon::Simplify( current );
            polys_clipped.set_poly( area, p, current );
        }
    }
#endif

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
                    base_nodes.unique_add( poly.GetTriNode( k, l ), TG_NODE_SMOOTHED );
                }
            }
        }
    }
}

void Airport::TextureBase( void )
{
    for ( unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly(area, p);
            poly.Texture( );
        }
    }
}

void Airport::CalcBaseElevations( const std::string& root, const string_list& elev_src )
{
    // first, generate the bounding rect, and average elevation
    tgRectangle bounds;

    std::vector<SGGeod> geods;
    base_nodes.get_geod_nodes(geods);

    if ( base_nodes.size() ) {
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
    base_surf.Create(  root, elev_src, bounds, average, slope_max, slope_eps );    
    base_nodes.CalcElevations( TG_NODE_SMOOTHED, base_surf );
}

void Airport::LookupBaseIndexes( void )
{
    // for each node, traverse all the triangles - and create face lists
    for ( unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly( area, p );

            for (unsigned int tri=0; tri < poly.Triangles(); tri++) {
                for (unsigned int vertex = 0; vertex < 3; vertex++) {
                    int idx = base_nodes.find( poly.GetTriNode( tri, vertex ) );
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

#if 0
#define AIRPORT_AREA_RUNWAY             (0)
#define AIRPORT_AREA_HELIPAD            (1)
#define AIRPORT_AREA_PAVEMENT           (2)
#define AIRPORT_AREA_TAXIWAY            (3)
#define AIRPORT_AREA_RUNWAY_SHOULDER    (4)
#define AIRPORT_AREA_HELIPAD_SHOULDER   (5)
#define AIRPORT_AREA_INNER_BASE         (6)
#define AIRPORT_AREA_OUTER_BASE         (7)
#define AIRPORT_AREA_RWY_FEATURES       (8)
#define AIRPORT_AREA_TAXI_FEATURES      (9)
#endif

void Airport::WriteBaseOutput( const std::string& root, const SGBucket& b )
{
    if ( base_nodes.size() ) {
        //
        // first write the pavements and grass as chopped polys for undetailed LOD
        //
        tgAccumulator base_ld_accum;
        tgAccumulator pvmt_ld_accum;
        
        for ( unsigned int area = AIRPORT_AREA_INNER_BASE; area <= AIRPORT_AREA_OUTER_BASE; area++ ) {
            for( unsigned int p = 0; p < polys_built.area_size(area); p++ ) {
                tgPolygon& poly = polys_built.get_poly( area, p );
                base_ld_accum.Add( poly );
            }
        }
        tgPolygon base_ld = base_ld_accum.Union();

        for ( unsigned int area = AIRPORT_AREA_RUNWAY; area <= AIRPORT_AREA_HELIPAD_SHOULDER; area++ ) {
            for( unsigned int p = 0; p < polys_built.area_size(area); p++ ) {
                tgPolygon& poly = polys_built.get_poly( area, p );
                pvmt_ld_accum.Add( poly );
            }
        }
        tgPolygon pvmt_ld = pvmt_ld_accum.Union();

        std::string   low_detail_path = root + "/Airport_lowdetail";
        tgChopper     ld_chopper( low_detail_path );
   
        base_ld.SetTexMethod( TG_TEX_BY_GEODE );
        base_ld.SetPreserve3D( false );
        ld_chopper.Add( base_ld, "Airport" );

        pvmt_ld.SetTexMethod( TG_TEX_BY_GEODE );
        pvmt_ld.SetPreserve3D( false );
        ld_chopper.Add( pvmt_ld, "Asphalt" );

        ld_chopper.Save(false);
        
        //
        // Then create seperate smoothed .btg for high detail airport
        //
        UniqueSGVec3fSet normals;
        UniqueSGVec2fSet texcoords;

        std::string objpath = root + "/AirportObj";
        std::string name = icao + ".btg";

        SGVec3f vnt = SGVec3f::fromGeod( base_nodes.get_node(0).GetPosition() );
        vnt = normalize(vnt);

        std::vector< SGVec3d > wgs84_nodes;
        base_nodes.get_wgs84_nodes( wgs84_nodes );
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

        std::string base = objpath;
        std::string binname = b.gen_index_str();
        binname += ".btg";
        std::string txtname = b.gen_index_str();
        txtname += ".txt";

        SGBinObject obj;
        
        for (unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++) {
            for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
                tgPolygon   poly      = polys_clipped.get_poly(area, p);
                std::string material  = poly.GetMaterial();
                SGBinObjectTriangle sgboTri;
                
                for (unsigned int k = 0; k < poly.Triangles(); ++k) {
                    sgboTri.clear();
                    
                    // Set Triangle Material
                    sgboTri.material = material;
                    
                    // For each triangle vertex, set the appropriate indexes
                    for (int l = 0; l < 3; ++l) {
                        int index;                        

                        index = poly.GetTriIdx( k, l );
                        sgboTri.v_list.push_back( index );
                        
                        // use 'the' normal
                        index = normals.add( vnt );
                        sgboTri.n_list.push_back( index );
                        
                        index = texcoords.add( poly.GetTriPriTexCoord( k, l ) );
                        sgboTri.tc_list[0].push_back( index );
                    }                    
                    obj.add_triangle( sgboTri );
                }
            }
        }
        
        obj.set_gbs_center( gbs_center );
        obj.set_gbs_radius( gbs_radius );
        obj.set_wgs84_nodes( wgs84_nodes );
        obj.set_normals( normals.get_list() );
        obj.set_texcoords( texcoords.get_list() );
                
        bool result;
        result = obj.write_bin( objpath, name, b );

        if ( !result )
        {
            throw sg_exception("error writing file. :-(");
        }

        // write out airport object reference
        write_index( objpath, b, name );


        //
        // Finally, write the 'connective tissue' between the outer airport base ( unsmoothed )
        // to the hole ( smoothed )
        //
        std::string holepath = root + "/AirportArea";
        tgChopper chopper( holepath );

        tgPolygon outer_base = tgPolygon::Union( polys_built.get_polys(AIRPORT_AREA_OUTER_BASE) );

        /* need to polulate the elevations in inner base */
        inner_base.SetElevations( base_nodes );
        inner_base.SetPreserve3D( true );
        chopper.Add( inner_base, "Hole" );

        outer_base.SetPreserve3D( false );
        outer_base.SetTexMethod( TG_TEX_BY_GEODE );
        chopper.Add( outer_base, "Airport" );

        chopper.Save(false);
    }
}
