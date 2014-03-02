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

void Airport::BuildFeatures( void )
{
    tgpolygon_list polys;
    
    TG_LOG(SG_GENERAL, SG_INFO, "Build Lines " );

    // need a new polys_buit / Polys clipped for linear features (including priority defs )
    for ( unsigned int i=0; i<features.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Feature Poly " << i + 1 << " of " << features.size() << " : " << features[i]->GetDescription() );
        features[i]->GetPolys( polys_built.get_polys(AIRPORT_AREA_FEATURES) );
    }

    for ( unsigned int i=0; i<pavements.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Pavement " << i + 1 << " of " << pavements.size() << " : " << pavements[i]->GetDescription());
        pavements[i]->GetFeaturePolys( polys_built.get_polys(AIRPORT_AREA_FEATURES) );
    }
}

void Airport::ClipFeatures()
{
    tgAccumulator accum;
    tgPolygon clipped;
    
    // first, collect all the base nodes
    TG_LOG(SG_GENERAL, SG_INFO, "Clipping Feature polys" );
    for( unsigned int p = 0; p < polys_built.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        tgPolygon& current = polys_built.get_poly(AIRPORT_AREA_FEATURES, p);
            
        clipped = accum.Diff( current );
            
        // only add to output list if the clip left us with a polygon
        if ( clipped.Contours() > 0 ) {                
            // add the sliverless result polygon to the clipped polys list
            if ( clipped.Contours() > 0  ) {
                // copy all of the superpolys and texparams
                clipped.SetId( polys_built.get_poly( AIRPORT_AREA_FEATURES, p ).GetId() );
                polys_clipped.add_poly( AIRPORT_AREA_FEATURES, clipped );
            }
        }
            
        accum.Add( current );
    }
    
    // now break into max segment size
    for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly( AIRPORT_AREA_FEATURES, p );
            
        poly = tgPolygon::SplitLongEdges(poly, 100);
            
        polys_clipped.set_poly( AIRPORT_AREA_FEATURES, p, poly );
    }
    
    // Now, Make sure we have all the base nodes added as smoothed elevation nodes
    for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly( AIRPORT_AREA_FEATURES, p );
            
        for (unsigned int con=0; con < poly.Contours(); con++) {
            for (unsigned int n = 0; n < poly.ContourSize( con ); n++) {
                // ensure
                SGGeod const& node = poly.GetNode( con, n );
                feat_nodes.unique_add( node, TG_NODE_DRAPED );
            }
        }
    }
}

void Airport::CleanFeatures()
{
    int before, after;
    std::vector<SGGeod> points;
    tgRectangle bb;
    
    // traverse each poly, and add intermediate nodes
    for( unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        tgPolygon current = polys_clipped.get_poly(AIRPORT_AREA_FEATURES, p);
        bb = current.GetBoundingBox();
        feat_nodes.get_geod_inside( bb.getMin(), bb.getMax(), points );
            
        before  = current.TotalNodes();
        current = tgPolygon::AddColinearNodes( current, points );
        current = tgPolygon::Snap(current, gSnap);
        current = tgPolygon::RemoveDups( current );
        current = tgPolygon::RemoveBadContours( current );
        after   = current.TotalNodes();
            
        if (before != after) {
            SG_LOG( SG_CLIPPER, SG_ALERT, "Fixed T-Junctions in " << p+1 << " of " << (int)polys_clipped.area_size(AIRPORT_AREA_FEATURES) << " nodes increased from " << before << " to " << after );
        }
            
        /* Save it back */
        polys_clipped.set_poly( AIRPORT_AREA_FEATURES, p, current );
    }
    
    // simplify the polys
    for( unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        tgPolygon& current = polys_clipped.get_poly(AIRPORT_AREA_FEATURES, p);
        current = tgPolygon::Simplify( current );
        polys_clipped.set_poly( AIRPORT_AREA_FEATURES, p, current );
    }
}

void Airport::IntersectFeaturesWithBase(void)
{
    int before, after;
    SGTimeStamp intersect_start;
    SGTimeStamp intersect_end;
    
    intersect_start.stamp();
    
    // generate the base mesh    
    for ( unsigned int area = 0; area <= AIRPORT_MAX_BASE; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly( area, p );
            
            for (unsigned int t=0; t < poly.Triangles(); t++) {
                // triangles don't have elevation - get it from base_nodes via index
                tgTriangle tri = poly.GetTriangle( t );
                for ( unsigned int n=0; n<3; n++ ) {
                    tri.SetNode( n, base_nodes[tri.GetIndex(n)].GetPosition() );
                }

                base_mesh.push_back( tri );
            }
        }
    }
    
    for( unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {        
        tgPolygon current = polys_clipped.get_poly(AIRPORT_AREA_FEATURES, p);

        before  = current.TotalNodes();
        current = tgPolygon::AddIntersectingNodes( current, base_mesh );
        after   = current.TotalNodes();
        
        if (before != after) {
            SG_LOG( SG_GENERAL, SG_DEBUG, "IntersectFeaturesWithBase feature " << p+1 << " of " << (int)polys_clipped.area_size(AIRPORT_AREA_FEATURES) << " nodes increased from " << before << " to " << after );
        }           
        
        /* Save it back */
        polys_clipped.set_poly( AIRPORT_AREA_FEATURES, p, current );
    }
    
    intersect_end.stamp();
    
    SG_LOG( SG_GENERAL, SG_ALERT, "IntersectFeaturesWithBase time " << intersect_end - intersect_start );
}

void Airport::TesselateFeatures()
{
    // tesselate the polygons and prepair them for final output
    for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        //TG_LOG(SG_GENERAL, SG_INFO, "Tesselating Base poly " << area << ", " << p );
        tgPolygon& poly = polys_clipped.get_poly(AIRPORT_AREA_FEATURES, p );
            
        #if DEBUG
        char layer[32];
        sprintf(layer, "tess_%d_%d", AIRPORT_AREA_FEATURES, p );
        tgShapefile::FromPolygon( poly, debug_path, layer, poly.GetMaterial().c_str() );
        #endif
            
        poly.Tesselate();
    }

    for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly(AIRPORT_AREA_FEATURES, p );
        
        // ensure all added nodes are accounted for
        for (unsigned int k=0; k < poly.Triangles(); k++) {
            for (int l = 0; l < 3; l++) {
                // ensure we have all nodes...
                feat_nodes.unique_add( poly.GetTriNode( k, l ), TG_NODE_DRAPED );
            }
        }
    }
}

void Airport::TextureFeatures( void )
{    
    for( unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly(AIRPORT_AREA_FEATURES, p);
        poly.Texture( );
    }    
}

void Airport::CalcFeatureElevations( void )
{
    SGTimeStamp drape_start;
    SGTimeStamp drape_end;
    
    drape_start.stamp();
    
    // drape over the base triangle mesh
    // first, get a list of all triangles to give to CalcElevations    
    feat_nodes.CalcElevations( TG_NODE_DRAPED, base_mesh );
    
    drape_end.stamp();
    
    SG_LOG( SG_GENERAL, SG_ALERT, "CalcFeatureElevations time " << drape_end - drape_start );    
}

void Airport::LookupFeatureIndexes( void )
{
    // for each node, traverse all the triangles - and create face lists
    for( unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly( AIRPORT_AREA_FEATURES, p );
        
        for (unsigned int tri=0; tri < poly.Triangles(); tri++) {
            for (unsigned int vertex = 0; vertex < 3; vertex++) {
                int idx = feat_nodes.find( poly.GetTriNode( tri, vertex ) );
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

void Airport::WriteFeatureOutput( const std::string& root, const SGBucket& b )
{
    if ( feat_nodes.size() ) {
        UniqueSGVec3fSet normals;
        UniqueSGVec2fSet texcoords;
        
        std::string objpath = root + "/AirportObj";
        std::string name = icao + "_lines.btg";
        
        SGVec3f vnt = SGVec3f::fromGeod( feat_nodes.get_node(0).GetPosition() );
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
        
        for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_FEATURES); p++ ) {
            tgPolygon   poly      = polys_clipped.get_poly(AIRPORT_AREA_FEATURES, p);
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
        
        std::vector< SGVec3d > wgs84_nodes;
        feat_nodes.get_wgs84_nodes( wgs84_nodes );
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
        obj.set_tris_pri_tc( tris_tc );
        obj.set_tri_materials( tri_materials );
        obj.set_strips_v( strips_v );
        obj.set_strips_n( strips_n );
        obj.set_strips_pri_tc( strips_tc );
        obj.set_strip_materials( strip_materials );
        obj.set_fans_v( fans_v );
        obj.set_fans_n( fans_n );
        obj.set_fans_pri_tc( fans_tc );
        obj.set_fan_materials( fan_materials );
        
        bool result;
        result = obj.write_bin( objpath, name, b );
        if ( !result )
        {
            throw sg_exception("error writing file. :-(");
        }
        
        // write out airport object reference
        write_index_lines( objpath, b, name );
    }        
}
