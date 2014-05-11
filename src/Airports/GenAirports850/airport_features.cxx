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
    
    TG_LOG(SG_GENERAL, SG_ALERT, "Building Runway Lines " );

    // need a new polys_buit / Polys clipped for linear features (including priority defs )
    for ( unsigned int i=0; i<runways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_ALERT, "Build Runway Feature Poly " << i + 1 << " of " << runways.size() );
        runways[i]->GetMarkingPolys( polys_built.get_polys(AIRPORT_AREA_RWY_FEATURES) );
    }
    
    TG_LOG(SG_GENERAL, SG_INFO, "Building Taxiway Lines " );
    
    // need a new polys_buit / Polys clipped for linear features (including priority defs )
    for ( unsigned int i=0; i<features.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Feature Poly " << i + 1 << " of " << features.size() << " : " << features[i]->GetDescription() );
        features[i]->GetPolys( polys_built.get_polys(AIRPORT_AREA_TAXI_FEATURES) );
    }

    for ( unsigned int i=0; i<pavements.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Pavement " << i + 1 << " of " << pavements.size() << " : " << pavements[i]->GetDescription());
        pavements[i]->GetFeaturePolys( polys_built.get_polys(AIRPORT_AREA_TAXI_FEATURES) );
    }
    
    // for now - caps are the same priority - just add them later...
    for ( unsigned int i=0; i<runways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_ALERT, "Build Runway Feature Poly " << i + 1 << " of " << runways.size() );
        runways[i]->GetCapPolys( polys_built.get_polys(AIRPORT_AREA_RWY_FEATURES) );
    }
    
    for ( unsigned int i=0; i<features.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Feature Poly (caps) " << i + 1 << " of " << features.size() << " : " << features[i]->GetDescription() );
        features[i]->GetCapPolys( polys_built.get_polys(AIRPORT_AREA_TAXI_FEATURES) );
    }
    
    for ( unsigned int i=0; i<pavements.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Pavement (caps)" << i + 1 << " of " << pavements.size() << " : " << pavements[i]->GetDescription());
        pavements[i]->GetFeatureCapPolys( polys_built.get_polys(AIRPORT_AREA_TAXI_FEATURES) );
    }
    
    // make sure we have a va method...
    TG_LOG(SG_GENERAL, SG_ALERT, "check all built features have va attrib " );

    for ( unsigned int x=0; x<polys_built.get_polys(AIRPORT_AREA_RWY_FEATURES).size(); x++ ) {
        if (polys_built.get_polys(AIRPORT_AREA_RWY_FEATURES)[x].GetNumIntVas() != 1 ) {
            TG_LOG(SG_GENERAL, SG_ALERT, "rwy poly " << x << " dows not have int va " );
        }
    }
    
    for ( unsigned int x=0; x<polys_built.get_polys(AIRPORT_AREA_TAXI_FEATURES).size(); x++ ) {
        if (polys_built.get_polys(AIRPORT_AREA_TAXI_FEATURES)[x].GetNumIntVas() != 1 ) {
            TG_LOG(SG_GENERAL, SG_ALERT, "taxi poly " << x << " dows not have int va " );
        }
    }
    
}

void Airport::ClipFeatures()
{
    tgAccumulator rwy_accum;
    tgAccumulator taxi_accum;
    tgPolygon clipped;
    
    // first, collect all the base nodes
    TG_LOG(SG_GENERAL, SG_INFO, "Clipping Feature polys" );

    for( unsigned int p = 0; p < polys_built.area_size(AIRPORT_AREA_RWY_FEATURES); p++ ) {
        tgPolygon& current = polys_built.get_poly(AIRPORT_AREA_RWY_FEATURES, p);
        
        clipped = rwy_accum.Diff( current );
        
        // only add to output list if the clip left us with a polygon
        if ( clipped.Contours() > 0 ) {                
            // add the sliverless result polygon to the clipped polys list
            if ( clipped.Contours() > 0  ) {
                // copy all of the superpolys and texparams
                clipped.SetId( polys_built.get_poly( AIRPORT_AREA_RWY_FEATURES, p ).GetId() );
                polys_clipped.add_poly( AIRPORT_AREA_RWY_FEATURES, clipped );
            }
        }
        
        rwy_accum.Add( current );
    }
    
    for( unsigned int p = 0; p < polys_built.area_size(AIRPORT_AREA_TAXI_FEATURES); p++ ) {
        tgPolygon& current = polys_built.get_poly(AIRPORT_AREA_TAXI_FEATURES, p);
            
        clipped = taxi_accum.Diff( current );
            
        // only add to output list if the clip left us with a polygon
        if ( clipped.Contours() > 0 ) {                
            // add the sliverless result polygon to the clipped polys list
            if ( clipped.Contours() > 0  ) {
                // copy all of the superpolys and texparams
                clipped.SetId( polys_built.get_poly( AIRPORT_AREA_TAXI_FEATURES, p ).GetId() );
                polys_clipped.add_poly( AIRPORT_AREA_TAXI_FEATURES, clipped );
            }
        }
            
        taxi_accum.Add( current );
    }
    
    // now break into max segment size
    for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_RWY_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly( AIRPORT_AREA_RWY_FEATURES, p );
        
        poly = tgPolygon::SplitLongEdges(poly, 100);
        
        polys_clipped.set_poly( AIRPORT_AREA_RWY_FEATURES, p, poly );
    }
    
    for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_TAXI_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly( AIRPORT_AREA_TAXI_FEATURES, p );
            
        poly = tgPolygon::SplitLongEdges(poly, 100);
            
        polys_clipped.set_poly( AIRPORT_AREA_TAXI_FEATURES, p, poly );
    }
    
    
    // Now, Make sure we have all the base nodes added as smoothed elevation nodes
    for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_RWY_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly( AIRPORT_AREA_RWY_FEATURES, p );
            
        for (unsigned int con=0; con < poly.Contours(); con++) {
            for (unsigned int n = 0; n < poly.ContourSize( con ); n++) {
                // ensure
                SGGeod const& node = poly.GetNode( con, n );
                feat_nodes.unique_add( node, TG_NODE_DRAPED );
            }
        }
    }
    
    for (unsigned int p = 0; p < polys_clipped.area_size(AIRPORT_AREA_TAXI_FEATURES); p++ ) {
        tgPolygon& poly = polys_clipped.get_poly( AIRPORT_AREA_TAXI_FEATURES, p );
        
        for (unsigned int con=0; con < poly.Contours(); con++) {
            for (unsigned int n = 0; n < poly.ContourSize( con ); n++) {
                // ensure
                SGGeod const& node = poly.GetNode( con, n );
                feat_nodes.unique_add( node, TG_NODE_DRAPED );
            }
        }
    }
   
   TG_LOG(SG_GENERAL, SG_ALERT, "check all features have va attrib after clipping" );
   
   for ( unsigned int x=0; x<polys_clipped.get_polys(AIRPORT_AREA_RWY_FEATURES).size(); x++ ) {
       if (polys_clipped.get_polys(AIRPORT_AREA_RWY_FEATURES)[x].GetNumIntVas() != 1 ) {
           TG_LOG(SG_GENERAL, SG_ALERT, "rwy poly " << x << " dows not have int va " );
           exit(0);
       }
   }
   
   for ( unsigned int x=0; x<polys_clipped.get_polys(AIRPORT_AREA_TAXI_FEATURES).size(); x++ ) {
       if (polys_clipped.get_polys(AIRPORT_AREA_TAXI_FEATURES)[x].GetNumIntVas() != 1 ) {
           TG_LOG(SG_GENERAL, SG_ALERT, "taxi poly " << x << " dows not have int va " );
           exit(0);
       }
   }

   TG_LOG(SG_GENERAL, SG_ALERT, "after clipping, all features have va attrib " );
   
}

void Airport::CleanFeatures()
{
    int before, after;
    std::vector<SGGeod> points;
    tgRectangle bb;
    
    // traverse each poly, and add intermediate nodes
    for ( unsigned int area=AIRPORT_AREA_RWY_FEATURES; area<=AIRPORT_AREA_TAXI_FEATURES; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon current = polys_clipped.get_poly(area, p);

            if ( current.GetNumIntVas() != 1 ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "AddColinearNodes broken before call" );
                exit(0);
            }
            
            bb = current.GetBoundingBox();
            feat_nodes.get_geod_inside( bb.getMin(), bb.getMax(), points );
            
            before  = current.TotalNodes();
            current = tgPolygon::AddColinearNodes( current, points );
            
            if ( current.GetNumIntVas() != 1 ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "AddColinearNodes broke us" );
                exit(0);
            }
            
            current = tgPolygon::Snap(current, gSnap);

            if ( current.GetNumIntVas() != 1 ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Snap broke us" );
                exit(0);
            }
            
            current = tgPolygon::RemoveDups( current );
            
            if ( current.GetNumIntVas() != 1 ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "RemoveDups broke us" );
                exit(0);
            }
            
            current = tgPolygon::RemoveBadContours( current );
            
            if ( current.GetNumIntVas() != 1 ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "RemoveBadContours broke us" );
                exit(0);
            }
            
            after   = current.TotalNodes();
            
            if (before != after) {
                SG_LOG( SG_CLIPPER, SG_DEBUG, "Fixed T-Junctions in " << p+1 << " of " << (int)polys_clipped.area_size(area) << " nodes increased from " << before << " to " << after );
            }
            
            /* Save it back */
            polys_clipped.set_poly( area, p, current );
        }
    
        // simplify the polys
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& current = polys_clipped.get_poly(area, p);
            current = tgPolygon::Simplify( current );

            if ( current.GetNumIntVas() != 1 ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Simplify broke us" );
                exit(0);
            }
            
            polys_clipped.set_poly( area, p, current );
        }
    }
    
    TG_LOG(SG_GENERAL, SG_ALERT, "check all features have va attrib after cleaning" );
    
    for ( unsigned int x=0; x<polys_clipped.get_polys(AIRPORT_AREA_TAXI_FEATURES).size(); x++ ) {
        if (polys_clipped.get_polys(AIRPORT_AREA_TAXI_FEATURES)[x].GetNumIntVas() != 1 ) {
            TG_LOG(SG_GENERAL, SG_ALERT, "poly " << x << " dows not have int va " );
        }
    }
    
    TG_LOG(SG_GENERAL, SG_ALERT, "after cleaning, all features have va attrib " );
    
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
    
    for ( unsigned int area=AIRPORT_AREA_RWY_FEATURES; area<=AIRPORT_AREA_TAXI_FEATURES; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {        
            tgPolygon current = polys_clipped.get_poly(area, p);

            before  = current.TotalNodes();
            current = tgPolygon::AddIntersectingNodes( current, base_mesh );
            after   = current.TotalNodes();
        
            if (before != after) {
                SG_LOG( SG_GENERAL, SG_DEBUG, "IntersectFeaturesWithBase feature " << p+1 << " of " << (int)polys_clipped.area_size(area) << " nodes increased from " << before << " to " << after );
            }           
        
            /* Save it back */
            polys_clipped.set_poly( area, p, current );
        }
    }
    
    intersect_end.stamp();
    
    SG_LOG( SG_GENERAL, SG_ALERT, "IntersectFeaturesWithBase time " << intersect_end - intersect_start );
}

void Airport::TesselateFeatures()
{
    //sglog().setLogLevels( SG_GENERAL, SG_INFO );
    
    // tesselate the polygons and prepair them for final output
    for ( unsigned int area=AIRPORT_AREA_RWY_FEATURES; area<=AIRPORT_AREA_TAXI_FEATURES; area++ ) {
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            //TG_LOG(SG_GENERAL, SG_INFO, "Tesselating Base poly " << area << ", " << p );
            tgPolygon& poly = polys_clipped.get_poly(area, p );
            
            #if DEBUG
            char layer[32];
            sprintf(layer, "tess_%d_%d", area, p );
            tgShapefile::FromPolygon( poly, debug_path, layer, poly.GetMaterial().c_str() );
            #endif
            
            poly.Tesselate();
        
#if 1            
            // for each triangle in the solution, find the triangle it is coplanar with
            for ( unsigned int t=0; t<poly.Triangles(); t++ ) {
                SGGeod c = poly.GetTriangle(t).GetCentroid();
                bool trifound = false;
            
                // now find the triangle in the base mesh this feature triangle is coplanar with
                for ( unsigned int bm=0; bm<base_mesh.size(); bm++ ) {
                    if ( base_mesh[bm].IsPointInside( c ) ) {
                        // assign secondary TexParams from base_mesh triangle parent poly
                        tgTexParams tp = base_mesh[bm].GetParent()->GetTexParams();
                        if ( tp.method == TG_TEX_UNKNOWN ) {
                            SG_LOG( SG_GENERAL, SG_DEBUG, "Tesselate poly " << p+1 << " of " << (int)polys_clipped.area_size(area) << " triangle " << t+1 << " found tp with unset method " );
                        } else {
                            SG_LOG( SG_GENERAL, SG_DEBUG, "Tesselate poly " << p+1 << " of " << (int)polys_clipped.area_size(area) << " triangle " << t+1 << " set tp with method " << tp.method );
                        }
                        poly.GetTriangle(t).SetSecondaryTexParams( tp );
                        trifound = true;
                        break;
                    }
                }
            
                if (!trifound) {
                    SG_LOG( SG_GENERAL, SG_ALERT, "Tesselate poly " << p+1 << " of " << (int)polys_clipped.area_size(area) << " triangle " << t+1 << " could not find base mesh texparams " );
                    //exit(-100);
                }
            }
#endif

        }

        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly(area, p );
        
            // ensure all added nodes are accounted for
            for (unsigned int k=0; k < poly.Triangles(); k++) {
                for (int l = 0; l < 3; l++) {
                    // ensure we have all nodes...
                    feat_nodes.unique_add( poly.GetTriNode( k, l ), TG_NODE_DRAPED );
                }
            }
        }
    }
}

void Airport::TextureFeatures( void )
{
    for ( unsigned int area=AIRPORT_AREA_RWY_FEATURES; area<=AIRPORT_AREA_TAXI_FEATURES; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly(area, p);
            poly.Texture();

            // make sure poly has a vertex mask
            if (poly.GetNumIntVas() != 1) {
                SG_LOG( SG_GENERAL, SG_ALERT, "poly with material " << poly.GetMaterial() << " does not have int vas " << poly.GetNumIntVas() );
                exit(0);
            }                
            
//          sglog().setLogLevels( SG_GENERAL, SG_DEBUG );
          poly.TextureSecondary();
        
//          sglog().setLogLevels( SG_GENERAL, SG_INFO );        
        }
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
    for ( unsigned int area=AIRPORT_AREA_RWY_FEATURES; area<=AIRPORT_AREA_TAXI_FEATURES; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly( area, p );
        
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
}

// belongs in terragear lib...
unsigned int add_unique_int( std::vector<int>& vaints, int attrib )
{
    // traverse the list and look for a match;
    unsigned int idx = 0;
    bool found = false;
    
    for ( unsigned int i=0; i<vaints.size(); i++ ) {
        if ( vaints[i] == attrib ) {
            found = true;
            idx = i;
            break;
        }
    }
    
    if ( !found ) {
        idx = vaints.size();
        vaints.push_back(attrib);
    }
    
    return idx;
}

unsigned int add_unique_float( std::vector<float>& vaflts, float attrib )
{
    // traverse the list and look for a match;
    unsigned int idx = 0;
    bool found = false;
    
    for ( unsigned int i=0; i<vaflts.size(); i++ ) {
        if ( fabs ( vaflts[i] - attrib ) < 0.0000000001 ) {
            found = true;
            idx = i;
            break;
        }
    }
    
    if ( !found ) {
        idx = vaflts.size();
        vaflts.push_back(attrib);
    }
    
    return idx;
}

void Airport::WriteFeatureOutput( const std::string& root, const SGBucket& b )
{
    SG_LOG(SG_GENERAL, SG_INFO, "WriteFeatureOutput" );
    
    if ( feat_nodes.size() ) {
        UniqueSGVec3fSet normals;
        UniqueSGVec2fSet texcoords;
        std::vector<int>   vaints;    // don't bother with uniqueness : we can just look it up ( may do this later )
        std::vector<float> vafloats;  // same
        
        std::string objpath = root + "/AirportObj";
        std::string name = icao + "_lines.btg";
        
        SGVec3f vnt = SGVec3f::fromGeod( feat_nodes.get_node(0).GetPosition() );
        vnt = normalize(vnt);
        
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
                
        std::string base = objpath;
        std::string binname = b.gen_index_str();
        binname += ".btg";
        std::string txtname = b.gen_index_str();
        txtname += ".txt";
        
        SGBinObject obj;

        for ( unsigned int area=AIRPORT_AREA_RWY_FEATURES; area<=AIRPORT_AREA_TAXI_FEATURES; area++ ) {
            for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
                tgPolygon   poly      = polys_clipped.get_poly(area, p);
                std::string material  = poly.GetMaterial();
                unsigned int num_int_vas = poly.GetNumIntVas();
                unsigned int num_flt_vas = poly.GetNumFltVas();                
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
                        
                        index = texcoords.add( poly.GetTriSecTexCoord( k, l ) );
                        sgboTri.tc_list[1].push_back( index );
                        
                        for ( unsigned int m=0; m<num_int_vas; m++ ) {
                            index = add_unique_int( vaints, poly.GetTriIntVA( k, l, m  ) );
                            sgboTri.va_list[m].push_back( index );
                        }
                        
                        for ( unsigned int m=0; m<num_flt_vas; m++ ) {
                            index = add_unique_float( vafloats, poly.GetTriFltVA( k, l, m  ) );
                            sgboTri.va_list[4+m].push_back( index );
                        }
                    }    
                    
                    if ( num_int_vas == 1 )
                    {
                        if ( ( sgboTri.va_list[0][0] != sgboTri.va_list[0][1] ) ||
                             ( sgboTri.va_list[0][1] != sgboTri.va_list[0][2] ) || 
                             ( sgboTri.va_list[0][2] != sgboTri.va_list[0][0] ) )
                        {
                            SG_LOG(SG_GENERAL, SG_INFO, "vertex atttrib mismatch!");
                            exit(0);
                        }
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
        if (!vaints.empty()) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "adding int va list of size " << vaints.size() );
//          TODO : Fix Simgear API
//          obj.set_intvetexattribs( vaints );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "crap - no int vas ");
        }
        
        if (!vafloats.empty()) {
//          TODO : Fix Simgear API
//          obj.set_floatvetexattribs( vafloats );
        }
        
        bool result = obj.write_bin( objpath, name, b );
        if ( !result )
        {
            throw sg_exception("error writing file. :-(");
        }
        
        // write out airport object reference
        write_index_lines( objpath, b, name );
    }        
}
