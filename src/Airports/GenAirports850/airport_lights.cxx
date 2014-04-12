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

void Airport::BuildLights( void )
{
    // build runway lights
    for ( unsigned int i=0; i<runways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Runway light " << i + 1 << " of " << runways.size());
        runways[i]->GetLights( lights );
    }
    
    for ( unsigned int i=0; i<helipads.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Helipad " << i + 1 << " of " << helipads.size());
        helipads[i]->GetLights( lights );
    }
    
    // build feature lights
    for ( unsigned int i=0; i<features.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Feature Poly " << i + 1 << " of " << features.size() << " : " << features[i]->GetDescription() );
        features[i]->GetLights( lights );
    }
    
    // build pavement lights
    for ( unsigned int i=0; i<pavements.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Pavement " << i + 1 << " of " << pavements.size() << " : " << pavements[i]->GetDescription());
        pavements[i]->GetFeatureLights( lights );
    }
    
    for ( unsigned int i=0; i<taxiways.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Taxiway " << i + 1 << " of " << taxiways.size());
        taxiways[i]->GetLights( lights );
    }
    
    TG_LOG(SG_GENERAL, SG_INFO, "Build lightobjects " << lightobjects.size() );    
    for ( unsigned int i=0; i<lightobjects.size(); i++ )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Build Light object" << i + 1 << " of " << lightobjects.size());
        lightobjects[i]->BuildBtg( lights );
    }
    
    TG_LOG(SG_GENERAL, SG_INFO, "we have " << lights.size() << " lights");    
}

void Airport::WriteLightsOutput( const std::string& root, const SGBucket& b )
{
    UniqueSGVec3fSet light_normals;
    UniqueSGVec2fSet light_texcoords;
    
    if ( lights.size() ) {        
        std::string objpath = root + "/AirportObj";
        std::string name = icao + "_lights.btg";
                
        SGBinObject obj;        
        // add light points        
        SGBinObjectPoint sgboPt;
        
        for ( unsigned int i = 0; i < lights.size(); ++i ) {
            if ( lights[i].ContourSize() ) {            
                sgboPt.clear();
                sgboPt.material = lights[i].GetType();
            
                for ( unsigned int j = 0; j < lights[i].ContourSize(); ++j )
                {
                    int index;
                
                    index = light_nodes.unique_add( lights[i].GetPosition(j), TG_NODE_SMOOTHED );
                    sgboPt.v_list.push_back( index );
                
                    index = light_normals.add( lights[i].GetNormal(j) );
                    sgboPt.n_list.push_back( index );
                }
            
                obj.add_point( sgboPt );
            } else {
                TG_LOG(SG_GENERAL, SG_ALERT, "light contour for mat " << sgboPt.material << " HAS NO LIGHTS " );
            }
        }
        
        // reuse the base surface to calc elevation
        light_nodes.CalcElevations( TG_NODE_SMOOTHED, base_surf );

        SGVec3d gbs_center = SGVec3d::fromGeod( b.get_center() );
        double dist_squared, radius_squared = 0;
        
        std::vector< SGVec3d > wgs84_nodes;
        light_nodes.get_wgs84_nodes( wgs84_nodes );
        
        TG_LOG(SG_GENERAL, SG_ALERT, "number of light nodes " << wgs84_nodes.size() );
        
        for (int i = 0; i < (int)wgs84_nodes.size(); ++i)
        {
            dist_squared = distSqr(gbs_center, wgs84_nodes[i]);
            if ( dist_squared > radius_squared ) {
                radius_squared = dist_squared;
            }
        }
        double gbs_radius = sqrt(radius_squared);
        
        TG_LOG(SG_GENERAL, SG_DEBUG, "gbs center = " << gbs_center);
        TG_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
        TG_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center << " radius = " << gbs_radius );
        
        obj.set_gbs_center( gbs_center );
        obj.set_gbs_radius( gbs_radius );
        obj.set_wgs84_nodes( wgs84_nodes );
        obj.set_normals( light_normals.get_list() );
        obj.set_texcoords( light_texcoords.get_list() );
        
        std::string base = objpath;
        std::string binname = b.gen_index_str();
        binname += ".btg";
        std::string txtname = b.gen_index_str();
        txtname += ".txt";        
        bool result;
        
        result = obj.write_bin( objpath, name, b );
        if ( !result )
        {
            throw sg_exception("error writing file. :-(");
        }
        
        // write out airport object reference
        write_index( objpath, b, name );
    }
}