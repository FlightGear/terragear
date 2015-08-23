// vector-decode.cxx -- process OGR vector data layers
//                      generate smoothing contours, and polyygons that can be
//                      textured
//
// Written by Peter Sadrozinski, started January 2015.
// Based on ogr_decode.cxx Copyright (C) 2007 Ralf Gerlich
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#include <string>
#include <map>
#include <limits>

#include <boost/thread.hpp>

#include <ogrsf_frmts.h>

#include <simgear/compiler.h>
#include <simgear/threads/SGThread.hxx>
#include <simgear/threads/SGQueue.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sg_path.hxx>

#include <Include/version.h>

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_accumulator.hxx>
#include <terragear/tg_intersection_generator.hxx>
#include <terragear/tg_chopper.hxx>
#include <terragear/tg_shapefile.hxx>

#ifdef _MSC_VER
#  include <windows.h>
#  define tgSleep(x) Sleep(x*1000)
#else
#  define tgSleep(x) sleep(x)
#endif

using std::string;

int line_width=50;
string line_width_col;
string area_type="Default";
string area_type_col;
int continue_on_errors=0;
int max_segment_length=0; // ==0 => don't split
int start_record=0;
bool use_attribute_query=false;
string attribute_query;
bool use_spatial_query=false;
double spat_min_x, spat_min_y, spat_max_x, spat_max_y;

struct areaDef 
{
public:
    areaDef( std::string m, unsigned int w, std::string ds ) : 
    material(m), width(w), datasource(ds) {}
    
    std::string  material;
    unsigned int width;
    std::string  datasource;
};

std::vector<areaDef>    areaDefs;

int GetTextureInfo( unsigned int type, bool cap, std::string& material, double& atlas_startu, double& atlas_endu, double& atlas_startv, double& atlas_endv, double& v_dist )
{
    material = areaDefs[type].material;
    atlas_startu = 0;
    atlas_endu   = 1;
    
    v_dist = 10.0l;
    
    return 0;
}

void processLineString(OGRLineString* poGeometry, unsigned int idx, int width, int zorder, tgIntersectionGenerator* pig )
{
    SGGeod p0, p1;
    int i, numPoints;
    double dbl_w = width;

    numPoints = poGeometry->getNumPoints();
    if (numPoints < 2) {
        SG_LOG( SG_GENERAL, SG_WARN, "Skipping line with less than two points" );
        return;
    }

    // add the points
    for ( i=1;i<numPoints;i++) {
        p0 = SGGeod::fromDeg( poGeometry->getX(i-1), poGeometry->getY(i-1) );
        p1 = SGGeod::fromDeg( poGeometry->getX(i),   poGeometry->getY(i) );

        pig->Insert( p0, p1, dbl_w, zorder, idx );
    }
}

void processLayer(OGRLayer* poLayer, const SGBucket& bucket, tgChopper& results, unsigned int idx, tgIntersectionGenerator* pig )
{
    int feature_count=poLayer->GetFeatureCount();
    int zorder;
    
    if (feature_count!=-1 && start_record>0 && start_record>=feature_count) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Layer has only " << feature_count << " records, but start record is set to " << start_record );
        exit( 1 );
    }
    
    // first, get default width
    line_width = areaDefs[idx].width;
    
    /* determine the indices of the required columns */
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    string layername=poFDefn->GetName();
    int line_width_field=-1;

    if (!line_width_col.empty()) {
        line_width_field=poFDefn->GetFieldIndex(line_width_col.c_str());
        if (line_width_field==-1) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Field " << line_width_col << " for line-width not found in layer" );
        if (!continue_on_errors)
            exit( 1 );
        }
    }

    /* setup a transformation to WGS84 */
    OGRSpatialReference *oSourceSRS, oTargetSRS;
    oSourceSRS=poLayer->GetSpatialRef();

    if (oSourceSRS == NULL) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has no defined spatial reference system" );
        exit( 1 );
    }

    char* srsWkt;
    oSourceSRS->exportToWkt(&srsWkt);
    SG_LOG( SG_GENERAL, SG_DEBUG, "Source spatial reference system: " << srsWkt );
    OGRFree(srsWkt);

    oTargetSRS.SetWellKnownGeogCS( "WGS84" );

    OGRCoordinateTransformation *poCT;

    poCT = OGRCreateCoordinateTransformation(oSourceSRS, &oTargetSRS);

    /* setup attribute and spatial queries */
    double trans_min_x,trans_min_y,trans_max_x,trans_max_y;
    /* do a simple reprojection of the source SRS */
    OGRCoordinateTransformation *poCTinverse;

    poCTinverse = OGRCreateCoordinateTransformation(&oTargetSRS, oSourceSRS);

    SGGeod min, max;
    min = bucket.get_corner( SG_BUCKET_SW );
    max = bucket.get_corner( SG_BUCKET_NE );
    
    trans_min_x=min.getLongitudeDeg();
    trans_min_y=min.getLatitudeDeg();
    trans_max_x=max.getLongitudeDeg();
    trans_max_y=max.getLatitudeDeg();

    poCTinverse->Transform(1,&trans_min_x,&trans_min_y);
    poCTinverse->Transform(1,&trans_max_x,&trans_max_y);

    poLayer->SetSpatialFilterRect(trans_min_x, trans_min_y,
                                  trans_max_x, trans_max_y);

    OGRFeature *poFeature;
    poLayer->SetNextByIndex(start_record);
    for ( ; (poFeature = poLayer->GetNextFeature()) != NULL; OGRFeature::DestroyFeature( poFeature ) )
    {
        OGRGeometry *poGeometry;

        poGeometry = poFeature->GetGeometryRef();

        if (poGeometry==NULL) {
            SG_LOG( SG_GENERAL, SG_INFO, "Found feature without geometry!" );
            if (!continue_on_errors) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Aborting!" );
                exit( 1 );
            } else {
                continue;
            }
        }

        assert(poGeometry!=NULL);

        OGRwkbGeometryType geoType=wkbFlatten(poGeometry->getGeometryType());

        if (geoType!=wkbPoint && geoType!=wkbMultiPoint &&
            geoType!=wkbLineString && geoType!=wkbMultiLineString &&
            geoType!=wkbPolygon && geoType!=wkbMultiPolygon) {
                SG_LOG( SG_GENERAL, SG_INFO, "Unknown feature" );
                continue;
        }

        poGeometry->transform( poCT );

        // get the intersection generator from z-order
        zorder=poFeature->GetFieldAsInteger("Z_ORDER");
        
        switch (geoType) {
        case wkbLineString: {
            SG_LOG( SG_GENERAL, SG_DEBUG, "LineString feature" );
            int width=line_width;
            if (line_width_field!=-1) {
                width=poFeature->GetFieldAsInteger(line_width_field);
                if (width == 0) {
                    width=line_width;
                }
            }

            processLineString((OGRLineString*)poGeometry, idx, width, zorder, pig);
            break;
        }
        case wkbMultiLineString: {
            SG_LOG( SG_GENERAL, SG_DEBUG, "MultiLineString feature" );
            int width=line_width;
            if (line_width_field!=-1) {
                width=poFeature->GetFieldAsInteger(line_width_field);
                if (width == 0) {
                    width=line_width;
                }
            }

            OGRMultiLineString* multils=(OGRMultiLineString*)poGeometry;
            for (int i=0;i<multils->getNumGeometries();i++) {
                processLineString((OGRLineString*)poGeometry, idx, width, zorder, pig);
            }
            break;
        }
        default:
            /* Ignore unhandled objects */
            break;
        }
    }

    OCTDestroyCoordinateTransformation ( poCT );
}

void usage(char* progname) {
    SG_LOG( SG_GENERAL, SG_ALERT, "Usage: " <<
              progname << " [options...] <work_dir> <datasource> [<layername>...]" );
    SG_LOG( SG_GENERAL, SG_ALERT, "Options:" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--line-width width" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Width in meters for the lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--line-width-column colname" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Use value from colname as width for lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Overrides --line-width if present" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--point-width width" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Size in meters of the squares generated from points" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--point-width-column colname" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Use value from colname as width for points" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Overrides --point-width if present" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--area-type type" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Area type for all objects from file" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--area-type-column colname" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Use string from colname as area type" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Overrides --area-type if present" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--continue-on-errors" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Continue even if the file seems fishy" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--max-segment max_segment_length" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Maximum segment length in meters" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--start-record record_no" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Start processing at the specified record number (first record num=0)" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--where attrib_query" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Use an attribute query (like SQL WHERE)" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--spat xmin ymin xmax ymax" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        spatial query extents" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--texture-lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Enable textured lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<work_dir>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Directory to put the polygon files in" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<datasource>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        The datasource from which to fetch the data" );
//    SG_LOG( SG_GENERAL, SG_ALERT, "<layername>..." );
//    SG_LOG( SG_GENERAL, SG_ALERT, "        The layers to process." );
//    SG_LOG( SG_GENERAL, SG_ALERT, "        If no layer is given, all layers in the datasource are used" );
    exit(-1);
}
    
static void ReadConfig( const std::string& filename )
{
    std::ifstream in ( filename.c_str() );
    
    if ( ! in ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Unable to open config file " << filename);
        return;
    }
    SG_LOG(SG_GENERAL, SG_ALERT, "Config file is " << filename);
    
    std::string material, datasource;
    int width;
    
    while ( !in.eof() ) {
        in >> material;
        in >> width;
        in >> datasource;

        if ( material.find("#") ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Read material " << material << " with width " << width << " datasourse is " << datasource );
            areaDefs.push_back( areaDef(material, width, datasource) );
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "Read comment " << material << " with width " << width << " datasourse is " << datasource );
        }
    }
    in.close();
    
    return;
}

void CreateWorkDirs( const std::string work_dir, const std::vector<SGBucket>& bucketList )
{
    for ( unsigned int i=0; i<bucketList.size(); i++ ) {
        SGBucket b = bucketList[i];
        std::string path = work_dir + "/" + b.gen_base_path();
        char tile_name[16];
        
        sprintf( tile_name, "%ld", b.gen_index() );
        std::string polyfile = path + "/" + tile_name;
    
        SGPath sgp( polyfile );
        sgp.create_dir( 0755 );
    }
}

#if 0
class Decoder : public SGThread
{
public:
    Decoder(SGLockedQueue<SGBucket>& wq, const std::string& dd, const std::string& wd, SGMutex* l) : workQueue(wq), dataDir(dd), workDir(wd) {
        lockOgr = l;
    }

private:
    virtual void run();
    void processLineString(OGRLineString* poGeometry, const string& area_type, int width, int with_texture );
    
private:
    SGLockedQueue<SGBucket>& workQueue;
    SGMutex*                 lockOgr;
    
    std::string              dataDir;
    std::string              workDir;
};

void Decoder::run()
{
    SGBucket bucket;
    
    // as long as we have buckets to parse, do so
    while (!workQueue.empty()) {

        SG_LOG( SG_GENERAL, SG_ALERT, "wq size is " << workQueue.size() );
        
        bucket = workQueue.pop();
        char debugdir[128];

        SG_LOG( SG_GENERAL, SG_ALERT, "Decode bucket " << bucket.gen_index_str() );

        lockOgr->lock();
        
//        sprintf( debugdir, "./vectordecode/%s.sqlite", bucket.gen_index_str().c_str() );
        sprintf( debugdir, "./vectordecode/%s", bucket.gen_index_str().c_str() );

        SG_LOG( SG_GENERAL, SG_ALERT, "debug path is " << debugdir );
        
        tgIntersectionGenerator* pig = new tgIntersectionGenerator( debugdir, 0, 1, GetTextureInfo );
        tgChopper results( workDir, bucket.gen_index() );

        GDALDataset *poDS;        
        for ( unsigned int i=0; i<areaDefs.size(); i++ ) {
            char pathname[256];
            
            sprintf( pathname, "%s/%s", dataDir.c_str(), areaDefs[i].datasource.c_str() );
            SG_LOG( SG_GENERAL, SG_ALERT, "Opening datasource " << pathname << " for reading." );
            poDS = (GDALDataset*)GDALOpenEx( pathname, GDAL_OF_VECTOR, NULL, NULL, NULL );

            if( poDS != NULL ) {
                OGRLayer  *poLayer;
                for (int j=0;j<poDS->GetLayerCount();j++) {
                    poLayer = poDS->GetLayer(j);
                    processLayer(poLayer, bucket, results, i, pig );
                }
                
                GDALClose( poDS );
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening datasource " << pathname );
            }
        }        
        // lockOgr->unlock();
        
        // add some additional Variables to the intersection generator
        // cleaning parameters
        // texture mode
        // simplify parameters
        // and add some data access
        // get skeleton segments
        // get skin segments
        // delta height info may be needed....
        // maybe needs a new class entirely based on intersectiongenerator.
        
        // we have all of the data - execute the intersection generator
        // don't clean the OSM map data - as we don't want to generate intersections
        // that don't really exist ( bridges and tunnels )
        // OSM data should have correct intersection nodes already.
        // - they need them to do routing.        
        // lockOgr->lock();
        pig->Execute();
        
        // now retreive the polygons in reverse z-order.  store them in lists
        std::map<int, tgpolygon_list*> polygons;
        
        for ( tgintersectionedge_it it = pig->edges_begin(); it != pig->edges_end(); it++ ) {
            tgPolygon poly = (*it)->GetPoly("complete");
            int       zo   = (*it)->GetZorder();
            
            if ( polygons.find( zo ) == polygons.end() ) {
                // add new polygon list at this zorder
                polygons[zo] = new tgpolygon_list;
            }
            polygons[zo]->push_back( poly );
        }
        
        // clip them in z order
        tgAccumulator accum;    
        std::map<int, tgpolygon_list*>::reverse_iterator pmap_it;
        
        std::cout << " start clip " << std::endl;
        
        for ( pmap_it = polygons.rbegin(); pmap_it != polygons.rend(); pmap_it++ ) {
            std::vector<tgPolygon>::iterator poly_it;
            
            std::cout << " clip zorder " << (*pmap_it).first << std::endl;
            
            unsigned int num_polys = (*pmap_it).second->size();
            unsigned int p = 1;
            
            for ( poly_it = (*pmap_it).second->begin(); poly_it != (*pmap_it).second->end(); poly_it++ ) {
                tgPolygon current = (*poly_it);
                current.RemoveDups();

                accum.Diff_and_Add_cgal( current );
                
                // only add to output list if the clip left us with a polygon
                if ( current.Contours() > 0 ) {
                    results.Add( current, current.GetMaterial() );
                }
            }
        }
        
        results.Save(false);
        
        delete pig;

        lockOgr->unlock();        
    }
}
#endif


int main( int argc, char **argv ) {
    char* progname=argv[0];
    string data_dir = ".";
    string work_dir = ".";
    string config = ".";
    int num_threads = 1;
    
    double minx =  std::numeric_limits<double>::infinity();
    double miny =  std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();
    
    std::vector<string> datasource;

    sglog().setLogLevels( SG_ALL, SG_INFO );
    
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        string arg = argv[arg_pos];
        
        if (arg.find("--data-dir=") == 0) {
            data_dir = arg.substr(11);
        } else if (arg.find("--work-dir=") == 0) {
            work_dir = arg.substr(11);
        } else if (arg.find("--config=") == 0) {
            config = arg.substr(9);
        } else if (arg.find("--threads=") == 0) {
            num_threads = atoi( arg.substr(10).c_str() );
        } else if (arg.find("--threads") == 0) {
            num_threads = boost::thread::hardware_concurrency();
        }
    }

    SG_LOG( SG_GENERAL, SG_ALERT, "vector-decode version " << getTGVersion() << "\n" );
    
    if (argc<4) {
        usage(progname);
    }

    SG_LOG( SG_GENERAL, SG_ALERT, "read config" );
    ReadConfig(config);

    GDALAllRegister();
    GDALDataset       *poDS;

    // if we haven't specified an spacial query - get the extents from the area defs.
    for ( unsigned int i=0; i<areaDefs.size(); i++ ) {
        char pathname[256];
        
        sprintf( pathname, "%s/%s", data_dir.c_str(), areaDefs[i].datasource.c_str() );
        SG_LOG( SG_GENERAL, SG_ALERT, "Opening datasource " << pathname << " for reading." );
        poDS = (GDALDataset*) GDALOpenEx( pathname, GDAL_OF_VECTOR, NULL, NULL, NULL );
        
        if( poDS != NULL ) {
            OGRLayer  *poLayer;
            for (int j=0;j<poDS->GetLayerCount();j++) {
                OGREnvelope oEnvelope;
                poLayer = poDS->GetLayer(j);
                poLayer->GetExtent( &oEnvelope );
                
                if ( oEnvelope.MinX < minx ) { 
                    minx = oEnvelope.MinX;
                }
                if ( oEnvelope.MinY < miny ) { 
                    miny = oEnvelope.MinY;
                }
                if ( oEnvelope.MaxX > maxx ) { 
                    maxx = oEnvelope.MaxX;
                }
                if ( oEnvelope.MaxY > maxy ) { 
                    maxy = oEnvelope.MaxY;
                }
            }
            
            GDALClose( poDS );         
        } else {
            SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening datasource " << pathname );
        }
    }

    SG_LOG( SG_GENERAL, SG_ALERT, "Area extents: (" << minx << "," << miny << ") - (" << maxx << "," << maxy << ")" );
    
    // now generate the WorkDirs - based on the buckets that cover the extents.
    std::vector<SGBucket> bucketList;    
    SGGeod min = SGGeod::fromDeg( minx, miny );
    SGGeod max = SGGeod::fromDeg( maxx, maxy );
    sgGetBuckets( min, max, bucketList );
    
    SG_LOG( SG_GENERAL, SG_ALERT, "Area extents: is covered by " << bucketList.size() << " buckets" );

    // I don't think this is threadsafe - pre create the workdir tree
    CreateWorkDirs( work_dir, bucketList);
    
    /* fill the workqueue */
    //SGLockedQueue<SGBucket> wq;
    //for (unsigned int i=0; i<bucketList.size(); i++) {
    //    wq.push( bucketList[i] );
    //}

    // only allow one thread reading from OGR at a time...
    //SGMutex ogrLock;
    
    // now create the worker threads for stage 1
    //std::vector<Decoder *> decoders;        
    //for (int i=0; i<num_threads; i++) {
    //    Decoder* decoder = new Decoder( wq, data_dir, work_dir, &ogrLock );
    //    decoders.push_back( decoder );
    //}
    
    // start all threads
    //for (unsigned int i=0; i<decoders.size(); i++) {
    //    decoders[i]->start();
    //}

    // wait for workqueue to empty
    //while( wq.size() ) {
    //    tgSleep( 5 );
    //}

    // wait for all threads to complete
    //for (unsigned int i=0; i<decoders.size(); i++) {
    //    decoders[i]->join();
    //}
    

    
    
        
    // as long as we have buckets to parse, do so
    for (unsigned int i=0; i<bucketList.size(); i++) {
        SGBucket bucket = bucketList[i];
        char debugdir[128];

        SG_LOG( SG_GENERAL, SG_ALERT, "Decode bucket " << bucket.gen_index_str() );
        
//        sprintf( debugdir, "./vectordecode/%s.sqlite", bucket.gen_index_str().c_str() );
        sprintf( debugdir, "./vectordecode/%s", bucket.gen_index_str().c_str() );

        SG_LOG( SG_GENERAL, SG_ALERT, "debug path is " << debugdir );
        
        tgIntersectionGenerator* pig = new tgIntersectionGenerator( debugdir, 0, 1, GetTextureInfo );
        tgChopper results( work_dir, bucket.gen_index() );

        GDALDataset *poDS;        
        for ( unsigned int i=0; i<areaDefs.size(); i++ ) {
            char pathname[256];
            
            sprintf( pathname, "%s/%s", data_dir.c_str(), areaDefs[i].datasource.c_str() );
            SG_LOG( SG_GENERAL, SG_ALERT, "Opening datasource " << pathname << " for reading." );
            poDS = (GDALDataset*)GDALOpenEx( pathname, GDAL_OF_VECTOR, NULL, NULL, NULL );

            if( poDS != NULL ) {
                OGRLayer  *poLayer;
                for (int j=0;j<poDS->GetLayerCount();j++) {
                    poLayer = poDS->GetLayer(j);
                    processLayer(poLayer, bucket, results, i, pig );
                }
                
                GDALClose( poDS );
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening datasource " << pathname );
            }
        }        
        
        // add some additional Variables to the intersection generator
        // cleaning parameters
        // texture mode
        // simplify parameters
        // and add some data access
        // get skeleton segments
        // get skin segments
        // delta height info may be needed....
        // maybe needs a new class entirely based on intersectiongenerator.
        
        // we have all of the data - execute the intersection generator
        // don't clean the OSM map data - as we don't want to generate intersections
        // that don't really exist ( bridges and tunnels )
        // OSM data should have correct intersection nodes already.
        // - they need them to do routing.        
        // lockOgr->lock();
        pig->Execute();
        
        // now retreive the polygons in reverse z-order.  store them in lists
        std::map<int, tgpolygon_list*> polygons;
        
        for ( tgintersectionedge_it it = pig->edges_begin(); it != pig->edges_end(); it++ ) {
            tgPolygon poly = (*it)->GetPoly("complete");
            int       zo   = (*it)->GetZorder();
            
            if ( polygons.find( zo ) == polygons.end() ) {
                // add new polygon list at this zorder
                polygons[zo] = new tgpolygon_list;
            }
            polygons[zo]->push_back( poly );
        }
        
        // clip them in z order
        tgAccumulator accum;    
        std::map<int, tgpolygon_list*>::reverse_iterator pmap_it;
        
        std::cout << " start clip " << std::endl;
        
        for ( pmap_it = polygons.rbegin(); pmap_it != polygons.rend(); pmap_it++ ) {
            std::vector<tgPolygon>::iterator poly_it;
            
            std::cout << " clip zorder " << (*pmap_it).first << std::endl;
            
            //unsigned int num_polys = (*pmap_it).second->size();
            //unsigned int p = 1;
            
            for ( poly_it = (*pmap_it).second->begin(); poly_it != (*pmap_it).second->end(); poly_it++ ) {
                tgPolygon current = (*poly_it);
                current.RemoveDups();

                accum.Diff_and_Add_cgal( current );
                
                // only add to output list if the clip left us with a polygon
                if ( current.Contours() > 0 ) {
                    results.Add( current, current.GetMaterial() );
                }
            }
        }
        
        results.Save(false);
        
        delete pig;
    }
    
    
    
    
    
    
    
    return 0;
}
