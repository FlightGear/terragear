// ogr-decode.cxx -- process OGR layers and extract polygon outlines,
//                   lines and points, clipping against and sorting
//                   them into the relevant tiles.
//
// Written by Ralf Gerlich, started February 2007.
//
// Copyright (C) 2007  Ralf Gerlich     - ralf.gerlich@custom-scenery.org
// Loosely based on shape-decode.cxx by Curtis L. Olson
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

#include <boost/thread.hpp>
#include <ogrsf_frmts.h>
#include <gdal_priv.h>

#include <simgear/compiler.h>
#include <simgear/threads/SGThread.hxx>
#include <simgear/threads/SGQueue.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/timing/timestamp.hxx>

#include <Include/version.h>

#include <terragear/polygon_set/tg_polygon_accumulator.hxx>
#include <terragear/polygon_set/tg_polygon_set.hxx>
#include <terragear/polygon_set/tg_polygon_chop.hxx>

using std::string;

string area_type="Default";
int num_threads = 1;

std::vector<SGBucket> bucketList;
SGLockedQueue<OGRFeature *> global_workQueue;

class Decoder : public SGThread
{
public:
    Decoder( OGRCoordinateTransformation *poct, tgChopper& c, tgPolygonSetList& all, SGMutex& l ) : chopper(c), allPolys(all), lock(l) {
        poCT      = poct;
    }

private:
    virtual void run();

    void processPolygon(OGRFeature *poFeature, OGRPolygon* poGeometry, const string& area_type );

private:
    // The transformation for each geometry object
    OGRCoordinateTransformation *poCT;

    // Store the reults per tile
    tgChopper&          chopper;
    
    // and in an array
    tgPolygonSetList&   allPolys; 

    int                 area_type_field;
    SGMutex&            lock;
};

void Decoder::processPolygon(OGRFeature *poFeature, OGRPolygon* poGeometry, const string& area_type )
{
    SGTimeStamp create_start, create_end, create_time;
    create_start.stamp();

    // generate metadata info from GDAL feature info
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, area_type );
    tgPolygonSet shapes( poGeometry, meta );
    
    // add the shape to allPolys
    lock.lock();
    allPolys.push_back( shapes );
    lock.unlock();
    
    create_end.stamp();
    create_time = create_end - create_start;
    
    chopper.Add( shapes, create_time  );
}

void Decoder::run()
{
    // as long as we have geometry to parse, do so
    while (!global_workQueue.empty()) {
        OGRFeature *poFeature = global_workQueue.pop();
        // SG_LOG( SG_GENERAL, SG_INFO, " chopping feature with thread " << current() << " remaining features is " << global_workQueue.size() );

        if ( poFeature ) {
            OGRGeometry *poGeometry = poFeature->GetGeometryRef();

            if (poGeometry==NULL) {
                SG_LOG( SG_GENERAL, SG_INFO, "Found feature without geometry! - Aborting!" );
                exit( 1 );
            }

            OGRwkbGeometryType geoType=wkbFlatten(poGeometry->getGeometryType());
            if (geoType!=wkbPolygon && geoType!=wkbMultiPolygon) {
                SG_LOG( SG_GENERAL, SG_INFO, "Unknown feature" );
                OGRFeature::DestroyFeature( poFeature );

                continue;
            }

            string area_type_name=area_type;
            poGeometry->transform( poCT );

            switch (geoType) {
            case wkbPolygon: {
                SG_LOG( SG_GENERAL, SG_DEBUG, "Polygon feature" );
                processPolygon(poFeature, (OGRPolygon*)poGeometry, area_type_name);
                break;
            }
            case wkbMultiPolygon: {
                SG_LOG( SG_GENERAL, SG_DEBUG, "MultiPolygon feature" );
                OGRMultiPolygon* multipoly=(OGRMultiPolygon*)poGeometry;
                for (int i=0;i<multipoly->getNumGeometries();i++) {
                    processPolygon(poFeature, (OGRPolygon*)(multipoly->getGeometryRef(i)), area_type_name);
                }
                break;
            }
            default:
                /* Ignore unhandled objects */
                break;
            }

            OGRFeature::DestroyFeature( poFeature );
        }
    }
}

// Main Thread
void processLayer(OGRLayer* poLayer, tgChopper& chopped, std::vector<SGBucket>& bucketList, tgPolygonSetList& all, SGMutex& l  )
{
    /* determine the indices of the required columns */
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    string layername=poFDefn->GetName();

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

    OGRCoordinateTransformation *poCT = OGRCreateCoordinateTransformation(oSourceSRS, &oTargetSRS);

    OGRFeature *poFeature;
    poLayer->SetNextByIndex(0);
    
    // get the extents of the layer
    OGREnvelope extents;
    poLayer->GetExtent(&extents);
    SGGeod gMin = SGGeod::fromDeg( extents.MinX, extents.MinY );
    SGGeod gMax = SGGeod::fromDeg( extents.MaxX, extents.MaxY );
    sgGetBuckets( gMin, gMax, bucketList );
    
    // Generate the work queue for this layer
    while ( ( poFeature = poLayer->GetNextFeature()) != NULL )
    {
        global_workQueue.push( poFeature );
    }

    // Now process the workqueue with threads
    // this just generates all the tgPolygons
    std::vector<Decoder *> decoders;
    for (int i=0; i<num_threads; i++) {
        Decoder* decoder = new Decoder( poCT, chopped, all, l );
        decoder->start();
        decoders.push_back( decoder );
    }

    // Then wait until they are finished
    for (unsigned int i=0; i<decoders.size(); i++) {
        decoders[i]->join();
    }
    
    // now we have all of the shapes.
    
    OCTDestroyCoordinateTransformation ( poCT );
}

void usage(char* progname) {
    SG_LOG( SG_GENERAL, SG_ALERT, "Usage: " <<
              progname << " [options...] <work_dir> <datasource> [<layername>...]" );
    SG_LOG( SG_GENERAL, SG_ALERT, "Options:" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--area-type type" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Area type for all objects from file" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--threads" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Enable multithreading with user specified number of threads" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<work_dir>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Directory to put the polygon files in" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<datasource>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        The datasource from which to fetch the data" );
    exit(-1);
}

int main( int argc, char **argv ) {
    char*                   progname=argv[0];
    string                  datasource,work_dir;
    std::vector<SGBucket>   bucketList;
    tgPolygonSetList        shapefilePolys;
    SGMutex                 lock;

    sglog().setLogLevels( SG_ALL, SG_INFO );

    while (argc>1) {
        if (!strcmp(argv[1],"--area-type")) {
            if (argc<3) {
                usage(progname);
            }
            area_type=argv[2];
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--help")) {
            usage(progname);
        } else {
            break;
        }
    }

    SG_LOG( SG_GENERAL, SG_ALERT, "\npoly-decode version " << getTGVersion() );

    if (argc<3) {
        usage(progname);
    }
    work_dir=argv[1];
    datasource=argv[2];

    SGPath sgp( work_dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    tgChopper results( work_dir );

    SG_LOG( SG_GENERAL, SG_DEBUG, "Opening datasource " << datasource << " for reading." );

    GDALAllRegister();
    GDALDataset       *poDS;

    poDS = (GDALDataset*)GDALOpenEx( datasource.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening datasource " << datasource );
        exit( 1 );
    }

    SG_LOG( SG_GENERAL, SG_ALERT, "Processing datasource " << datasource );

    OGRLayer  *poLayer;
    if (argc>3) {
        for (int i=3;i<argc;i++) {
            poLayer = poDS->GetLayerByName( argv[i] );

            if (poLayer == NULL )
            {
                SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening layer " << argv[i] << " from datasource " << datasource );
                exit( 1 );
            }
            processLayer(poLayer, results, bucketList, shapefilePolys, lock );
        }
    } else {
        for (int i=0;i<poDS->GetLayerCount();i++) {
            poLayer = poDS->GetLayer(i);

            assert(poLayer != NULL);

            processLayer(poLayer, results, bucketList, shapefilePolys, lock );
        }
    }

    GDALClose(poDS);

    // first, join all the polys
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, area_type );
    tgPolygonSet     allShapefilePolys = tgPolygonSet::join( shapefilePolys, meta );
    
    // now load the chopped shapefiles into a new shape
    tgPolygonSetList choppedPolys;
    
    for ( unsigned int i=0; i<bucketList.size(); i++ ) {
        // open the shapefile in the bucket
        std::string poly_path;
        SGBucket    bucket = bucketList[i];
        tgPolygonSetList polys;
        
        // load 2D polygons from correct path
        poly_path = work_dir + "/" + bucket.gen_base_path() + '/' + bucket.gen_index_str() + '/' + area_type + ".shp";
        tgPolygonSet::fromShapefile( poly_path, polys );
        
        for ( unsigned int j=0; j<polys.size(); j++ ) {
            choppedPolys.push_back( polys[j] );
        }
    }
    
    tgPolygonSet allChoppedPolys = tgPolygonSet::join( choppedPolys, meta );
    
    // allChoppedPolys should be equal to allShapefilePolys
    // i.e. the symetric difference should be empty
    tgPolygonSet resultPolys = tgPolygonSet::symmetricDifference( allChoppedPolys, allShapefilePolys, meta );
    
    if ( !resultPolys.isEmpty() ) {
        resultPolys.toShapefile( "./result", "difference" );
    }
    
    
    return 0;
}
