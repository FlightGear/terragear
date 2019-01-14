// cliff-decode.cxx -- process OGR layers and extract lines
//                   representing discontinuities, clipping
//                   against and sorting
//                   them into the relevant tiles.
//
// Written by James Hester 2018
//
// Largely copied from ogr-decode by Ralf Gerlich.
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
#include <chrono>
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

#include <Include/version.h>

#include <terragear/tg_chopper.hxx>
#include <terragear/tg_shapefile.hxx>

using std::string;

int continue_on_errors=0;
int num_threads=1;
bool use_attribute_query=false;
string attribute_query;
int start_record=0;
bool use_spatial_query=false;
int seperate_segments = 0;
double spat_min_x, spat_min_y, spat_max_x, spat_max_y;
bool save_shapefiles=false;
SGLockedQueue<OGRFeature *> global_workQueue;

class Decoder : public SGThread
{
public:
    Decoder( OGRCoordinateTransformation *poct, tgChopper& c ) : chopper(c) {
        poCT = poct;
    }

private:
    virtual void run();

    void processLineString(OGRLineString* poGeometry);

private:
    // The transformation for each geometry object
    OGRCoordinateTransformation *poCT;

    // Store the reults per tile
    tgChopper& chopper;

};

void Decoder::processLineString(OGRLineString* poGeometry)
{
    tgContour line;

    SGGeod p0, p1;
    double heading, dist, az2;
    int i, j, numPoints, numSegs;
    double max_dist;

    numPoints = poGeometry->getNumPoints();
    if (numPoints < 2) {
        SG_LOG( SG_GENERAL, SG_WARN, "Skipping line with less than two points" );
        return;
    }

    heading = SGGeodesy::courseDeg( p1, p0 );

    // now add the middle points : if they are too far apart, add intermediate nodes
    for ( i=0;i<numPoints;i++) {
        p0 = SGGeod::fromDeg( poGeometry->getX(i), poGeometry->getY(i) );
        line.AddNode( p0 );
    }

    // Do not close this contour
    line.SetOpen(true);
    
    // clip the contour.
    // TODO: don't bother clipping, as the contour is informational
    // only, and simply output to all relevant buckets instead.
    tgPolygon open_poly;
    open_poly.SetOpen();   //do not try to close this one up
    open_poly.AddContour(line);
    chopper.Add( open_poly, "Cliffs" );
}

void Decoder::run()
{
    // as long as we have geometry to parse, do so
    while (!global_workQueue.empty()) {
        OGRFeature *poFeature = global_workQueue.pop();
        SG_LOG( SG_GENERAL, SG_BULK, " remaining features is " << global_workQueue.size() );

        if ( poFeature ) {
            OGRGeometry *poGeometry = poFeature->GetGeometryRef();

            if (poGeometry==NULL) {
                SG_LOG( SG_GENERAL, SG_INFO, "Found feature without geometry!" );
                if (!continue_on_errors) {
                    SG_LOG( SG_GENERAL, SG_ALERT, "Aborting!" );
                    exit( 1 );
                } else {
                    continue;
                }
            }

            OGRwkbGeometryType geoType=wkbFlatten(poGeometry->getGeometryType());
            if (geoType!=wkbLineString && geoType!=wkbMultiLineString) {
                    SG_LOG( SG_GENERAL, SG_INFO, "Non-line feature" );
                    continue;
            }

            poGeometry->transform( poCT );

            switch (geoType) {
            case wkbLineString: {
                SG_LOG( SG_GENERAL, SG_DEBUG, "LineString feature" );
                processLineString((OGRLineString*)poGeometry);
                break;
            }
            case wkbMultiLineString: {
                SG_LOG( SG_GENERAL, SG_DEBUG, "MultiLineString feature" );
                OGRMultiLineString* multilines=(OGRMultiLineString*)poGeometry;
                for (int i=0;i<multilines->getNumGeometries();i++) {
                    processLineString((OGRLineString*)(multilines->getGeometryRef(i)));
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
void processLayer(OGRLayer* poLayer, tgChopper& results )
{
    int feature_count=poLayer->GetFeatureCount();

    if (feature_count!=-1 && start_record>0 && start_record>=feature_count) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Layer has only " << feature_count << " records, but start record is set to " << start_record );
        exit( 1 );
    }

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

    /* setup attribute and spatial queries */
    if (use_spatial_query) {
        double trans_min_x,trans_min_y,trans_max_x,trans_max_y;
        /* do a simple reprojection of the source SRS */
        OGRCoordinateTransformation *poCTinverse;

        poCTinverse = OGRCreateCoordinateTransformation(&oTargetSRS, oSourceSRS);

        trans_min_x=spat_min_x;
        trans_min_y=spat_min_y;
        trans_max_x=spat_max_x;
        trans_max_y=spat_max_y;

        poCTinverse->Transform(1,&trans_min_x,&trans_min_y);
        poCTinverse->Transform(1,&trans_max_x,&trans_max_y);

        poLayer->SetSpatialFilterRect(trans_min_x, trans_min_y,
                                      trans_max_x, trans_max_y);
    }

    if (use_attribute_query) {
        if (poLayer->SetAttributeFilter(attribute_query.c_str()) != OGRERR_NONE) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Error in query expression '" << attribute_query << "'" );
            exit( 1 );
        }
    }

    // Generate the work queue for this layer
    OGRFeature *poFeature;
    poLayer->SetNextByIndex(start_record);
    while ( ( poFeature = poLayer->GetNextFeature()) != NULL )
    {
        global_workQueue.push( poFeature );
    }

    // Now process the workqueue with threads
    std::vector<Decoder *> decoders;
    for (int i=0; i<num_threads; i++) {
        Decoder* decoder = new Decoder( poCT, results );
        decoder->start();
        decoders.push_back( decoder );
    }

    // Then wait until they are finished
    for (unsigned int i=0; i<decoders.size(); i++) {
        decoders[i]->join();
    }

    OCTDestroyCoordinateTransformation ( poCT );
}

void usage(char* progname) {
    SG_LOG( SG_GENERAL, SG_ALERT, "Usage: " <<
              progname << " [options...] <work_dir> <datasource> [<layername>...]" );
    SG_LOG( SG_GENERAL, SG_ALERT, "Options:" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--log-level priority" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Width in priority being bulk|debug|info|warn|alert" );
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
    SG_LOG( SG_GENERAL, SG_ALERT, "--threads" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Enable multithreading with user specified number of threads" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--all-threads" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Enable multithreading with all available cpu cores" );
    SG_LOG( SG_GENERAL, SG_ALERT, "" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<work_dir>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Directory to put the cliff files in" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<datasource>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        The datasource from which to fetch the data" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<layername>..." );
    SG_LOG( SG_GENERAL, SG_ALERT, "        The layers to process." );
    SG_LOG( SG_GENERAL, SG_ALERT, "        If no layer is given, all layers in the datasource are used" );
    exit(-1);
}

void
setLoggingPriority (const char * p)
{
  if (p == 0)
      return;
  string priority = p;
  if (priority == "bulk") {
    sglog().set_log_priority(SG_BULK);
  } else if (priority == "debug") {
    sglog().set_log_priority(SG_DEBUG);
  } else if (priority.empty() || priority == "info") { // default
    sglog().set_log_priority(SG_INFO);
  } else if (priority == "warn") {
    sglog().set_log_priority(SG_WARN);
  } else if (priority == "alert") {
    sglog().set_log_priority(SG_ALERT);
  } else {
    SG_LOG(SG_GENERAL, SG_WARN, "Unknown logging priority " << priority);
  }
}

int main( int argc, char **argv ) {
    char* progname=argv[0];
    string datasource,work_dir;

    auto start_time = std::chrono::high_resolution_clock::now();

    sglog().setLogLevels( SG_ALL, SG_INFO );

    while (argc>1) {
        if (!strcmp(argv[1],"--log-level")) {
            if (argc<3) {
                usage(progname);
            }
            setLoggingPriority(argv[2]);
            argv+=2;
            argc-=2;
        }  else if (!strcmp(argv[1],"--seperate-segments")) {
            argv++;
            argc--;
            seperate_segments=1;
        } else if (!strcmp(argv[1],"--continue-on-errors")) {
            argv++;
            argc--;
            continue_on_errors=1;
        } else if (!strcmp(argv[1],"--start-record")) {
            if (argc<3) {
                usage(progname);
            }
            start_record=atoi(argv[2]);
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--where")) {
            if (argc<3) {
                usage(progname);
            }
            use_attribute_query=true;
            attribute_query=argv[2];
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--spat")) {
            if (argc<6) {
                usage(progname);
            }
            use_spatial_query=true;
            spat_min_x=atof(argv[2]);
            spat_min_y=atof(argv[3]);
            spat_max_x=atof(argv[4]);
            spat_max_y=atof(argv[5]);
            argv+=5;
            argc-=5;
        } else if (!strcmp(argv[1],"--threads")) {
            if (argc<3) {
                usage(progname);
            }
            num_threads=atoi(argv[2]);
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--all-threads")) {
            num_threads=boost::thread::hardware_concurrency(); 
            argv+=1;
            argc-=1;
        } else if (!strcmp(argv[1],"--debug")) {
            argv++;
            argc--;
            save_shapefiles=true;
        } else if (!strcmp(argv[1],"--help")) {
            usage(progname);
        } else {
            break;
        }
    }

    SG_LOG( SG_GENERAL, SG_ALERT, "\ncliff-decode version " << getTGVersion() );

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

    poDS = (GDALDataset*) GDALOpenEx( datasource.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening datasource " << datasource );
        return EXIT_FAILURE;
    }

    SG_LOG( SG_GENERAL, SG_ALERT, "Processing datasource " << datasource );

    OGRLayer  *poLayer;
    if (argc>3) {
        for (int i=3;i<argc;i++) {
            poLayer = poDS->GetLayerByName( argv[i] );

            if (poLayer == NULL )
            {
                SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening layer " << argv[i] << " from datasource " << datasource );
                return EXIT_FAILURE;
            }
            processLayer(poLayer, results );
        }
    } else {
        for (int i=0;i<poDS->GetLayerCount();i++) {
            poLayer = poDS->GetLayer(i);

            assert(poLayer != NULL);

            processLayer(poLayer, results );
        }
    }

    GDALClose(poDS);

    SG_LOG(SG_GENERAL, SG_ALERT, "Saving to buckets");
    results.Add_Extension("cliffs");
    results.Save( save_shapefiles );

    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;
    std::cout << std::endl << "Elapsed time: " << elapsed.count() << " seconds" << std::endl << std::endl;

 
    return 0;
}


