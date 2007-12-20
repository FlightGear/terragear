// poly2ogr.cxx -- Translate the polygon definitions in a given TerraGear
//                 working directory to vector data, writing it out using
//                 the OGR library.
//
// Written by Ralf Gerlich, started December 2007.
//
// Copyright (C) 2007  Ralf Gerlich     - ralf.gerlich@custom-scenery.org
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

#include <simgear/compiler.h>

#include STL_STRING
#include <map>

#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <dirent.h>
#include <unistd.h>

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>

#include <Polygon/polygon.hxx>
#include <Polygon/point2d.hxx>

#include <ogrsf_frmts.h>

typedef std::map<std::string,OGRLayer*> LayerMap;

const char* format_name="ESRI Shapefile";
bool do_split=false;

OGRDataSource *datasource;
OGRLayer *defaultLayer;
LayerMap layerMap;

bool endswith(const std::string& s, const std::string& suffix) {
        size_t slen,sufflen;
        slen=s.size();
        sufflen=suffix.size();
        if (slen<sufflen)
                return false;
        return s.compare(slen-sufflen,sufflen,suffix)==0;
}

OGRLayer* create_layer(const std::string& material) {
        OGRLayer* layer;
        
        OGRSpatialReference srs;
        srs.SetWellKnownGeogCS("WGS84");
        layer=datasource->CreateLayer(material.c_str(),&srs,wkbPolygon25D,NULL);
        if (!layer) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Creation of layer '" << material << "' failed");
                return NULL;
        }
        
        OGRFieldDefn materialField("Material", OFTString);
        materialField.SetWidth(128);
        
        OGRFieldDefn fileField("File",OFTString);
        fileField.SetWidth(256);
        
        if( layer->CreateField( &materialField ) != OGRERR_NONE ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Creation of field 'Material' failed");
        }
        
        if( layer->CreateField( &fileField ) != OGRERR_NONE ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Creation of field 'Material' failed");
        }
        
        return layer;
}

OGRLayer* get_layer_for_material(const std::string& material) {
        if (!do_split) {
                if (!defaultLayer) {
                        defaultLayer=create_layer("default");
                }
                return defaultLayer;
        }
        
        OGRLayer* layer;
        LayerMap::iterator it=layerMap.find(material);
        if (it==layerMap.end()) {
                layer=create_layer(material);
                if (!layer)
                        return NULL;
                layerMap[material]=layer;
        } else {
                layer=(*it).second;
        }
        return layer;
}

void process_polygon_file(const std::string& path) {
        SG_LOG(SG_GENERAL, SG_INFO, "Loading polygon file " << path);
        
        sg_gzifstream in( path );
        
        while (!in.eof()) {
                string first_line,material;
                bool poly3d=false;
                in >> first_line;
                if ( first_line == "#2D" ) {
                    poly3d = false;
                    in >> material;
                } else if ( first_line == "#3D" ) {
                    poly3d = true;
                    in >> material;
                } else {
                    // support old format (default to 2d)
                    poly3d = false;
                    material=first_line;
                }
                
                int contours;
                in >> contours;
                
                OGRPolygon* polygon=new OGRPolygon();
                
                for (int contour=0;contour<contours;contour++) {
                        int count,hole_flag;
                        bool skip_ring=false;
                        
                        in >> count;
                        
                        if (count<3) {
                                SG_LOG(SG_GENERAL, SG_ALERT, "Polygon with less than 3 points");
                                skip_ring=true;
                        }
                        
                        in >> hole_flag;
                        
                        // FIXME: Current we ignore the hole-flag and instead assume
                        //        that the first ring is not a hole and the rest
                        //        are holes
                        
                        OGRLinearRing *ring=new OGRLinearRing();
                        
                        for (int pt=0;pt<count;pt++) {
                                OGRPoint *point=new OGRPoint();
                                double x,y,z;
                                
                                in >> x >> y;
                                point->setX(x);
                                point->setY(y);
                                if (poly3d) {
                                        in >> z;
                                        point->setZ(z);
                                } else {
                                        point->setZ(0.0);
                                }
                                
                                ring->addPoint(point);
                        }
                        
                        ring->closeRings();
                        
                        if (!skip_ring)
                                polygon->addRingDirectly(ring);
                }
                
                OGRLayer* layer=get_layer_for_material(material);
                OGRFeature* feature;
                
                feature = new OGRFeature( layer->GetLayerDefn() );
                feature->SetField("Material", material.c_str());
                feature->SetField("File", path.c_str());
                feature->SetGeometry(polygon);
                
                if( layer->CreateFeature( feature ) != OGRERR_NONE )
                {
                        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
                }
                
                OGRFeature::DestroyFeature(feature);
        }
}

void process_file(const std::string& path) {
        struct stat sbuf;
        
        if ( stat(path.c_str(),&sbuf) != 0 ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Unable to stat path '" << path << "'");
                return;
        }
        
        if (S_ISDIR(sbuf.st_mode)) {
                DIR* dir;
                
                dir=opendir(path.c_str());
                if (!dir) {
                        SG_LOG(SG_GENERAL, SG_ALERT, "Unable to open directory '" << path << "'");
                        return;
                }
                
                struct dirent *de;
                
                while ((de=readdir(dir))) {
                        if (!strcmp(de->d_name,".") || !strcmp(de->d_name,"..")) {
                                continue;
                        }
                        
                        std::string subpath=path+"/"+de->d_name;
                        process_file(subpath);
                }
                
                closedir(dir);
        } else if (!endswith(path,".gz") &&
                        !endswith(path,".arr") &&
                        !endswith(path,".fit") &&
                        !endswith(path,".btg") &&
                        !endswith(path,".stg") &&
                        !endswith(path,".ind")) {
                // should be a polygon file
                process_polygon_file(path);
        }
}

void usage(const char* progname, const std::string& msg) {
        if (msg.size()!=0)
                SG_LOG(SG_GENERAL,SG_ALERT, msg);
        SG_LOG(SG_GENERAL, SG_INFO, "Usage: " << progname << " [options] dst_datasource path...");
        SG_LOG(SG_GENERAL, SG_INFO, "");
        SG_LOG(SG_GENERAL, SG_INFO, "Options:");
        SG_LOG(SG_GENERAL, SG_INFO, "\t-h");
        SG_LOG(SG_GENERAL, SG_INFO, "\t--help");
        SG_LOG(SG_GENERAL, SG_INFO, "\t\tShow this help screen");
        SG_LOG(SG_GENERAL, SG_INFO, "");
        SG_LOG(SG_GENERAL, SG_INFO, "\t-v");
        SG_LOG(SG_GENERAL, SG_INFO, "\t--version");
        SG_LOG(SG_GENERAL, SG_INFO, "\t\tShow the version");
        SG_LOG(SG_GENERAL, SG_INFO, "");
        SG_LOG(SG_GENERAL, SG_INFO, "\t-s");
        SG_LOG(SG_GENERAL, SG_INFO, "\t--split");
        SG_LOG(SG_GENERAL, SG_INFO, "\t\tCreate one layer per material");
        SG_LOG(SG_GENERAL, SG_INFO, "");
        SG_LOG(SG_GENERAL, SG_INFO, "\t-f format");
        SG_LOG(SG_GENERAL, SG_INFO, "\t--format format");
        SG_LOG(SG_GENERAL, SG_INFO, "\t\tSpecify the output format");
        SG_LOG(SG_GENERAL, SG_INFO, "\t\tAvailable formats:");
        OGRSFDriverRegistrar* registrar=OGRSFDriverRegistrar::GetRegistrar();
        for (int i=0;i<registrar->GetDriverCount();i++) {
                SG_LOG(SG_GENERAL, SG_INFO, "\t\t\t-f \"" << registrar->GetDriver(i)->GetName() << "\"");
        }
        SG_LOG(SG_GENERAL, SG_INFO, "\t\tDefault: ESRI Shapefile");
        SG_LOG(SG_GENERAL, SG_INFO, "");
        SG_LOG(SG_GENERAL, SG_INFO, "The polygons from the given paths are read and transferred");
        SG_LOG(SG_GENERAL, SG_INFO, "to layers in the given destination datasource.");
        SG_LOG(SG_GENERAL, SG_INFO, "");
        SG_LOG(SG_GENERAL, SG_INFO, "If one of the paths is a directory, the files in this directory");
        SG_LOG(SG_GENERAL, SG_INFO, "and its subdirectories are read.");
}

struct option options[]={
        {"help",no_argument,NULL,'h'},
        {"version",no_argument,NULL,'v'},
        {"split",no_argument,NULL,'s'},
        {"format",required_argument,NULL,'f'},
        {NULL,0,NULL,0}
};

int main(int argc, char** argv) {
        sglog().setLogLevels( SG_ALL, SG_DEBUG );
        
        OGRRegisterAll();
        
        int option;
        
        while ((option=getopt_long(argc,argv,"hvsf:",options,NULL))!=-1) {
                switch (option) {
                case 'h':
                        usage(argv[0],"");
                        break;
                case 'f':
                        format_name=optarg;
                        break;
                case 's':
                        do_split=true;
                        break;
                case 'v':
                        SG_LOG(SG_GENERAL,SG_INFO,argv[0] << " Version 1.0");
                        exit(0);
                        break;
                case '?':
                        usage(argv[0],std::string("Unknown option:")+(char)optopt);
                        exit(1);
                }
        }
        
        if (optind+1>argc) {
                usage(argv[0],"A datasource must be specified");
                exit(1);
        }
        
        if (optind+2>argc) {
                usage(argv[0],"At least one input file must be specified");
                exit(1);
        }
        
        const char* dst_datasource=argv[optind++];
        OGRSFDriver *ogrdriver;
        
        ogrdriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(format_name);
        if (!ogrdriver) {
                usage(argv[0],std::string("Unknown datasource format driver:")+format_name);
                exit(1);
        }
        
        datasource = ogrdriver->CreateDataSource(dst_datasource,NULL);
        if (!datasource) {
                usage(argv[0],std::string("Unable to create datasource:")+dst_datasource);
                exit(1);
        }
        
        for (int i=optind;i<argc;i++) {
                process_file(argv[i]);
        }
        
        OGRDataSource::DestroyDataSource( datasource );
        
        return 0;
}
