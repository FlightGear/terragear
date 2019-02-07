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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.

#include <simgear/compiler.h>

#include <string>
#include <map>

#include <sys/types.h>
#include <sys/stat.h>
#ifdef _MSC_VER
#  define S_ISDIR(a)	((a)&_S_IFDIR)
#  include <Prep/Terra/getopt.h>
#else
#  include <getopt.h>
#  include <unistd.h>
#endif

#include <ogrsf_frmts.h>

#include <simgear/debug/logstream.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/io/iostreams/sgstream.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/misc/sg_dir.hxx>

#include <terragear/tg_polygon.hxx>

using std::string;

typedef std::map<std::string,OGRLayer*> LayerMap;

const char* format_name="ESRI Shapefile";
bool do_split=false;

GDALDataset *datasource;
OGRLayer *defaultLayer;
OGRLayer *pointsLayer=NULL;
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

OGRLayer* create_pointsLayer() {
        OGRLayer* layer;

        OGRSpatialReference srs;
        srs.SetWellKnownGeogCS("WGS84");
        layer=datasource->CreateLayer("points",&srs,wkbPoint,NULL);
        if (!layer) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Creation of layer 'points' failed");
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

OGRLinearRing* make_ring_from_fan(const int_list& fan, const std::vector<SGGeod>& nodes) {
        OGRLinearRing* ring = new OGRLinearRing();
        int_list::const_iterator vertex = fan.begin();
        
        if (fan[1]==fan[fan.size()-1]) {
                /* The fan is closed, so the first vertex is in the interior */
                ++vertex;
        }

        for (;vertex!=fan.end();++vertex) {
                OGRPoint *point=new OGRPoint();
                const SGGeod& node = nodes[*vertex];
                point->setX(node.getLongitudeDeg());
                point->setY(node.getLatitudeDeg());
                point->setZ(node.getElevationM());

                ring->addPoint(point);
        }

        ring->closeRings();

        return ring;
}

OGRLinearRing* make_ring_from_strip(const int_list& strip, const std::vector<SGGeod>& nodes) {
        OGRLinearRing* ring = new OGRLinearRing();
        const size_t vertex_count = strip.size();
        unsigned int i;
        for (i=0;i<vertex_count;i+=2) {
                OGRPoint *point=new OGRPoint();
                const SGGeod& node = nodes[strip[i]];
                point->setX(node.getLongitudeDeg());
                point->setY(node.getLatitudeDeg());
                point->setZ(node.getElevationM());

                ring->addPoint(point);
        }
        for (i--;i>0;i-=2) {
                OGRPoint *point=new OGRPoint();
                const SGGeod& node = nodes[strip[i]];
                point->setX(node.getLongitudeDeg());
                point->setY(node.getLatitudeDeg());
                point->setZ(node.getElevationM());

                ring->addPoint(point);
        }
        std::cout << "\n";

        ring->closeRings();

        return ring;
}

void make_feature_from_polygon(OGRPolygon* polygon, const std::string& material, const std::string& path) {
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

void make_feature_from_ring(OGRLinearRing* ring, const std::string& material, const std::string& path) {
        OGRPolygon* polygon = new OGRPolygon();
        polygon->addRingDirectly(ring);

        make_feature_from_polygon(polygon, material, path);
}

void convert_triangles(const std::string& path, const group_list& verts, const string_list& materials, const std::vector<SGGeod>& wgs84_nodes) {
        const size_t groups_count = verts.size();

        for (unsigned int i=0;i<groups_count;i++) {
                const string& material = materials[i];
                const int_list& tri_verts = verts[i];
                const size_t vertices = tri_verts.size();
                for (unsigned int j=0;j<vertices;j+=3) {
                        OGRLinearRing* ring = new OGRLinearRing();
                        for (int k=0;k<3;k++) {
                                OGRPoint *point=new OGRPoint();
                                const SGGeod& node = wgs84_nodes[tri_verts[j+k]];
                                point->setX(node.getLongitudeDeg());
                                point->setY(node.getLatitudeDeg());
                                point->setZ(node.getElevationM());

                                ring->addPoint(point);
                        }
                        ring->closeRings();
                        make_feature_from_ring(ring, material, path);
                }
        }
}

void convert_triangle_fans(const std::string& path, const group_list& verts, const string_list& materials, const std::vector<SGGeod>& wgs84_nodes) {
        const size_t groups_count = verts.size();

        for (unsigned int i=0;i<groups_count;i++) {
                const string& material = materials[i];
                OGRLinearRing* ring = make_ring_from_fan(verts[i], wgs84_nodes);
                make_feature_from_ring(ring, material, path);
        }
}

void convert_triangle_strips(const std::string& path, const group_list& verts, const string_list& materials, const std::vector<SGGeod>& wgs84_nodes) {
        const size_t groups_count = verts.size();

        for (unsigned int i=0;i<groups_count;i++) {
                const string& material = materials[i];
                OGRLinearRing* ring = make_ring_from_strip(verts[i], wgs84_nodes);
                make_feature_from_ring(ring, material, path);
        }
}

void process_scenery_file(const std::string& path) {
        SGBinObject binObject;
        if (!binObject.read_bin(path)) {
                return;
        }

        SGVec3d gbs_center = binObject.get_gbs_center();
        const std::vector<SGVec3d>& wgs84_nodes = binObject.get_wgs84_nodes();
        std::vector<SGGeod> geod_nodes;
        const size_t node_count = wgs84_nodes.size();
        for (unsigned int i=0;i<node_count;i++) {
                SGVec3d wgs84 = wgs84_nodes[i];
                SGVec3d raw = SGVec3d( gbs_center.x() + wgs84.x(),
                                       gbs_center.y() + wgs84.y(),
                                       gbs_center.z() + wgs84.z() );
                SGGeod geod = SGGeod::fromCart( raw );
                geod_nodes.push_back(geod);
        }

        /* Convert individual triangles */
        convert_triangles(path,
                binObject.get_tris_v(),
                binObject.get_tri_materials(),
                geod_nodes);

        /* Convert triangle fans */
        convert_triangle_fans(path,
                binObject.get_fans_v(),
                binObject.get_fan_materials(),
                geod_nodes);

        /* Convert triangle strips */
        convert_triangle_strips(path,
                binObject.get_strips_v(),
                binObject.get_strip_materials(),
                geod_nodes);
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
                                double x, y;
                                in >> x >> y;
                                point->setX(x);
                                point->setY(y);

                                if (poly3d) {
                                        double z;
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

                make_feature_from_polygon(polygon, material, path);
        }
}

void process_points_file(const std::string& path) {
        SG_LOG(SG_GENERAL, SG_INFO, "Loading points file " << path);

        sg_gzifstream in( path );

        if (pointsLayer==NULL)
        {
                pointsLayer=create_pointsLayer();
        }

        while (!in.eof()) {
                std::string material;
                double x,y;
                in >> x >> y >> material;

                if (in.eof())
                        break;

                OGRPoint* point=new OGRPoint(x,y);

                OGRFeature* feature;
                feature = new OGRFeature( pointsLayer->GetLayerDefn() );
                feature->SetField("Material", material.c_str());
                feature->SetField("File", path.c_str());
                feature->SetGeometry(point);

                if( pointsLayer->CreateFeature( feature ) != OGRERR_NONE )
                {
                        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
                }

                OGRFeature::DestroyFeature(feature);
        }
}

void process_file(const SGPath& path)
{
    if (path.isDir()) {
    // recurse downwards!
        simgear::Dir d(path);
        int flags = simgear::Dir::TYPE_FILE | simgear::Dir::TYPE_DIR |
            simgear::Dir::NO_DOT_OR_DOTDOT;
        for (const SGPath& c : d.children(flags)) {
            process_file(c);
        }

        return;
    }

    string lext = path.complete_lower_extension();
    if (lext == "pts") {
        process_points_file(path.str());
    } else if ((lext == "btg.gz") || (lext == "btg")) {
        process_scenery_file(path.str());
    } else if ((lext != "gz") && (lext != "arr") && (lext != "fit") &&
               (lext != "stg") && (lext != "ind"))
    {
        // should be a polygon file
        process_polygon_file(path.str());
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

        auto driverManager = GetGDALDriverManager();
        for (int i = 0; i < driverManager->GetDriverCount(); ++i) {
            auto ogrDriver = driverManager->GetDriver(i);
            SG_LOG(SG_GENERAL, SG_INFO, "\t\t\t-f \"" << ogrDriver->GetDescription() << "\"");
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

        GDALAllRegister();

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

        auto driverManager = GetGDALDriverManager();

        auto gdalDriver = driverManager->GetDriverByName(format_name);
        if (!gdalDriver) {
                usage(argv[0], std::string("Unknown datasource format driver:") + format_name);
                exit(1);
        }

        const char* dst_datasource = argv[optind++];
        datasource = gdalDriver->Create(dst_datasource, 0, 0, 0, GDALDataType::GDT_Unknown, NULL);
        if (!datasource) {
                usage(argv[0],std::string("Unable to create datasource:") + dst_datasource);
                exit(1);
        }

        for (int i=optind;i<argc;i++) {
                process_file(SGPath(argv[i]));
        }

        GDALClose((GDALDatasetH) datasource );
        GDALDestroyDriverManager();

        return 0;
}
