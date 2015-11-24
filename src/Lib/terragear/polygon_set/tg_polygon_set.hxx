#ifndef __TG_POLYGON_SET_HXX__
#define __TG_POLYGON_SET_HXX__

#include <ogrsf_frmts.h>

#include <terragear/clipper.hpp>
#include <terragear/tg_surface.hxx>
#include <terragear/tg_cluster.hxx>

#include "tg_polygon_def.hxx"

// point offsetting...
#include <simgear/math/SGMath.hxx>
SGGeod OffsetPointMiddle( const cgalPoly_Point& gPrev, const cgalPoly_Point& gCur, const cgalPoly_Point& gNext, double offset_by );
SGGeod OffsetPointFirst( const cgalPoly_Point& cur, const cgalPoly_Point& next, double offset_by );
SGGeod OffsetPointLast( const cgalPoly_Point& prev, const cgalPoly_Point& cur, double offset_by );

class tgPolygonSetMeta
{
public:
    // what type of meta info are we saving with the geometry
    typedef enum {
        META_NONE,
        META_TEXTURED,
        META_TEXTURED_SURFACE,
        META_CONSTRAIN
    } MetaInfo_e;

    tgPolygonSetMeta() : info(META_NONE), id(tgPolygonSetMeta::cur_id++) {}
    tgPolygonSetMeta( MetaInfo_e i) : info(i), id(tgPolygonSetMeta::cur_id++) {}
    tgPolygonSetMeta( MetaInfo_e i, const std::string& mat, const std::string& desc ) : info(i), material(mat), id(tgPolygonSetMeta::cur_id++), description(desc) {}
    tgPolygonSetMeta( MetaInfo_e i, const std::string& mat ) : info(i), material(mat), id(tgPolygonSetMeta::cur_id++) {}

    /* All Meta Info types */
    void setDescription( const char* desc ) { description = desc; }
        
    std::string getMetaType( void ) const {
        std::string type;
        
        switch( info ) {
            case META_NONE:
                type = "none";
                break;
            
            case META_TEXTURED:
                type = "textured";
                break;
                
            case META_TEXTURED_SURFACE:
                type = "textured_surface";
                break;                
                
            case META_CONSTRAIN:
                type = "constraint";
                break;            
        }
        
        return type;
    }
    
    /* Texture Information */
    typedef enum {
        TEX_UNKNOWN,
        TEX_BY_GEODE,
        TEX_BY_TPS_NOCLIP,
        TEX_BY_TPS_CLIPU,
        TEX_BY_TPS_CLIPV,
        TEX_BY_TPS_CLIPUV,
        TEX_1X1_ATLAS
    } TextureMethod_e;

    void setTextureMethod( TextureMethod_e m ) { 
        method     = m; 
    }
    void setTextureMethod( TextureMethod_e m, double cl ) { 
        method     = m; 
        center_lat = cl;
    }
    void setTextureMethod( TextureMethod_e m, double min_u, double min_v, double max_u, double max_v ) { 
        method     = m; 
        min_clipu  = min_u;
        min_clipv  = min_v;
        max_clipu  = max_u;
        max_clipv  = max_v;
    }
    
    void setMaterial( const std::string& mat ) { material = mat; }
    
    void setTextureRef( const cgalPoly_Point& r, double w, double l, double h ) { 
        reflon  = CGAL::to_double( r.x() );
        reflat  = CGAL::to_double( r.y() );
        width   = w;
        length  = l;
        heading = h;
    }
   
    void setTextureLimits( double min_u, double min_v, double max_u, double max_v ) { 
        minu  = min_u;
        minv  = min_v;
        maxu  = max_u;
        maxv  = max_v;
    }
    
    // Smoothing Surface Info
    void setSurfaceInfo( const tgSurface& base_surf );
    
    // I/O
    void getFeatureFields( OGRFeature* poFeature );
    void setFeatureFields( OGRFeature* poFeature ) const;
    
    MetaInfo_e      info;

    std::string     material;
    TextureMethod_e method;

    double          reflon;
    double          reflat;
    double          width;
    double          length;
    double          heading;

    double          minu;
    double          maxu;
    double          minv;
    double          maxv;

    double          min_clipu;
    double          max_clipu;
    double          min_clipv;
    double          max_clipv;

    double          center_lat;

    std::vector<double> surfaceCoefficients;
    SGGeod surfaceMin, surfaceMax, surfaceCenter;
    
    // metadata
    unsigned long       flags;
    unsigned long       id;
    unsigned long       fid;
    std::string         description;
    
private:    
    static unsigned long    cur_id;    

    void                    getCommonFields( OGRFeature* poFeature );
    void                    setCommonFields( OGRFeature* poFeature ) const;

    void                    getTextureFields( OGRFeature* poFeature );
    void                    setTextureFields( OGRFeature* poFeature ) const;

    void                    getSurfaceFields( OGRFeature* poFeature );
    void                    setSurfaceFields( OGRFeature* poFeature ) const;
    
    void                    getFieldAsInteger( OGRFeature* poFeature, const char* field, long unsigned int* setting );
    void                    getFieldAsDouble( OGRFeature* poFeature, const char* field, double* setting );
    void                    getFieldAsString( OGRFeature* poFeature, const char* field, char* setting, size_t size );    
};

class tgPolygonSet;
typedef std::vector<tgPolygonSet>   tgPolygonSetList;

class tgPolygonSet 
{
public:
    tgPolygonSet( void ) {}
    
    // generate new polygon sets
    tgPolygonSet( const cgalPoly_Polygon& poly ) {
        ps = cgalPoly_PolygonSet(poly);
    }
    
    tgPolygonSet( const cgalPoly_Polygon& poly, const tgPolygonSetMeta& metaInfo ) : meta(metaInfo) {
        ps = cgalPoly_PolygonSet(poly);
    }
    tgPolygonSet( const cgalPoly_PolygonWithHoles& polyWithHoles, const tgPolygonSetMeta& metaInfo ) : meta(metaInfo) {
        ps = cgalPoly_PolygonSet(polyWithHoles);
    }

    tgPolygonSet( const cgalPoly_PolygonSet& set ) : ps(set) {}
    tgPolygonSet( const cgalPoly_PolygonSet& polyset, const tgPolygonSetMeta& metaInfo ) : ps(polyset), meta(metaInfo) {}

    tgPolygonSet( OGRFeature* poFeature, OGRPolygon* poGeometry );
    
    tgPolygonSet( OGRPolygon* poGeometry, const tgPolygonSetMeta& metaInfo );

    void                                clusterNodes( const tgCluster& clusteredNodes );
    
    void                                toShapefile( const char* datasource, const char* layer ) const;
    void                                toShapefile( OGRLayer* layer, const char* description ) const;
    void                                toShapefile( OGRLayer* layer ) const;

    static void                         toShapefile( const cgalPoly_Polygon& poly, const char* datasource, const char* layer );

    void                                toSegments( std::vector<cgalPoly_Segment>& segs, bool withHoles ) const;

    const std::vector<cgalPoly_Point>&  getInteriorPoints( void ) const;
    void                                calcInteriorPoints( void );
    
    CGAL::Bbox_2                        getBoundingBox( void ) const;
    bool                                isEmpty( void ) const { return ps.is_empty(); }
    void                                erase( void ) { ps.clear(); }

    void                                splitLongEdges( int maxSegmentLength );
    
    void                                setPs( const cgalPoly_PolygonSet& polyset ) { ps = polyset; }
    const cgalPoly_PolygonSet&          getPs( void ) const   { return ps; }
    const tgPolygonSetMeta&             getMeta( void ) const { return meta; }
    tgPolygonSetMeta&                   getMeta( void ) { return meta; }
    
    tgPolygonSet                        intersection( const cgalPoly_Polygon& other ) const;
    void                                intersection2( const cgalPoly_Polygon& other );

    void                                difference( const cgalPoly_Polygon& other );
    void                                join( const cgalPoly_Polygon& other );
    static tgPolygonSet                 join( const tgPolygonSetList& sets, const tgPolygonSetMeta& meta );

    tgPolygonSet                        offset( double oset ) const;
    
    static GDALDataset*                 openDatasource( const char* datasource_name );
    static OGRLayer*                    openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, const char* layer_name );
    
private:        
    void                                toShapefile( OGRLayer* poLayer, const cgalPoly_PolygonSet& polySet ) const;
    void                                toShapefile( OGRLayer* poLayer, const cgalPoly_PolygonWithHoles& pwh ) const;
    void                                toShapefile( OGRLayer* poLayer, const cgalPoly_Polygon& poly ) const;
    void                                toShapefile( OGRLayer* poLayer, const cgalPoly_Arrangement& arr ) const;

    void                                polygonToSegmentList( const cgalPoly_Polygon& p, std::vector<cgalPoly_Segment>& segs ) const;
    void                                findIntersections( const cgalPoly_PolygonWithHoles& pwh, const cgalPoly_Line& line, std::vector<cgalPoly_Point>& intersections ) const;
    cgalPoly_Point                      getInteriorPoint( const cgalPoly_PolygonWithHoles& pwh ) const;
    
    void                                facesFromUntrustedNodes( std::vector<cgalPoly_Point> nodes, std::vector<cgalPoly_Polygon>& faces );

    cgalPoly_Polygon                    splitLongEdges( cgalPoly_Polygon& p, int maxSegmentSize );
    cgalPoly_PolygonWithHoles           splitLongEdges( cgalPoly_PolygonWithHoles& pwh, int maxSegmentLength );
  
// to / from clipper for Polygon Offsetting ( Can't get CGAL to propery shrink Polygons....  TODO, maybe
    double                              toClipper( double dist ) const;
    
    ClipperLib::IntPoint                toClipper( const cgalPoly_Point& p ) const;
    cgalPoly_Point                      fromClipper( const ClipperLib::IntPoint& p ) const;
    
    ClipperLib::Path                    toClipper( const cgalPoly_Polygon& subject, bool isHole ) const;
    cgalPoly_Polygon                    fromClipper( const ClipperLib::Path& subject ) const;
    
    void                                toClipper( const cgalPoly_PolygonWithHoles& pwh, ClipperLib::Paths& paths ) const;
    ClipperLib::Paths                   toClipper( const cgalPoly_PolygonSet& ps ) const;
    
    cgalPoly_PolygonSet                 fromClipper( const ClipperLib::Paths& subject ) const;
    
    cgalPoly_PolygonSet                 ps;
    std::vector<cgalPoly_Point>         interiorPoints;
    tgPolygonSetMeta                    meta;
};

#endif /* __TG_POLYGON_SET_HXX__ */
