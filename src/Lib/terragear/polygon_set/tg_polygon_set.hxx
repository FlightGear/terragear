#ifndef __TG_POLYGON_SET_HXX__
#define __TG_POLYGON_SET_HXX__

#include <ogrsf_frmts.h>
#include "tg_polygon_def.hxx"

class tgTexInfo
{
public:
    
    typedef enum {
        TEX_UNKNOWN,
        TEX_BY_GEODE,
        TEX_BY_TPS_NOCLIP,
        TEX_BY_TPS_CLIPU,
        TEX_BY_TPS_CLIPV,
        TEX_BY_TPS_CLIPUV,
        TEX_1X1_ATLAS
    } method_e;
    
    std::string     material;
    method_e        method;

    cgalPoly_Point  ref;
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
};

class tgPolygonSet 
{
public:
    tgPolygonSet( void ) : id(tgPolygonSet::cur_id++) {}
    tgPolygonSet( const cgalPoly_PolygonSet& set ) : ps(set), id(tgPolygonSet::cur_id++) {}
    tgPolygonSet( const cgalPoly_PolygonSet& polyset, const tgTexInfo& texinfo, unsigned long f ) : ps(polyset), ti(texinfo), flags(f), id(tgPolygonSet::cur_id++) {}

    tgPolygonSet( OGRFeature* poFeature, OGRPolygon* poGeometry );
    tgPolygonSet( OGRFeature* poFeature, OGRPolygon* poGeometry, const std::string& material );

    void                toShapefile( const char* datasource, const char* layer ) const;
    void                toShapefile( OGRLayer* layer, const char* description ) const;

    CGAL::Bbox_2        getBoundingBox( void ) const;
    bool                isEmpty( void ) const { return ps.is_empty(); }

    void                erase( void ) { ps.clear(); }

    void                splitLongEdges( int maxSegmentLength );
    
    void                setTexParams( cgalPoly_Point& ref, double width, double length, double heading ) {
        ti.ref     = ref;
        ti.width   = width;
        ti.length  = length;
        ti.heading = heading;
    }
    
    void setTexMethod( tgTexInfo::method_e method ) {
        ti.method = method;
    }
    void setTexMethod( tgTexInfo::method_e method, double min_cu, double min_cv, double max_cu, double max_cv ) {
        ti.method = method;
        ti.min_clipu = min_cu;
        ti.min_clipv = min_cv;
        ti.max_clipu = max_cu;
        ti.max_clipv = max_cv;        
    }
    
    void setTexLimits( double minu, double minv, double maxu, double maxv ) {
        ti.minu = minu;
        ti.minv = minv;
        ti.maxu = maxu;
        ti.maxv = maxv;
    }
    
    void                setMaterial( std::string mat ) { ti.material = mat; }
    std::string         getMaterial( void ) const { return ti.material; }

    void                setPs( const cgalPoly_PolygonSet& polyset ) { ps = polyset; }
    cgalPoly_PolygonSet getPs( void ) const { return ps; }
    tgTexInfo           getTi( void ) const { return ti; }
    unsigned long       getId( void ) const { return id; }
    
    void                intersection( const cgalPoly_Polygon& other );
    tgPolygonSet        intersection( const cgalPoly_Polygon& other ) const;

    void                difference( const cgalPoly_Polygon& other );
    void                join( const cgalPoly_Polygon& other );
    
private:
    static unsigned long      cur_id;
    
    GDALDataset*              openDatasource( const char* datasource_name ) const;
    OGRLayer*                 openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, const char* layer_name ) const;
    
    void                      toShapefile( OGRLayer* poLayer, const cgalPoly_PolygonSet& polySet ) const;
    void                      toShapefile( OGRLayer* poLayer, const cgalPoly_PolygonWithHoles& pwh ) const;
    void                      toShapefile( OGRLayer* poLayer, const cgalPoly_Arrangement& arr ) const;

    void                      getFeatureFields( OGRFeature* poFeature );
    void                      setFeatureFields( OGRFeature* poFeature ) const;

    int                       getFieldAsInteger( OGRFeature* poFeature, const char* field, int defValue );
    double                    getFieldAsDouble( OGRFeature* poFeature, const char* field, double defValue );
    const char*               getFieldAsString( OGRFeature* poFeature, const char* field, const char* defValue );

    void                      polygonToSegmentList( const cgalPoly_Polygon& p, std::vector<cgalPoly_Segment>& segs ) const;
    void                      findIntersections( const cgalPoly_PolygonWithHoles& pwh, const cgalPoly_Line& line, std::vector<cgalPoly_Point>& intersections ) const;
    cgalPoly_Point            getInteriorPoint( const cgalPoly_PolygonWithHoles& pwh ) const;
    
    cgalPoly_PolygonSet       ogrRingToPolygonSet( OGRLinearRing const *ring );

    cgalPoly_Polygon          splitLongEdges( cgalPoly_Polygon& p, int maxSegmentSize );
    cgalPoly_PolygonWithHoles splitLongEdges( cgalPoly_PolygonWithHoles& pwh, int maxSegmentLength );

    cgalPoly_PolygonSet ps;
    tgTexInfo           ti;
    unsigned long       flags;
    unsigned long       id;
};

typedef std::vector<tgPolygonSet>   tgPolygonSetList;

#endif /* __TG_POLYGON_SET_HXX__ */
