#ifndef __TG_POLYGON_SET_HXX__
#define __TG_POLYGON_SET_HXX__

#include <ogrsf_frmts.h>
#include "tg_polygon_def.hxx"

// point offsetting...
#include <simgear/math/SGMath.hxx>
SGGeod OffsetPointMiddle( const cgalPoly_Point& gPrev, const cgalPoly_Point& gCur, const cgalPoly_Point& gNext, double offset_by );
SGGeod OffsetPointFirst( const cgalPoly_Point& cur, const cgalPoly_Point& next, double offset_by );
SGGeod OffsetPointLast( const cgalPoly_Point& prev, const cgalPoly_Point& cur, double offset_by );


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
    
    tgTexInfo() {}
    tgTexInfo( const std::string& mat ) : material(mat) {}
    
    void SetMethod( method_e m ) { 
        method     = m; 
    }
    void SetMethod( method_e m, double cl ) { 
        method     = m; 
        center_lat = cl;
    }
    void SetMethod( method_e m, double min_u, double min_v, double max_u, double max_v ) { 
        method     = m; 
        min_clipu  = min_u;
        min_clipv  = min_v;
        max_clipu  = max_u;
        max_clipv  = max_v;
    }
    
    void SetRef( const cgalPoly_Point& r, double w, double l, double h ) { 
        ref     = r;
        width   = w;
        length  = l;
        heading = h;
    }
   
    void SetLimits( double min_u, double min_v, double max_u, double max_v ) { 
        minu  = min_u;
        minv  = min_v;
        maxu  = max_u;
        maxv  = max_v;
    }
    
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

class tgPolygonSet;
typedef std::vector<tgPolygonSet>   tgPolygonSetList;

class tgPolygonSet 
{
public:
    tgPolygonSet( void ) : id(tgPolygonSet::cur_id++) {}
    tgPolygonSet( const cgalPoly_Polygon& poly ) : id(tgPolygonSet::cur_id++) {
        ps = cgalPoly_PolygonSet(poly);
    }
    tgPolygonSet( const cgalPoly_Polygon& poly, const tgTexInfo& texinfo, const char* desc ) : ti(texinfo), flags(0), id(tgPolygonSet::cur_id++) {
        ps = cgalPoly_PolygonSet(poly);
        description = desc;
    }
    tgPolygonSet( const cgalPoly_PolygonWithHoles& poly, const tgTexInfo& texinfo, const char* desc ) : ti(texinfo), flags(0), id(tgPolygonSet::cur_id++) {
        ps = cgalPoly_PolygonSet(poly);
        description = desc;
    }

    tgPolygonSet( const cgalPoly_PolygonSet& set ) : ps(set), id(tgPolygonSet::cur_id++) {}
    tgPolygonSet( const cgalPoly_PolygonSet& polyset, const tgTexInfo& texinfo, unsigned long f ) : ps(polyset), ti(texinfo), flags(f), id(tgPolygonSet::cur_id++) {}

    tgPolygonSet( OGRFeature* poFeature, OGRPolygon* poGeometry );
    tgPolygonSet( OGRFeature* poFeature, OGRPolygon* poGeometry, const std::string& material );

    void                toShapefile( const char* datasource, const char* layer ) const;
    void                toShapefile( OGRLayer* layer, const char* description ) const;
    void                toShapefile( OGRLayer* layer ) const;

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
    
    tgPolygonSet        intersection( const cgalPoly_Polygon& other ) const;
    void                intersection2( const cgalPoly_Polygon& other );

    void                difference( const cgalPoly_Polygon& other );
    void                join( const cgalPoly_Polygon& other );
    static tgPolygonSet join( const tgPolygonSetList& sets );

    tgPolygonSet        offset( double oset ) const;
    
    static GDALDataset* openDatasource( const char* datasource_name );
    static OGRLayer*    openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, const char* layer_name );
    
private:
    static unsigned long      cur_id;
        
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
    
    void                      ogrRingToPolygonSet( OGRLinearRing const *ring, std::vector<cgalPoly_Polygon>& faces );

    cgalPoly_Polygon          splitLongEdges( cgalPoly_Polygon& p, int maxSegmentSize );
    cgalPoly_PolygonWithHoles splitLongEdges( cgalPoly_PolygonWithHoles& pwh, int maxSegmentLength );

    void                      contractPolygon( double oset, const cgalPoly_Polygon& poly, std::vector<cgalPoly_Polygon>& offsetPWHs ) const;
    void                      expandPolygon( double oset, const cgalPoly_Polygon& poly, std::vector<cgalPoly_Polygon>& offsetPWHs ) const;
    
    cgalPoly_PolygonSet ps;
    tgTexInfo           ti;
    unsigned long       flags;
    unsigned long       id;
    unsigned long       fid;
    std::string         description;
};

#endif /* __TG_POLYGON_SET_HXX__ */
