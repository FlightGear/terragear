
#include "tg_polygon.hxx"
#include "tg_contour.hxx"
#include "clipper.hpp"

class tgShapefile
{
public:
    static void  FromContour( const tgContour& subject, const std::string& datasource, const std::string& layer, const std::string& description );
    static void  FromPolygon( const tgPolygon& subject, const std::string& datasource, const std::string& layer, const std::string& description );
    static void  FromTriangles( const tgPolygon& subject, const std::string& datasource, const std::string& layer, const std::string& description );
    static tgPolygon ToPolygon( const void* subject );

    static void  FromClipper( const ClipperLib::Paths& subject, const std::string& datasource, const std::string& layer, const std::string& description );

private:
    static bool initialized;

    static void* OpenDatasource( const char* datasource_name );
    static void* OpenLayer( void* ds_id, const char* layer_name );
    static void* OpenLineLayer( void* ds_id, const char* layer_name );
    static void* CloseDatasource( void* ds_id );
};