#include <simgear/debug/logstream.hxx>
#include "tg_polygon_set.hxx"

// every polygon set (metadata) gets its own unique identifier
unsigned long tgPolygonSetMeta::cur_id = 1;

void tgPolygonSetMeta::initFields( void )
{
    reflon  = 0.0l;
    reflat  = 0.0l;
    width   = 0.0;
    length  = 0.0;
    heading = 0.0;
    
    minu = 0.0;
    maxu = 0.0;
    minv = 0.0;
    maxv = 0.0;
    
    min_clipu = 0.0;
    max_clipu = 0.0;
    min_clipv = 0.0;
    max_clipv = 0.0;
    
    center_lat = 0.0;
    
    flags = 0;
}

void tgPolygonSetMeta::setSurfaceInfo(tgSurface const& surf)
{    
    surf.getCoefficients( surfaceCoefficients );
    surf.getExtents( surfaceMin, surfaceMax, surfaceCenter );
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "tgPolygonSetMeta::setSurfaceInfo - got " << surfaceCoefficients.size() << " coeffs" );
    
    for ( unsigned int i=0; i<surfaceCoefficients.size(); i++ ) {
        SG_LOG( SG_GENERAL, SG_DEBUG, "tgPolygonSetMeta::setSurfaceInfo - coeff[" << i << "] = " <<  surfaceCoefficients[i] );
    }        
}

void tgPolygonSetMeta::getFeatureFields( OGRFeature* poFeature )
{
    getCommonFields( poFeature );
    
    switch( info ) {
        case META_NONE:
            SG_LOG( SG_GENERAL, SG_DEBUG, "tgPolygonSetMeta::getFeatureFields - info is META_NONE" );
            break;
        
        case META_TEXTURED:
            SG_LOG( SG_GENERAL, SG_DEBUG, "tgPolygonSetMeta::getFeatureFields - info is META_TEXTURED" );
            getTextureFields( poFeature );
            break;
            
        case META_TEXTURED_SURFACE:
            SG_LOG( SG_GENERAL, SG_DEBUG, "tgPolygonSetMeta::getFeatureFields - info is META_TEXTURED_SURFACE" );
            getTextureFields( poFeature );
            getSurfaceFields( poFeature );
            break;                
            
        case META_CONSTRAIN:
            SG_LOG( SG_GENERAL, SG_DEBUG, "tgPolygonSetMeta::getFeatureFields - info is META_CONSTRAIN" );
            break;               
    }
}

void tgPolygonSetMeta::setFeatureFields( OGRFeature* poFeature ) const
{
    setCommonFields( poFeature );
    
    switch( info ) {
        case META_NONE:
            break;
        
        case META_TEXTURED:
            setTextureFields( poFeature );
            break;
            
        case META_TEXTURED_SURFACE:
            setTextureFields( poFeature );
            setSurfaceFields( poFeature );
            break;                
            
        case META_CONSTRAIN:
            break;               
    }
}

void tgPolygonSetMeta::getCommonFields( OGRFeature* poFeature )
{
    char strbuff[256];

    getFieldAsInteger( poFeature, "tg_id",   &id );
    getFieldAsInteger( poFeature, "tg_meta", (unsigned long int *)&info );
    getFieldAsInteger( poFeature, "OGC_FID", &fid );
    getFieldAsInteger( poFeature, "tg_flags", &flags );    

    getFieldAsString( poFeature, "tg_desc", strbuff, 256 );
    if ( strlen( strbuff ) ) {
        description = strbuff;
    }    
}

void tgPolygonSetMeta::getTextureFields( OGRFeature* poFeature )
{
    char strbuff[256];
    
    getFieldAsString( poFeature, "tg_mat", strbuff, 256 );
    if ( strlen( strbuff ) ) {
        material = strbuff;
    }
    
    getFieldAsInteger( poFeature, "tg_texmeth", (unsigned long int *)&method );
    if ( method == TEX_BY_GEODE ) {
        getFieldAsDouble( poFeature, "tg_clat", &center_lat );
    } else {
        getFieldAsDouble( poFeature, "tg_reflon", &reflon );
        getFieldAsDouble( poFeature, "tg_reflon", &reflat );
                
        getFieldAsDouble( poFeature, "tg_heading", &heading );
        getFieldAsDouble( poFeature, "tg_width", &width );
        getFieldAsDouble( poFeature, "tg_length", &length );
        getFieldAsDouble( poFeature, "tg_minu", &minu );
        getFieldAsDouble( poFeature, "tg_minv", &minv );
        getFieldAsDouble( poFeature, "tg_maxu", &maxu );
        getFieldAsDouble( poFeature, "tg_maxv", &maxv );
        getFieldAsDouble( poFeature, "tg_mincu", &min_clipu );
        getFieldAsDouble( poFeature, "tg_mincv", &min_clipv );
        getFieldAsDouble( poFeature, "tg_maxcu", &max_clipu );
        getFieldAsDouble( poFeature, "tg_maxcv", &max_clipv );
    }    
}

void tgPolygonSetMeta::getSurfaceFields( OGRFeature* poFeature )
{
}

void tgPolygonSetMeta::setCommonFields( OGRFeature* poFeature ) const
{
    poFeature->SetField("tg_id",        (int)id );
    poFeature->SetField("tg_meta",      (int)info );
    poFeature->SetField("OGC_FID",      (int)fid );
    poFeature->SetField("tg_flags",     (int)flags );

    poFeature->SetField("tg_desc",      description.c_str() );    
}

void tgPolygonSetMeta::setTextureFields( OGRFeature* poFeature ) const
{    
    poFeature->SetField("tg_mat",       material.c_str() );
    poFeature->SetField("tg_texmeth",   (int)method );
    
    poFeature->SetField("tg_reflon",    reflon );
    poFeature->SetField("tg_reflat",    reflat );
    poFeature->SetField("tg_heading",   heading );
    poFeature->SetField("tg_width",     width );
    poFeature->SetField("tg_length",    length );
    poFeature->SetField("tg_minu",      minu );
    poFeature->SetField("tg_minv",      minv );
    poFeature->SetField("tg_maxu",      maxu );
    poFeature->SetField("tg_maxv",      maxv );
    poFeature->SetField("tg_mincu",     min_clipu );
    poFeature->SetField("tg_mincv",     min_clipv );
    poFeature->SetField("tg_maxcu",     max_clipu );
    poFeature->SetField("tg_maxcv",     max_clipv );
}

void tgPolygonSetMeta::setSurfaceFields( OGRFeature* poFeature ) const
{
    poFeature->SetField("tgsrf_mnln", surfaceMin.getLongitudeDeg() );
    poFeature->SetField("tgsrf_mnlt", surfaceMin.getLatitudeDeg() );
    poFeature->SetField("tgsrf_mxln", surfaceMax.getLongitudeDeg() );
    poFeature->SetField("tgsrf_mxlt", surfaceMax.getLatitudeDeg() );

    poFeature->SetField("tgsrf_co00", surfaceCoefficients[ 0] );
    poFeature->SetField("tgsrf_co01", surfaceCoefficients[ 1] );
    poFeature->SetField("tgsrf_co02", surfaceCoefficients[ 2] );
    poFeature->SetField("tgsrf_co03", surfaceCoefficients[ 3] );
    poFeature->SetField("tgsrf_co04", surfaceCoefficients[ 4] );
    poFeature->SetField("tgsrf_co05", surfaceCoefficients[ 5] );
    poFeature->SetField("tgsrf_co06", surfaceCoefficients[ 6] );
    poFeature->SetField("tgsrf_co07", surfaceCoefficients[ 7] );
    poFeature->SetField("tgsrf_co08", surfaceCoefficients[ 8] );
    poFeature->SetField("tgsrf_co09", surfaceCoefficients[ 9] );
    poFeature->SetField("tgsrf_co10", surfaceCoefficients[10] );
    poFeature->SetField("tgsrf_co11", surfaceCoefficients[11] );
}

void tgPolygonSetMeta::getFieldAsInteger( OGRFeature* poFeature, const char* field, unsigned long int* setting )
{
    int fieldIdx = poFeature->GetFieldIndex( field );
    
    if ( fieldIdx >= 0 ) {
        *setting = poFeature->GetFieldAsInteger(fieldIdx);
    }    
}

void tgPolygonSetMeta::getFieldAsDouble( OGRFeature* poFeature, const char* field, double* setting )
{
    int fieldIdx = poFeature->GetFieldIndex( field );
    
    if ( fieldIdx >= 0 ) {
        *setting = poFeature->GetFieldAsDouble(fieldIdx);
    }    
}

void tgPolygonSetMeta::getFieldAsString( OGRFeature* poFeature, const char* field, char* setting, size_t size )
{
    int fieldIdx = poFeature->GetFieldIndex( field );
    
    if ( fieldIdx >= 0 ) {
        strncpy( setting, poFeature->GetFieldAsString(fieldIdx), size );
    } else {
        memset( setting, 0, size );
    }
}

