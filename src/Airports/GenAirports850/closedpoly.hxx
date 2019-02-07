#ifndef _BEZPOLY_H_
#define _BEZPOLY_H_

#include <memory>

#include <terragear/tg_polygon.hxx>

#include "beznode.hxx"
#include "linearfeature.hxx"

class ClosedPoly
{
public:
    explicit ClosedPoly( char* desc );
    ClosedPoly( int st, float s, float th, char* desc );
	~ClosedPoly();

    inline std::string GetDescription() { return description; }
    void AddNode( std::shared_ptr<BezNode> node );
    void CloseCurContour();
    void Finish();

    // Build BTG for airport base for airports with boundary
    int  BuildBtg( tgPolygon& apt_base,
                   tgPolygon& apt_clearing,
                   std::string& shapefile_name );

    // Build BTG for pavements for airports with no boundary
    int  BuildBtg( tgpolygon_list& rwy_polys,
                   tgcontour_list& slivers,
                   tgAccumulator& accum,
                   std::string& shapefile_name );

    int  BuildBtg( tgpolygon_list& rwy_polys,
                   tgcontour_list& slivers,
                   tgpolygon_list& apt_base_polys,
                   tgpolygon_list& apt_clearing_polys,
                   tgAccumulator& accum,
                   std::string& shapefile_name );

    FeatureList& GetFeatures()
    {
        return features;
    }

private:
    // convert the BezierPoly to a normal Poly (adding nodes for the curves)
    void CreateConvexHull( void );
    void ConvertContour( const BezContour& src, tgContour& dst );
    std::string GetMaterial( int surface );


    bool   is_pavement;
    bool   is_border;
    bool   has_feature;
    
    int    surface_type;
    float  smoothness;
    double texture_heading;
    std::string description;

    // outer boundary definition as bezier nodes
    BezContour boundary;

    // holes
    BezContourArray holes;

    // contour that nodes will be added until done
    BezContour cur_contour;

    // Converted polygon after parsing complete
    tgPolygon pre_tess;

    // shoulders after BTG built
    tgpolygon_list shoulder_polys;

    // pavement definitions have multiple linear features (markings and lights for each contour)
    std::shared_ptr<LinearFeature> cur_feature;
    FeatureList features;
};

typedef std::vector<std::shared_ptr<ClosedPoly>> PavementList;

#endif
