#ifndef _BEZPOLY_H_
#define _BEZPOLY_H_

#include <terragear/polygon_set/tg_polygon_set.hxx>

#include "beznode.hxx"
#include "linearfeature.hxx"

class Airport;

class ClosedPoly
{
public:
    ClosedPoly( char* desc );
    ClosedPoly( int st, float s, float th, char* desc );
	~ClosedPoly();

    inline std::string GetDescription() { return description; }
    void AddNode( BezNode* node );
    void CloseCurContour( Airport* ap );
    void Finish();

    tgPolygonSetList& GetPolys( void );
    tgPolygonSetList& GetInnerBasePolys( void );
    tgPolygonSetList& GetOuterBasePolys( void );
    tgPolygonSetList& GetInnerBoundaryPolys( void );
    tgPolygonSetList& GetOuterBoundaryPolys( void );
    
    tgPolygonSetList&  GetFeaturePolys( void );
    tgPolygonSetList&  GetFeatureCapPolys( void );
    void GetFeatureLights( tglightcontour_list& lights );
    
private:
    // convert the BezierPoly to a normal Poly (adding nodes for the curves)
    // void CreateConvexHull( void );
    cgalPoly_Polygon ConvertContour( BezContour* src );
    std::string GetMaterial( int surface );

    bool   is_pavement;
    bool   is_border;
    bool   has_feature;
    
    int    surface_type;
    float  smoothness;
    double texture_heading;
    std::string description;

    // outer boundary definition as bezier nodes
    BezContour* boundary;

    // holes
    BezContourArray holes;

    // contour that nodes will be added until done
    BezContour* cur_contour;

    // Converted polygon after parsing complete
    cgalPoly_PolygonWithHoles pre_tess;

    // shoulders after BTG built
    tgPolygonSetList feature_polys;
    tgPolygonSetList feature_cap_polys;
    tgPolygonSetList pavement_polys;
    tgPolygonSetList shoulder_polys;
    tgPolygonSetList inner_base_polys;
    tgPolygonSetList outer_base_polys;
    tgPolygonSetList inner_boundary_polys;
    tgPolygonSetList outer_boundary_polys;

    // pavement definitions have multiple linear features (markings and lights for each contour)
    LinearFeature* cur_feature;
    FeatureList features;
};

typedef std::vector <ClosedPoly *> PavementList;

#endif
