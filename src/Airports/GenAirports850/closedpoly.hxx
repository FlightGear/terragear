#ifndef _BEZPOLY_H_
#define _BEZPOLY_H_

#include <terragear/tg_polygon.hxx>

#include "beznode.hxx"
#include "linearfeature.hxx"

class ClosedPoly
{
public:
    ClosedPoly( char* desc );
    ClosedPoly( int st, float s, float th, char* desc );
	~ClosedPoly();

    inline std::string GetDescription() { return description; }
    void AddNode( BezNode* node );
    void CloseCurContour();
    void Finish();

    void GetPolys( tgpolygon_list& polys );
    void GetInnerBasePolys( tgpolygon_list& polys );
    void GetOuterBasePolys( tgpolygon_list& polys );
    void GetInnerBoundaryPolys( tgpolygon_list& polys );
    void GetOuterBoundaryPolys( tgpolygon_list& polys );
    
    void GetFeaturePolys( tgpolygon_list& polys );
    void GetFeatureCapPolys( tgpolygon_list& polys );
    void GetFeatureLights( tglightcontour_list& lights );
    
private:
    // convert the BezierPoly to a normal Poly (adding nodes for the curves)
    void CreateConvexHull( void );
    void ConvertContour( BezContour* src, tgContour& dst );
    std::string GetMaterial( int surface );


    bool   is_pavement;
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
    tgPolygon pre_tess;

    // shoulders after BTG built
    tgpolygon_list shoulder_polys;

    // pavement definitions have multiple linear features (markings and lights for each contour)
    LinearFeature* cur_feature;
    FeatureList features;
};

typedef std::vector <ClosedPoly *> PavementList;

#endif
