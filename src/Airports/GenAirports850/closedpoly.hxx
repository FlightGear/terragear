#ifndef _BEZPOLY_H_
#define _BEZPOLY_H_

#include "beznode.hxx"
#include "linearfeature.hxx"

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>

#include <Geometry/point3d.hxx>

using std::string;

class ClosedPoly
{
public:
    ClosedPoly( char* desc );
    ClosedPoly( int st, float s, float th, char* desc );
	~ClosedPoly();
    
    inline string GetDescription() { return description; }
    void AddNode( BezNode* node );
    void CloseCurContour();
    void Finish();

    // Build BTG for airport base for airports with boundary
    int  BuildBtg( tgPolygon& apt_base,
                   tgPolygon& apt_clearing,
                   std::string& shapefile_name );

    // Build BTG for pavements for airports with no boundary
    int  BuildBtg( tgpolygon_list& rwy_polys,
                   tgcontour_list& slivers,
                   std::string& shapefile_name );

    int  BuildBtg( tgpolygon_list& rwy_polys,
                   tgcontour_list& slivers,
                   tgPolygon& apt_base,
                   tgPolygon& apt_clearing,
                   std::string& shapefile_name );

    FeatureList* GetFeatures()
    {
        return &features;
    }

private:
    // convert the BezierPoly to a normal Poly (adding nodes for the curves)
    void CreateConvexHull( void );
    void ConvertContour( BezContour* src, tgContour& dst );
    void ExpandContour( point_list& src, TGPolygon& dst, double dist );
    std::string GetMaterial( int surface );


    bool   is_pavement;
    int    surface_type;
    float  smoothness;
    double texture_heading;
    string description;
    
    // outer boundary definition as bezier nodes
    BezContour* boundary;

    // holes
    BezContourArray holes;

    // contour that nodes will be added until done
    BezContour* cur_contour;

    // outer boundary as convex hull
    point_list hull;

    // Converted polygon after parsing complete
    tgPolygon pre_tess;

    // pavement definitions have multiple linear features (markings and lights for each contour)
    LinearFeature* cur_feature;
    FeatureList features;
};

typedef std::vector <ClosedPoly *> PavementList;

#endif
