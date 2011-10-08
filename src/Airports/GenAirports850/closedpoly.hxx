#ifndef _BEZPOLY_H_
#define _BEZPOLY_H_

#include "beznode.hxx"
#include "linearfeature.hxx"

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Geometry/point3d.hxx>

#include "texparams.hxx"

#include <osg/Geometry>
#include <osg/Vec3d>

using std::string;

class ClosedPoly
{
public:
    ClosedPoly( char* desc );
    ClosedPoly( int st, float s, float th, char* desc );
    
    inline string GetDescription() { return description; }
    void AddNode( BezNode* node );
    int  CloseCurContour();
    int  Finish();

    int  BuildOsg( osg::Group* airport );

    // Build BTG for airport base for airports with boundary
    int  BuildBtg( float alt_m, TGPolygon* apt_base, TGPolygon* apt_clearing );

    // Build BTG for pavements for airports with no boundary
    int  BuildBtg( float alt_m, superpoly_list* rwy_polys, texparams_list* texparams, TGPolygon* accum, TGPolygon* apt_base, TGPolygon* apt_clearing );

    FeatureList* GetFeatures()
    {
        return &features;
    }

private:
    //osg::DrawArrays* CreatePrimitive( BezContour* contour, osg::Vec3Array* v_pave );
    // convert the BezierPoly to a normal Poly (adding nodes for the curves)
    void CreateConvexHull( void );
    void ConvertContour( BezContour* src, point_list *dst );
    osg::DrawArrays* CreateOsgPrimitive( point_list contour, osg::Vec3Array* vpave );
    void ExpandContour( point_list& src, TGPolygon& dst, double dist );

    bool   is_pavement;
    int    surface_type;
    float  smoothness;
    float  texture_heading;
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
    TGPolygon pre_tess;

    // pavement definitions have multiple linear features (markings and lights for each contour)
    LinearFeature* cur_feature;
    FeatureList features;
};

typedef std::vector <ClosedPoly *> PavementList;

#endif
