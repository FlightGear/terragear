#ifndef _LINEARFEATURE_H_
#define _LINEARFEATURE_H_

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Geometry/point3d.hxx>

#include "texparams.hxx"

#include <osg/Group>

using std::string;

class LinearFeature
{
public:
    LinearFeature( char* desc )
    {
        if ( desc )
        {
            strcpy( description, desc );
        }
        else
        {
            strcpy( description, "none" );
        }

        offset = 0.0f;
        closed = false;
    }
    
    void AddNode( BezNode* b )
    {
        contour.push_back( b );
    }

    void SetClosed()
    {
        closed = true;
    }

    int Finish( osg::Group* airport );
    int BuildBtg( float alt_m, superpoly_list* rwy_polys, texparams_list* texparams, TGPolygon* accum, TGPolygon* apt_base, TGPolygon* apt_clearing );
    
private:
    char  description[256];
    
    // contour definition
    BezContour  contour;

    // TODO : Implement offset
    double      offset;

    bool        closed;
};

typedef std::vector <LinearFeature *> FeatureList;

// add this to the class
extern double CalcMarkingVerticies( Point3D *prev, Point3D *cur, Point3D *next, int wind, int *prev_dir, double *dist1, double *dist2 );

// don't know what to do with these
extern osg::Geode* FinishMarking( osg::Vec3dArray* verticies );
extern osg::Vec3dArray* CheckMarking(int cur_marking, int new_marking, osg::Vec3dArray* v_marking, osg::Group* airport);

#endif

