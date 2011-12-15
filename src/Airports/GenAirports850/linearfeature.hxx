#ifndef _LINEARFEATURE_H_
#define _LINEARFEATURE_H_

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>

#include <Geometry/point3d.hxx>

using std::string;


#define LF_NONE                     (0)
#define LF_SOLID_YELLOW             (1)
#define LF_BROKEN_YELLOW            (2)
#define LF_SOLID_DBL_YELLOW         (3)
#define LF_RUNWAY_HOLD              (4)
#define LF_OTHER_HOLD               (5)
#define LF_ILS_HOLD                 (6)
#define LF_SAFETYZONE_CENTERLINE    (7)
#define LF_SINGLE_LANE_QUEUE        (8)
#define LF_DOUBLE_LANE_QUEUE        (9)

#define LF_B_SOLID_YELLOW           (51)
#define LF_B_BROKEN_YELLOW          (52)
#define LF_B_SOLID_DBL_YELLOW       (53)
#define LF_B_RUNWAY_HOLD            (54)
#define LF_B_OTHER_HOLD             (55)
#define LF_B_ILS_HOLD               (56)
#define LF_B_SAFETYZONE_CENTERLINE  (57)
#define LF_B_SINGLE_LANE_QUEUE      (58)
#define LF_B_DOUBLE_LANE_QUEUE      (59)

#define LF_SOLID_WHITE              (20)
#define LF_CHECKERBOARD_WHITE       (21)
#define LF_BROKEN_WHITE             (22)

#define LF_BIDIR_GREEN              (101)
#define LF_OMNIDIR_BLUE             (102)
#define LF_UNIDIR_CLOSE_AMBER       (103)
#define LF_UNIDIR_CLOSE_AMBER_PULSE (104)
#define LF_BIDIR_GREEN_AMBER        (105)
#define LF_OMNIDIR_RED              (106)

struct Marking
{
public:
    int type;
    int start_idx;
    int end_idx;
};
typedef std::vector <Marking*> MarkingList;

struct Lighting
{
public:
    int type;
    int start_idx;
    int end_idx;
};
typedef std::vector <Lighting*> LightingList;

class LinearFeature
{
public:
    LinearFeature( char* desc, double o )
    {
        if ( desc )
        {
            description = desc;
        }
        else
        {
            description = "none";
        }
        offset = o;
    }

    LinearFeature( string desc, double o )
    {
        description = desc;
        offset = o;
    }

    inline string GetDescription() { return description; }

    void AddNode( BezNode* b )
    {
        contour.push_back( b );
    }

    int Finish();
    int BuildBtg( float alt_m, superpoly_list* line_polys, texparams_list* line_tps, ClipPolyType* line_accum, superpoly_list* lights ); 

private:
    Point3D OffsetPointFirst( Point3D *cur, Point3D *next, double offset_by );
    Point3D OffsetPointMiddle( Point3D *prev, Point3D *cur, Point3D *next, double offset_by );
    Point3D OffsetPointLast( Point3D *prev, Point3D *cur, double offset_by );

    double      offset;
    double      width;

    MarkingList     marks;
    Marking*        cur_mark;

    LightingList    lights;
    Lighting*       cur_light;

    void ConvertContour( BezContour* src );

    // text description
    string description;
    
    // contour definition (each beznode has marking type)
    BezContour  contour;

    // contour definition after bezier interpolation
    point_list  points;
    
    superpoly_list marking_polys;
    texparams_list marking_tps;

    superpoly_list lighting_polys;
};

typedef std::vector <LinearFeature *> FeatureList;

// add this to the class
extern double CalcMarkingVerticies( Point3D *prev, Point3D *cur, Point3D *next, double *dist1, double *dist2 );

#endif

