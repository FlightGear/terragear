#ifndef _LINEARFEATURE_H_
#define _LINEARFEATURE_H_

#include <terragear/polygon_set/tg_polygon_set.hxx>
#include <terragear/tg_light.hxx>
#include <terragear/vector_intersections/tg_intersection_generator.hxx>

#include "beznode.hxx"

// Linear Feature Markings (from apt.dat spec)

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

// Runway Markings (FG Specific)
#define RWY_BORDER                  (1000)
#define RWY_THRESH                  (1001)
#define RWY_DISP_TAIL               (1002)
#define RWY_TZONE                   (1003)
#define RWY_AIM                     (1004)
#define RWY_CENTERLINE              (1005)

class Airport;

struct Marking
{
public:
    unsigned int type;
    unsigned int start_idx;
    unsigned int end_idx;
};
typedef std::vector <Marking*> MarkingList;

struct Lighting
{
public:
    unsigned int type;
    unsigned int start_idx;
    unsigned int end_idx;

    // 1 for unidirectional
    // 2 for bidirectional
    // 0 for omnidirectional
    int LightDirection()
    {
        if (type == 103 || type == 104) return 1;
        else if (type == 101 || type == 105) return 2;
        return 0;
    }
};
typedef std::vector <Lighting*> LightingList;

class LinearFeature
{
public:
    LinearFeature( const char* desc, double o )
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

    LinearFeature( std::string desc, double o )
    {
        description = desc;
        offset = o;
    }

    ~LinearFeature();

    double GetWidth( unsigned int type );
    static int GetTextureInfo(unsigned int info, bool cap, std::string& material, double& atlas_startu, double& atlas_endu, double& atlas_startv, double& atlas_endv, double& v_dist);
    
    inline std::string GetDescription() { return description; }

    void AddNode( BezNode* b )
    {
        contour.push_back( b );
    }

    int Finish( Airport* ap, bool closed, double width = 0.0f );
    
    void GetPolys( tgPolygonSetList& polys );
    void GetCapPolys( tgPolygonSetList& polys );
    void GetLights( tglightcontour_list& lights );
    

private:
    unsigned int CheckMarkStart(BezNode* curNode);
    unsigned int CheckMarkChange(BezNode* curNode, unsigned int cur_edge);    
    void         ConvertContour( BezContour* src, bool closed );

    double          offset;
    double          width;

    MarkingList     marks;
    Marking*        cur_mark;

    LightingList    lights;
    Lighting*       cur_light;

    // text description
    std::string description;

    // contour definition (each beznode has marking type)
    BezContour  contour;

    // contour definition after bezier interpolation
    cgalPoly_Polygon    points;
    
    tgPolygonSetList    marking_polys;
    tgPolygonSetList    cap_polys;      // lower priority than the marks themselves
    tglightcontour_list lighting_polys;
};

typedef std::vector <LinearFeature *> FeatureList;

#endif
