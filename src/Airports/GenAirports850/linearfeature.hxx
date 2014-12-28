#ifndef _LINEARFEATURE_H_
#define _LINEARFEATURE_H_

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_accumulator.hxx>
#include <terragear/tg_light.hxx>
#include <terragear/tg_intersection_generator.hxx>

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
    unsigned int cap_end_idx;
    unsigned int repeat_start_idx;
    unsigned int repeat_end_idx;
    unsigned int cap_start_idx;
    bool cap_started;
};
typedef std::vector <Marking*> MarkingList;

struct Lighting
{
public:
    unsigned int type;
    unsigned int start_idx;
    unsigned int end_idx;

    bool IsDirectional()
    {
        return (type == 103 || type == 104) ? true : false;
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
    static int GetTextureInfo(unsigned int info, std::string& material, double& atlas_start, double& atlas_end, double& v_dist);
    
    inline std::string GetDescription() { return description; }

    void AddNode( BezNode* b )
    {
        contour.push_back( b );
    }

    int Finish( Airport* ap, bool closed, double width = 0.0f );
    
    void GetPolys( tgpolygon_list& polys );
    void GetCapPolys( tgpolygon_list& polys );
    void GetLights( tglightcontour_list& lights );
    

private:
    double          offset;
    double          width;

    MarkingList         marks;
    Marking*            cur_mark;
    
    LightingList        lights;
    Lighting*           cur_light;
    
    double AddMarkingPolyCapStart( const SGGeod& prev_inner, const SGGeod& prev_outer, const SGGeod& cur_outer, const SGGeod& cur_inner, std::string material, double width, double v_dist, double heading, double atlas_start, double atlas_end, double v_start, double v_end );
    double AddMarkingPolyRepeat( const SGGeod& prev_inner, const SGGeod& prev_outer, const SGGeod& cur_outer, const SGGeod& cur_inner, std::string material, double width, double v_dist, double heading, double atlas_start, double atlas_end, double v_start, double v_end );
    double AddMarkingPolyCapEnd( const SGGeod& prev_inner, const SGGeod& prev_outer, const SGGeod& cur_outer, const SGGeod& cur_inner, std::string material, double width, double v_dist, double heading, double atlas_start, double atlas_end, double v_start, double v_end );
    
    double AddMarkingStartTriRepeat( const SGGeod& prev, const SGGeod& cur_outer, const SGGeod& cur_inner, std::string material, double width, double v_dist, double heading, double atlas_start, double atlas_end, double v_start, double v_end );
    
    double   GetCapDist( unsigned int type );
    Marking* CheckStartCap(BezNode* curNode);
    Marking* CheckEndCap(BezNode* curNode, Marking* curMark);
    void     FinishStartCap(const SGGeod& curLoc, Marking* cur_mark);

    
    
    
    unsigned int CheckMarkStart(BezNode* curNode);
    unsigned int CheckMarkChange(BezNode* curNode, unsigned int cur_edge);
    void         GenerateMarkingPolys(void);
    
    
    void         GenerateIntersectionTris(void);
    //void         GenerateNonIntersectingPolys(void);
    
    
    
    void GetMarkInfo( unsigned int type, double& width, std::string& material, double& atlas_start, double& atlas_end, double& v_dist );
    void ConvertContour( Airport* ap, BezContour* src, bool closed );

    // text description
    std::string description;

    // contour definition (each beznode has marking type)
    BezContour  contour;

    // contour definition after bezier interpolation - still used for lights - TODO: convert to tggraphnode_list / edge
    tgContour  points;
    
    // all linear features are stored in the lf_intersector
    //tgIntersectionGenerator lf_insersector;
    
    tgpolygon_list      marking_polys;
    tgpolygon_list      cap_polys;      // lower priority than the marks themselves
    tglightcontour_list lighting_polys;
};

typedef std::vector <LinearFeature *> FeatureList;

#endif