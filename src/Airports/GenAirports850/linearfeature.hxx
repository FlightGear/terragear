#ifndef _LINEARFEATURE_H_
#define _LINEARFEATURE_H_

#include <memory>

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_accumulator.hxx>
#include <terragear/tg_light.hxx>
#include "beznode.hxx"


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
#define LF_SOLID_YELLOW_WIDE        (10)
#define LF_TAXI_HOLD_WIDE           (11)
#define LF_DOUBLE_HOLD_WIDE         (12)
#define LF_OTHER_HOLD_WIDE          (13)
#define LF_ILS_HOLD_WIDE            (14)
#define LF_SOLID_YELLOW_HATCH       (19)

#define LF_SOLID_WHITE              (20)
#define LF_CHECKERBOARD_WHITE       (21)
#define LF_BROKEN_WHITE             (22)
#define LF_BROKEN_ROAD              (23)
#define LF_SOLID_ROAD_WIDE          (24)
#define LF_BROKEN_ROAD_WIDE         (25)

#define LF_SOLID_RED                (30)
#define LF_BROKEN_RED               (31)
#define LF_SOLID_RED_WIDE           (32)

#define LF_SOLID_ORANGE             (40)
#define LF_SOLID_BLUE               (41)
#define LF_SOLID_GREEN              (42)

#define LF_B_SOLID_YELLOW           (51)
#define LF_B_BROKEN_YELLOW          (52)
#define LF_B_SOLID_DBL_YELLOW       (53)
#define LF_B_RUNWAY_HOLD            (54)
#define LF_B_OTHER_HOLD             (55)
#define LF_B_ILS_HOLD               (56)
#define LF_B_SAFETYZONE_CENTERLINE  (57)
#define LF_B_SINGLE_LANE_QUEUE      (58)
#define LF_B_DOUBLE_LANE_QUEUE      (59)

#define LF_B_SOLID_YELLOW_WIDE      (60)
#define LF_B_TAXI_HOLD_WIDE         (61)
#define LF_B_RUNWAY_HOLD_WIDE       (62)
#define LF_B_OTHER_HOLD_WIDE        (63)
#define LF_B_ILS_HOLD_WIDE          (64)

#define LF_B_ROAD_EDGE_SOLID        (70)
#define LF_B_ROAD_EDGE_ZIPPER       (71)
#define LF_B_ROAD_EDGE_DASH         (72)
#define LF_B_ROAD_DASH              (73)
#define LF_B_ROAD_EDGE_WIDE         (74)
#define LF_B_ROAD_DASH_WIDE         (75)

#define LF_B_SOLID_RED              (80)
#define LF_B_BROKEN_RED             (81)
#define LF_B_SOLID_RED_WIDE         (82)

#define LF_B_SOLID_ORANGE           (90)
#define LF_B_SOLID_BLUE             (91)
#define LF_B_SOLID_GREEN            (92)

#define LF_BIDIR_GREEN              (101)
#define LF_OMNIDIR_BLUE             (102)
#define LF_UNIDIR_CLOSE_AMBER       (103)
#define LF_UNIDIR_CLOSE_AMBER_PULSE (104)
#define LF_BIDIR_GREEN_AMBER        (105)
#define LF_OMNIDIR_RED              (106)
#define LF_UNIDIR_GREEN             (107)
#define LF_UNIDIR_GREEN_AMBER       (108)

struct Marking
{
public:
    unsigned int type;
    unsigned int start_idx;
    unsigned int end_idx;
};
typedef std::vector<Marking*> MarkingList;

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
typedef std::vector<Lighting*> LightingList;

class LinearFeature
{
public:
    LinearFeature( char* desc, double o ) :
        LinearFeature(std::string(desc ? desc : "none"), o)
    {
    }

    LinearFeature( const std::string& desc, double o ) :
        cur_mark(nullptr),
        cur_light(nullptr),
        description(desc)
    {
        offset = o;
        width = 0;
    }

    ~LinearFeature();

    inline std::string GetDescription() { return description; }

    void AddNode( std::shared_ptr<BezNode> b )
    {
        contour.push_back( b );
    }

    int Finish( bool closed, unsigned int idx );
    int BuildBtg( tgpolygon_list& line_polys, tglightcontour_list& lights, tgAccumulator& accum, bool debug );

private:
    double          offset;
    double          width;

    MarkingList     marks;
    Marking*        cur_mark;

    LightingList    lights;
    Lighting*       cur_light;

    void ConvertContour( const BezContour& src, bool closed );

    // text description
    std::string description;

    // contour definition (each beznode has marking type)
    BezContour  contour;

    // contour definition after bezier interpolation
    tgContour  points;

    tgpolygon_list      marking_polys;
    tglightcontour_list lighting_polys;
};

typedef std::vector<std::shared_ptr<LinearFeature>> FeatureList;

#endif
