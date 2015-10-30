#ifndef __TG_INTERSECTION_EDGE_HXX__
#define __TG_INTERSECTION_EDGE_HXX__

#include <ogrsf_frmts.h>

// temp temp temp
#include <terragear/tg_polygon.hxx>

#include <terragear/polygon_set/tg_polygon_set.hxx>

#include "tg_constraint.hxx"

typedef int (*tgIntersectionGeneratorTexInfoCb)(unsigned int info, bool cap, std::string& material, double& atlas_startu, double& atlas_endu, double& atlas_startv, double& atlas_endv, double& v_dist);


// forward declarations
class tgIntersectionEdge;
class tgIntersectionNode;

class tgIntersectionEdgeInfo
{
public:
    tgIntersectionEdgeInfo( bool orig, tgIntersectionEdge* e );
    
    bool operator< (const tgIntersectionEdgeInfo* e) const;
    
    const tgIntersectionEdge* GetEdge( void ) const {
        return edge;
    }
    tgIntersectionEdge* GetEdge( void ) {
        return edge;
    }
    
    double GetHeading(void) const {
        return heading;
    }
    
    double GetGeodesyHeading(void) const {
        return geodesy_heading;
    }
    
    tgRay GetDirectionRay( void ) const;
    
    bool IsOriginating(void) const {
        return originating; 
    }
    bool IsStartCap(void) const;
    bool IsEndCap(void) const;
    
    bool IsTextured(void) const;
    
    double Texture( double vEnd, tgIntersectionGeneratorTexInfoCb texInfoCb, double ratio );
    void   TextureStartCap( tgIntersectionGeneratorTexInfoCb texInfoCb );
    void   TextureEndCap( tgIntersectionGeneratorTexInfoCb texInfoCb );

private:
    tgIntersectionEdge*     edge;
    bool                    originating;        // edge originates at this node
    bool                    textured;
    double                  heading;            // edge heading from this node 
    double                  geodesy_heading;
};

struct EdgeInfoPtrComp
{
    bool operator()(const tgIntersectionEdgeInfo* lhs, const tgIntersectionEdgeInfo* rhs) const  { 
        bool result = true;
        if( lhs->GetHeading() >= rhs->GetHeading() ) {
            result = false;
        }
        return result;
    }
};

typedef std::set<tgIntersectionEdgeInfo *, EdgeInfoPtrComp> tgintersectionedgeinfo_set;
typedef tgintersectionedgeinfo_set::iterator                tgintersectionedgeinfo_it;

typedef std::vector<tgIntersectionEdgeInfo *>               tgintersectionedgeinfo_vector;
typedef std::map<double, tgintersectionedgeinfo_vector>     tgintersectionedgeinfo_map;

#define FLAGS_INTERSECTED_BOTTOM_CONSTRAINTS       (0x00000001)
#define FLAGS_INTERSECTED_TOP_CONSTRAINTS          (0x00000002)
#define FLAGS_INTERSECT_CONSTRAINTS_COMPLETE       (0x00000003)

#define FLAGS_TEXTURED                             (0x00000004)



class tgIntersectionEdge
{
public:
    tgIntersectionEdge( tgIntersectionNode* s, tgIntersectionNode* e, double w, int z, unsigned int t, const std::string& db );
    
    bool operator==(const tgIntersectionEdge& e) { 
        return ((start == e.start) && (end == e.end)); 
    }

    tgRectangle GetBoundingBox( void ) const;
    const char* GetDataset( void ) const {
        return debugDataset.c_str();
    }
    
    bool IntersectWithBisector( bool originating, bool right, const tgConstraint& bisector, edgeArrPoint& ce_end_intersect, double& ce_end_dist, edgeArrPoint& ce_side_intersect, double& ce_side_dist );
    bool VerifyIntersectionLocation( bool originating, const tgConstraint& bisector, const edgeArrPoint& verify_pos );
        
    void GenerateSideConstraints( void );
    void Generate( void );
    bool Verify( unsigned long int f );
    void AddDebugPoint( const SGGeod& pos, const char* desc );
    void AddDebugPoint( const edgeArrPoint& pos, const char* desc );
    
    void SetLeftConstraint( bool originating, const std::list<edgeArrPoint>& cons );
    void SetRightConstraint( bool originating, const std::list<edgeArrPoint>& cons );

    void SetLeftProjectList( bool originating, const std::list<edgeArrPoint>& pl );
    void SetRightProjectList( bool originating, const std::list<edgeArrPoint>& pl );

    void ApplyConstraint( bool apply );
    
    int  GetZorder( void ) const { return zorder; }
    void Complete( void );
    
    double GetHeading( bool originating ) const;
    double GetGeodesyLength( void ) const;
        
    edgeArrLine GetRightSide( bool originating ) const {
        if ( originating ) {
            return side_r;
        } else {
            return side_l; 
        }
    }

    edgeArrLine GetLeftSide( bool originating ) const {
        if ( originating ) {
            return side_l;
        } else {
            return side_r; 
        }
    }

    edgeArrPoint GetStart( bool originating ) const;
    
    void AddConstraint( ConstraintPos_e pos, tgConstraint cons );
    void DumpArrangement( OGRLayer* skeleton_lid, OGRLayer* constraints_lid, OGRLayer* startv_lid, OGRLayer* poly_lid );
    
    tgIntersectionEdge* Split( bool originating, tgIntersectionNode* newNode );
    
    tgLine    ToLine( void ) const;
    CGAL::Vector_2<edgeArrKernel> ToVector( void ) const;
    
    tgPolygonSet GetPoly(const char* prefix);
    
    bool IsTextured( void ) const {
        return (flags & FLAGS_TEXTURED);
    }
    double Texture( bool originating, double vEnd, tgIntersectionGeneratorTexInfoCb texInfoCb, double ratio );
    void   TextureCap( bool originating, tgIntersectionGeneratorTexInfoCb texInfoCb );
    
    tgIntersectionNode* start;
    tgIntersectionNode* end;
    double              width;
    int                 zorder;
    unsigned int        type;
    
    edgeArrLine         side_l;
    edgeArrLine         side_r;

    std::vector<edgeArrPoint>   constrain_msbl;
    std::vector<edgeArrPoint>   constrain_msbr;
    std::vector<edgeArrPoint>   constrain_mstl;
    std::vector<edgeArrPoint>   constrain_mstr;
    
    std::vector<edgeArrPoint>   projectlist_msbl;
    std::vector<edgeArrPoint>   projectlist_msbr;
    std::vector<edgeArrPoint>   projectlist_mstl;
    std::vector<edgeArrPoint>   projectlist_mstr;
    
    unsigned long int   id;
    unsigned long int   flags;
    
private:    
    cgalPoly_Polygon    poly;
    tgPolygonSetMeta    meta;
    
    cgalPoly_Point      vStart;    
    cgalPoly_Point      texRefTopRight;
    cgalPoly_Point      texRefBotLeft;
    std::vector<tgConstraint>  constraints[NUM_CONSTRAINTS];
    
    std::string         debugDataset;
    std::string         debugPointLayer;
    std::string         debugLineLayer;
    std::string         debugConsPointLayer;
    std::string         debugConsLineLayer;
};

typedef std::vector<tgIntersectionEdge*>    tgintersectionedge_list;
typedef tgintersectionedge_list::iterator   tgintersectionedge_it;

#endif /* __TG_INTERSECTION_EDGE_HXX__ */