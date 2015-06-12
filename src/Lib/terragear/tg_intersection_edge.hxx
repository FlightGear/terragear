#ifndef __TG_INTERSECTION_EDGE_HXX__
#define __TG_INTERSECTION_EDGE_HXX__

#include "tg_polygon.hxx"

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

typedef std::set<tgIntersectionEdgeInfo *, EdgeInfoPtrComp> tgintersectionedgeinfo_list;
typedef tgintersectionedgeinfo_list::iterator               tgintersectionedgeinfo_it;

#define FLAGS_INTERSECTED_BOTTOM_CONSTRAINTS       (0x00000001)
#define FLAGS_INTERSECTED_TOP_CONSTRAINTS          (0x00000002)
#define FLAGS_INTERSECT_CONSTRAINTS_COMPLETE       (0x00000003)

#define FLAGS_TEXTURED                             (0x00000004)


class tgIntersectionEdge
{
public:
    tgIntersectionEdge( tgIntersectionNode* s, tgIntersectionNode* e, double w, int z, unsigned int t, const std::string& dr );
    
    bool operator==(const tgIntersectionEdge& e) { 
        return ((start == e.start) && (end == e.end)); 
    }

    tgRectangle GetBoundingBox( void ) const;
    const char* GetDatasource( void ) const {
        return datasource;
    }
    
    bool Verify( unsigned long int f );
    
    void SetBottomRightConstraint( const tgRay& c ) {
        if ( br_set ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::SetBottomRightConstraint : already set");
        }
        constrain_br = c;
        br_set = true;
    }
    void SetBottomLeftConstraint( const tgRay& c ) {
        if ( bl_set ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::SetBottomLeftConstraint : already set");
        }
        constrain_bl= c;
        bl_set = true;
    }
    void SetTopLeftConstraint( const tgRay& c ) {
        if ( tl_set ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::SetTopLeftConstraint : already set");
        }
        constrain_tl = c;
        tl_set = true;
    }
    void SetTopRightConstraint( const tgRay& c ) {
        if ( tr_set ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::SetTopRightConstraint : already set");
        }
        constrain_tr = c;
        tr_set = true;
    }
    
    typedef enum {
        ADDCON_INCOMPLETE = 0,
        ADDCON_COMPLETE_EDGE = 1,
        ADDCON_COMPLETE_SIDE = 2
    } AddCon_e;
    
    void SetLeftConstraint( bool originating, const std::list<SGGeod>& cons );
    void SetRightConstraint( bool originating, const std::list<SGGeod>& cons );

    void SetLeftProjectList( bool originating, const std::list<SGGeod>& pl );
    void SetRightProjectList( bool originating, const std::list<SGGeod>& pl );

    void ApplyConstraint( bool apply );
    
    int  GetZorder( void ) const { return zorder; }
    void Complete( void );
    
    double GetHeading( bool originating ) const;
    double GetGeodesyLength( void ) const;
    
    tgRay GetTopRightConstraint( bool originating ) const {
        if ( originating ) {
            if ( tr_set ) {
                return constrain_tr;
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetTopRightConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        } else {
            if ( bl_set ) {
                return constrain_bl;
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetTopRightConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        }
    }
    
    tgRay GetTopLeftConstraint( bool originating ) const {
        if ( originating ) {
            if ( tl_set ) {
                return constrain_tl;
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetTopLeftConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        } else {
            if ( br_set ) {
                return constrain_br;
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetTopLeftConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        }
    }
    
    tgRay GetBottomRightConstraint( bool originating ) const {
        if ( originating ) {
            if ( br_set ) {
                return constrain_br;
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetBottomRightConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        } else {
            if ( tl_set ) {
                return constrain_tl;
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetBottomRightConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        }
    }
    
    tgRay GetBottomLeftConstraint( bool originating ) const {
        if ( originating ) {
            if ( bl_set ) {
                return constrain_bl;
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetBottomLeftConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        } else {
            if ( tr_set ) {
                return constrain_tr;
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetBottomLeftConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        }
    }
    
    tgLine GetRightSide( bool originating ) const {
        if ( originating ) {
            return side_r;
        } else {
            return side_l; 
        }
    }

    tgLine GetLeftSide( bool originating ) const {
        if ( originating ) {
            return side_l;
        } else {
            return side_r; 
        }
    }

    tgIntersectionEdge* Split( bool originating, tgIntersectionNode* newNode );
    
    void ToShapefile( void ) const;
    tgSegment ToSegment( void ) const;
    
    void IntersectConstraintsAndSides(tgIntersectionEdgeInfo* cur);
    
    tgPolygon CreatePolygon(int& type, double& heading, double& dist, double& width, SGGeod& texref, const char* dbg_layer);
    tgPolygon GetPoly(const char* prefix);
    
    void DumpConstraint( const char* layer, const char* label, const std::list<SGGeod>& contour ) const;
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
    
    SGGeod              botLeft, botRight;
    SGGeod              topLeft, topRight;
    
    SGGeod              conBotLeft, conBotRight;
    SGGeod              conTopLeft, conTopRight;
    
    tgLine              side_l;
    tgLine              side_r;
        
    tgRay               constrain_br;
    tgRay               constrain_bl;
    tgRay               constrain_tl;
    tgRay               constrain_tr;

    bool                bl_set;
    bool                br_set;
    bool                tl_set;
    bool                tr_set;
    
    std::list<SGGeod>   constrain_msbl;
    std::list<SGGeod>   constrain_msbr;
    std::list<SGGeod>   constrain_mstl;
    std::list<SGGeod>   constrain_mstr;

    bool                msbl_valid;
    bool                msbr_valid;
    bool                mstl_valid;
    bool                mstr_valid;
    
    bool                msbl_set;
    bool                msbr_set;
    bool                mstl_set;
    bool                mstr_set;
    
    std::list<SGGeod>   projectlist_msbl;
    std::list<SGGeod>   projectlist_msbr;
    std::list<SGGeod>   projectlist_mstl;
    std::list<SGGeod>   projectlist_mstr;
    
//    bool                msblpl_valid;
//    bool                msbrpl_valid;
//    bool                mstlpl_valid;
//    bool                mstrpl_valid;
    
    bool                msblpl_set;
    bool                msbrpl_set;
    bool                mstlpl_set;
    bool                mstrpl_set;
    
    std::list<SGGeod>   left_contour;
    std::list<SGGeod>   right_contour;
    
    unsigned long int   id;
    unsigned long int   flags;
    
private:    
    tgPolygon           poly;
    std::string         debugRoot;
    char                datasource[64];
};

typedef std::vector<tgIntersectionEdge*>    tgintersectionedge_list;
typedef tgintersectionedge_list::iterator   tgintersectionedge_it;

#endif /* __TG_INTERSECTION_EDGE_HXX__ */