#ifndef __TG_INTERSECTION_EDGE_HXX__
#define __TG_INTERSECTION_EDGE_HXX__

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
    
    bool IsOriginating(void) const {
        return originating; 
    }

private:
    tgIntersectionEdge*     edge;
    bool                    originating;        // edge originates at this node
    double                  heading;            // edge heading from this node 
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

class tgIntersectionEdge
{
public:
    tgIntersectionEdge( tgIntersectionNode* s, tgIntersectionNode* e, double w, unsigned int t, const std::string& dr );
    
    bool operator==(const tgIntersectionEdge& e) { 
        return ((start == e.start) && (end == e.end)); 
    }

    tgRectangle GetBoundingBox( void ) const;
    const char* GetDatasource( void ) const {
        return datasource;
    }
    
    void AddBottomRightConstraint( const tgRay& c ) {
        constrain_br.push_back( c );
    }
    void AddBottomLeftConstraint( const tgRay& c ) {
        constrain_bl.push_back( c );
    }
    void AddTopLeftConstraint( const tgRay& c ) {
        constrain_tl.push_back( c );
    }
    void AddTopRightConstraint( const tgRay& c ) {
        constrain_tr.push_back( c );
    }
    
    typedef enum {
        ADDCON_INCOMPLETE = 0,
        ADDCON_COMPLETE_EDGE = 1,
        ADDCON_COMPLETE_SIDE = 2
    } AddCon_e;
    
    void SetLeftConstraint( bool originating, const std::list<SGGeod>& cons );
    void SetRightConstraint( bool originating, const std::list<SGGeod>& cons );
    void ApplyConstraint( bool apply );
    
    void Complete( void );
    
    double GetHeading( bool originating ) const;
    double GetLength( void ) const;
    
    tgRay GetTopRightConstraint( bool originating ) const {
        if ( originating ) {
            if ( !constrain_tr.empty() ) {
                return constrain_tr[0];
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetTopRightConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        } else {
            if ( !constrain_bl.empty() ) {
                return constrain_bl[0];
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetTopRightConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        }
    }
    
    tgRay GetTopLeftConstraint( bool originating ) const {
        if ( originating ) {
            if ( !constrain_tl.empty() ) {
                return constrain_tl[0];
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetTopLeftConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        } else {
            if ( !constrain_br.empty() ) {
                return constrain_br[0];
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetTopLeftConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        }
    }
    
    tgRay GetBottomRightConstraint( bool originating ) const {
        if ( originating ) {
            if ( !constrain_br.empty() ) {
                return constrain_br[0];
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetBottomRightConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        } else {
            if ( !constrain_tl.empty() ) {
                return constrain_tl[0];
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetBottomRightConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        }
    }
    
    tgRay GetBottomLeftConstraint( bool originating ) const {
        if ( originating ) {
            if ( !constrain_bl.empty() ) {
                return constrain_bl[0];
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::GetBottomLeftConstraint : but we don't have any");
                SGGeod invalid;
                return tgRay( invalid, 0 );
            }
        } else {
            if ( !constrain_tr.empty() ) {
                return constrain_tr[0];
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

    tgIntersectionEdge* Split( tgIntersectionNode* newEnd );
    
    void ToShapefile( void ) const;
    tgSegment ToSegment( void ) const;
    
    void IntersectConstraintsAndSides(tgIntersectionEdgeInfo* cur);
    
    tgPolygon CreatePolygon(int& type, double& heading, double& dist, double& width, SGGeod& texref, const char* dbg_layer);
    tgPolygon GetPoly(const char* prefix);
    
    void DumpConstraint( const char* layer, const char* label, const std::list<SGGeod>& contour ) const;
    
    tgIntersectionNode* start;
    tgIntersectionNode* end;
    double              width;
    unsigned int        type;
    
    SGGeod              botLeft, botRight;
    SGGeod              topLeft, topRight;
    
    SGGeod              conBotLeft, conBotRight;
    SGGeod              conTopLeft, conTopRight;
    
    tgLine              side_l;
    tgLine              side_r;
        
    tgray_list          constrain_br;
    tgray_list          constrain_bl;
    tgray_list          constrain_tl;
    tgray_list          constrain_tr;
    
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
    
    std::list<SGGeod>   left_contour;
    std::list<SGGeod>   right_contour;
    
    unsigned long int   id;
    
private:
    SGGeod IntersectCorner( const SGGeod& pos, tgray_list& constraint1, tgray_list& constraint2, tgLine& side, 
                            const char* c1name, const char* c2name, const char* sname );
    
    std::string         debugRoot;
    char                datasource[64];
};

typedef std::vector<tgIntersectionEdge*>    tgintersectionedge_list;
typedef tgintersectionedge_list::iterator   tgintersectionedge_it;

#endif /* __TG_INTERSECTION_EDGE_HXX__ */