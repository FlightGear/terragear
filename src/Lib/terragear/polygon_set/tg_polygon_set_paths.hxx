#ifndef __TG_POLYGON_SET_PATHS_HXX__
#define __TG_POLYGON_SET_PATHS_HXX__

/* thPaths : used to convert CGAL arrangement faces created from untructed geometry
 * into a valid collection of faces for CGAL PolygonSet_2.
 * 
 * The reason is CGAL arrangement faces may not be simple - they can be reletively 
 * simple.
 * 
 * CGAL PolygonSet Polygons must be simple.
 */

// remember each face as we tranverse from outside in.  every halfege should have a
// hole on one side, and a polygon on the other.
class tgPathsFaceType {
public:
    tgPathsFaceType( cgalPoly_FaceConstHandle h, bool ih ) : face(h), isHole(ih) {}
    tgPathsFaceType() {};
    
    cgalPoly_FaceConstHandle    face;
    bool                        isHole;
};

// This is really just for debuging.  It's impossible to Print a CGAL Face handle.
// so we assign an ID to each handle, and print that...
class tgPathsFaceID {
public:
    tgPathsFaceID( cgalPoly_FaceConstHandle h, unsigned long i ) : face(h), id(i) {}
    tgPathsFaceID() {};
    
    cgalPoly_FaceConstHandle    face;
    unsigned long               id;
};

// a junction occurs at a point when an inside face touches the outside face.  
// we need to assign a priority to every path leaving such a vertex to decide 
// how to continue traversing a ccb - We may leave the arrangement ccb to create
// a simple polygon, then generate a different polygon from the part of the CCB
// we jumped off of...
class tgPathsJunction {
public:
    typedef enum {
        UNKNOWN,
        SAME_OUT,
        SAME_IN,
        SAME_IN_AND_OUT
    } pathPriority_e;
    
    tgPathsJunction( cgalPoly_HeConstHandle h, pathPriority_e p ) : priority(p) 
    {
        startCcb = h->ccb();
    }
    
    cgalPoly_CcbHeConstCirculator startCcb;
    pathPriority_e                priority;
};

// This represents a single path - it will be used to construct a Polygon 
// once completed
class tgPolygonSetPath {
public:
    tgPolygonSetPath( cgalPoly_CcbHeConstCirculator sccb, bool isHole );
    
    cgalPoly_CcbHeConstCirculator   startCcb;   // starting ccb
    cgalPoly_VertexConstHandle      endVertex;  // end condition
    tgPathsFaceType                 inner;      // the incident face
    tgPathsFaceType                 outer;      // the outer face ( MUST be the same throughout )
    std::vector<cgalPoly_Point>     nodes;      // nodes making up the path
    std::vector<cgalPoly_Point>     junctions;  // junction Nodes ( for debugging )
    bool                            complete;   // path forms a loop
    unsigned long                   id;         // for debugging
    
    void toShapefile( const char* ds );
    
private:
    static unsigned long            cur_id;
};

// Here's the helper class to traverse the arrangement faces, and generate 
// the polygons
class tgPolygonSetPaths
{
public:
    tgPolygonSetPaths(const cgalPoly_Arrangement& arr);
    ~tgPolygonSetPaths();
    
    unsigned int                        numPaths( void ) const { return paths.size(); } 
    void                                traversePaths( void  );
    void                                getPolys( std::vector<cgalPoly_Polygon>& boundaries, std::vector<cgalPoly_Polygon>& holes ) const;
    
private:    
    void                                setVisited( cgalPoly_HeConstHandle he) ;
    bool                                isVisited( cgalPoly_HeConstHandle he );
    void                                identifyFaces( const cgalPoly_Arrangement& arr );
    void                                dumpFaces( void ) const;
    unsigned long                       lookupFace( cgalPoly_FaceConstHandle h ) const;
    void                                printFace( const char* layer, cgalPoly_FaceConstHandle fh ) const;

private:
    std::vector<cgalPoly_HeConstHandle> visited;
    std::vector<tgPolygonSetPath *>     paths;
    std::vector<tgPathsFaceID>          faces;
};

#endif /* __TG_POLYGON_SET_PATHS_HXX__ */
