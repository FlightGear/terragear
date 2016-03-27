#ifndef __TG_MESH_DEF_HXX__
#define __TG_MESH_DEF_HXX__

// the tg mesh class utilizes two main cgal features

// an arrangement ( with exact constructions ) 
// This defines faces from segments.
// Note that this automatically handles clipped polygon colinear nodes.
// as all of the individual polygon segments are added to the arrangement,
// a segment may be broken into many, as adjacent polys may touch our segment.
// We also modify the arrangement by clustering nodes within a certain radius.
// this is similar ( but better than ) snapping.  snapping may sometimes make
// nodes further away from one another, and can modify topology in unexpected 
// ways. clustering pulls nodes together at their centroid.

// The other is the constrained triangulation
// with required inexact constructions ( for refining )
// it MAY be possible to use exact constructions ( with square root ),
// but it is really slow...

// arrangement
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_landmarks_point_location.h>

// triangulation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

// mesh refinement
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesher_no_edge_refinement_2.h>

// the arrangement uses EPECK, the triangulation is EPICK
typedef CGAL::Exact_predicates_exact_constructions_kernel   meshArrKernel;
typedef CGAL::Arr_segment_traits_2<meshArrKernel>           meshArrTraits;
typedef meshArrKernel::FT                                   meshArr_FT;

typedef meshArrTraits::Point_2                              meshArrPoint;
typedef meshArrTraits::Curve_2                              meshArrSegment;
typedef CGAL::Arrangement_2<meshArrTraits>                  meshArrangement;
typedef meshArrangement::Face_handle                        meshArrFaceHandle;
typedef meshArrangement::Face_const_handle                  meshArrFaceConstHandle;
typedef meshArrangement::Halfedge_const_handle              meshArrHalfedgeConstHandle;
typedef meshArrangement::Ccb_halfedge_const_circulator      meshArrHalfedgeConstCirculator;
typedef meshArrangement::Edge_iterator                      meshArrEdgeIterator;
typedef meshArrangement::Edge_const_iterator                meshArrEdgeConstIterator;
typedef meshArrangement::Vertex_handle                      meshArrVertexHandle;
typedef meshArrangement::Vertex_const_handle                meshArrVertexConstHandle;
typedef meshArrangement::Vertex_iterator                    meshArrVertexIterator;
typedef meshArrangement::Vertex_const_iterator              meshArrVertexConstIterator;
typedef CGAL::Arr_landmarks_point_location<meshArrangement> meshArrLandmarks_pl;

// face info for providing a link back to an arrangement face per triangle
struct tgMeshArrFaceInfo
{
    tgMeshArrFaceInfo() : arrMeshFace(NULL), visited(false) {}
    
    void clear( void ) {
        visited     = false;
        arrMeshFace = (meshArrFaceConstHandle)NULL;
    }
    
    void setFace( meshArrFaceConstHandle f ) {
        arrMeshFace = f;
        visited = true;
    }
    
    bool isVisited( void ) const { 
        return visited;
    }
    
    bool hasFace( void ) const {
        return arrMeshFace != (meshArrFaceConstHandle)NULL;
    }
    
private:
    // internal arrangement / mesh lookup
    meshArrFaceConstHandle arrMeshFace;
    bool                   visited;
    
    // saved data
    std::string            material;
};

// vertex info for per vertex data ( elevation )
struct tgMeshVertexInfo
{
public:    
    tgMeshVertexInfo() { elv = 0; }
    
    void   setElevation( double e ) { elv = e; }
    double getElevation( void ) { return elv; }

private:
    // elevation
    double elv;
    
    // point normal
    double normx;
    double normy;
    double normz;
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel                                 meshTriKernel;
typedef meshTriKernel::Point_2                                                              meshTriPoint;
typedef meshTriKernel::Segment_2                                                            meshTriSegment;

typedef CGAL::Triangulation_vertex_base_with_info_2<tgMeshVertexInfo, meshTriKernel>        meshTriVertexBase;
// The meshTriFaceBase template is a bit complex.
// It needs to be refinable (Delaunay_mesh_face_base_2) 
// with info (Triangulation_face_base_with_info_2), and of course
// derived from plain face base (Constrained_triangulation_face_base_2)
// I found this in a forum - not in any of the CGAL examples.

typedef CGAL::Constrained_triangulation_face_base_2<meshTriKernel>                          Fbb;
typedef CGAL::Triangulation_face_base_with_info_2<tgMeshArrFaceInfo, meshTriKernel, Fbb>    Fb;
//typedef CGAL::Constrained_triangulation_face_base_2<meshTriKernel>                        Fb;

typedef CGAL::Delaunay_mesh_face_base_2<meshTriKernel,Fb>                                   meshTriFaceBase;

typedef CGAL::Triangulation_data_structure_2<meshTriVertexBase,meshTriFaceBase>             meshTriTDS;
typedef CGAL::Exact_intersections_tag                                                       meshTriItag;
typedef CGAL::Constrained_Delaunay_triangulation_2<meshTriKernel, meshTriTDS, meshTriItag>  meshTriCDT;
typedef CGAL::Constrained_triangulation_plus_2<meshTriCDT>                                  meshTriCDTPlus;

typedef meshTriCDTPlus::Edge                                                                meshTriEdge;
typedef meshTriCDTPlus::Vertex_handle                                                       meshTriVertexHandle;
typedef meshTriCDTPlus::Face_handle                                                         meshTriFaceHandle;
typedef meshTriCDTPlus::Face_circulator                                                     meshTriFaceCirculator;
typedef meshTriCDTPlus::Finite_faces_iterator                                               meshTriFaceIterator;
typedef CGAL::Triangle_2<meshTriKernel>                                                     meshTriangle;

typedef CGAL::Delaunay_mesh_size_criteria_2<meshTriCDTPlus>                                 meshCriteria;

// we perform 2 triangulations for the entire tile ( goes back all the way to Curt's original design )
// 1st triangulation adds steiner points to edges - so we end up with extra nodes on boundaries.
// once we generate two tiles, and match their boundaries, we need to retriangulate.
// But this time, we only add nodes in faces, not on edges, so we maintain our matched edges.
typedef CGAL::Delaunay_mesher_2<meshTriCDTPlus, meshCriteria>                               meshRefinerWithEdgeModification;
typedef CGAL::Delaunay_mesher_no_edge_refinement_2<meshTriCDTPlus, meshCriteria>            meshRefinerWithoutEdgeModification;

// finally, we need a face info structure to read from the shapefile metadata - and set in the TDS for triangulation serialization
class meshVertexInfo {
public:
    meshVertexInfo() : id(-1) {}
    
    meshVertexInfo( int i, meshTriTDS::Vertex_handle v ) {
        id        = i;
        vh        = v;
        
        // id 0 is infinite vertex
        if ( id ) {
            pt        = vh->point();
            elevation = vh->info().getElevation();
        }
    }
    
    meshVertexInfo( int i, const OGRPoint* p ) {
        id        = i;
        vh        = meshTriTDS::Vertex_handle();
        pt        = meshTriPoint( p->getX(), p->getY() );
        elevation = p->getZ();
    }
    
    int          getId( void ) const    { return id; }
    meshTriPoint getPoint( void ) const { return pt; }
    double       getX( void ) const     { return pt.x(); }
    double       getY( void ) const     { return pt.y(); }
    double       getZ( void ) const     { return elevation; }

private:    
    int                 id;         // our id
    meshTriVertexHandle vh;
    meshTriPoint        pt;         // 2d point
    double              elevation;  // point elevation
};

class meshFaceInfo {
public:
    meshFaceInfo() : fid(-1) {};    
    meshFaceInfo( int i, meshTriTDS::Face_handle f, CGAL::Unique_hash_map<meshTriTDS::Vertex_handle, int>& map ) {
        fid         = i;
        fh          = f;

        for ( unsigned int i=0; i<3; i++ ) {
            if ( map.is_defined( fh->vertex(i) ) ) {
                vid[i]      = map[fh->vertex(i)];
            } else {
                SG_LOG(SG_GENERAL, SG_INFO, "vertex handle not in map " );
            }
            con[i]      = fh->is_constrained(i) ? 1 : 0;
        }
    }
    
    meshFaceInfo( int i, int vi[3], int ni[3], int c[3] ) {
        fid         = i;
        fh          = meshTriFaceHandle();
        
        for ( unsigned int i=0; i<3; i++ ) {
            vid[i] = vi[i];
            nid[i] = ni[i];
            con[i] = c[i];
        }
    }
    
    void setNeighbors( CGAL::Unique_hash_map<meshTriTDS::Face_handle, int>& map ) {        
        if ( fh != meshTriTDS::Face_handle() ) {
            for ( unsigned int i=0; i<3; i++ ) {
                if ( map.is_defined(fh->neighbor(i)) ) {
                    // SG_LOG(SG_GENERAL, SG_INFO, "ni for " << i << " is " << map[fh->neighbor(i)] );            
                    nid[i] = map[fh->neighbor(i)];
                } else {
                    SG_LOG(SG_GENERAL, SG_INFO, "fh->neighbor(i) not in map" );
                }
            } 
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "fh invalid" );            
        }
    }
    
    void getConstrainedField( char* constrained ) const {
        sprintf(constrained, "%01d,%01d,%01d", con[0], con[1], con[2] );
    }        

    void getVIdxField( char* indicies ) const {
        sprintf(indicies, "%08d,%08d,%08d", vid[0], vid[1], vid[2] );
    }        
    
    void getNIdxField( char* indicies ) const {
        sprintf(indicies, "%08d,%08d,%08d", nid[0], nid[1], nid[2] );
    }
    
    int getFid( void ) const {
        return fid;
    }
    
    int getVid( int i ) const {
        return vid[i];
    }

    int getNid( int i ) const {
        return nid[i];
    }
    
    int getConstrained( int i ) const {
        return con[i];
    }
    
private:
    int                 fid;        // our face id
    meshTriFaceHandle   fh;         // out hanfle
    int                 vid[3];     // vertex ids
    int                 nid[3];     // neighbor face ids
    int                 con[3];     // constrained edges
};


#endif