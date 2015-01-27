#ifndef __TG_GEOMETRY_ARRAYS_HXX__
#define __TG_GEOMETRY_ARRAYS_HXX__


#include <simgear/debug/logstream.hxx>

#include <terragear/tg_unique_vec2f.hxx>
#include <terragear/tg_unique_vec3f.hxx>
#include <terragear/tg_unique_vec3d.hxx>
#include <terragear/tg_polygon.hxx>


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Fuzzy_sphere.h>


// search tree kernet - just using double for number type
typedef CGAL::Simple_cartesian<double>                      gaK;

// the 3d point (SGVec3d), stored in the tree with its index
typedef gaK::Point_2                                        gaPoint;
typedef boost::tuple<gaPoint,unsigned int>                  gaPointWithIndex;

typedef CGAL::Search_traits_2<gaK>                          gaTraitsBase;
typedef CGAL::Search_traits_adapter<gaPointWithIndex, CGAL::Nth_of_tuple_property_map<0, gaPointWithIndex>,gaTraitsBase>  gaTraits;

typedef CGAL::Orthogonal_k_neighbor_search<gaTraits>        gaNeighborSearch;
typedef CGAL::Fuzzy_sphere<gaTraits>                        gaFuzzyCircle;
typedef gaNeighborSearch::Tree                              gaTree;

struct VertNormTex {
  VertNormTex() { }
  VertNormTex(const SGVec3d& v, const SGVec3f& n, const SGVec2f& t) :
    vertex(v), normal(n), texCoord(t) { }

  SGVec3d vertex;
  SGVec3f normal;
  SGVec2f texCoord;
};

struct VertNormTexIndex {
  VertNormTexIndex() { }
  VertNormTexIndex(unsigned int v, unsigned int n, unsigned int t) :
    vertex(v), normal(n), texCoord(t) { }

  unsigned int vertex;
  unsigned int normal;
  unsigned int texCoord;
};

struct PointList {
public: 
    void AddPoint( unsigned vi, unsigned ni, unsigned tci ) {
        vertexIndex.push_back( vi );
        normalIndex.push_back( ni );
        texcoordIndex.push_back( tci );
    }

    std::vector<unsigned> vertexIndex;
    std::vector<unsigned> normalIndex;
    std::vector<unsigned> texcoordIndex;
};

typedef std::map< const std::string, PointList > matPoints;
typedef std::map< const std::string, PointList > matTris;

class Arrays {
public:
    unsigned int addVertex( const SGGeod& min, const SGGeod& max, const SGVec3d& v ) {
        // we compare the vertices in 2d 
        // ( 3d sphere picks up false positives before we stitch, 
        //   and we need to know the tile boundaries )
        
        SGGeod          node = SGGeod::fromCart(v);
        gaPoint         pt( node.getLongitudeDeg(), node.getLatitudeDeg() );
        unsigned int    index;
        std::list<gaPointWithIndex>             result;
        std::list<gaPointWithIndex>::iterator   it;
        
        // if node is near the tile border, make sure it isn't a dupe
        if ( (fabs ( node.getLongitudeDeg() - min.getLongitudeDeg() ) < 0.00001) ||
             (fabs ( node.getLongitudeDeg() - max.getLongitudeDeg() ) < 0.00001) ||
             (fabs ( node.getLatitudeDeg()  - min.getLatitudeDeg() )  < 0.00001) || 
             (fabs ( node.getLatitudeDeg()  - max.getLatitudeDeg() )  < 0.00001) ) {
            //SG_LOG(SG_TERRAIN, SG_ALERT, "Found border node " << std::setprecision(8) << node << " min is " << min << " max is " << max );
                
            // first search the tree for the vertex
            gaFuzzyCircle search = gaFuzzyCircle( pt, 0.0000005 );
            
            // perform the query
            vertexTree.search( std::back_inserter( result ), search );
        } 

        if ( result.empty() ) {
            // ad new vertex to the tree
            index = vertexVector.size();
            vertexVector.push_back( v );

            gaPointWithIndex pi( pt, index );
            vertexTree.insert( pi );
        } else {
            for ( it = result.begin(); it != result.end(); it++ ) {
                SG_LOG(SG_TERRAIN, SG_ALERT, "Found " << result.size() << " points in search circle : " << std::setprecision(16) << 
                    "(" << pt.x() << "," << pt.y() << ")  found (" <<
                    boost::get<0>(*it).x() << "," << boost::get<0>(*it).y() << ")" );
            }
            
            // return the index of the nearest neighbor
            index = boost::get<1>( *(result.begin()) );
        }
        
        return index;
    }
    
    void insertPoint( const SGGeod& min, const SGGeod& max, const SGVec3d& center, const std::string& material, const SGVec3d& v, const SGVec3f& n, const SGVec2f& t)
    {
        insertPoint( min, max, center, material, VertNormTex(v, n, t) );
    }

    void insertPoint( const SGGeod& min, const SGGeod& max, const SGVec3d& center, const std::string& material, const VertNormTex& v)
    {        
        unsigned vIndex, nIndex, tIndex;

        matPoints::iterator mpi = pts.find( material );
        if ( mpi == pts.end() ) {
            // insert new material
            pts[material] = PointList();
        }
     
        vIndex = addVertex(min, max, v.vertex + center);
        nIndex = normals.add(v.normal);
        tIndex = texcoords.add(v.texCoord);
     
        pts[material].AddPoint( vIndex, nIndex, tIndex );
    }
    
    void insertTriangle(const SGGeod& min, const SGGeod& max, const SGVec3d& center, const std::string& material, const VertNormTex& v0, const VertNormTex& v1, const VertNormTex& v2)
    {        
        unsigned vIndex, nIndex, tIndex;

        matTris::iterator mti = tris.find( material );
        if ( mti == pts.end() ) {
            // insert new material
            tris[material] = PointList();
        }
        
        vIndex = addVertex(min, max, v0.vertex + center);
        nIndex = normals.add(v0.normal);
        tIndex = texcoords.add(v0.texCoord);
        tris[material].AddPoint( vIndex, nIndex, tIndex );

        vIndex = addVertex(min, max, v1.vertex + center);
        nIndex = normals.add(v1.normal);
        tIndex = texcoords.add(v1.texCoord);
        tris[material].AddPoint( vIndex, nIndex, tIndex );

        vIndex = addVertex(min, max, v2.vertex + center);
        nIndex = normals.add(v2.normal);
        tIndex = texcoords.add(v2.texCoord);
        tris[material].AddPoint( vIndex, nIndex, tIndex );
    }

    void insertTriangle( const std::string& material, const VertNormTexIndex& i0, const VertNormTexIndex& i1, const VertNormTexIndex& i2 )
    {
        matTris::iterator mti = tris.find( material );
        if ( mti == pts.end() ) {
            // insert new material
            tris[material] = PointList();
        }
        
        tris[material].AddPoint( i0.vertex, i0.normal, i0.texCoord );
        tris[material].AddPoint( i1.vertex, i1.normal, i1.texCoord );
        tris[material].AddPoint( i2.vertex, i2.normal, i2.texCoord );
    }
    
    void
    insertFanGeometry(const std::string& material,
                      const SGGeod& min,
                      const SGGeod& max, 
                      const SGVec3d& center,
                      const std::vector<SGVec3d>& fan_vertices,
                      const std::vector<SGVec3f>& fan_normals,
                      const std::vector<SGVec2f>& fan_texCoords,
                      const int_list& fans_v,
                      const int_list& fans_n,
                      const int_list& fans_tc)
    {
        std::map< unsigned int, unsigned int >  vertexMap;
        std::map< unsigned int, unsigned int >  normalMap;
        std::map< unsigned int, unsigned int >  texCoordMap;

        VertNormTexIndex v0, v1, v2;
                
        for ( unsigned int i=0; i<fan_vertices.size(); i++ ) {
            vertexMap[i] = addVertex( min, max, fan_vertices[i] + center );
        }
        for ( unsigned int i=0; i<fan_normals.size(); i++ ) {
            normalMap[i] = normals.add( fan_normals[i] );
        }
        for ( unsigned int i=0; i<fan_texCoords.size(); i++ ) {
            texCoordMap[i] = texcoords.add( fan_texCoords[i] );
        }

        v0.vertex   = vertexMap[fans_v[0]];
        v0.normal   = normalMap[fans_n[0]];
        v0.texCoord = texCoordMap[fans_tc[0]];

        v1.vertex   = vertexMap[fans_v[1]];
        v1.normal   = normalMap[fans_n[1]];
        v1.texCoord = texCoordMap[fans_tc[1]];
        
        for (unsigned i = 2; i < fans_v.size(); ++i) {
            v2.vertex   = vertexMap[fans_v[i]];
            v2.normal   = normalMap[fans_n[i]];
            v2.texCoord = texCoordMap[fans_tc[i]];
            
            insertTriangle(material, v0, v1, v2);
            v1 = v2;
        }
    }

    
    bool insert(const SGGeod& min, const SGGeod& max, const SGBinObject& obj)
    {
        if (obj.get_tris_n().size() < obj.get_tris_v().size() ||
            obj.get_tris_tcs().size() < obj.get_tris_v().size()) {
            SG_LOG(SG_TERRAIN, SG_ALERT, "Group list sizes for triangles do not match! v : " << obj.get_tris_v().size() << " n : " << obj.get_tris_n().size() << " tc : " << obj.get_tris_tcs().size() );
            return false;
        }

        // insert the .btg vertexes into the array tree.  remember the new index
        // duplicate vertex ( shared between btg will be dropped )
        // 2nd btg indexes will not match the geometry - need to look them up
        std::map< unsigned int, unsigned int >  vertexMap;
        std::map< unsigned int, unsigned int >  normalMap;
        std::map< unsigned int, unsigned int >  texCoordMap;
        SGVec3d center  = obj.get_gbs_center();
        
        // first, read in the vertex information
        for ( unsigned int i=0; i<obj.get_wgs84_nodes().size(); i++ ) {
            SGVec3d vertex = obj.get_wgs84_nodes()[i]+center;
            vertexMap[i] = addVertex( min, max, vertex );
        }
        for ( unsigned int i=0; i<obj.get_normals().size(); i++ ) {
            SGVec3f normal = obj.get_normals()[i];
            normalMap[i] = normals.add( normal );
        }
        for ( unsigned int i=0; i<obj.get_texcoords().size(); i++ ) {
            SGVec2f texCoord = obj.get_texcoords()[i];
            texCoordMap[i] = texcoords.add( texCoord );
        }
        
        // now add the geometry
        const group_list& tris_v      = obj.get_tris_v();
        const group_list& tris_n      = obj.get_tris_n();
        const group_tci_list& tris_tc = obj.get_tris_tcs();
        
        for (unsigned grp = 0; grp < tris_v.size(); ++grp) {
            // verify int list size is 3 for triangles
            if ( tris_v[grp].size() != 3 ) {
                SG_LOG(SG_TERRAIN, SG_ALERT, "Triangle size != 3");
                return false;
            }
            
            const int_list& ints_v  = tris_v[grp];
            const int_list& ints_n  = tris_n[grp];
            const int_list& ints_tc = tris_tc[grp][0];

            std::string materialName = obj.get_tri_materials()[grp];
            VertNormTexIndex v0( vertexMap[ints_v[0]], normalMap[ints_n[0]], texCoordMap[ints_tc[0]] );
            VertNormTexIndex v1( vertexMap[ints_v[1]], normalMap[ints_n[1]], texCoordMap[ints_tc[1]] );
            VertNormTexIndex v2( vertexMap[ints_v[2]], normalMap[ints_n[2]], texCoordMap[ints_tc[2]] );
            
            insertTriangle( materialName, v0, v1, v2 );
        }
        
        if ( obj.get_strips_v().size() ) {
            SG_LOG(SG_TERRAIN, SG_ALERT, "Strips not supported!");
            return false;
        }
        
        if (obj.get_fans_v().size() ) {
            SG_LOG(SG_TERRAIN, SG_ALERT, "Fans not supported!");
            return false;
        }
        
        return true;
    }
    
    unsigned int getTriangleCount( void ) const {
        matTris::const_iterator mti;
        unsigned int num_tris = 0;
        
        for ( mti=pts.begin(); mti != pts.end(); mti++ ) {
            num_tris += (mti->second.vertexIndex.size()/3);
        }
        
        return num_tris;
    }
    
    const std::vector<SGVec3d>& getVertexList( void ) const {
        return vertexVector;
    }
    
    gaTree                                  vertexTree;
    std::vector<SGVec3d>                    vertexVector;

    UniqueSGVec3fSet     normals;
    UniqueSGVec2fSet     texcoords;
    matTris              tris;
    matPoints            pts;    
};

#endif /* __TG_GEOMETRY_ARRAYS_HXX__ */
