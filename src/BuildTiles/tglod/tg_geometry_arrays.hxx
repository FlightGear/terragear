#ifndef __TG_GEOMETRY_ARRAYS_HXX__
#define __TG_GEOMETRY_ARRAYS_HXX__


#include <simgear/debug/logstream.hxx>

#include <terragear/tg_unique_vec2f.hxx>
#include <terragear/tg_unique_vec3f.hxx>
#include <terragear/tg_unique_vec3d.hxx>


struct VertNormTex {
  VertNormTex() { }
  VertNormTex(const SGVec3d& v, const SGVec3f& n, const SGVec2f& t) :
    vertex(v), normal(n), texCoord(t) { }

  SGVec3d vertex;
  SGVec3f normal;
  SGVec2f texCoord;
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
    void insertPoint(const SGVec3d& center, const std::string& material, const SGVec3d& v, const SGVec3f& n, const SGVec2f& t)
    {
        insertPoint( center, material, VertNormTex(v, n, t) );
    }

    void insertPoint(const SGVec3d& center, const std::string& material, const VertNormTex& v)
    {        
        unsigned vIndex, nIndex, tIndex;

        matPoints::iterator mpi = pts.find( material );
        if ( mpi == pts.end() ) {
            // insert new material
            pts[material] = PointList();
        }
     
        vIndex = vertices.add(v.vertex + center);
        nIndex = normals.add(v.normal);
        tIndex = texcoords.add(v.texCoord);
     
        pts[material].AddPoint( vIndex, nIndex, tIndex );
    }
    
    void insertTriangle(const SGVec3d& center, const std::string& material, const VertNormTex& v0, const VertNormTex& v1, const VertNormTex& v2)
    {        
        unsigned vIndex, nIndex, tIndex;

        matTris::iterator mti = tris.find( material );
        if ( mti == pts.end() ) {
            // insert new material
            tris[material] = PointList();
        }
        
        vIndex = vertices.add(v0.vertex + center);
        nIndex = normals.add(v0.normal);
        tIndex = texcoords.add(v0.texCoord);
        tris[material].AddPoint( vIndex, nIndex, tIndex );

        vIndex = vertices.add(v1.vertex + center);
        nIndex = normals.add(v1.normal);
        tIndex = texcoords.add(v1.texCoord);
        tris[material].AddPoint( vIndex, nIndex, tIndex );

        vIndex = vertices.add(v2.vertex + center);
        nIndex = normals.add(v2.normal);
        tIndex = texcoords.add(v2.texCoord);
        tris[material].AddPoint( vIndex, nIndex, tIndex );        
    }

    void
    insertFanGeometry(const std::string& material,
                      const SGVec3d& center,
                      const std::vector<SGVec3d>& vertices,
                      const std::vector<SGVec3f>& normals,
                      const std::vector<SGVec2f>& texCoords,
                      const int_list& fans_v,
                      const int_list& fans_n,
                      const int_list& fans_tc)
    {
        VertNormTex v0;
        v0.vertex   = vertices[fans_v[0]] + center;
        v0.normal   = normals[fans_n[0]];
        v0.texCoord = texCoords[fans_tc[0]];

        VertNormTex v1;
        v1.vertex   = vertices[fans_v[1]] + center;
        v1.normal   = normals[fans_n[1]];
        v1.texCoord = texCoords[fans_tc[1]];
        
        for (unsigned i = 2; i < fans_v.size(); ++i) {
            VertNormTex v2;
            v2.vertex   = vertices[fans_v[i]] + center;
            v2.normal   = normals[fans_n[i]];
            v2.texCoord = texCoords[fans_tc[i]];
            
            insertTriangle(center, material, v0, v1, v2);
            v1 = v2;
        }
    }

    
    bool insert(const SGBinObject& obj)
    {
        if (obj.get_tris_n().size() < obj.get_tris_v().size() ||
            obj.get_tris_tcs().size() < obj.get_tris_v().size()) {
            SG_LOG(SG_TERRAIN, SG_ALERT, "Group list sizes for triangles do not match! v : " << obj.get_tris_v().size() << " n : " << obj.get_tris_n().size() << " tc : " << obj.get_tris_tcs().size() );
            return false;
        }

        SGVec3d center = obj.get_gbs_center();
        const std::vector<SGVec3d>& vertices  = obj.get_wgs84_nodes();
        const std::vector<SGVec3f>& normals   = obj.get_normals();
        const std::vector<SGVec2f>& texcoords = obj.get_texcoords();

        const group_list& tris_v  = obj.get_tris_v();
        const group_list& tris_n  = obj.get_tris_n();
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
            VertNormTex v0( vertices[ints_v[0]], normals[ints_n[0]], texcoords[ints_tc[0]] );
            VertNormTex v1( vertices[ints_v[1]], normals[ints_n[1]], texcoords[ints_tc[1]] );
            VertNormTex v2( vertices[ints_v[2]], normals[ints_n[2]], texcoords[ints_tc[2]] );
            
            insertTriangle( center, materialName, v0, v1, v2 );
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
    
    unsigned int getTriangleCount( void ) {
        matTris::iterator mti;
        unsigned int num_tris = 0;
        
        for ( mti=pts.begin(); mti != pts.end(); mti++ ) {
            num_tris += (mti->second.vertexIndex.size()/3);
        }
        
        return num_tris;
    }
    
    
    UniqueSGVec3dSet vertices;
    UniqueSGVec3fSet normals;
    UniqueSGVec2fSet texcoords;
    matTris          tris;
    matPoints        pts;    
};

#endif /* __TG_GEOMETRY_ARRAYS_HXX__ */
