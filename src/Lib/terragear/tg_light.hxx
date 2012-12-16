#ifndef _TG_LIGHT_HXX
#define _TG_LIGHT_HXX

#include <vector>

#include <simgear/math/SGMath.hxx>

class tgLight
{
public:
    SGGeod      pos;
    SGVec3f     norm;
};

typedef std::vector <tgLight>  tglight_list;
typedef tglight_list::iterator tglight_list_iterator;
typedef tglight_list::const_iterator const_tglight_list_iterator;

class tgLightContour
{
public:
    unsigned int ContourSize( void ) const {
        return lights.size();
    }

    void AddLight( SGGeod p, SGVec3f n ) {
        tgLight light;
        light.pos  = p;
        light.norm = n;
        lights.push_back(light);
    }

    void SetElevation( unsigned int i, double elev ) {
        lights[i].pos.setElevationM( elev );
    }

    SGGeod GetNode( unsigned int i ) const {
        return lights[i].pos;
    }

    SGGeod GetPosition( unsigned int i ) const {
        return lights[i].pos;
    }

    SGVec3f GetNormal( unsigned int i ) const {
        return lights[i].norm;
    }

    std::string GetFlag( void ) const {
        return flag;
    }
    void SetFlag( const std::string f ) {
        flag = f;
    }

    std::string GetType( void ) const {
        return type;
    }
    void SetType( const std::string t ) {
        type = t;
    }

    std::vector<SGGeod> GetPositionList( void ) {
        std::vector<SGGeod> positions;

        for (unsigned int i=0; i<lights.size(); i++) {
            positions.push_back( lights[i].pos );
        }

        return positions;
    }

    std::vector<SGVec3f> GetNormalList( void ) {
        std::vector<SGVec3f> normals;

        for (unsigned int i=0; i<lights.size(); i++) {
            normals.push_back( lights[i].norm );
        }

        return normals;
    }

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgLightContour& );

    std::string  type;
    std::string  flag;
    tglight_list lights;
};

typedef std::vector <tgLightContour>  tglightcontour_list;
typedef tglightcontour_list::iterator tglightcontour_list_iterator;
typedef tglightcontour_list::const_iterator const_tglightcontour_list_iterator;

#endif // _TG_LIGHT_HXX