// srtmchop.cxx -- chop up a srtm tiff file into it's corresponding pieces and stuff
//                them into the workspace directory
//
// Written by Frederic Bouvier, started February 2009.
//
// Copyright (C) 1997  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id:$

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#ifdef _MSC_VER
#  include <direct.h>
#endif

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/misc/sg_dir.hxx>

#include <boost/foreach.hpp>

#include <tiffio.h>

#include <Polygon/point2d.hxx>

#include <zlib.h>

#include <Lib/HGT/srtmbase.hxx>

using std::cout;
using std::endl;
using std::setfill;
using std::setw;
using std::string;
using std::ifstream;
using std::ostringstream;
using std::ios;

#define MAX_HGT_SIZE 6001
class TGSrtmTiff : public TGSrtmBase {
public:
    TGSrtmTiff( const SGPath &file );
    ~TGSrtmTiff();
    bool open( const SGPath &f );
    bool close();

    // load an hgt file
    bool load();
    bool is_opened() const { return opened; }

    virtual short height( int x, int y ) const { return data[x][y]; }

private:
    enum LoadKind { BottomLeft, BottomRight, TopLeft, TopRight };
    TGSrtmTiff( const SGPath &file, LoadKind lk );
    bool pos_from_name( string name, string &pfx, int &x, int &y );

    TIFF* tif;
    LoadKind lkind;
    string prefix, ext;
    SGPath dir;
    bool opened;
    
    // pointers to the actual grid data allocated here
    short int (*data)[MAX_HGT_SIZE];
    short int (*output_data)[MAX_HGT_SIZE];
};

TGSrtmTiff::TGSrtmTiff( const SGPath &file ) {
    lkind = BottomLeft;
    tif = 0;
    remove_tmp_file = false;
    data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    output_data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    opened = TGSrtmTiff::open( file );
}

TGSrtmTiff::TGSrtmTiff( const SGPath &file, LoadKind lk ) {
    lkind = lk;
    tif = 0;
    remove_tmp_file = false;
    output_data = 0;
    if ( lkind == BottomLeft ) {
        data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
        output_data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    } else if ( lkind == TopLeft ) {
        data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    } else if ( lkind == BottomRight ) {
        data = new short int[1][MAX_HGT_SIZE];
    } else /* if ( lkind == TopRight ) */ {
        data = new short int[1][MAX_HGT_SIZE];
    }
    TGSrtmTiff::open( file );
}

TGSrtmTiff::~TGSrtmTiff() {
    delete[] data;
    delete[] output_data;
    if ( remove_tmp_file ) {
        simgear::Dir dir(remove_file_name.dir());
        simgear::PathList files = dir.children(simgear::Dir::TYPE_FILE | simgear::Dir::NO_DOT_OR_DOTDOT);
        BOOST_FOREACH(const SGPath& file, files) {
            unlink( file.c_str() );
        }
        
        rmdir( remove_file_name.dir().c_str() );
    }
    if ( tif )
        TIFFClose( tif );
}

bool TGSrtmTiff::pos_from_name( string name, string &pfx, int &x, int &y ) {
    size_t p = name.find( '_' );
    if ( p == string::npos )
        return false;
    pfx = name.substr( 0, p );
    name.erase( 0, p + 1 );
    p = name.find( '.' );
    if ( p == string::npos )
        return false;
    name.erase( p );
    p = name.find( '_' );
    if ( p == string::npos )
        return false;

    x = atoi( name.substr( 0, p ).c_str() );
    y = atoi( name.substr( p+1 ).c_str() );
    return true;
}

bool TGSrtmTiff::open( const SGPath &f ) {
    SGPath file_name = f;
    ext = file_name.extension();
    dir = file_name.dir();
    int x, y;
    pos_from_name( file_name.file(), prefix, x, y );
    if ( ext == "zip" ) {
        // extract the .zip file to /tmp and point the file name
        // to the extracted file
        SGPath tmp_dir_path = string( tempnam( 0, "srtm" ) );
        simgear::Dir tmp_dir(tmp_dir_path);
        SGPath dummy = tmp_dir.file( "dummy" );
        dummy.create_dir( 0700 );
        cout << "Extracting " << file_name.str() << " to " << tmp_dir_path.str() << endl;
        string command = "unzip -d \"" + tmp_dir_path.str() + "\" " + file_name.base();
        system( command.c_str() );

        simgear::PathList files = tmp_dir.children(simgear::Dir::TYPE_FILE | simgear::Dir::NO_DOT_OR_DOTDOT);
        BOOST_FOREACH(const SGPath& file, files) {
            string ext = file.extension();
            if ( ext == "TIF" || ext == "tif" ) {
                file_name = file;
                break;
            }
        }
        
        remove_tmp_file = true;
        remove_file_name = file_name.str();

        cout << "Proceeding with " << file_name.str() << endl;
    }

    tif = TIFFOpen( file_name.c_str(), "r" );
    if ( !tif ) {
        cout << "ERROR: opening " << file_name.str() << " for reading!" << endl;
        return false;
    }

    // Determine originx/originy from file name
    originx = ( double( x ) - 37.0 ) * 18000.0;
    originy = ( 12.0 - double( y ) ) * 18000.0;
    cout << "  Origin = " << originx << ", " << originy << endl;

    return true;
}

bool TGSrtmTiff::load() {
    int size;
    cols = rows = size = 6000;
    col_step = row_step = 3;

    uint32 w, h, d;
    uint16 dataType;
    uint16 samplesperpixel;
    uint16 bitspersample;

    TIFFGetField( tif, TIFFTAG_IMAGEWIDTH, &w );
    TIFFGetField( tif, TIFFTAG_IMAGELENGTH, &h );
    TIFFGetField( tif, TIFFTAG_IMAGEDEPTH, &d );
    TIFFGetField( tif, TIFFTAG_SAMPLESPERPIXEL, &samplesperpixel );
    TIFFGetField( tif, TIFFTAG_BITSPERSAMPLE, &bitspersample );
    TIFFGetField( tif, TIFFTAG_DATATYPE, &dataType );

    tdata_t buf = _TIFFmalloc( TIFFScanlineSize( tif ) );
    if ( lkind == BottomLeft ) {
        uint32 row = 0;
        for ( ; row < h; row++ ) {
            TIFFReadScanline( tif, buf, row );
            uint32 col = 0;
            for ( ; col < w; col++ ) {
                int16 v = ((int16*)buf)[col];
                if ( v == -32768 )
                    v = 0;
                data[col][6000-1-row] = v;
            }
            for ( ; col < 6000; col++ ) {
                data[col][6000-1-row] = 0;
            }
        }
        for ( ; row < 6000; row++ ) {
            uint32 col = 0;
            for ( ; col < 6000; col++ ) {
                data[col][6000-1-row] = 0;
            }
        }
        int x1 = int( originx / 18000.0 ) + 37,
            y1 = int( 12 - ( originy / 18000.0 ) ),
            x2 = x1 + 1,
            y2 = y1 - 1;
        if ( x2 > 72 )
            x2 -= 72;
        {
            ostringstream name;
            name << prefix << "_" << std::setfill( '0' ) << std::setw( 2 ) << x2 << "_" << std::setfill( '0' ) << std::setw( 2 ) << y1 << "." << ext;
            SGPath f = dir;
            f.append( name.str() );
            if ( f.exists() ) {
                TGSrtmTiff s( f.str(), BottomRight );
                s.load();
                s.close();
                for ( int i = 0; i < 6000; ++i ) {
                    data[6000][i] = s.data[0][i];
                }
            } else {
                for ( int i = 0; i < 6000; ++i ) {
                    data[6000][i] = 0;
                }
            }
        }
        if ( y2 != 0 ) {
            ostringstream name;
            name << prefix << "_" << std::setfill( '0' ) << std::setw( 2 ) << x1 << "_" << std::setfill( '0' ) << std::setw( 2 ) << y2 << "." << ext;
            SGPath f = dir;
            f.append( name.str() );
            if ( f.exists() ) {
                TGSrtmTiff s( f.str(), TopLeft );
                s.load();
                s.close();
                for ( int i = 0; i < 6000; ++i ) {
                    data[i][6000] = s.data[i][0];
                }
            } else {
                for ( int i = 0; i < 6000; ++i ) {
                    data[i][6000] = 0;
                }
            }
        } else {
            for ( int i = 0; i < 6000; ++i ) {
                data[i][6000] = data[i][6000-1];
            }
        }
        if ( y2 != 0 ) {
            ostringstream name;
            name << prefix << "_" << std::setfill( '0' ) << std::setw( 2 ) << x2 << "_" << std::setfill( '0' ) << std::setw( 2 ) << y2 << "." << ext;
            SGPath f = dir;
            f.append( name.str() );
            if ( f.exists() ) {
                TGSrtmTiff s( f.str(), TopRight );
                s.load();
                s.close();
                data[6000][6000] = s.data[0][0];
            } else {
                data[6000][6000] = 0;
            }
        } else {
            data[6000][6000] = data[6000][6000-1];
        }
    } else if ( lkind == TopLeft ) {
        TIFFReadScanline( tif, buf, 0 );
        uint32 col = 0;
        for ( ; col < w; col++ ) {
            int16 v = ((int16*)buf)[col];
            if ( v == -32768 )
                v = 0;
            data[col][0] = v;
        }
        for ( ; col < 6000; col++ ) {
            data[col][0] = 0;
        }
    } else if ( lkind == BottomRight ) {
        uint32 row = 0;
        for ( ; row < h; row++ ) {
            TIFFReadScanline( tif, buf, row );
            int16 v = ((int16*)buf)[0];
            if ( v == -32768 )
                v = 0;
            data[0][6000-1-row] = v;
        }
        for ( ; row < 6000; row++ ) {
            data[0][6000-1-row] = 0;
        }
    } else /* if ( lkind == TopRight ) */ {
        if ( h == 6000 ) {
            TIFFReadScanline( tif, buf, h-1 );
            int16 v = ((int16*)buf)[0];
            if ( v == -32768 )
                v = 0;
            data[0][0] = v;
        } else {
            data[0][0] = 0;
        }
    }
    _TIFFfree(buf);

    return true;
}

bool TGSrtmTiff::close() {
    if ( tif )
        TIFFClose( tif );
    tif = 0;
    return true;
}

int main(int argc, char **argv) {
    sglog().setLogLevels( SG_ALL, SG_WARN );

    if ( argc != 3 ) {
        cout << "Usage " << argv[0] << " <hgt_file> <work_dir>"
             << endl;
        cout << endl;
        exit(-1);
    }

    string hgt_name = argv[1];
    string work_dir = argv[2];

    SGPath sgp( work_dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    TGSrtmTiff hgt( hgt_name );
    hgt.load();
    hgt.close();

    point2d min, max;
    min.x = hgt.get_originx() / 3600.0 + SG_HALF_BUCKET_SPAN;
    min.y = hgt.get_originy() / 3600.0 + SG_HALF_BUCKET_SPAN;
    SGBucket b_min( min.x, min.y );

    max.x = (hgt.get_originx() + hgt.get_cols() * hgt.get_col_step()) / 3600.0 
        - SG_HALF_BUCKET_SPAN;
    max.y = (hgt.get_originy() + hgt.get_rows() * hgt.get_row_step()) / 3600.0 
        - SG_HALF_BUCKET_SPAN;
    SGBucket b_max( max.x, max.y );

    if ( b_min == b_max ) {
        hgt.write_area( work_dir, b_min );
    } else {
        SGBucket b_cur;
        int dx, dy, i, j;

        sgBucketDiff(b_min, b_max, &dx, &dy);
        cout << "HGT file spans tile boundaries (ok)" << endl;
        cout << "  dx = " << dx << "  dy = " << dy << endl;

        if ( (dx > 50) || (dy > 50) ) {
            cout << "somethings really wrong!!!!" << endl;
            exit(-1);
        }

        for ( j = 0; j <= dy; j++ ) {
            for ( i = 0; i <= dx; i++ ) {
                b_cur = sgBucketOffset(min.x, min.y, i, j);
                hgt.write_area( work_dir, b_cur );
            }
        }
    }

    return 0;
}
