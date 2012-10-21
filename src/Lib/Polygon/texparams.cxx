#include <simgear/io/lowlevel.hxx>

#include "texparams.hxx"

// TEMP TEMP TEMP : Need to get my merge request into simgear

inline void sgReadGeod  ( gzFile fd, SGGeod& var ) {
    double data[3];
    sgReadDouble ( fd, 3, data );
    var = SGGeod::fromDegM( data[0], data[1], data[2] );
}
inline void sgWriteGeod ( gzFile fd, const SGGeod& var ) {
    sgWriteDouble( fd, var.getLongitudeDeg() );
    sgWriteDouble( fd, var.getLatitudeDeg() );
    sgWriteDouble( fd, var.getElevationM() );
}

// TEMP TEMP TEMP

// Send a TexParam to standard output.
std::ostream& operator << (std::ostream &output, const TGTexParams &tp)
{
    // Save the data
    output << tp.ref;
    output << tp.width << " ";
    output << tp.length << " ";
    output << tp.heading << "\n";

    output << tp.minu << " ";
    output << tp.maxu << " ";
    output << tp.minv << " ";
    output << tp.maxv << "\n";

    return output;
}

void TGTexParams::SaveToGzFile(gzFile& fp)
{
    sgWriteGeod( fp, ref );
    sgWriteDouble( fp, width );
    sgWriteDouble( fp, length );
    sgWriteDouble( fp, heading );

    sgWriteDouble( fp, minu );
    sgWriteDouble( fp, maxu );
    sgWriteDouble( fp, minv );
    sgWriteDouble( fp, maxv );
}

void TGTexParams::LoadFromGzFile(gzFile& fp)
{
    sgReadGeod( fp, ref );
    sgReadDouble( fp, &width );
    sgReadDouble( fp, &length );
    sgReadDouble( fp, &heading );

    sgReadDouble( fp, &minu );
    sgReadDouble( fp, &maxu );
    sgReadDouble( fp, &minv );
    sgReadDouble( fp, &maxv );
}
