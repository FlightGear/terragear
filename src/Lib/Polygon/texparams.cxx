#include <simgear/io/lowlevel.hxx>

#include "texparams.hxx"

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
