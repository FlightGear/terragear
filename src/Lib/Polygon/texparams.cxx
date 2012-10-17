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
    sgWritePoint3D( fp, ref );
    sgWriteDouble( fp, width );
    sgWriteDouble( fp, length );
    sgWriteDouble( fp, heading );

    sgWriteDouble( fp, minu );
    sgWriteDouble( fp, maxu );
    sgWriteDouble( fp, minv );
    sgWriteDouble( fp, maxv );
}

// Read a polygon from input buffer.
std::istream& operator >> (std::istream &input, TGTexParams &tp)
{
    // Load the data
    input >> tp.ref;
    input >> tp.width;
    input >> tp.length;
    input >> tp.heading;

    input >> tp.minu;
    input >> tp.maxu;
    input >> tp.minv;
    input >> tp.maxv;

    return input;
}

void TGTexParams::LoadFromGzFile(gzFile& fp)
{
    sgReadPoint3D( fp, ref );
    sgReadDouble( fp, &width );
    sgReadDouble( fp, &length );
    sgReadDouble( fp, &heading );

    sgReadDouble( fp, &minu );
    sgReadDouble( fp, &maxu );
    sgReadDouble( fp, &minv );
    sgReadDouble( fp, &maxv );
}
