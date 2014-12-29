// tg_texparams.hxx -- texture parameters class
//
// Written by Curtis Olson, started March 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
#ifndef _TG_TEXPARAMS_HXX_
#define _TG_TEXPARAMS_HXX_

#ifndef __cplusplus
# error This library requires C++
#endif

#include <iostream>
#include <fstream>

#include <simgear/constants.h>
#include <simgear/misc/texcoord.hxx>

typedef enum {
    TG_TEX_UNKNOWN,
    TG_TEX_BY_GEODE,
    TG_TEX_BY_TPS_NOCLIP,
    TG_TEX_BY_TPS_CLIPU,
    TG_TEX_BY_TPS_CLIPV,
    TG_TEX_BY_TPS_CLIPUV,
    TG_TEX_1X1_ATLAS
} tgTexMethod;

class tgTexParams
{
public:
    SGGeod ref;
    double width;
    double length;
    double heading;

    double minu;
    double maxu;
    double minv;
    double maxv;

    double min_clipu;
    double max_clipu;
    double min_clipv;
    double max_clipv;

    tgTexMethod method;

    double center_lat;

    void SaveToGzFile( gzFile& fp ) const;
    void LoadFromGzFile( gzFile& fp );

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgTexParams& );
};

#endif /* _TG_TEXPARAMS_ */