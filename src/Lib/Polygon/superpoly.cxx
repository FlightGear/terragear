// superpoly.cxx -- Manage all aspects of a rendered polygon
//
// Written by Curtis Olson, started June 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: superpoly.cxx,v 1.6 2004-11-19 22:25:50 curt Exp $


#include "superpoly.hxx"
#include <simgear/debug/logstream.hxx>


// Constructor
TGSuperPoly::TGSuperPoly() :
    flag( "" )
{
}


// Destructor
TGSuperPoly::~TGSuperPoly()
{
}

// erase the "super" polygon
void TGSuperPoly::erase()
{
    material = "";
    poly.erase();
    normals.erase();
    texcoords.erase();
    tris.erase();
    face_normals.clear();
}

// Friends for serialization
std::ostream& operator<< ( std::ostream& output, const TGSuperPoly& sp )
{
    int     nFaceNormals;
    int     nFaceAreas;

    // Save the data
    output << sp.material << "\n";
    output << sp.poly;
    output << sp.normals;
    output << sp.texcoords;

    output << sp.tris;
    output << sp.tri_idxs;

    nFaceNormals = sp.face_normals.size();
    output << nFaceNormals << "\n";
    for ( int i = 0; i < nFaceNormals; i++ ) {
        output << sp.face_normals[i];
    }

    nFaceAreas = sp.face_areas.size();
    output << nFaceAreas;
    for ( int i = 0; i < nFaceAreas; i++ ) {
        output << sp.face_areas[i] << " ";
    }
    output << "\n";

    if ( sp.flag.empty() ) {
        output << "none\n";
    } else {
        output << sp.flag << "\n";
    }
    
    return output;
}

std::istream& operator>> ( std::istream& input, TGSuperPoly& sp )
{
    int     nFaceNormals;
    int     nFaceAreas;
    Point3D normal;
    double  area;

    // Load the data
    input >> sp.material;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: material " << sp.material );

    input >> sp.poly;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: poly " << sp.poly );

    input >> sp.normals;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: normals " << sp.normals );

    input >> sp.texcoords;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: texcoords " << sp.texcoords );

    input >> sp.tris;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: tris " << sp.tris );

    input >> sp.tri_idxs;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: tri_idxs " << sp.tri_idxs );

    input >> nFaceNormals;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: nFaceNormals " << nFaceNormals );

    for ( int i = 0; i < nFaceNormals; i++ ) {
        input >> normal;
        sp.face_normals.push_back(normal);
    }

    input >> nFaceAreas;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: nFaceAreas " << nFaceAreas );
    for ( int i = 0; i < nFaceAreas; i++ ) {
        input >> area;
        sp.face_areas.push_back(area);
    }

    input >> sp.flag;
    SG_LOG(SG_GENERAL, SG_ALERT, "\t\tsp: flag " << sp.flag );

    return input;
}

std::ostream& operator<< ( std::ostream& output, const TGPolyNodes& pn )
{
    int     nContours;
    int     nPoints;

    // Save the data
    nContours = pn.poly.size();
    output << nContours << "\n";
    for(int i=0; i<nContours; i++) {
        nPoints = pn.poly[i].size();
        output << nPoints << "\n";
        for (int j=0; j<nPoints; j++) {
            output << pn.poly[i][j] << " ";
        }
        output << "\n";
    }

    return output;
}


std::istream& operator>> ( std::istream& input, TGPolyNodes& pn )
{
    int     nContours;
    int     nPoints;
    int     point;

    // Load the data
    input >> nContours;
    for(int i=0; i<nContours; i++) {
        int_list    points;

        input >> nPoints;
        for (int j=0; j<nPoints; j++) {
            input >> point;
            points.push_back( point );
        }
        pn.poly.push_back( points );
    }

    return input;
}