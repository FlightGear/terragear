// tgconstruct_mesh.cxx -- BTG mesh simplification
//
// Written by Peter Sadrozinski, started Dec 2014.
//
// Copyright (C) 2014  Curtis L. Olson  - http://www.flightgear.org/~curt
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
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifdef _MSC_VER
#  include <windows.h>
#endif

#include <simgear/debug/logstream.hxx>

//#include <terragear/tg_polygon.hxx>
//#include <terragear/tg_shapefile.hxx>
//#include <terragear/tg_shapefile.hxx>
//#include <terragear/tg_cgal.hxx>

//#include "tgconstruct_mesh.hxx"

#if 0
tgMesh::tgMesh( const TGAreaDefinitions& ad, const tgAreas& p ) : area_defs(ad), polys(p)
{
    meshObserver obs(mesh, this);
    
    // init the face info list with an unknown material;
    // get a handle to the unbounded face
    //materialLookup.push_back( face_material( mesh.faces_begin(), "unknown" ) );
    
    // start inserting poly outer boundaries for each area type
    for ( current_area = 0; current_area < area_defs.size(); current_area++ ) {
        for( current_poly = 0; current_poly < polys.area_size(current_area); current_poly++ ) {            
            tgPolygon current = polys.get_poly(current_area, current_poly);

            for ( unsigned int contour = 0; contour < current.Contours(); contour++ ) {
                tgContour curContour = current.GetContour( contour );
                if ( !curContour.GetHole() ) {
                    SG_LOG( SG_CLIPPER, SG_INFO, "tgMesh::tgMesh - inserting contour " << contour << " of material " << current.GetMaterial() );
                    
                    Insert( curContour );
                }
            }
        }
    }
    
    // once all of the contours are added, get all of the interior points, and mark faces with correct attributes
    meshLandmarks_pl landmarks_pl;
    landmarks_pl.attach(mesh);
  
    for ( current_area = 0; current_area < area_defs.size(); current_area++ ) {
        for( current_poly = 0; current_poly < polys.area_size(current_area); current_poly++ ) {            
            tgPolygon current = polys.get_poly(current_area, current_poly);
            std::vector<tgPoint> intPoints = current.GetInteriorPoints();

            SG_LOG( SG_CLIPPER, SG_INFO, "tgMesh::tgMesh - found " << intPoints.size() << " interior points" );
            
            for ( unsigned int i=0; i<intPoints.size(); i++ ) {
                // lookup the points face
                CGAL::Object obj = landmarks_pl.locate(intPoints[i]);
                meshArrangement::Face_const_handle      f;
                meshArrangement::Halfedge_const_handle  e;
                meshArrangement::Vertex_const_handle    v;

                if (CGAL::assign(f, obj)) {
                    // point is in face - set the material
                    if ( !f->is_unbounded() ) {
                        materialLookup.push_back( face_material(f, current.GetMaterial()) );
                    }
                } else if (CGAL::assign(e, obj)) {
                    SG_LOG( SG_CLIPPER, SG_INFO, "tgMesh::tgMesh - POINT " << i << " found on edge!" );                    
                } else if (CGAL::assign(v, obj)) {
                    SG_LOG( SG_CLIPPER, SG_INFO, "tgMesh::tgMesh - POINT " << i << " found on vertex!" );                    
                } else {
                    SG_LOG( SG_CLIPPER, SG_INFO, "tgMesh::tgMesh - POINT " << i << " not found!" );                    
                }
            }
        }
    }
}

void tgMesh::Insert( const tgContour& c )
{
    std::vector<meshSegment>    segs;    
    meshPoint                   src,  trg;    
    unsigned int                j;
    
    src  = c.GetPoint(0);
    for ( j = 1; j < c.GetSize(); ++j ) {
        trg = c.GetPoint(j);
        if ( src != trg ) {
            segs.push_back( meshSegment(src, trg) );
        }
        
        src  = trg;
    }
    trg = c.GetPoint(0);    
    if ( src != trg ) {
        segs.push_back( meshSegment( src, trg ) );
    }
    
    insert( mesh, segs.begin(), segs.end() );
}

void tgMesh::AddFaceHandle( meshFaceHandle oldFace, meshFaceHandle newFace, bool isHole )
{
#if 0    
    // lookup the old face
    bool found = false;
    
    for ( unsigned int i=0; i<materialLookup.size(); i++ ) {
        if ( oldFace == materialLookup[i].face ) {
            SG_LOG( SG_CLIPPER, SG_INFO, " Splitting old face " << materialLookup[i].material );
            found = true;
            break;
        }
    }
    
    if (!found) {
        SG_LOG( SG_CLIPPER, SG_INFO, " Splitting old face, but couldn't find it " );
    }
#endif    
}

void meshObserver::after_split_face(meshFaceHandle oldFace, meshFaceHandle newFace, bool isHole )
{
//  pMesh->AddFaceHandle( oldFace, newFace, isHole );
}

std::string tgMesh::GetMaterial( meshFaceHandle fh )
{
    std::string material = "unknown";
    
    for ( unsigned int i=0; i<polyLookup.size(); i++ ) {
        if ( polyLookup[i].face == fh ) {
            tgPolygon found = polys.get_poly( polyLookup[i].area, polyLookup[i].poly );
            material = found.GetMaterial();
            break;
        }
    }
    
    return material;
}

void tgMesh::SaveFace( meshArrangement::Face_const_handle fh, const char* path, const char* layer )
{
    meshArrangement::Ccb_halfedge_const_circulator ccb = fh->outer_ccb();
    meshArrangement::Ccb_halfedge_const_circulator cur = ccb;
    meshArrangement::Halfedge_const_handle         he;
    
    tgContour cont;
    cont.AddPoint( cur->source()->point() );

    do {
        he = cur;
        cont.AddPoint( he->target()->point() );
        
        ++cur;
    } while (cur != ccb);
    
    tgShapefile::FromContour( cont, true, false, path, layer, "poly" );
}

void tgMesh::ToShapefile( const char* path )
{
    // we dump all the arrangement faces to the correct layer...    
    for ( unsigned int i=0; i<materialLookup.size(); i++ ) {
        SaveFace( materialLookup[i].face, path, materialLookup[i].material.c_str() );
    }
}
#endif