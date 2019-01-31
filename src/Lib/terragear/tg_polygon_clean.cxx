#include <simgear/debug/logstream.hxx>

#include "tg_polygon.hxx"

tgPolygon tgPolygon::Snap( const tgPolygon& subject, double snap )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    for (unsigned int c = 0; c < subject.Contours(); c++) {
        result.AddContour( tgContour::Snap( subject.GetContour( c ), snap ) );
    }

    return result;
}

tgPolygon tgPolygon::RemoveDups( const tgPolygon& subject )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::RemoveDups( subject.GetContour( c ) ) );
    }

    return result;
}

tgPolygon tgPolygon::RemoveBadContours( const tgPolygon& subject )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        tgContour contour = subject.GetContour(c);
        if ( contour.GetSize() >= 3 ) {
            /* keeping the contour */
            result.AddContour( contour );
        }
    }

    return result;
}

tgPolygon tgPolygon::RemoveCycles( const tgPolygon& subject )
{
    tgPolygon result;
    tgcontour_list contours;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        contours.clear();

        // sglog().setLogLevels( SG_ALL, SG_DEBUG );
        SG_LOG(SG_GENERAL, SG_DEBUG, " REMOVE DUPES FOR CONTOUR : " << c << " with " << subject.ContourSize(c) << " nodes ");
        tgContour::RemoveCycles( subject.GetContour( c ), contours );
        SG_LOG(SG_GENERAL, SG_DEBUG, " REMOVE DUPES FOR CONTOUR : " << c << " COMPLETE");
        //sglog().setLogLevels( SG_ALL, SG_INFO );

        for ( unsigned int i=0; i< contours.size(); i++ ) {
            result.AddContour( contours[i] );
        }
    }

    return result;
}

tgPolygon tgPolygon::SplitLongEdges( const tgPolygon& subject, double dist )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    for ( unsigned c = 0; c < subject.Contours(); c++ )
    {
        result.AddContour( tgContour::SplitLongEdges( subject.GetContour(c), dist ) );
    }

    return result;
}

tgPolygon tgPolygon::StripHoles( const tgPolygon& subject )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    ClipperLib::Paths clipper_result;
    ClipperLib::Clipper c;
    c.Clear();

    for ( unsigned int i = 0; i < subject.Contours(); i++ ) {
        tgContour contour = subject.GetContour( i );
        if ( !contour.GetHole() ) {
            c.AddPath( tgContour::ToClipper( contour ), ClipperLib::PolyType::Clip, true );
        }
    }
    c.Execute(ClipperLib::ClipType::Union, clipper_result, ClipperLib::PolyFillType::EvenOdd, ClipperLib::PolyFillType::EvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    return result;
}

tgPolygon tgPolygon::Simplify( const tgPolygon& subject )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    ClipperLib::Paths clipper_poly = tgPolygon::ToClipper( subject );
    SimplifyPolygons( clipper_poly );

    result = tgPolygon::FromClipper( clipper_poly );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    return result;
}

tgPolygon tgPolygon::RemoveTinyContours( const tgPolygon& subject )
{
    double min_area = SG_EPSILON*SG_EPSILON;
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        tgContour contour = subject.GetContour( c );
        double area = contour.GetArea();

        if ( area >= min_area) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "remove_tiny_contours NO - " << c << " area is " << area << " requirement is " << min_area);
            result.AddContour( contour );
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, "remove_tiny_contours " << c << " area is " << area << ": removing");
        }
    }

    return result;
}

tgPolygon tgPolygon::RemoveSpikes( const tgPolygon& subject )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    
    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::RemoveSpikes( subject.GetContour(c) ) );
    }

    return result;
}

// Move slivers from in polygon to out polygon.
void tgPolygon::RemoveSlivers( tgPolygon& subject, tgcontour_list& slivers )
{
    // traverse each contour of the polygon and attempt to identify
    // likely slivers
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygon::RemoveSlivers()");

    tgPolygon result;
    tgContour contour;
    int       i;

    double angle_cutoff = 10.0 * SGD_DEGREES_TO_RADIANS;
    double area_cutoff = 0.000000001;

    // process contours in reverse order so deleting a contour doesn't
    // foul up our sequence
    for ( i = subject.Contours() - 1; i >= 0; --i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "contour " << i );

        contour = subject.GetContour(i);

        SG_LOG(SG_GENERAL, SG_DEBUG, "  calc min angle for contour " << i);
        double min_angle = contour.GetMinimumAngle();
        SG_LOG(SG_GENERAL, SG_DEBUG, "  min_angle (rad) = " << min_angle );

        double area = contour.GetArea();
        SG_LOG(SG_GENERAL, SG_DEBUG, "  area = " << area );

        if ( ((min_angle < angle_cutoff) && (area < area_cutoff)) ||
           ( area < area_cutoff / 10.0) )
        {
            if ((min_angle < angle_cutoff) && (area < area_cutoff))
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "      WE THINK IT'S A SLIVER! - min angle < 10 deg, and area < 10 sq meters");
            }
            else
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "      WE THINK IT'S A SLIVER! - min angle > 10 deg, but area < 1 sq meters");
            }

            // Remove the sliver from source
            subject.DeleteContourAt( i );

            // And add it to the slive list if it isn't a hole
            if ( !contour.GetHole() ) {
                // move sliver contour to sliver list
                SG_LOG(SG_GENERAL, SG_DEBUG, "      Found SLIVER!");

                slivers.push_back( contour );
            }
        }
    }
}

tgcontour_list tgPolygon::MergeSlivers( tgpolygon_list& polys, tgcontour_list& sliver_list ) {
    tgPolygon poly, result;
    tgContour sliver;
    tgContour contour;
    tgcontour_list unmerged;
    unsigned int original_contours, result_contours;

    for ( unsigned int i = 0; i < sliver_list.size(); i++ ) {
        sliver = sliver_list[i];
        SG_LOG(SG_GENERAL, SG_DEBUG, "Merging sliver = " << i );

        sliver.SetHole( false );

        bool done = false;

        // try to merge the slivers with the list of clipped polys
        for ( unsigned int j = 0; j < polys.size() && !done; j++ ) {
            poly = polys[j];
            original_contours = poly.Contours();
            result = tgContour::Union( sliver, poly );
            result_contours = result.Contours();

            if ( original_contours == result_contours ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "    FOUND a poly to merge the sliver with");
                result.SetMaterial( polys[j].GetMaterial() );
                result.SetTexParams( polys[j].GetTexParams() );
                result.SetId( polys[j].GetId() );
                result.SetPreserve3D( polys[j].GetPreserve3D() );
                polys[j] = result;
                done = true;
            }
        }

        if ( !done ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "couldn't merge sliver " << i );
            unmerged.push_back( sliver );
        }
    }

    return unmerged;
}

