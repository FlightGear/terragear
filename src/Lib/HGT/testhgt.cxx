#include <simgear/misc/sg_path.hxx>

#include "hgt.hxx"

int main() {
    // SGPath path1( "/stage/fgfs03/SRTM/United_States/N33W080/1arcsec/N33W080.hgt.gz" );
    SGPath path3( "N37W123/3arcsec/N37W123.hgt.gz" );

    // TGHgt hgt1( 1, path1 );
    TGHgt hgt3( 3, path3 );

    // hgt1.load();
    hgt3.load();

    // SGBucket b(-79.5, 33.5);
    // hgt1.write_area( ".", b );
    hgt3.write_whole_ascii( "test.ahgt" );
}
