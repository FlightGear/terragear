#include <simgear/misc/sg_path.hxx>

#include "hgt.hxx"

int main() {
    // SGPath path3( "/stage/fgfs03/SRTM/United_States/N38W124/3arcsec/N38W124.hgt.gz" );
    SGPath path3( "/home/curt/trash/N38W124.hgt.gz" );

    // TGHgt hgt1( 1, path1 );
    TGHgt hgt3( 3, path3 );

    // hgt1.load();
    hgt3.load();

    // SGBucket b(-79.5, 33.5);
    // hgt1.write_area( ".", b );
    hgt3.write_whole_ascii( "w124n38.txt.gz" );
}
