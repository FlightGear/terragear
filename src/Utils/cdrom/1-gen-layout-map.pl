#!/usr/bin/perl

$tgzdir = "/stage/fgfs07/ftp/pub/fgfs/Scenery-1.0.0/";
$verbose = 0;

if ( $#ARGV != 1 ) {
    print "usage: $0 <reserve> <disk_size_mb>\n";
    print "  - where <reserve> is some amount to reserve on the first disk\n";
    print "    for things like the win32 setup program or aircraft or src.\n";
    print "    At the moment 700 (Mb) works well.\n";
    print "  - and <disk_size_mb> is the size in MB of each individual disk.\n";
    print "    DVD=4500 (Mb), CDROM=700 (Mb)\n";
    exit;
} else {
    $reserve = shift(@ARGV);
    $disksize = shift(@ARGV);
}

@lons = ( "w180", "w170", "w160", "w150", "w140", "w130", "w120",
         "w110", "w100", "w090", "w080", "w070", "w060", "w050",
         "w040", "w030", "w020", "w010", "e000", "e010", "e020",
         "e030", "e040", "e050", "e060", "e070", "e080", "e090",
         "e100", "e110", "e120", "e130", "e140", "e150", "e160",
         "e170" );

@lats = ( "n80", "n70", "n60", "n50", "n40", "n30", "n20", "n10",
         "n00", "s10", "s20", "s30", "s40", "s50", "s60", "s70",
         "s80", "s90" );

# @files = `ls $tgzdir/*.tar.gz`;
@files = ();

for $lon (@lons) {
    for $lat (@lats) {
        push( @files, "$lon$lat.tgz" );
    }
}


$disk = 1;
$total_disk = $reserve;
$total_collection = 0;

foreach $file ( @files ) {
    chomp $file;

    while ( $file =~ m/\// ) {
        $file =~ s/.*\///;
    }

    if ( -f "$tgzdir/$file" ) {
        ($dev, $ino, $mode, $nlink, $uid, $gid, $rdev, $size, $atime,
         $mtime, $ctime, $blksize, $blocks) = stat( "$tgzdir/$file" );

        $kb = $size / 1024;
        $mb = $kb / 1024;
        # printf( "%s %9.2f kb\n", $file, $kb );

        if ( $total_disk + $mb < $disksize ) {
            $total_disk += $mb;
        } else {
            $disk++;
            $total_disk = $mb;
        }
        $total_collection += $mb;

        printf( "Disk%02d/%s", $disk, $file );
        printf( " %.2f %.2f", $mb, $total_disk ) if $verbose;
        printf( "\n" );
    }
}
# print "\n";

printf( "Total size = %.2f mb\n", $total_collection ) if $verbose;
