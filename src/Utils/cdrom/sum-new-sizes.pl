#!/usr/bin/perl

$tgzdir = "/stage/fgfs03/curt/Scenery/Scenery-0.9.5";

if ( $#ARGV < 0 ) {
    die "usage: $0 <layout_dir>\n";
} else {
    $cdromdir = shift(@ARGV);
}

@files = `ls $cdromdir/*.tgz`;
$total = 0;

foreach $file ( @files ) {
    chomp $file;

    while ( $file =~ m/\// ) {
        $file =~ s/.*\///;
    }

    ($dev, $ino, $mode, $nlink, $uid, $gid, $rdev, $size, $atime,
     $mtime, $ctime, $blksize, $blocks) = stat( "$tgzdir/$file" );

    $kb = $size / 1024;
    # printf( "%s %9.2f kb\n", $file, $kb );

    $total += $size;
}
# print "\n";

$mb = $total / (1024*1024);
printf( "Total size = %.2f mb\n", $mb );
