#!/usr/bin/perl

$tgzdir = "/stage/fgfs01/ftp/pub/fgfs/Scenery";

if ( $#ARGV < 0 ) {
    $defdir = ".";
} else {
    $defdir = shift(@ARGV);
}

@files = `ls $defdir/*.tar.gz`;
$total = 0;

foreach $file ( @files ) {
    chomp $file;

    while ( $file =~ m/\// ) {
        $file =~ s/.*\///;
    }

    ($dev, $ino, $mode, $nlink, $uid, $gid, $rdev, $size, $atime,
     $mtime, $ctime, $blksize, $blocks) = stat( "$tgzdir/$file" );

    $kb = $size / 1024;
    printf( "%s %9.2f kb\n", $file, $kb );

    $total += $size;
}
print "\n";

$mb = $total / (1024*1024);
printf( "Total size = %.2f mb\n", $mb );
