#!/usr/bin/perl

$layout = "/stage/fgfs02/curt/layout-cd";
$source = "/stage/fgfs01/ftp/pub/fgfs/Scenery";

@files = `ls $layout/Africa/* $layout/Asia-East/* $layout/Asia-West/* $layout/Europe/* $layout/NorthAmerica-East/* $layout/NorthAmerica-West/* $layout/SouthAmerica-Antartica/* $layout/USA/*`;

foreach $file ( @files ) {
    chomp $file;
    # print "$file\n";

    $base = $file;
    while ( $base =~ m/\// ) {
        $base =~ s/.*\///;
    }

    print "cp $source/$base $file\n";
}
