#!/usr/bin/perl

use GD;

$version = "0.9.7";
if ( $#ARGV < 0 ) {
    push( @ARGV, "/stage/fgfs01/ftp/pub/fgfs/Scenery-" . $version );
}

$ftpurl = "ftp://ftp.flightgear.org/pub/fgfs/Scenery-" . $version;
$htmlout = "scenery-" . $version . ".html";
$mapout = "map-" . $version . ".png";
$outputdir = "mapresult/";

$daysecs = 3600*24;

# load raw image
open( INPUT, "rawmap.png" ) || die "cannot open raw image\n";
$im = newFromPng GD::Image( INPUT );
($width, $height) = $im->getBounds();
close( INPUT );

$red1 = $im->colorAllocate(100,30,30);
$red2 = $im->colorAllocate(255,0,0);
$red3 = $im->colorAllocate(180,30,30);

$orange1 = $im->colorAllocate(100,70,30);
$orange2 = $im->colorAllocate(255,165,0);

$yellow1 = $im->colorAllocate(100,100,30);
$yellow2 = $im->colorAllocate(255,255,0);

$green1 = $im->colorAllocate(30,100,30);
$green2 = $im->colorAllocate(0,255,0);

$blue1 = $im->colorAllocate(30,30,100);
$blue2 = $im->colorAllocate(0,0,255);

$xstep = $width / 36.0;
$ystep = $height / 18.0;

# draw background grid
for ( $i = 0; $i <= $width; $i += $xstep ) {
    $im->line( $i, 0, $i, $height, $red1 );
}

for ( $i = 0; $i <= $height; $i += $ystep ) {
    $im->line( 0, $i, $width, $i, $red1 );
}

# draw equater and GMT lines
$im->line( $width / 2, 0, $width / 2, $height, $red3 );
$im->line( 0, $height / 2, $width, $height / 2, $red3 );

# create html file
open ( HTML, ">$outputdir/$htmlout" ) || 
    die "cannot open $outputdir/$htmlout\n";
print HTML "<HTML>\n";
print HTML "<TITLE>FGFS Scenery Downloads Version $version</TITLE>\n";
print HTML "<BODY>\n";
print HTML "<H2>FGFS Scenery Downloads Version $version</H2>\n";
print HTML "Click on any of the 10x10 degree chunks in the image below to\n";
print HTML "download that area.<BR>\n";
print HTML "An area with no corresponding link means that area is all ocean\n";
print HTML "so there is nothing to download.  (Or if a rebuild is in\n";
print HTML "progress, that chunk may not yet be generated.)\n";
print HTML "<P>\n";
print HTML "<IMG SRC=\"$mapout\" WIDTH=\"$width\" HEIGHT=\"$height\" ";
print HTML "USEMAP=\"#map\">\n";
print HTML "<MAP NAME=\"map\">\n";

$toggle = 0;
foreach $dir ( @ARGV ) {
    @files = `ls $dir/*.tgz`;

    foreach $file ( @files ) {
        chop($file);

        ($dev, $ino, $mode, $nlink, $uid, $gid, $rdev, $size, $atime,
         $mtime, $ctime, $blksize, $blocks) = stat($file);

        $mb = $size / (1024*1024);
        # print "$file size = $mb\n";

        # recover the modification date from the stat
        ($sec, $min, $hour, $mday, $mon, $year, $wday, $yday, $isdst) =
            localtime($mtime);

        $date = sprintf("%2d/%02d/%02d", $mon + 1, $mday, 1900 + $year);

        $age = (time() - $mtime) / $daysecs;

        if ( !$toggle ) {
            if ( $age < 7 ) {
                $color1 = $green1;
                $color2 = $green2;
            } elsif ( $age < 14 ) {
                $color1 = $yellow1;
                $color2 = $yellow2;
            } else {
                $color1 = $orange1;
                $color2 = $orange2;
            }
        } else {
            if ( $age < 7 ) {
                $color1 = $orange1;
                $color2 = $orange2;
            } elsif ( $age < 14 ) {
                $color1 = $yellow1;
                $color2 = $yellow2;
            } else {
                $color1 = $orange1;
                $color2 = $orange2;
            }
        }

        $file =~ s/.*\///g;
        $file =~ s/.tgz//;
        # print "$file\n";
        ($ew, $lon, $ns, $lat) = $file =~ m/(\w)(\d\d\d)(\w)(\d\d)/;
        # print "$ew $lon, $ns, $lat\n";

        if ( $ew eq "w" ) {
            $lon = $lon * -1;
        } else {
            $lon = $lon * 1;
        }

        if ( $ns eq "s" ) {
            $lat = $lat * -1;
        } else {
            $lat = $lat * 1;
        }

        # print "$lon $lat\n";
        $x1 = ($lon + 180) * $xstep / 10.0;
        $y1 = $height - ($lat + 90) * $ystep / 10.0;
        $x2 = ($lon + 10 + 180) * $xstep / 10.0;
        $y2 = $height - ($lat + 10 + 90) * $ystep / 10.0;

        $im->line($x1, $y1, $x2, $y2, $color1);
        $im->line($x1, $y2, $x2, $y1, $color1);
        $im->rectangle($x1, $y1, $x2, $y2, $color2);

        # $y1 = $height - $y1;
        # $y2 = $height - $y2;
        print HTML "<AREA SHAPE=rect COORDS=$x1,$y2,$x2,$y1 ";
        print HTML "HREF=$ftpurl/$file.tgz ";
        printf(HTML "ALT=\"%s  %.2f Mb  $date\">\n", $file, $mb);
    }

    $toggle = !$toggle;
}

# write out gif
$png_data = $im->png;
open( OUTPUT, ">$outputdir/$mapout" ) || 
    die "cannot open output $outputdir/$mapout\n";
binmode OUTPUT;
print OUTPUT $png_data;
close OUTPUT;

# finish off html file
print HTML "</MAP>\n";
print HTML "</BODY>\n";
print HTML "</HTML>\n";
close(HTML);

