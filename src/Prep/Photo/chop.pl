#!/usr/bin/perl

die "Usage: $0 file xdiv ydiv res\n" if ( $#ARGV != 3 );
    
$file = shift @ARGV;
$xdiv = shift @ARGV;
$ydiv = shift @ARGV;
$res = shift @ARGV;

die "Must start with a .png file\n" if ( $file !~ m/\.png$/ );

# extract image dimensions

$info = `file $file`;
$info =~ s/,//g;
($junk, $type, $junk, $junk, $width, $junk, $height, $junk)
    = split(/\s+/, $info, 8);
print "$width - $height\n";

$basename = $file;
$basename =~ s/\.png$//;

# convert source image to pnm
# `pngtopnm $basename.png > $basename.pnm`

$dx = $width / $xdiv;
$dy = $height / $ydiv;

for ( $j = 0; $j < $ydiv; $j++ ) {
    for ( $i = 0; $i < $xdiv; $i++ ) {
        print "$i $j\n";
        $x = $dx * $i;
        $y = $height - $dy * ($j + 1);
        $outputpnm = sprintf("$basename%X%X.pnm", $i, $j);
        $outputsgi = sprintf("$basename%X%X.sgi", $i, $j);
        $outputrgb = sprintf("$basename%X%X.rgb", $i, $j);
        printf "pnmcut $x $y $dx $dy $basename.pnm | pnmscale -xysize $res $res > $outputpnm\n";
        `pnmcut $x $y $dx $dy $basename.pnm | pnmscale -xysize $res $res > $outputpnm`;
        `convert $outputpnm $outputsgi`;
        unlink($outputpnm);
        rename($outputsgi, $outputrgb);
    }
}
