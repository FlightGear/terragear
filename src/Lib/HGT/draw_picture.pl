#!/usr/bin/perl

use GD;

$input = shift( @ARGV );
$output = shift( @ARGV );

print " < $input > $output\n";

# load data
open( INPUT, "<$input" );
$width = <INPUT>; chomp($width);
$height = <INPUT>; chomp($height);
$min = 9999;
$max = -9999;

for ( $i = 0; $i < $height; $i++ ) {
    print "$i ";
    for ( $j = 0; $j < $width; $j++ ) {
        $d = <INPUT>; chomp($d);
        if ( $d < -9999 ) {
            $d = 0;
        }
        if ( $d < $min ) {
            $min = $d;
        }
        if ( $d > $max ) {
            $max = $d;
        }
    }
}
close(INPUT);
print "min = $min  max = $max\n";

$im = new GD::Image( $width, $height );
($w, $h) = $im->getBounds();
print "create $w x $h\n";

for ( $i = 0; $i < 255; $i++ ) {
    $color{$i} = $im->colorAllocate( $i, $i, $i );
}
$red = $im->colorAllocate( 255, 10, 10 );

# draw image
open( INPUT, "<$input" );
$width = <INPUT>; chomp($width);
$height = <INPUT>; chomp($height);
for ( $i = 0; $i < $height; $i++ ) {
    for ( $j = 0; $j < $width; $j++ ) {
        $d = <INPUT>; chomp($d);
        if ( $d > 0 ) {
            $level = 55 + int(200 * ( $d - $min ) / ( $max - $min ));
        } else {
            $level = 0;
        }
        
        $im->setPixel( $j, $i, $color{$level} );
    }
}

$im->line( 0, 0, $width, 0, $red );
$im->line( 0, 1, $width, 1, $red );
$im->line( 0, 2, $width, 2, $red );

# write out gif
$data = $im->png;
open( OUTPUT, ">$output" ) ||
    die "cannot open $output\n";
binmode OUTPUT;
print OUTPUT $data;
close OUTPUT;
