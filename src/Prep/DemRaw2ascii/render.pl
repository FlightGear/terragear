#!/usr/bin/perl

use GD;

$width = 960;
$height = 1200;
$im = new GD::Image( $width, $height );

for ( $i = 0; $i < 200; ++$i ) {
    $red = int($i * 255 / 200);
    $green = 255 - $red;
    $color[$i] = $im->colorAllocate($red,$green,0);
    # print "$red $green\n";
}

# for ( $i = 0; $i < 200; ++$i ) {
#     print "$color[$i]\n";
# }

$ocean = $im->colorAllocate(0,0,255);

$xstep = 5;
$ystep = 5;

for ( $j = 0; $j < $height * 5; ++$j ) {
    if ( $j % 10 == 0 ) {
        print "row $j\n";
    }
    for ( $i = 0; $i < $width * 5; ++$i ) {
	$value = <>; chop($value);
	if ( ($i % 5 == 0) && ($j % 5 == 0) ) {
	    # print "$i $j $value\n";
	    if ( $value >= 0 ) {
		# print "$i $j $value\n";
		$index = int($value * 200 / 6732);
		# print "$index\n";
		$im->setPixel( int($i/5), int($j/5), 
			      $color[$index]);
	    } else {
                # print "ocean\n";
		$im->setPixel( int($i/5), int($j/5), $ocean );
	    }
	}
    }
}

# write out gif
$gif_data = $im->gif;
open( OUTPUT, ">map.gif" ) || 
    die "cannot open output map.gif\n";
binmode OUTPUT;
print OUTPUT $gif_data;
close OUTPUT;
