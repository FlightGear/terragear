#!/usr/bin/perl

use strict;

require "calc-tile.pl";

my( $arg );
my( $infile ) = "";
my( $outdir ) = "";
my( $countonly ) = 0;

sub usage {
    die "Usage: $0 --input=<infile> --outdir=<output_dir_tree> --count-only\n";
}


# process arguments
while( $arg = shift(@ARGV) ) {
    if ( $arg =~ m/^--input=/ ) {
        $arg =~ s/^--input=//;
        $infile = $arg;
        print "infile = $infile\n";
    } elsif ( $arg =~ m/^--outdir=/ ) {
        $arg =~ s/^--outdir=//;
        $outdir = $arg;
        print "outdir = $outdir\n";
    } elsif ( $arg =~ m/^--count-only/ ) {
        $countonly = 1;
    } else {
        usage();
    }
}

if ( $infile eq "" || ($countonly == 0 && $outdir eq "") ) {
    usage();
}

open( IN, "<$infile" ) || die "Cannot open $infile\n";

my( @F );
my( $total ) = 0.0;
my( $count ) = 0;
my( $shortcount ) = 0;
my( $mediumcount ) = 0;
my( $tallcount ) = 0;
my( $min ) = 10000.0;
my( $max ) = 0.0;

while( <IN> ) {
    # print "-> $_";
    @F = split( /\|/, $_ );

    my( $lat ) = $F[3] + $F[4]/60.0 + $F[5]/3600.0;
    my( $lon ) = $F[8] + $F[9]/60.0 + $F[10]/3600.0;

    # strip white space
    $F[6] =~ s/\s+//g;
    $F[11] =~ s/\s+//g;

    if ( $F[6] eq "" || $F[6] eq " " || $F[6] eq "N" ) {
        # do nothing
    } else {
        print $_;
        $lat = -$lat;
    }
    if ( $F[11] eq "" || $F[11] eq " " || $F[11] eq "W" ) {
        $lon = -$lon
    } else {
        # do nothing
        print $_;
    }

    my( $ground ) = $F[13];
    my( $height ) = $F[14];
    my( $top_msl ) = $F[15];

    if ( $height < $min ) {
        $min = $height;
    }
    if ( $height > $max ) {
        $max = $height;
    }
    $total += $height;
    $count++;

    my( $dir ) = directory_name($lon, $lat);
    my( $index ) = tile_index($lon, $lat);

    my( $model ) = "";
    my( $base_elev ) = 0.0;

    if ( $height < 100.0 ) {
        # short tower
        $model = "Models/Structures/radio-short.xml";
        $base_elev = $top_msl - 100.0;
        $shortcount++;

        # but let's skip because too many of these just get too crazy
        next;
    } elsif ( $height < 200.0 ) {
        # medium tower
        $model = "Models/Structures/radio-medium.xml";
        $base_elev = $top_msl - 200.0;
        $mediumcount++;
    } else {
        # tall tower
        $model = "Models/Structures/radio-tall.xml";
        $base_elev = $top_msl - 610.0;
        $tallcount++;
    }

    # printf("%11.6f %10.6f %.1f %.1f %.1f \n",
    #        $lon, $lat, $ground, $height, $top_msl);
    # printf(" %s/%s/%s.ind -> OBJECT_SHARED %s %.6f %.6f %.1f 0.00\n", $outdir, $dir, $index, $model, $lon, $lat, $base_elev );

    if ( ! $countonly ) {
        system( "mkdir -p $outdir/$dir" );
    
        my( $indfile ) = "$outdir/$dir/$index.ind";
        open( INDEX, ">>$indfile" );
        printf( INDEX "OBJECT_SHARED %s %.6f %.6f %.1f 0.00\n",
                $model, $lon, $lat, $base_elev );
    }
}

print "short count = $shortcount\n";
print "medium count = $mediumcount\n";
print "tall count = $tallcount\n";

print "average height = " . $total / $count . "\n";
print "min = $min  max = $max\n";

close( IN );
