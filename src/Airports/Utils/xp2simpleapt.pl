#!/usr/bin/perl
#
# Convert the XP apt.dat format to FlightGear's "simple.apt" format
#
# Data source is:
#
#     http://www.x-plane.org/users/robinp/
#
# Written by Curt Olson <http://www.flightgear.org/~curt> Started Aug 2003

use strict;

my( $line );
my( @F );
my( $last_apt ) = "";
my( $last_elev ) = "";
my( $last_name ) = "";
my( $apt_type ) = "C";
my( $rwy_lon ) = 0.0;
my( $rwy_lat ) = 0.0;
my( $has_tower ) = 0;
my( $default_bldgs ) = 0;
my( $count ) = 0;

# strip the first line
$line = <>;

# copy the license / source / credits line
$line = <>;
print "// " . $line;

while ( <> ) {
    @F = split( /\s+/, $_ );
    if ( $F[0] == 1 || $F[0] == 16 || $F[0] == 17 ) {
        # print "airport = " . $_;

        my( $type );
        if ( $F[0] == 1 ) {
            $type = "A";
        } elsif ( $F[0] == 16 ) {
            $type = "S";
        } elsif ( $F[0] == 17 ) {
            $type = "H";
        }

        if ( $last_apt ne "" ) {
            # print out airport definition line
            my( $lon ) = $rwy_lon / $count;
            my( $lat ) = $rwy_lat / $count;
            printf( "%s %-4s %10.6f %11.6f %5d %s%s%s %s\n",
                    $type, $last_apt, $lat, $lon, $last_elev,
                    $apt_type, $has_tower, $default_bldgs, $last_name );
        }

        # current airport definition
        $last_elev = $F[1];
        $apt_type = "C";
        if( $F[2] ) {
            $has_tower = "Y";
        } else {
            $has_tower = "N";
        }
        if ( $F[3] ) {
            $default_bldgs = "Y";
        } else {
            $default_bldgs = "N";
        }
        $last_apt = $F[4];
        $last_name = $F[5];
        for( my($i) = 6; $i <= $#F; ++$i ) {
            $last_name .= " " . $F[$i];
        }
        $count = 0;
        $rwy_lon = 0.0;
        $rwy_lat = 0.0;
    } elsif ( $F[0] == 10 ) {
        if ( $F[3] ne "xxx" ) {
            # runway definition
            # print "runway = " . $_;
            $rwy_lon += $F[2];
            $rwy_lat += $F[1];
            $count++;
        } else {
            # taxiway definition
        }
    } else {
        # something we don't know how to handle right now
    }
}

# grab that last data point
if ( $last_apt ne "" ) {
    # print out airport definition line
    my( $lon ) = $rwy_lon / $count;
    my( $lat ) = $rwy_lat / $count;
    printf( "A %-4s %10.6f %11.6f %5d %s%s%s %s\n",
            $last_apt, $lat, $lon, $last_elev,
            $apt_type, $has_tower, $default_bldgs, $last_name );
}
