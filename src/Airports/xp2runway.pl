#!/usr/bin/perl
#
# Convert the XP apt.dat format to FlightGear's default.rwy format
#
# Data source is:
#
#     http://www.x-plane.org/users/robinp/
#
# Written by Curt Olson <curt@flightgear.org> Started Aug 2003

use strict;

my( $line );
my( @F );
my( $apt_id ) = "";

# strip the first line
$line = <>;

# copy the license / source / credits line
$line = <>;
print "// " . $line;

while ( <> ) {
    @F = split( /\s+/, $_ );
    if ( $F[0] == 1 ) {
        # print "airport = " . $_;

        # current airport definition
        $apt_id = $F[4];
    } elsif ( $F[0] == 10 ) {
        # runway/taxiway definition
        # print "runway = " . $_;
        my( $rwy_no ) = $F[3];
        $rwy_no =~ s/x+$//;
        my( $xpvasi1, $xprwy1, $xpappr1,
            $xpvasi2, $xprwy2, $xpappr2 )
            = $F[9] =~ m/(\d)(\d)(\d)(\d)(\d)(\d)/;

        my( $cll );
        if ( $xprwy1 >= 4 || $xprwy2 >= 4 ) {
            $cll = "Y";
        } else {
            $cll = "N";
        }

        my( $xpsurf ) = $F[10];
        my( $surface );
        if ( $xpsurf eq "01" ) {
            $surface = "A";
        } elsif ( $xpsurf eq "02" ) {
            $surface = "C";
        } elsif ( $xpsurf eq "03" ) {
            $surface = "T";
        } elsif ( $xpsurf eq "04" ) {
            $surface = "D";
        } elsif ( $xpsurf eq "05" ) {
            $surface = "G";
        } elsif ( $xpsurf eq "06" ) {
            $surface = "A";
        } elsif ( $xpsurf eq "07" ) {
            $surface = "C";
        } elsif ( $xpsurf eq "08" ) {
            $surface = "T";
        } elsif ( $xpsurf eq "09" ) {
            $surface = "D";
        } elsif ( $xpsurf eq "10" ) {
            $surface = "A";
        } elsif ( $xpsurf eq "11" ) {
            $surface = "C";
        } elsif ( $xpsurf eq "12" ) {
            $surface = "L";
        } elsif ( $xpsurf eq "13" ) {
            $surface = "W";
        } else {
            die "unknown surface code = $xpsurf\n";
        }

        my( $xpmarkings ) = $F[12];
        my( $markings );
        if ( $xpmarkings == 0 ) {
            $markings = "V";
        } elsif ( $xpmarkings == 1 ) {
            $markings = "V";
        } elsif ( $xpmarkings == 2 ) {
            $markings = "R";
        } elsif ( $xpmarkings == 3 ) {
            $markings = "P";
        } elsif ( $xpmarkings == 4 ) {
            $markings = "H";
        } else {
            die "unknown markings code = $xpmarkings\n";
        }

        my( $edgelights );
        if ( $xprwy1 >= 2 || $xprwy2 >= 2 ) {
            $edgelights = "H";
        } else {
            $edgelights = "N";
        }

        my( $rwy_codes ) = "$cll$surface$markings$edgelights" . "N";

        my( $tdz1 );
        if ( $xprwy1 >= 5 ) {
            $tdz1 = "Y";
        } else {
            $tdz1 = "N";
        }

        my( $tdz2 );
        if ( $xprwy2 >= 5 ) {
            $tdz2 = "Y";
        } else {
            $tdz2 = "N";
        }

        my( $reil1 );
        if ( $xprwy1 >= 3 ) {
            $reil1 = "Y";
        } else {
            $reil1 = "N";
        }

        my( $reil2 );
        if ( $xprwy2 >= 3 ) {
            $reil2 = "Y";
        } else {
            $reil2 = "N";
        }

        my( $vasi1 );
        if ( $xpvasi1 == 1 ) {
            $vasi1 = "N";
        } elsif ( $xpvasi1 == 2 ) {
            $vasi1 = "V";
        } elsif ( $xpvasi1 == 3 ) {
            $vasi1 = "P";
        } elsif ( $xpvasi1 == 4 ) {
            $vasi1 = "P";
        }

        my( $vasi2 );
        if ( $xpvasi2 == 1 ) {
            $vasi2 = "N";
        } elsif ( $xpvasi2 == 2 ) {
            $vasi2 = "V";
        } elsif ( $xpvasi2 == 3 ) {
            $vasi2 = "P";
        } elsif ( $xpvasi2 == 4 ) {
            $vasi2 = "P";
        }

        my( $appr1 );
        if ( $xpappr1 == 0 ) {
            $appr1 = "N";
        } elsif ( $xpappr1 == 1 ) {
            $appr1 = "N";
        } elsif ( $xpappr1 == 2 ) {
            $appr1 = "S";
        } elsif ( $xpappr1 == 3 ) {
            $appr1 = "P";
        } elsif ( $xpappr1 == 4 ) {
            $appr1 = "B";
        } elsif ( $xpappr1 == 5 ) {
            $appr1 = "C";
        } elsif ( $xpappr1 == 6 ) {
            $appr1 = "L";
        } elsif ( $xpappr1 == 7 ) {
            $appr1 = "D";
        } elsif ( $xpappr1 == 8 ) {
            $appr1 = "E";
        } else {
            die "unknown approach lighting code = $xpappr1\n";
        }
        
        my( $appr2 );
        if ( $xpappr2 == 0 ) {
            $appr2 = "N";
        } elsif ( $xpappr2 == 1 ) {
            $appr2 = "N";
        } elsif ( $xpappr2 == 2 ) {
            $appr2 = "S";
        } elsif ( $xpappr2 == 3 ) {
            $appr2 = "P";
        } elsif ( $xpappr2 == 4 ) {
            $appr2 = "B";
        } elsif ( $xpappr2 == 5 ) {
            $appr2 = "C";
        } elsif ( $xpappr2 == 6 ) {
            $appr2 = "L";
        } elsif ( $xpappr2 == 7 ) {
            $appr2 = "D";
        } elsif ( $xpappr2 == 8 ) {
            $appr2 = "E";
        } else {
            die "unknown approach lighting code = $xpappr2\n";
        }
        
        my( $end1_codes, $end2_codes );

        $end1_codes = "$tdz1$reil1$vasi1$appr1";
        $end2_codes = "$tdz2$reil2$vasi2$appr2";

        my( $end1_thresh, $end2_thresh ) = split( /\./, $F[6] );
        my( $end1_stopway, $end2_stopway ) = split( /\./, $F[7] );

        my( $taxi_edge );
        if ( $xprwy1 >= 6 || $xprwy2 >= 6 ) {
            $taxi_edge = "B";
        } else {
            $taxi_edge = "N";
        }

        my( $taxi_codes );
        $taxi_codes = "$cll$surface$taxi_edge";

        if ( $F[3] ne "xxx" ) {
            # runway definition
            printf("R %-4s %-3s %10.6f %11.6f %6.2f %5d %3d %s %s %4d %4d %s %4d %4d\n",
                   $apt_id, $rwy_no, $F[1], $F[2], $F[4], $F[5], $F[8],
                   $rwy_codes,
                   $end1_codes, $end1_thresh, $end1_stopway,
                   $end2_codes, $end2_thresh, $end2_stopway );
        } else {
            # taxiway definition
            printf("T %-4s xxx %10.6f %11.6f %6.2f %5d %3d %s\n",
                   $apt_id, $F[1], $F[2], $F[4], $F[5], $F[8],
                   $taxi_codes );
        }
    } else {
        # something we don't know how to handle right now
    }
}
