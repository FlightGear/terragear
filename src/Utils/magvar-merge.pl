#!/usr/bin/perl

# read magvar database

$file = shift;
open( MAGVAR, "<$file" ) || die "cannot open $file\n";

%VAR = ();

while ( <MAGVAR> ) {
    chomp;
    ($id, $city, $state, $type, $lat, $lon, $magvar, $rawfreq ) =
	split( /\t/ );
    $freq = sprintf("%.2f", $rawfreq);
    $keylat = sprintf("%.2f", $lat);
    $key = $id . $freq . $keylat;
    if ( $VAR{$key} ne "" ) {
	print "warning, dup key = $key\n";
    }
    $VAR{$key} = $_;
    # print "$key $id $magvar\n";
    if ( $lat < 0 ) {
	# print "$lat $key\n";
    }
}

close( MAGVAR );

# read master data base

$file = shift;
open( NAV, "<$file" ) || die "cannot open $file\n";

$line = <NAV>;
print $line;

while ( <NAV> ) {
    if ( $_ eq "[End]\n" ) {
	print $_;
	print "\n";
	break;
    } else {
	chomp();
	($type, $lat, $lon, $elev, $freq, $range, $dme, $id, $magvar, $name) =
	    split( /\s+/, $_, 10);
	$keylat = sprintf("%.2f", $lat);
	$key = $id . $freq . $keylat;
	if ( $VAR{$key} eq "" ) {
	    # print "warning $id $freq not in magvar database\n";
	} else {
	    # print "found $id $freq\n";
	    ($junk, $junk, $junk, $junk, $lat, $lon, $magvar, $junk ) =
		split( /\t/, $VAR{$key} );
	    if ( $lon > -100 && $lon < 100 ) {
		$lon =~ s/\-/\-0/;
	    }
	    delete $VAR{$key};
	}

	if ( $lat >= 0 ) {
	    $prettylat = " " . $lat;
	} else {
	    $prettylat = $lat;
	}

	if ( $lon >= 0 ) {
	    $prettylon = " " . $lon;
	} else {
	    $prettylon = $lon;
	}

	printf("%s %s %s %5d  %06.2f %4d %s %-4s %3s %s\n",
	       $type, $prettylat, $prettylon, $elev, $freq, $range, $dme,
	       $id, $magvar, $name );
    }
}

close( NAV );

print "Unmatched:\n\n";

foreach $key (sort keys %VAR) {
    ($id, $city, $state, $basetype, $lat, $lon, $magvar, $rawfreq ) =
	split( /\t/, $VAR{$key} );
    if ( $basetype =~ m/NDB/ ) {
	$type = "N";
    } elsif ( $basetype =~ m/VOR/ ) {
	$type = "V";
    } else { 
	$type = "X";
    }

    if ( $basetype =~ m/DME/ || $basetype =~ m/VORTAC/ ) {
	$dme = "Y";
    } else {
	$dme = "N";
    }

    $name = "$city";
    if ( $basetype =~ m/NDB/ ) {
	$name .= " NDB";
    } elsif ( $basetype =~ m/VORTAC/ ) {
	$name .= " VORTAC";
    } elsif ( $basetype =~ m/VOR/ ) {
	$name .= " VOR";
    }

    if ( $magvar eq "" ) {
	$magvar = "XXX";
    }

    if ( $lat >= 0 ) {
	$prettylat = " " . $lat;
    } else {
	$prettylat = $lat;
    }

    if ( $lon >= 0 ) {
	$prettylon = " " . $lon;
    } else {
	$prettylon = $lon;
    }
    if ( $prettylon > -100 && $prettylon < 100 ) {
	$prettylon =~ s/\-/\-0/;
    }

    $freq = sprintf("%.2f", $rawfreq);

    printf("%s %s %s %5d  %06.2f %4d %s %-4s %3s %s\n",
	   $type, $prettylat, $prettylon, $elev, $freq, $range, $dme,
	   $id, $magvar, $name );
}
