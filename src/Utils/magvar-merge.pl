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
    $key = $id . $freq . $lat;
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
	exit;
    } else {
	chomp();
	($type, $lat, $lon, $elev, $freq, $range, $dme, $id, $magvar, $name) =
	    split( /\s+/, $_, 10);
	$key = $id . $freq . $lat;
	if ( $VAR{$key} eq "" ) {
	    # print "warning $id $freq not in magvar database\n";
	} else {
	    # print "found $id $freq\n";
	    ($junk, $junk, $junk, $junk, $junk, $junk, $magvar, $junk ) =
		split( /\t/, $VAR{$key} );
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
