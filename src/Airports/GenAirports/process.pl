#!/usr/bin/perl

# this is a sad testament to building your software off of other
# libraries that have bugs (or perhaps less than disired features.)
# Unfortunatley I don't know enough about the functionality they
# provide to go fix them.  Maybe someday.

# At any rate.  This script is a wrapper around the main airport
# generation utility.  It run it until it finishes or crashes.  If it
# finishes without saying, yup I'm all done, a crash is assumed.  We
# re-run the airport generate from the crash point with a different
# nudge factor in a sorry attempt to work around the bug.

# yes, I know this is a really ugly hack, I apologize in advance to
# those who might have trouble running these tools on non-unix
# platforms, but I'm not sure what else to do at this point.  If
# someone can fix the polygon clipping library so it doesn't leave
# tiny shards of polygons or cracks, I'd be very grateful.

# So here we go, children under 13 years of age should probably have
# parental supervision if playing with something this ugly.


# Edit the following values to set up your preferences:

$workdir = "/fgfs01/curt/Work";
$inputfile = "./default.apt";
$binary = "./genapts";
$startid = "";

# end of user configurable section


$done = 0;
$nudge = 0;

while ( ! $done ) {

    # update the nudge value
    $nudge += 5;
    if ( $nudge > 25 ) {
	$nudge = 5;
    }

    # launch the airport generator

    $command = "$binary --input=$inputfile --work=$workdir --nudge=$nudge";

    if ( $startid ne "" ) {
	$command .= " --start-id=$startid";
    }

    print "Executing $command\n";
    open( PIPE, "$command |" ) || die "Cannot run $command\n";

    while ( <PIPE> ) {
	if ( m/Id portion/ ) {
	    # print $_;
	}

	if ( m/\[FINISHED CORRECTLY\]/ ) {
	    $done = 1;
	    print "FINISHED!\n";
	}
    }

    close ( PIPE );

    if ( ! $done ) {
	$startid = `cat last_apt`; chop( $startid );
	print "Restarting at $startid.\n";
    }
}
