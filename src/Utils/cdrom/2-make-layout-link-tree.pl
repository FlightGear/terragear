#!/usr/bin/perl

$source = "/stage/fgfs03/curt/Scenery/Scenery-0.9.5";

$layout_master = shift( @ARGV );
$dest = shift( @ARGV );

if ( $layout_master eq "" ) {
    $layout_master = "./layout-disks-0.9.5";
}
if ( $dest eq "" ) {
    $dest = "./Images";
}

@rawfiles = `ls $source/*`;

@layout = ();

open ( LAYOUT, "<$layout_master" ) || die "cannot open $layout_master\n";
while ( <LAYOUT> ) {
    chomp;
    push( @layout, $_ );
}

print "Delete current $dest (y/N): ";
$response = <STDIN>;
chomp($response);
if ( $response eq "y" || $response eq "Y" ) {
    system( "/bin/rm -rf $dest" );
} else {
    die "Stopped with no action.\n";
}

foreach $file ( @layout ) {
    $base = $file;
    while ( $base =~ m/\// ) {
        $base =~ s/.*\///;
    }
    $dir = $file;
    $dir =~ s/\/[^\/]+$//;

    if ( ! -d "$dest/$dir" ) {
        system( "mkdir -p $dest/$dir" );
    }

    system( "ln -sf $source/$base $dest/$file\n" );
}
