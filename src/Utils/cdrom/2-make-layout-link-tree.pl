#!/usr/bin/perl

$source = "/stage/fgfs07/ftp/pub/fgfs/Scenery-1.0.0";

$layout_master = shift( @ARGV );
$dest = shift( @ARGV );

if ( $layout_master eq "" ) {
    $layout_master = "./layout-dvd-1.0.0";
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

    printf( "ln -sf $source/$base $dest/$file\n" );
    system( "ln -sf $source/$base $dest/$file" );
}
