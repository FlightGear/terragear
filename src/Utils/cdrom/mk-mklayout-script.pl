#!/usr/bin/perl

$layout_master = "./layout-0.9.2";
$source = "/stage/fgfs03/curt/Scenery-0.9.2";
$dest = "./Images";

@rawfiles = `ls $source/*`;

@layout = ();

open ( LAYOUT, "<$layout_master" ) || die "cannot open $layout_master\n";
while ( <LAYOUT> ) {
    chomp;
    push( @layout, $_ );
}

system( "/bin/rm -rf $dest" );

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
