#!/usr/bin/perl

%BYFILE = ();

while ( <> ) {
    @pieces = split( /\s+/, $_ );
    $file = $pieces[8];
    $file =~ s/^.*\///g;
    # print $file\n";
    $BYFILE{$file}++;
}

%BYCOUNT = ();

foreach $file ( sort keys %BYFILE ) {
    $count = $BYFILE{$file};
    $BYCOUNT{$count} .= "$file,";
}


foreach $count ( keys %BYCOUNT ) {
    print "$count $BYCOUNT{$count}\n";
}
