#!/usr/bin/perl
#-------------------------------------------------------------------------
# Script for creating maps out of Flight Gear scenery data
#
# Written by Alexei Novikov, May-July 1999.
#
# Copyright (C) 1999 Alexei Novikov, anovikov@heron.itep.ru
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#------------------------------------------------------------------------
# Version 1.0.1
#------------------------------------------------------------------------
#
# Changes from 1.0
#
# Some bugs were fixed, better colors for the maps used, some grid code 
# clean up. More options are added by  Brian C. Wiles <brian@speakfreely.org>
#
#------------------------------------------------------------------------
#
# Changes from 1.0BETA2
#
# Some code clean is done. Code is runing something like 10% faster on
# the larger maps.
#
#------------------------------------------------------------------------
#
# Changes from 1.0BETA1 to 1.0BETA0
#
# 1) All colored surfaces are ploted correctly
# 2) Shading based on the steepness of the slope is added
# 3) Glaciers are displayed
# 4) Equiheight lines are ploted at correct altitude
# 5) More user-friendly colors used 
# 6) Digits on the grid lines corresponds to degrees.minutes.1/100 minutes
# instead of degrees.1/1000 degrees
#
#------------------------------------------------------------------------
# Changes from 1.0BETA0 to 1.0BETA
#
# 1) Works with USA-0.2 scenery data
# 2) Plots equiheight lines
# 3) Plots colored surfaces much more correctly 
#
#------------------------------------------------------------------------
#
# Known problems
#
# 1) There still exist some color artifacts due to the drawing of 2
# triangles of different color
#
#------------------------------------------------------------------------
use GD;
use Getopt::Long;
# 
# This is all you have to change.
#
my $fgfs_root="/stage/fgfs01/curt/FlightGear";
#
# Some local definitions
#
my $scenery_dir=$fgfs_root . "/Scenery/";
my $airports=$fgfs_root . "/Airports/default.apt.gz";
my ($x,$y,$z,$r,$theta,$alpha)=0;
my $v_c=0;
my $pi=3.141592653589793238;
local $thet=0;
local $alph=0;
local $to_shade=1;
my $id=0;
local $scale=100000;
local $length=600;
local $airportfilter=".*";
local $show_names=1;
local $use_display=0;
#
# Geting options
#
$ret=&GetOptions("lon:f",\$alph,"lat:f",\$thet,"size:i",\$length,"length:i",\$scale,"airport-id:s",\$id,"shading:i",\$to_shade,"airport-filter:s",\$airportfilter,"airport-names:i",\$show_names,"use-display:i",\$use_display);
if (((!$alph) & (!$thet)) & (!$id)) {
  usage();
  die;
}
if ($id) {
#
# Loading long and lat from the airport description
#
  open (AIRPRT, "zcat $airports |") || warn "Can\'t open $airports: $!\n";
  my ($lid,$name)=0;
  while (<AIRPRT>) {
    chomp($_);
    if ($_=~/^A /) {
+      ($tmp,$lid,$thet,$alph,$tmp,$tmp,$name)=split(/\s+/,$_,7);
     }
    elsif ($lid eq $id) {
      $_=~s/^\s+//;
      ($tmp,$tmp,$tmp,$tmp,$heading,$tmp)=split(/\s/,$_);
      $thet=~s/(^|-)(0+)/$1/;
      $alph=~s/(^|-)(0+)/$1/;
      last;
    }
  }
  close(AIRPRT);
}
if (!$thet && !$alph) {die "Can't find airport !";} 
#
# Color and image definitions
#
local @tmp_color=();
local $im = new GD::Image($length,$length);
local $white = $im->colorAllocate(255,255,255);
$tmp_color[0] =  1;  
#
# Warning there are another 1 place where one have to change colors
# this is in the subroutine sh_color
$tmp_color[1] =  $im->colorAllocate(165,186,165); #Under the sea
$tmp_color[2] =  $im->colorAllocate(219,231,205); # <1000f
$tmp_color[3] =  $im->colorAllocate(192,212,174); # <2000f
$tmp_color[4] =  $im->colorAllocate(242,238,193); # <3000f
$tmp_color[5] =  $im->colorAllocate(240,222,155); # <5000f
$tmp_color[6] =  $im->colorAllocate(224,185,124); # <7000f
$tmp_color[7] =  $im->colorAllocate(208,157,113); # <9000f
$tmp_color[8] =  $im->colorAllocate(198,135,87);   # <12000f  
$tmp_color[9] =  $im->colorAllocate(112,69,107); # color for airports
$tmp_color[10] = $im->colorAllocate(174,146,94);# color for levels
$tmp_color[11] = $im->colorAllocate(70,70,70); # color for grid lines
$tmp_color[12] = $im->colorAllocate(194,214,216); # lakes
$tmp_color[13] = $im->colorAllocate(194,214,216); # oceans
$tmp_color[14] =  $im->colorAllocate(112,69,107); # color for airport labels
#$tmp_color[15] =  $im->colorAllocate(93,126,115); # color for urban areas from TIGER
$tmp_color[15] = $im->colorAllocate(236,225,98); # color for urban areas
$tmp_color[16] = $im->colorAllocate(255,0,0); # color for unknown
local $color_tube=17; # !!! This shoud be size of array (counting 0)  


local $zoom=$length/$scale;

$thet=$thet/180*$pi;
$alph=$alph/180*$pi;
my ($x,$y,$z,$r_e)=xyz_lat($thet,$alph);
# We first transfer our scale in km to lat. long.
my $ts=$scale/2;
my ($dth, $dal)=lat_ab($ts,$ts,$thet,$alph);
$dth -=$thet;
$dal -=$alph;
# and load all files from the needed directories
my $signt=0;
$signt=1 if ($thet<0);
my $min_t=int(($thet-$dth)*180/$pi)-$signt;
my $max_t=int(($thet+$dth)*180/$pi)-$signt;
my $signa=0;
$signa=1 if ($alph<0);
my $min_a=int(($alph-$dal)*180/$pi)-$signa;
my $max_a=int(($alph+$dal)*180/$pi)-$signa;
if ($alph==0) {
  $min_a=int(($alph-$dal)*180/$pi)-1;
  $max_a=int(($alph+$dal)*180/$pi);
}

print "$min_t $max_t\n";

for (my $k=$min_t;$k<=$max_t;$k++) {
  for (my $l=$min_a;$l<=$max_a;$l++) {
    $kd=$ks=$k;
    $ld=$ls=$l;
    my ($ks,$ls)=to_ew_dir($ks,$ls);
    my $subdir=$ls . $ks;
    $sign2=0;
    $sign2=1 if ($ld < 0);
    $kd=int($kd/10)*10;
    $ld=int(($ld+$sign2)/10)*10-$sign2*10; 
    my ($kd,$ld)=to_ew_dir($kd,$ld);
    $dir=$ld . $kd;
    my @files=glob("$scenery_dir$dir/$subdir/*.gz");
    print "Loading files from: $dir/$subdir\n";
    my $size=@files;
    for (my $i=0;$i<$size;$i++) {
      # Here we also create graphics
	print "  loading $files[$i]\n";
      load_file($files[$i], $x, $y, $z);
    
    }
  }
}


open (AIRPRT, "$airports") || warn "Can\'t open $airports: $!\n";
my ($id,$name)=0;
my ($alat,$along)=0;
my ($c,$d)=0;
my $show_airport=0;
my $tmp_color=0;
while (<AIRPRT>) {
  chomp($_);
  next if ($_=~/^\#/);
  # Here we look if we have to do something with that airport
  
  my ($alat,$along)=0;
  if ($_=~/^A /) {
    $show_airport=0;
    ($tmp,$id,$along,$alat,$tmp,$tmp,$name)=split(/\s+/,$_,7);
    $name=~s/\s+$//;
    $alat=~s/(^|-)(0+)/$1/;
    $alat=$alat*$pi/180;
    $along=~s/(^|-)(0+)/$1/;
    $along=$along*$pi/180;
    if ((abs($alat-$alph)<$dal) & (abs($along-$thet)<$dth)) {
      my $ten=25; # radius of the airport circle
      ($c,$d)=ab_lat($along,$alat,$thet,$alph);
      $c=scale($c);
      $d=scale($d);
      $tmp_color=$im->getPixel($c,$d);
      if ($name=~m/$airportfilter/i) {
	if($name) {
	  $show_airport=1;
	  # We want to print the name
	  if($show_names){
	    my $string=$name . " (" . $id . ")";
	    print "$string\n";
	    my $size=1;
	    if ($string=~/.{15,}/) {
	      # name is too long; 
	      $string=~/(\w+)\s(.*)/; # we subdivied by two
	      print "2: $1,$2\n";
	      # well I'd like to subdivide it in n min 10 leters chunks
	      # but I don't know how to do this
	      $im->string(gdMediumBoldFont,$c+10,$d+10,"$1",$tmp_color[14]);
	      $im->string(gdMediumBoldFont,$c+10,$d+22,"$2",$tmp_color[14]);
	    }
	    else {
	      $im->string(gdMediumBoldFont,$c+10,$d+10,"$string",$tmp_color[14]);
	    }
	  }
	  else {
	    $im->string(gdMediumBoldFont,$c+10,$d+10,"$id",$tmp_color[14]);
	  }
	  $name=0;
	  $im->arc($c,$d,$ten,$ten,0,365,$tmp_color[9]);
	  if (((abs($c-$length/2)<($length/2-20)) || (abs($d-$length/2)<($length/2-20))) && ($tmp_color ne "0")){
	  $im->fillToBorder($c,$d,$tmp_color[9],$tmp_color[9]); 
	}
	}
	# We are ready to show runaways
      }
    }
  }
  elsif (($_=~/^R /) & $show_airport) {
    ($tmp,$tmp,$tmp,$tmp,$heading,$tmp)=split(/\s+/,$_,6);
    $heading=$heading*$pi/180;
    print "$tmp_color\n";
    my @rgb=$im->rgb($tmp_color);
    $ten=$ten/2-5;
    my $brush=new GD::Image(2,2);
    my $tmp_color2=$brush->colorClosest($rgb[0],$rgb[1],$rgb[2]);
    $brush-> filledRectangle(0,0,2,2,$tmp_color);
    $im->setBrush($brush);
    $a1=int($c+$ten*sin($heading));
    $b1=int($d-$ten*cos($heading));
    $a=int($c-$ten*sin($heading));
    $b=int($d+$ten*cos($heading));
    $im->line($a1,$b1,$a,$b,gdBrushed); # one have to draw this line at least double thick.
  }
}
close (AIRPRT);

my $grid=0.5;
my $ddth=360*$dth/$pi;

foreach my $number (10, 5, 1, 0.5) {
  my $times=int($ddth/$number);
  if ($times<1) {next;}
  else {$grid=$number;last}
}
$sth=$sal=$grid;
$dthet=int($thet/$pi*180*$grid)/$grid;
$dalph=int($alph/$pi*180*$grid)/$grid;

for (my $k=-5;$k<=5;$k++) {
#
# Here we are ploting grid
#
  my $d_al=$dalph+$k*$sal;
  my $d_th=$dthet+$k*$sth;
  my $al=$d_al*$pi/180;
  my $th=$d_th*$pi/180;
  my ($a1)=ab_lat($thet+$dth,$al,$thet,$alph);
  my ($a2)=ab_lat($thet-$dth,$al,$thet,$alph);
  my ($tmp,$b1)=ab_lat($th,$alph+$dal,$thet,$alph);
  my ($tmp,$b2)=ab_lat($th,$alph-$dal,$thet,$alph);
  foreach my $param ($a1,$a2,$b1,$b2) {
    $param=scale($param);
  }
  
  $im->line($a1,0,$a2,$length,$tmp_color[11]);
  $im->line(0,$b1,$length,$b2,$tmp_color[11]);
  #
  # Ticks
  #
  my $a_ticks_delta=$sal/6;
  my $b_ticks_delta=$sth/6;
  for (my $i=-60-$k*10;$i<(60-$k*10);$i++) {
    my $a_position=$d_al+$a_ticks_delta*$i;
    my $b_position=$d_th+$b_ticks_delta*$i;
    my $al2=$a_position*$pi/180;
    my $th2=$b_position*$pi/180;
    my ($at1)=ab_lat($th,$al2,$thet,$alph);
    my ($at2,$bt1)=ab_lat($th2,$al,$thet,$alph);
    $at1=scale($at1);
    $bt1=scale($bt1);
    $at2=scale($at2);
    #print "$a1\n";
    my $tick_length=3;
    $tick_length=7 if (int($i/3)*3 == $i);
    #print "$a1, $b1\n";
    $im->line($at1,$b1,$at1,$b1+$tick_length,$tmp_color[11]);
    $im->line($at2,$bt1,$at2+$tick_length,$bt1,$tmp_color[11]);
  }
  ($th,$al)=to_ew($d_th,$d_al);
  ($th,$thm)=split(/\./,$th);
  $thm=(int($thm*6));
  $th .="." . $thm;
  ($al,$alm)=split(/\./,$al);
  $alm=(int($alm*6));
  $al .="." . $alm;
  $im->string(gdTinyFont,$a1+2,10,"$al",$tmp_color[11]);
  $im->string(gdTinyFont,10,$b1+2,"$th",$tmp_color[11]);
}

if($use_display)
{
open (DISPLAY, "| display") || warn "Can\'t open display: $!\n";
}
else
{
open (DISPLAY, ">output.gif") || warn "Can\'t open output.gif: $!\n";
}
binmode DISPLAY;
# Convert the image to GIF and print it on standard output
print DISPLAY $im->gif;
close (DISPLAY);
exit;

sub load_file {
  my ($file, $x, $y, $z)=@_; 
  
  my $num=0;
  local @rvert=();
  #
  # $x, $y, $z are the coordinates of the central point
  #
  my $v_c=$num;
  open (FILE, "zcat -c -d $file|") || die "Can\'t open $file :$!\n";
  my $usemtl=0;
  my $color=0;
  my ($xcr,$ycr,$zcr)=0;
  while (<FILE>) {
    if ($_=~/^\# gbs/) {
      chomp($_);
      ($tmp,$tmp,$xcr,$ycr,$zcr,$rad)=split(/\s/,$_);
      my $dif=($xcr-$x)**2+($ycr-$y)**2+($zcr-$z)**2;
      if (abs($dif-$rad**2)>4*$scale**2) {
	# This tile is too far away
	last;
      }
    }
    elsif ($_=~/^v\s/) {
      chomp($_);
      my ($tmp,$xp, $yp, $zp)=split(/\s/,$_);
      # here we transfer point coordinates to points on the map
      $xp +=$xcr;
      $yp +=$ycr;
      $zp +=$zcr;
      my $pr=sqrt($xp**2+$yp**2+$zp**2);
      my ($a,$b,$r)=ab_xy($xp,$yp,$zp,$x,$y,$z);
      $rvert[$v_c]{r} = $pr-$r;
      $rvert[$v_c]{b} = $b;
      $rvert[$v_c]{a} = $a;
      $v_c++;
    }
    elsif ($_=~/^vn/) {
      next;
      #Nothing to do with normales
    }
    elsif ($_=~/^tf/) {
      chomp($_);
      $_=~s/^tf\s//;
      # here we are ploting alot of triangles
      local @triang=split(/\s/,$_);
      my $size=@triang;
      for (my $i=0;$i<$size;$i++) {
	$triang[$i]=~s/\/.*$//;
	#print "$triang[$i] $color\n";
      }
      
      &colored($color)
      &levels();
    }
    # here we choose colors based on the description of the
    # surface
    elsif ($_=~/usemtl/) {
      if ($_=~/\sLake$|Reserv|Stream$/) {
	$color=12;
	
      }
      elsif ($_=~/Ocean$/) {
	$color=13;
      }
      elsif ($_=~/AirportKeep$/) {
	$color=-1;
      }
      elsif ($_=~/Glacier$/) {
	$color=0;
      }
      elsif ($_=~/Urban$/) {
	$color=15;
      }
      elsif ($_=~/Default|Marsh$|DryLake/) {
	$color=-1;
      }
      elsif ($_=~/Unknown/) {
	$color=16;
      }
      next;
    }
    else {next}
  }
  close (FILE);
}
#
# various trig functions
#

# Calculate Lat/Lon directory names
sub to_ew_dir {
  my ($thet,$alph)=@_;
  $thet=sprintf "%03d",$thet;
  $thet=~s/0/n/ if ($thet >= 0);  
  $thet=~s/-/s/ if ($thet < 0);
  $alph=sprintf "%04d",$alph;
  $alph=~s/0/e/ if ($alph >= 0);  
  $alph=~s/-/w/ if ($alph < 0); 
  return ($thet,$alph)
}

# Calculate full Lat/Lon Coordinates
sub to_ew {
  my ($thet,$alph)=@_;
  $thet="n" . $thet if ($thet >= 0);  
  $thet=~s/-/s/ if ($thet < 0);
  $alph= "e" . $alph if ($alph >= 0);  
  $alph=~s/-/w/ if ($alph < 0); 
  return ($thet,$alph)
}

sub earth_radius_lat {
  my ($lat)=@_;
  my $rec=6378137;
  my $rpol=6356752.314;
  my $r=1/sqrt((cos($lat)/$rec)**2+(sin($lat)/$rpol)**2);
  return $r;
}

sub earth_radius_xy {
  my ($x,$y,$z)=@_;
  my $rec=6378137;
  my $rpol=6356752.314;
  my $xy=sqrt($x**2+$y**2);
  my $r=sqrt($xy**2+$z**2);
  my $s_lat=$z/$r;
  my $c_lat=$xy/$r;
  my $r=1/sqrt(($c_lat/$rec)**2+($s_lat/$rpol)**2);
  return $r;
}

sub ab_xy {
  my ($x,$y,$z,$xr,$yr,$zr)=@_;
  #
  # (x,y,z) are the point coordinate, (xr,yr,zr) -- the reference one
  #
  my $r_ec=6378137;
  my $r_pol=6356752.314;
  my $xy=sqrt($x**2+$y**2);
  my $r=sqrt($xy**2+$z**2);
  my $s_lat=$z/$r;
  my $c_lat=$xy/$r;
  my $r_e=1/sqrt(($c_lat/$r_ec)**2+($s_lat/$r_pol)**2);
  my $xyr=sqrt($xr**2+$yr**2);
  my $rr=sqrt($xyr**2+$zr**2);
  my $a=$r_e/($r*$xyr)*($y*$xr-$yr*$x);
  my $b=-$r_e/($r*$rr)*($xyr*$z-$xy*$zr); #-because of GD
  return ($a,$b,$r_e);
}

sub ab_lat {
  my ($lat, $long, $lat_r,$long_r)=@_;
  my $r=earth_radius_lat($lat);

  my $a=$r*cos($lat)*($long-$long_r); # ?? not sure
  my $b=-$r*($lat-$lat_r); #-because of GD
  return ($a,$b,$r);
}

sub ab_lat2 {
  my ($lat, $long, $lat_r,$long_r)=@_;
  my $r=earth_radius_lat($lat);
  my $a=$r*cos($lat)*($long-$long_r);
  my $b=-$r*($lat-$lat_r); #-because of GD
  return ($a,$b,$r);
}

sub xyz_lat {
  my ($lat, $long)=@_;
  my $r=earth_radius_lat($lat);
  my ($s,$c)=geod_geoc($lat);
  my $x=$r*$c*cos($long);
  my $y=$r*$c*sin($long);
  my $z=$r*$s;
  return ($x,$y,$z,$r);
}
sub lat_ab {
  my ($a,$b,$lat_r,$long_r)=@_;
  my $r=earth_radius_lat($lat_r); # There is no obvious other 
  # (more correct way) :-(
  $lat=$lat_r+$b/$r; #because of GD
  my ($s,$c)=geod_geoc($lat);
  $long=$long_r+$a/($c*$r);
  return ($lat,$long);
}

sub scale {
  my ($x)=@_;
  $x=int($length/2+$x*$zoom);
  return $x;
}

sub geod_geoc {
  my ($ang)=@_;
  my $rec=6378137;
  my $rpol=6356752.314;
  my $s=sin($ang);
  my $div=sqrt($rec**4+$s**2*($rpol**4-$rec**4));
  my $Sin=$rpol**2*$s/$div;
  my $Cos=$rec**2*cos($ang)/$div;
  return ($Sin, $Cos)
}

sub usage {
  print "FGFS map: Version 1.0BETA0\n";
  print "Usage:\n";
  print "\t--lon=degrees:  starting longitude in degrees (west = -)\n";
  print "\t--lat=degrees:  starting latitude in degrees (south = -)\n";
  print "\t--airport-id=ABCD:  specify starting postion by airport id\n";
  print "\t--size=pixels: image size in pixels (default 600)\n";
  print "\t--length=meters: max distance between points on the earth\n";
  print "\t\t(default 100000m)\n";
  print "\t--shading=(0,1) :either to put shading or not (default to put)\n";
  print "\t--airport-filter=string: only display airports with \"string\" in the name\n";
  print "\t--airport-names={0,1}: display airport names with ID's (defaults to 1)\n";
  print "\t--use-display={0,1}: use X display program instead of output.gif (defaults to 0)\n";
}

sub levels {
# a first try to draw levels on the graph
  #my ($triang,$rvert)=@_;
  my $size=@triang;

  my $cp=$triang[0];
  my ($xc,$yc,$hc)=load_points($cp);
  my $sc_lv=500; # levels each 500 m
  
  for (my $k=1;$k<$size-1;$k++) {
	$k2=$k+1;
	my $p1=$triang[$k];
	my $p2=$triang[$k2];  
	my ($x1,$y1,$h1)=load_points($p1);
	my ($x2,$y2,$h2)=load_points($p2);
	my $length2=0.6*$scale;
	if ((abs($x1)>$lenght2) & (abs($y1)>$lenght2) & (abs($x2)>$length2) & (abs($y2)>$lenght2) & (abs($xc)>$lenght2) & (abs($yc)>$lenght2)) { next }
	my %points=( h => [$hc, $h1, $h2, $hc],
		     x => [$xc, $x1, $x2, $xc],
		     y => [$yc, $y1, $y2, $yc]
		   );
	my %levels=();
	for (my $i=0;$i<3;$i++) {
	  my $pmin=(($points{h}[$i] <=> $points{h}[$i+1])+1)/2+$i;
	  $sc_lv=100  if ($points{h}[$pmin]<500);
	  my $times=abs(int($points{h}[$i]/$sc_lv)-int($points{h}[$i+1]/$sc_lv));
	  for (my $j=1;$j<=$times;$j++){
	    my ($x,$y,$h)=level_point($points{x}[$i],$points{y}[$i],$points{h}[$i],$points{x}[$i+1],$points{y}[$i+1],$points{h}[$i+1],$sc_lv,$j );
	    if (($h>500) && ($h/500 != int($h/500)))  {
	      # it is still possible that there can be useless curves
	      next;
	    }
	    $x=scale($x);
	    $y=scale($y);
	    if ($levels{$h}{x}) {
	      $im->line($x,$y,$levels{$h}{x},$levels{$h}{y},$tmp_color[10]);
	    }
	    else {
	      $levels{$h}{x}=$x;
	      $levels{$h}{y}=$y;
	    }
	  }
	}
      }
}

sub colored {
  #
  # This part should be able to produce fast large rough polygons 
  # 
  my ($color)=@_;
  my $size=@triang;
  my $cp=$triang[0];
  my ($xc,$yc,$hc)=load_points($cp);
  # we keep central point forever
  if ($color<=0) {
    my $c=0;
    my $c_prev=0;
    #
    # We have to draw each triangle separately
    #
    my $l=1; #the last point with drawn triangle       
    while ($l<$size-1) {
      for ($k=$l;$k<$size-1;$k++) {
	my $tmpc=$hc;
	$k2=$k+1;
	my $p1=$triang[$k];
	my $p2=$triang[$k2];
	my ($x1,$y1,$h1)=load_points($p1);
	my ($x2,$y2,$h2)=load_points($p2);
	my $length2=0.6*$scale;
	if ((abs($x1)>$lenght2) & (abs($y1)>$lenght2) & (abs($x2)>$length2) & (abs($y2)>$lenght2) & (abs($xc)>$lenght2) & (abs($yc)>$lenght2)) { 
	  $im->filledPolygon($poly,$tmp_color[$c_prev]) if ($c_prev!=0);
	  $l=$k2;
	  $c_prev=0;
	  last;
	}
	
	$tmpc=$h1 if ($h1>$tmpc); # we are painting everything in the 
	# maximum color
	$tmpc=$h2 if ($h2>$tmpc);
	#
	# here we choose colors based on the height
	#
	if ($color==0) {
	  $tmpc=-999; # this is a glacier and it can be shaded
	}
	($r,$g,$b)=sh_color($tmpc,$cp,$p1,$p2);
	$c=color_allocate($r,$g,$b);
	if ($c==$c_prev) {	 
	  $poly->addPt(scale($x2),scale($y2));
	  $l=$k2;
	}
	# it is a triangle with new color (or just a new one)
	else {
	  $im->filledPolygon($poly,$tmp_color[$c_prev]) if ($c_prev!=0); 
	  # draw an old one if it is not empty
	  $poly = new GD::Polygon; # start new
	  $poly->addPt(scale($xc),scale($yc)); # add central point
	  $poly->addPt(scale($x1),scale($y1));# and all aready calculated 
	  $poly->addPt(scale($x2),scale($y2));
	  $c_prev=$c; # save the color
	  $l=$k2;
	  
	  last; # go on with this polygon
	}
	#
      }
    }
    $im->filledPolygon($poly,$tmp_color[$c_prev]) if ($c_prev!=0);
  }
  else {
    #
    # We just want to draw a huge polygon with the prescribed color
    #
    my $poly = new GD::Polygon;
    if ($triang[1]!=$triang[$size-1]) {
      # we have a opened polygon
      $poly->addPt(scale($xc),scale($yc));
    }
    for (my $i=1;$i<$size;$i++) {
      # and we are just adding points
      $poly->addPt(scale($rvert[$triang[$i]]->{a}),scale($rvert[$triang[$i]]->{b}));
    }
    $c=$color;      
    $im->filledPolygon($poly,$tmp_color[$c]);
  }
  if ($color<0) {
    #
    # If this is not a lake or smth we have to check if there is need
    # to draw a more subtile triangles over the existing triangle strip 
    #
    &sub_triangles();
  }
}



sub color_select {
  my ($tmpc)=@_;
  my $c=0;
  #my @color_change=("-1000", "-200", "0", "100", "200", "500", "1000", "2000",
  #		    "3000", "5000");
  my @color_change=("-1000", "-200", "0", "200", "500", "1000", "2000",
                    "3000", "5000");
  for (my $i=0;$i<8;$i++) {
    if ($tmpc >= $color_change[$i]) {
      $c=$i+1;
    }
    else {
      last;
    }
  }

  return $c;
}

sub sh_color {
  my ($tmpc,$p1,$p2,$p3)=@_;
  @tmp_rgb= ([255,255,255],
	     [165,186,165],
	     [219,231,205],
	     [192,212,174],
	     [242,238,193],
	     [240,222,155],
	     [224,185,124],
	     [208,157,113],
	     [198,135,87]);
  my $c=color_select($tmpc);
  $c--;
  if ($to_shade) {
    my $sh=shading($p1,$p2,$p3);
    for (my $k=0;$k<3;$k++) {
      $tmp_rgb[$c][$k] *=($sh*0.9+0.1);
      $tmp_rgb[$c][$k]=int($tmp_rgb[$c][$k]/4)*4;
    }
  }
  return ($tmp_rgb[$c][0],$tmp_rgb[$c][1],$tmp_rgb[$c][2]);
}

sub level_point {
  my ($x1,$y1,$h1,$x2,$y2,$h2,$sc_lv,$j)=@_;
  my $min=$h1;
  $min=$h2 if ($h1>$h2);
  my $h=(int($min/$sc_lv)+$j)*$sc_lv;
  my $t=($h-$h1)/($h2-$h1);
  my $x=$x1+$t*($x2-$x1);
  my $y=$y1+$t*($y2-$y1);
  return ($x,$y,$h)
}

sub level_triangle {
  my ($x1,$y1,$h1,$x2,$y2,$h2,$k)=@_;
#  my @color_change=("-1000", "-200", "0", "100", "200", "500", "1000", "2000",
#		    "3000", "5000");
  my @color_change=("-1000", "-200", "0", "200", "500", "1000", "2000",
                    "3000", "5000");
  my $min=$h1;
  $min=$h2 if ($h1>$h2);
 
  my $j=color_select($min);
  my $h=@color_change[$j+$k-1];
  my $t=($h-$h1)/($h2-$h1);
  my $x=$x1+$t*($x2-$x1);
  my $y=$y1+$t*($y2-$y1);
  return ($x,$y,$h)
}

sub shading {
  my ($p1,$p2,$p3)=@_;
  my ($x1,$y1,$z1)=load_points($p1);
  my ($x2,$y2,$z2)=load_points($p2);
  my ($x3,$y3,$z3)=load_points($p3);
  my $c=(($y1-$y2)*($x1-$x3)-($y1-$y3)*($x1-$x2))/(($z1-$z3)*($x1-$x2)-($z1-$z2)*($x1-$x3));
  my $a=-($y1-$y2+$c*($z1-$z2))/($x1-$x2);
  my $b=1;
  my $sum=sqrt(1+$a**2+$c**2);
  foreach my $coord ($a,$b,$c) {
    $coord /=$sum;
  }
  #my $shading=($a*sqrt(3)/4+$b/4+$c/2*sqrt(3));
  #$shading=0 if ($shading<0);
  return abs($c);
} 

sub color_allocate {
  my ($r,$g,$b)=@_;
  my $c=@tmp_color;
  if ($c>253) {
    $c=$im->colorClosest($r,$g,$b);
  }
  else {
    if (($im->colorExact($r,$g,$b)) <=> -1) {
      $c=$im->colorExact($r,$g,$b);
    }
    else {
      $tmp_color[$c]=$im->colorAllocate($r,$g,$b);
    }
  }
  return $c;
}

sub sub_triangles {
  my ($color)=@_;
  my $c=0;
  my $cp=$triang[0];
  my ($xc,$yc,$hc)=load_points($cp);
  my $size=@triang;
  for (my $k=1;$k<$size-1;$k++) {
    # Just the same as for large triangles but with additional cheking
    my $tmpc=$hc;
    $k2=$k+1;
    my $p1=$triang[$k];
    my $p2=$triang[$k2];  
    my ($x1,$y1,$h1)=load_points($p1);
    my ($x2,$y2,$h2)=load_points($p2);
    my $length2=0.6*$scale;
    if ((abs($x1)>$lenght2) & (abs($y1)>$lenght2) & (abs($x2)>$length2) & (abs($y2)>$lenght2) & (abs($xc)>$lenght2) & (abs($yc)>$lenght2)) { next }
    
    my %points=( h => [$hc, $h1, $h2, $hc],
		 x => [$xc, $x1, $x2, $xc],
		 y => [$yc, $y1, $y2, $yc]
	       );
    # we first check each line
    my %levels=();
    my $ctimes=0;      
    #
    # here we have to sort lines so that the one with the higest
    # altitudes are in the begining
    #
    my %order=();
    my @times=();
    my $max_ctimes=0;
    for (my $i=0;$i<3;$i++) {
      $ctimes=abs(color_select($points{h}[$i])-color_select($points{h}[$i+1]));
      $times[$i]=$ctimes; # we will have to calculate it once again
      if ($ctimes) {
	my ($x,$y,$h)=level_triangle($points{x}[$i],$points{y}[$i],$points{h}[$i],$points{x}[$i+1],$points{y}[$i+1],$points{h}[$i+1],$ctimes);
	#this is the highest point
	$order{$i}=$h;
	$max_ctimes=$ctimes if ($ctimes>$max_times);
      }
    }
    # In case we have to plot an additionals smaller triangles 
    if ($max_ctimes) {
      foreach my $i (sort {$order{$b} <=> $order{$a}} keys %order) {
	# we sort so that triangles with larger max altitude are ploted first
	$ctimes=$times[$i];
	for (my $j=$ctimes;$j>0;$j--){
	  my ($x,$y,$h)=level_triangle($points{x}[$i],$points{y}[$i],$points{h}[$i],$points{x}[$i+1],$points{y}[$i+1],$points{h}[$i+1],$j);	
	  my $pmin=(($points{h}[$i] <=> $points{h}[$i+1])+1)/2+$i;
	  if ($levels{$h}{x}) {
	    #
	    # we already have point for this height and can draw a triangle
	    #
	    my $poly = new GD::Polygon;
	    $poly->addPt(scale($levels{$h}{x1}),scale($levels{$h}{y1}));
	    #
	    # This should be the minimum point
	    #
	    $poly->addPt(scale($levels{$h}{x}),scale($levels{$h}{y}));
	    $poly->addPt(scale($x),scale($y));
	    $x1=$points{x}[$pmin];
	    $y1=$points{y}[$pmin];
	    $poly->addPt(scale($x1),scale($y1));	    
	    if ($color==0) {
	      $tmpc=-998; # Don't forget the glaciers
	    }
	    ($r,$g,$b)=sh_color($h-1,$cp,$p1,$p2);
	    $c=color_allocate($r,$g,$b);
	    $im->filledPolygon($poly,$tmp_color[$c]);
	  }
	  else {
	    #
	    # if we don't have anything defined yet we store the needed data
	    #
	    $levels{$h}{x}=$x;
	    $levels{$h}{y}=$y;
	    $levels{$h}{x1}=$points{x}[$pmin];
	    $levels{$h}{y1}=$points{y}[$pmin];
	  }
	}
      }
    }
  }
}

sub load_points {
  my ($point)=@_;
  my $x=$rvert[$point]->{a};
  my $y=$rvert[$point]->{b};
  my $h=$rvert[$point]->{r};
  return ($x, $y, $h);
}
