#!/usr/bin/perl -w

eval 'exec /usr/bin/perl -S $0 ${1+"$@"}'
    if 0;

use strict;
use Cwd;

my $VERSION = "0.1";
my $PACKAGE = "am2dsp";

# String constants.
my $IGNORE_PATTERN = "^##([^#].*)?\$";
my $WHITE_PATTERN = "^[ \t]*\$";
my $COMMENT_PATTERN = "^#";
my $RULE_PATTERN = "^([\$a-zA-Z_.][-.a-zA-Z0-9_(){}/\$]*) *:([^=].*|)\$";
my $SUFFIX_RULE_PATTERN = "^\\.([a-zA-Z]+)\\.([a-zA-Z]+)\$";
my $MACRO_PATTERN = "^([A-Za-z][A-Za-z0-9_]*)[ \t]*([:+]?)=[ \t]*(.*)\$";
my $BOGUS_MACRO_PATTERN = "^([^ \t]*)[ \t]*([:+]?)=[ \t]*(.*)\$";
my $IF_PATTERN = "^if[ \t]+\([A-Za-z][A-Za-z0-9_]*\)[ \t]*\(#.*\)?\$";
my $ELSE_PATTERN = "^else[ \t]*\(#.*\)?\$";
my $ENDIF_PATTERN = "^endif[ \t]*\(#.*\)?\$";
my $PATH_PATTERN='(\\w|/|\\.)+';
# This will pass through anything not of the prescribed form.
my $INCLUDE_PATTERN = "^include[ \t]+((\\\$\\\(top_srcdir\\\)/${PATH_PATTERN})|(\\\$\\\(srcdir\\\)/${PATH_PATTERN})|([^/\\\$]${PATH_PATTERN}))[ \t]*(#.*)?\$";

my $AM_CONDITIONAL_PATTERN = "AM_CONDITIONAL\\((\\w+)";
my $AM_INIT_AUTOMAKE = "AM_INIT_AUTOMAKE\\(([^,]+),[ \t]*([^)]+)";

# Hash table of AM_CONDITIONAL variables seen in configure.
my %configure_cond = ();

# This holds the names which are targets.  These also appear in
# %contents.
my %targets = ();

# This holds the line numbers at which various elements of
# %contents are defined.
my %content_lines = ();

my %content_seen = ();

# This maps the source extension of a suffix rule to its
# corresponding output extension.
my %suffix_rules = ();

# Hash table of discovered configure substitutions.  Keys are names,
# values are `FILE:LINE' strings which are used by error message
# generation.
my %configure_vars = ();

# This holds the set of included files.
my @include_stack = ();

my $verbose = 0;
my $vcond;
my @conditional_stack = ();
my %contents = ();
my %conditional = ();

# This holds our (eventual) exit status.  We don't actually exit until
# we have processed all input files.
my $exit_status = 0;

my @make_input_list = ();
my %make_list = ();

# Names used in AC_CONFIG_HEADER call.  @config_fullnames holds the
# name which appears in AC_CONFIG_HEADER, colon and all.
# @config_names holds the file names.  @config_headers holds the '.in'
# files.  Ordinarily these are similar, but they can be different if
# the weird "NAME:FILE" syntax is used.
my @config_fullnames = ();
my @config_names = ();
my @config_headers = ();
# Line number at which AC_CONFIG_HEADER appears in configure.ac.
my $config_header_line = 0;

# Relative location of top build directory.
my $top_builddir = '';

my $relative_dir = '.';

my $output_vars = '';
my $output_trailer = '';

# List of Makefile.am's to process, and their corresponding outputs.
my @input_files = ();
my %output_files = ();

# List of files in AC_OUTPUT without Makefile.am, and their outputs.
my @other_input_files = ();

my @var_list = ();
my %am_vars = ();
my %def_type = ();

my @excluded_dirs = ();
my @excluded_files = ();
my $msvc_cflags = "";
my $msvc_cflagsd = "";
my $msvc_threads = "";
my $msvc_libs = "";
my @extra_sources = ();
my @extra_projects = ();

# Extracted from AM_INIT_AUTOMAKE(package,version)
my $dsp_package = 'FGFS';
my $dsp_version = '0.2';

my $static_lib = 0;

parse_arguments(@ARGV);

# Read project configuration file.
my $rcfile = "am2dsp.cfg";
read_am2dsprc($rcfile) if -r $rcfile;

scan_configure();
die "am2dsp: no \`Makefile.am' found or specified\n"
    if ! @input_files;

if ($static_lib) {
    static_lib_dsp_init($dsp_package);
} else {
    console_app_dsp_init($dsp_package);
}

my $am_file;
foreach $am_file (@input_files) {
    dsp_add_group($dsp_package, $am_file) if !exclude_dir($am_file);
}
dsp_finish($dsp_package);
generate_dsw( $dsp_package );

sub exclude_dir {
    my $dir = shift;
    foreach my $d (@excluded_dirs) {
	if ($dir =~ "/$d/") {
	    return 1;
	}
    }
    return 0;
}

sub exclude_file {
    my $file = shift;
    foreach my $f (@excluded_files) {
	if ($file =~ /$f/) {
	    return 1;
	}
    }
    return 0;
}

#
# TODO: option to specify static library or console app.
#
sub parse_arguments {
    my @av = @_;

    while (@av) {
	if ($av[0] eq '--version') {
	} elsif ($av[0] eq '--help') {
	} elsif ($av[0] eq '--verbose' || $av[0] eq '-v') {
	    $verbose = 1;
	} elsif ($av[0] eq '--package' || $av[0] eq '-p') {
	    require_argument(@av);
	    shift @av;
	    $dsp_package = $av[0];
	} elsif ($av[0] eq '--lib' || $av[0] eq '-l') {
	    # Create a static library
	    $static_lib = 1;
	} elsif ($av[0] =~ /^-/) {
	    die "am2dsp: unrecognised option -- `$av[0]'\nTry am2dsp --help for more information.\n";
	} else {
	}
	shift @av;
    }
}

sub read_am2dsprc {
    my $rc_file = shift;

    open( RC_FILE, $rc_file )
	or die "Can't open $rc_file: $!\n";
    my $line;
    while (defined($line = <RC_FILE>)) {
	chomp $line;
	if ($line =~ s/\\$//) {
	    # continuation line
	    $line .= "%";
	    $line .= <RC_FILE>;
	    redo unless eof(RC_FILE);
	}

	next if $line =~ /^$/; # ignore blank lines
	next if $line =~ /^#/; # ignore comments

	if ($line =~ /exclude_dir\s*=\s*(\S+)/) {
	    push( @excluded_dirs, $1 );
	}
	elsif ($line =~ /exclude_file\s*=\s*(\S+)/) {
	    my $f;
	    ($f = $1) =~ s/\\/\\\\/g; # escape path separators, "\" -> "\\"
	    push( @excluded_files, $f );
	    print "Excluding file: $f\n" if $verbose;
	}
 	elsif ($line =~ /include_path\s*=\s*\"(.*)\"/) {
 	    $msvc_cflags .= " /I \"$1\"";
 	    $msvc_cflagsd .= " /I \"$1\"";
 	}
	elsif ($line =~ /include_path\s*=\s*(\S+)/) {
	    $msvc_cflags .= " /I \"$1\"";
	    $msvc_cflagsd .= " /I \"$1\"";
	}
	elsif ($line =~ /define\s*=\s*(\S+)/) {
	    $msvc_cflags .= " /D \"$1\"";
	    $msvc_cflagsd .= " /D \"$1\"";
	}
	elsif ($line =~ /lib_path\s*=\s*(\S+)/) {
	    $msvc_libs .= " /libpath:\"$1\"";
	}
	elsif ($line =~ /add_lib\s*=\s*(\S+)/) {
	    $msvc_libs .= " $1.lib";
	}
	elsif ($line =~ /type\s*=\s*(\S+)/) {
	    my ($type,$threads,$debug) = split /,/, $1;
	    print "type=$type, threading=$threads, debug=$debug\n" if $verbose;
	    if ($type =~ "ConsoleApplication") {
		$static_lib = 0
	    }
	    elsif ($type =~ "StaticLibrary") {
		$static_lib = 1
	    }
	    else {
		# Invalid type
	    }

	    my $flags = " /ML"; # single threaded.
	    if ($threads =~ /MultithreadedDLL/) {
		$flags = " /MD";
	    }
	    elsif ($threads =~ /Multithreaded/) {
		$flags = " /MT";
	    }
	    elsif ($threads =~ /Singlethreaded/) {
		$flags = " /ML";
	    }
	    else {
		# Invalid threading option.
	    }

	    $msvc_cflags .= $flags;
	    $flags .= "d";
	    $msvc_cflagsd .= $flags;
	}
	elsif ($line =~ /add_source_file\s*=\s*(.*)/) {
	    my $rule;
	    ($rule = $1) =~ s/%/\r\n/g;
	    push( @extra_sources, $rule );
	}
	elsif ($line =~ /add_project\s*=\s*(.*)/) {
	    push( @extra_projects, $1 );
	}
    }
    close(RC_FILE);
}

# Ensure argument exists, or die.
sub require_argument
{
    my ($arg, @arglist) = @_;
    die "am2dsp: no argument given for option \`$arg'\n"
        if ! @arglist;
}

sub scan_configure {
    scan_one_configure_file('configure.ac');
    scan_one_configure_file('aclocal.m4')
	if -f 'aclocal.m4';

    if (! @input_files) {
	@input_files = @make_input_list;
	%output_files = %make_list;
    }
}

sub scan_one_configure_file {
    my $filename = shift;
    open(CONFIGURE, $filename)
	|| die "am2dsp: can't open \`$filename': $!\n";
    print "am2dsp: reading $filename\n" if $verbose;

    my $in_ac_output = 0;
    my $ac_output_line = '';

    while (<CONFIGURE>) {
	# Remove comments from current line.
	s/\bdnl\b.*$//;
	s/\#.*$//;

        # Skip macro definitions.  Otherwise we might be confused into
        # thinking that a macro that was only defined was actually
        # used.
        next if /AC_DEFUN/;

        # Follow includes.  This is a weirdness commonly in use at
        # Cygnus and hopefully nowhere else.
        if (/sinclude\((.*)\)/ && -f $1)
        {
            &scan_one_configure_file ($1);
        }

	if (! $in_ac_output && ( s/AC_OUTPUT\s*\(\[?// || s/AC_CONFIG_FILES\s*\(\[?// ) ) {
	    $in_ac_output = 1;
	    $ac_output_line = $.;
	}

	if ($in_ac_output)
	{
	    my $closing = 0;
	    if (s/[\]\),].*$//)
	    {
		$in_ac_output = 0;
		$closing = 1;
	    }

	    # Look at potential Makefile.am's
	    foreach (split)
	    {
                # Must skip empty string for Perl 4.
                next if $_ eq "\\" || $_ eq '';

		my ($local,$input,@rest) = split(/:/);
		if (! $input)
		{
		    $input = $local;
		}
		else
		{
		    $input =~ s/\.in$//;
		}

		if (-f $input . '.am')
		{
		    push(@make_input_list, $input);
		    $make_list{$input} = join(':', ($local,@rest));
		}
		else
		{
                    # We have a file that automake should cause to be
                    # rebuilt, but shouldn't generate itself.
                    push (@other_input_files, $_);
		}
	    }
	}

        # Handle configuration headers.  A config header of `[$1]'
        # means we are actually scanning AM_CONFIG_HEADER from
        # aclocal.m4.
        if (/A([CM])_CONFIG_HEADER\s*\((.*)\)/
            && $2 ne '[$1]')
        {
            &am_conf_line_error
                ($filename, $., "\`automake requires \`AM_CONFIG_HEADER', not \`AC_CONFIG_HEADER'")
                    if $1 eq 'C';

            $config_header_line = $.;
            my ($one_hdr);
            foreach $one_hdr (split (' ', $2))
            {
                push (@config_fullnames, $one_hdr);
                if ($one_hdr =~ /^([^:]+):(.+)$/)
                {
                    push (@config_names, $1);
                    push (@config_headers, $2);
                }
                else
                {
                    push (@config_names, $one_hdr);
                    push (@config_headers, $one_hdr . '.in');
                }
            }
        }

	if (/$AM_CONDITIONAL_PATTERN/o)
	{
	    $configure_cond{$1} = 1;
	}

	if (/$AM_INIT_AUTOMAKE/o)
	{
	    $dsp_package = $1;
	    $dsp_version = $2;
	}
    }

    close(CONFIGURE);
}

sub read_main_am_file {
    $am_file = shift;

    read_am_file($am_file);

    my @topdir = ();
    foreach (split(/\//, $relative_dir)) {
	next if $_ eq '.' || $_ eq '';
	if ($_ eq '..') {
	    pop @topdir;
	} else {
	    push(@topdir, '..');
	}
    }
    @topdir = ('.') if ! @topdir;

    my $top_builddir = join('/', @topdir);
    
}

sub read_am_file {
    $am_file = shift;
    open( AM_FILE, $am_file )
	or die "Can't open $am_file: $!\n";
    print "am2dsp: reading $am_file\n" if $verbose;

    my $saw_bk = 0;
    my $was_rule = 0;
    my $spacing = '';
    my $comment = '';
    my $last_var_name = '';
    my $blank = 0;

    while (<AM_FILE>)
    {
	if (/$IGNORE_PATTERN/o)
	{
	    # Merely delete comments beginning with two hashes.
	}
	elsif (/$WHITE_PATTERN/o)
	{
	    # Stick a single white line before the incoming macro or rule.
	    $spacing = "\n";
	    $blank = 1;
	}
	elsif (/$COMMENT_PATTERN/o)
	{
	    # Stick comments before the incoming macro or rule.  Make
	    # sure a blank line preceeds first block of comments.
	    $spacing = "\n" unless $blank;
	    $blank = 1;
	    $comment .= $spacing . $_;
	    $spacing = '';
	}
	else
	{
	    last;
	}
    }

    $output_vars .= $comment . "\n";
    $comment = '';
    $spacing = "\n";
    my $source_suffix_pattern = '';

    my $is_ok_macro = 0;
    while ($_)
    {
	$_ .= "\n"
	    unless substr ($_, -1, 1) eq "\n";

	if (/$IGNORE_PATTERN/o)
	{
	    # Merely delete comments beginning with two hashes.
	}
	elsif (/$WHITE_PATTERN/o)
	{
	    # Stick a single white line before the incoming macro or rule.
	    $spacing = "\n";
	    &am_line_error ($., "blank line following trailing backslash")
		if $saw_bk;
	}
	elsif (/$COMMENT_PATTERN/o)
	{
	    # Stick comments before the incoming macro or rule.
	    $comment .= $spacing . $_;
	    $spacing = '';
	    &am_line_error ($., "comment following trailing backslash")
		if $saw_bk;
	}
	elsif ($saw_bk)
	{
	    if ($was_rule)
	    {
		$output_trailer .= join ('', @conditional_stack) . $_;
		$saw_bk = /\\$/;
	    }
	    else
	    {
		$saw_bk = /\\$/;
		# Chop newline and backslash if this line is
		# continued.  ensure trailing whitespace exists.
		chop if $saw_bk;
		chop if $saw_bk;
		$contents{$last_var_name} .= ' '
		    unless $contents{$last_var_name} =~ /\s$/;
		$contents{$last_var_name} .= $_;
		if (@conditional_stack)
		{
		    $conditional{$last_var_name} .= &quote_cond_val ($_);
		}
	    }
	}
	elsif (/$IF_PATTERN/o)
	{
	    &am_line_error ($., "$1 does not appear in AM_CONDITIONAL")
		if (! $configure_cond{$1});
	    push (@conditional_stack, "\@" . $1 . "_TRUE\@");
	}
	elsif (/$ELSE_PATTERN/o)
	{
	    if (! @conditional_stack)
	    {
		&am_line_error ($., "else without if");
	    }
	    elsif ($conditional_stack[$#conditional_stack] =~ /_FALSE\@$/)
	    {
		&am_line_error ($., "else after else");
	    }
	    else
	    {
		$conditional_stack[$#conditional_stack]
		    =~ s/_TRUE\@$/_FALSE\@/;
	    }
	}
	elsif (/$ENDIF_PATTERN/o)
	{
	    if (! @conditional_stack)
	    {
		&am_line_error ($., ": endif without if");
	    }
	    else
	    {
		pop @conditional_stack;
	    }
	}
	elsif (/$RULE_PATTERN/o)
	{
	    # Found a rule.
	    $was_rule = 1;
	    if (defined $contents{$1}
		&& (@conditional_stack
		    ? ! defined $conditional{$1}
		    : defined $conditional{$1}))
	    {
		&am_line_error ($1,
				"$1 defined both conditionally and unconditionally");
	    }
	    # Value here doesn't matter; for targets we only note
	    # existence.
	    $contents{$1} = 1;
	    $targets{$1} = 1;
	    my $cond_string = join ('', @conditional_stack);
	    if (@conditional_stack)
	    {
		if ($conditional{$1})
		{
		    &check_ambiguous_conditional ($1, $cond_string);
		    $conditional{$1} .= ' ';
		}
		else
		{
		    $conditional{$1} = '';
		}
		$conditional{$1} .= $cond_string . ' 1';
	    }
	    $content_lines{$1} = $.;
	    $output_trailer .= $comment . $spacing . $cond_string . $_;
	    $comment = $spacing = '';
	    $saw_bk = /\\$/;

	    # Check the rule for being a suffix rule. If so, store in
	    # a hash.

	    my $source_suffix;
	    my $object_suffix;

	    if (($source_suffix, $object_suffix) = ($1 =~ $SUFFIX_RULE_PATTERN)) 
	    {
	      $suffix_rules{$source_suffix} = $object_suffix;
	      print "Sources ending in .$source_suffix become .$object_suffix\n" if $verbose;
	      $source_suffix_pattern = "(" . join('|', keys %suffix_rules) . ")";
	    }

	    # FIXME: make sure both suffixes are in SUFFIXES? Or set
	    # SUFFIXES from suffix_rules?
	}
	elsif (($is_ok_macro = /$MACRO_PATTERN/o)
	       || /$BOGUS_MACRO_PATTERN/o)
	{
	    # Found a macro definition.
	    $was_rule = 0;
	    $last_var_name = $1;
	    if (defined $contents{$1}
		&& (@conditional_stack
		    ? ! defined $conditional{$1}
		    : defined $conditional{$1}))
	    {
		&am_line_error ($1,
				"$1 defined both conditionally and unconditionally");
	    }
	    my $value;
	    if ($3 ne '' && substr ($3, -1) eq "\\")
	    {
		$value = substr ($3, 0, length ($3) - 1);
	    }
	    else
	    {
		$value = $3;
	    }
	    my $type = $2;
	    if ($type eq '+')
	    {
		if (! defined $contents{$last_var_name}
		    && defined $configure_vars{$last_var_name})
		{
		    $contents{$last_var_name} = '@' . $last_var_name . '@';
		}
		$contents{$last_var_name} .= ' ' . $value;
	    }
	    else
	    {
		$contents{$last_var_name} = $value;
		# The first assignment to a macro sets the line
		# number.  Ideally I suppose we would associate line
		# numbers with random bits of text.
		$content_lines{$last_var_name} = $.;
	    }
	    my $cond_string = join ('', @conditional_stack);
	    if (@conditional_stack)
	    {
		my $found = 0;
		my $val;
		if ($conditional{$last_var_name})
		{
		    if ($type eq '+')
		    {
			# If we're adding to the conditional, and it
			# exists, then we might want to simply replace
			# the old value with the new one.
			my (@new_vals, @cond_vals);
			@cond_vals = split (' ', $conditional{$last_var_name});
			while (@cond_vals)
			{
			    $vcond = shift (@cond_vals);
			    push (@new_vals, $vcond);
			    if (&conditional_same ($vcond, $cond_string))
			    {
				$found = 1;
				$val = (&unquote_cond_val (shift (@cond_vals))
					. ' ' . $value);
				push (@new_vals, &quote_cond_val ($val));
			    }
			    else
			    {
				push (@new_vals, shift (@cond_vals));
			    }
			}
			if ($found)
			{
			    $conditional{$last_var_name}
			        = join (' ', @new_vals);
			}
		    }

		    if (! $found)
		    {
			&check_ambiguous_conditional ($last_var_name,
						      $cond_string);
			$conditional{$last_var_name} .= ' ';
			$val = $value;
		    }
		}
		else
		{
		    $conditional{$last_var_name} = '';
		    $val = $contents{$last_var_name};
		}
		if (! $found)
		{
		    $conditional{$last_var_name} .= ($cond_string
						     . ' '
						     . &quote_cond_val ($val));
		}
	    }
	    # FIXME: this doesn't always work correctly; it will group
	    # all comments for a given variable, no matter where
	    # defined.
	    $am_vars{$last_var_name} = $comment . $spacing;
	    $def_type{$last_var_name} = ($type eq ':') ? ':' : '';
	    push (@var_list, $last_var_name);
	    $comment = $spacing = '';
	    $saw_bk = /\\$/;

	    # Error if bogus.
	    &am_line_error ($., "bad macro name \`$last_var_name'")
		if ! $is_ok_macro;
	}
        elsif (/$INCLUDE_PATTERN/o)
        {
            my ($path) = $1;

            if ($path =~ s/^\$\(top_srcdir\)\///)
            {
                push (@include_stack, "\$\(top_srcdir\)/$path");
            }
            else
            {
                $path =~ s/\$\(srcdir\)\///;
                push (@include_stack, "\$\(srcdir\)/$path");
                $path = $relative_dir . "/" . $path;
            }
            &read_am_file ($path);
        }
	else
        {
	    # This isn't an error; it is probably a continued rule.
	    # In fact, this is what we assume.
	    $was_rule = 1;
	    $output_trailer .= ($comment . $spacing
				. join ('', @conditional_stack) . $_);
	    $comment = $spacing = '';
	    $saw_bk = /\\$/;
	}

	$_ = <AM_FILE>;
    }

    close(AM_FILE);
    $output_trailer .= $comment;

    &am_error ("unterminated conditionals: " . join (' ', @conditional_stack))
	if (@conditional_stack);
}

sub initialize_per_input {
    # These two variables are used when generating each Makefile.in.
    # They hold the Makefile.in until it is ready to be printed.
    $output_vars = '';
    $output_trailer = '';

    # This holds the contents of a Makefile.am, as parsed by
    # read_am_file.
    %contents = ();

    # This holds the names which are targets.  These also appear in
    # %contents.
    %targets = ();

    # For a variable or target which is defined conditionally, this
    # holds an array of the conditional values.  The array is composed
    # of pairs of condition strings (the variables which configure
    # will substitute) and values (the value of a target is
    # meaningless).  For an unconditional variable, this is empty.
    %conditional = ();

    # This holds the line numbers at which various elements of
    # %contents are defined.
    %content_lines = ();

    # This holds a 1 if a particular variable was examined.
    %content_seen = ();

    # This is the conditional stack.
    @conditional_stack = ();

    # This holds the set of included files.
    @include_stack = ();

    # This holds the "relative directory" of the current Makefile.in.
    # Eg for src/Makefile.in, this is "src".
    $relative_dir = '';

    # This maps the source extension of a suffix rule to its
    # corresponding output extension.
    %suffix_rules = ();

}

# Quote a value in order to put it in $conditional.  We need to quote
# spaces, and we need to handle null strings, so that we can later
# retrieve values by splitting on space.
sub quote_cond_val
{
    my ($val) = @_;
    $val =~ s/ /\001/g;
    $val =~ s/\t/\003/g;
    $val = "\002" if $val eq '';
    return $val;
}

# See if a conditional is true.  Both arguments are conditional
# strings.  This returns true if the first conditional is true when
# the second conditional is true.
sub conditional_true_when
{
    my ($cond, $when) = @_;

    # Check the easy case first.
    if ($cond eq $when)
    {
	return 1;
    }

    # Check each component of $cond, which looks @COND1@@COND2@.
    foreach my $comp (split ('@', $cond))
    {
	# The way we split will give null strings between each
	# condition.
	next if ! $comp;

	if (index ($when, '@' . $comp . '@') == -1)
	{
	    return 0;
	}
    }

    return 1;
}

# Check for an ambiguous conditional.  This is called when a variable
# or target is being defined conditionally.  If we already know about
# a definition that is true under the same conditions, then we have an
# ambiguity.
sub check_ambiguous_conditional
{
    my ($var_name, $cond) = @_;
    my (@cond_vals) = split (' ', $conditional{$var_name});
    while (@cond_vals)
    {
	my ($vcond) = shift (@cond_vals);
	shift (@cond_vals);
	if (&conditional_true_when ($vcond, $cond)
	    || &conditional_true_when ($cond, $vcond))
	{
	    print "$var_name multiply defined in condition";
	}
    }
}

sub am_line_error
{
    my ($symbol, @args) = @_;

    if ($symbol && "$symbol" ne '-1')
    {
	my ($file) = "${am_file}";

	if ($symbol =~ /^\d+$/)
	{
	    # SYMBOL is a line number, so just add the colon.
	    $file .= ':' . $symbol;
	}
	elsif (defined $content_lines{$symbol})
	{
	    # SYMBOL is a variable defined in Makefile.am, so add the
	    # line number we saved from there.
	    $file .= ':' . $content_lines{$symbol};
	}
	elsif (defined $configure_vars{$symbol})
	{
	    # SYMBOL is a variable defined in configure.ac, so add the
	    # appropriate line number.
	    $file = $configure_vars{$symbol};
	}
	else
	{
	    # Couldn't find the line number.
	}
	warn $file, ": ", join (' ', @args), "\n";
	$exit_status = 1;
    }
    else
    {
	&am_error (@args);
    }
}

sub generate_dsw
{
    my $name = shift;
    my $dsw_name = $name . '.dsw';
    open(DSW, ">$dsw_name")
	|| die "Can't create $dsw_name: $!\n";

    print "Creating $dsw_name\n" if $verbose;

    print DSW <<"EOF";
Microsoft Developer Studio Workspace File, Format Version 6.00\r
# WARNING: DO NOT EDIT OR DELETE THIS WORKSPACE FILE!\r
\r
###############################################################################\r
\r
EOF
    print DSW 'Project: ', "\"$name\"=\".\\", $name, ".dsp\" - Package Owner=<4>\r\n";
    print DSW <<"EOF";
\r
Package=<5>\r
{{{\r
}}}\r
\r
Package=<4>\r
{{{\r
}}}\r
\r
EOF

    foreach my $p (@extra_projects) {
	print DSW "###############################################################################\r\n\r\n";
	my ($dsp,$name) = split ',', $p;
	print "Project $name=$dsp\n" if $verbose;
	print DSW "Project: \"$name\"=\"$dsp\" - Package Owner=<4>\r\n\r\n";
	print DSW <<"EOF";
Package=<5>\r
{{{\r
}}}\r
\r
Package=<4>\r
{{{\r
}}}\r
\r
EOF
    }

    print DSW <<"EOF";
###############################################################################\r
\r
Global:\r
\r
Package=<5>\r
{{{\r
}}}\r
\r
Package=<3>\r
{{{\r
}}}\r
\r
###############################################################################\r
\r
EOF
    close(DSW);
}

sub expand_here {
    local $_ = shift;
    s/\%cflags\%/$msvc_cflags/g;
    s/\%cflagsd\%/$msvc_cflagsd/g;
    s/\%libs\%/$msvc_libs/g;
    return $_;
}

sub console_app_dsp_init
{
    my $name = shift;
    my $dsp_name = $name . '.dsp';

    open(DSP, ">$dsp_name")
	|| die "Can't create $dsp_name: $!\n";

    print "Creating $dsp_name\n" if $verbose;

    print DSP expand_here(<<"EOF");
# Microsoft Developer Studio Project File - Name="$name" - Package Owner=<4>\r
# Microsoft Developer Studio Generated Build File, Format Version 6.00\r
# ** DO NOT EDIT **\r
\r
# TARGTYPE "Win32 (x86) Console Application" 0x0103\r
\r
CFG=$name - Win32 Debug\r
!MESSAGE This is not a valid makefile. To build this project using NMAKE,\r
!MESSAGE use the Export Makefile command and run\r
!MESSAGE \r
!MESSAGE NMAKE /f "$name.mak".\r
!MESSAGE \r
!MESSAGE You can specify a configuration when running NMAKE\r
!MESSAGE by defining the macro CFG on the command line. For example:\r
!MESSAGE \r
!MESSAGE NMAKE /f "$name.mak" CFG="$name - Win32 Debug"\r
!MESSAGE \r
!MESSAGE Possible choices for configuration are:\r
!MESSAGE \r
!MESSAGE "$name - Win32 Release" (based on "Win32 (x86) Console Application")\r
!MESSAGE "$name - Win32 Debug" (based on "Win32 (x86) Console Application")\r
!MESSAGE \r
\r
# Begin Project\r
# PROP AllowPerConfigDependencies 0\r
# PROP Scc_ProjName ""\r
# PROP Scc_LocalPath ""\r
CPP=cl.exe\r
RSC=rc.exe\r
\r
!IF  "\$(CFG)" == "$name - Win32 Release"\r
\r
# PROP Use_MFC 0\r
# PROP Use_Debug_Libraries 0\r
# PROP Output_Dir "Release"\r
# PROP Intermediate_Dir "Release"\r
# PROP Target_Dir ""\r
# ADD CPP /nologo /W3 /GX /O2 /D "NDEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /FD /c %cflags%\r
# SUBTRACT CPP /YX\r
# ADD RSC /l 0xc09 /d "NDEBUG"\r
BSC32=bscmake.exe\r
# ADD BSC32 /nologo\r
LINK32=link.exe\r
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib uuid.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386 %libs%\r
\r
!ELSEIF  "\$(CFG)" == "$name - Win32 Debug"\r
\r
# PROP Use_MFC 0\r
# PROP Use_Debug_Libraries 1\r
# PROP Output_Dir "Debug"\r
# PROP Intermediate_Dir "Debug"\r
# PROP Ignore_Export_Lib 0\r
# PROP Target_Dir ""\r
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /D "_DEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /FR /FD /GZ /c %cflagsd%\r
# ADD RSC /l 0xc09 /d "_DEBUG"\r
BSC32=bscmake.exe\r
# ADD BSC32 /nologo\r
LINK32=link.exe\r
# ADD LINK32 kernel32.lib user32.lib winspool.lib comdlg32.lib gdi32.lib shell32.lib wsock32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept %libs%\r
\r
!ENDIF \r
\r
# Begin Target\r
\r
# Name "$name - Win32 Release"\r
# Name "$name - Win32 Debug"\r
EOF

    close(DSP);
}

sub static_lib_dsp_init
{
    my $name = shift;
    my $dsp_name = $name . '.dsp';

    open(DSP, ">$dsp_name")
	|| die "Can't create $dsp_name: $!\n";

    print "Creating $dsp_name\n" if $verbose;

    print DSP expand_here(<<"EOF");
# Microsoft Developer Studio Project File - Name="$name" - Package Owner=<4>\r
# Microsoft Developer Studio Generated Build File, Format Version 6.00\r
# ** DO NOT EDIT **\r
\r
# TARGTYPE "Win32 (x86) Static Library" 0x0104\r
\r
CFG=$name - Win32 Debug\r
!MESSAGE This is not a valid makefile. To build this project using NMAKE,\r
!MESSAGE use the Export Makefile command and run\r
!MESSAGE \r
!MESSAGE NMAKE /f "$name.mak".\r
!MESSAGE \r
!MESSAGE You can specify a configuration when running NMAKE\r
!MESSAGE by defining the macro CFG on the command line. For example:\r
!MESSAGE \r
!MESSAGE NMAKE /f "$name.mak" CFG="$name - Win32 Debug"\r
!MESSAGE \r
!MESSAGE Possible choices for configuration are:\r
!MESSAGE \r
!MESSAGE "$name - Win32 Release" (based on "Win32 (x86) Static Library")\r
!MESSAGE "$name - Win32 Debug" (based on "Win32 (x86) Static Library")\r
!MESSAGE \r
\r
# Begin Project\r
# PROP AllowPerConfigDependencies 0\r
# PROP Scc_ProjName ""\r
# PROP Scc_LocalPath ""\r
CPP=cl.exe\r
RSC=rc.exe\r
\r
!IF  "\$(CFG)" == "$name - Win32 Release"\r
\r
# PROP Use_MFC 0\r
# PROP Use_Debug_Libraries 0\r
# PROP Output_Dir "Release"\r
# PROP Intermediate_Dir "Release"\r
# PROP Target_Dir ""\r
# ADD CPP /nologo /W3 /GX /O2 /D "NDEBUG" /D "WIN32" /D "_MBCS" /FD /c %cflags%\r
# ADD RSC /l 0x409 /d "NDEBUG"\r
BSC32=bscmake.exe\r
# ADD BASE BSC32 /nologo\r
# ADD BSC32 /nologo\r
LINK32=link.exe -lib\r
# ADD BASE LIB32 /nologo\r
# ADD LIB32 /nologo\r
\r
!ELSEIF  "\$(CFG)" == "$name - Win32 Debug"\r
\r
# PROP Use_MFC 0\r
# PROP Use_Debug_Libraries 1\r
# PROP Output_Dir "Debug"\r
# PROP Intermediate_Dir "Debug"\r
# PROP Target_Dir ""\r
# ADD CPP /nologo /W3 /GX /ZI /Od /D "_DEBUG" /D "WIN32" /D "_MBCS" /FR /FD /GZ /c %cflagsd%\r
# ADD RSC /l 0x409 /d "_DEBUG"\r
BSC32=bscmake.exe\r
# ADD BASE BSC32 /nologo\r
# ADD BSC32 /nologo\r
LINK32=link.exe -lib\r
# ADD BASE LIB32 /nologo\r
# ADD LIB32 /nologo\r
\r
!ENDIF \r
\r
# Begin Target\r
\r
# Name "$name - Win32 Release"\r
# Name "$name - Win32 Debug"\r
EOF

    close(DSP);
}

sub dsp_add_source_rule {
    my ($fh,$dsp_name,$group,$file) = @_;
    print $fh "# Begin Source File\r\n";
    print $fh "\r\n";
    print $fh "SOURCE=$file\r\n";
    print $fh "\r\n";
    print $fh "!IF  \"\$(CFG)\" == \"$dsp_name - Win32 Release\"\r\n";
    print $fh "\r\n";
    print $fh "# PROP Intermediate_Dir \"Release\\$group\"\r\n";
    print $fh "# PROP Exclude_From_Build 1\r\n" if exclude_file($file);
    print $fh "\r\n";
    print $fh "!ELSEIF  \"\$(CFG)\" == \"$dsp_name - Win32 Debug\"\r\n";
    print $fh "\r\n";
    print $fh "# PROP Intermediate_Dir \"Debug\\$group\"\r\n";
    print $fh "# PROP Exclude_From_Build 1\r\n" if exclude_file($file);
    print $fh "\r\n";
    print $fh "!ENDIF \r\n";
    print $fh "\r\n";
    print $fh "# End Source File\r\n";
}

sub dsp_add_group {
    my ($dsp_name,$makefile) = @_;
    my $base_dir = './';

    initialize_per_input();
    my $relative_dir = dirname($makefile);

    read_main_am_file($makefile . '.am');

    open(DSP, ">>$dsp_name" . '.dsp')
	|| die "Can't append to $dsp_name: $!\n";

    foreach my $key (sort keys %contents) {
	if ($key eq "include_HEADERS") {
	}
	if ($key =~ /^lib(.*)_a_SOURCES/) {
	    my $group = 'Lib_' . $1;
	    print DSP "# Begin Group \"$group\"\r\n";
	    print DSP "\r\n";
	    print DSP "# PROP Default_Filter \"\"\r\n";
	    my @files = split(' ', $contents{$key});
	    foreach (@files) {
		my $file;
		my $src_dir = $base_dir . $relative_dir . '/';
		$src_dir =~ s/\//\\/g; # fixup DOS path separators

		if (/^\$\(([^\)]*)\)$/) {
		    # Found a variable.
		    my $varname = $1;
		    foreach (split(' ', $contents{$varname})) {
			$file = $src_dir . $_;
			dsp_add_source_rule(\*DSP, $dsp_name, $group, $file);
		    }
		} else {
		    $file = $src_dir . $_;
		    dsp_add_source_rule(\*DSP, $dsp_name, $group, $file);
		}
	    }

	    print DSP "# End Group\r\n";
	}
	#elsif ($key =~ /(.*)_SOURCES/) {
	elsif ($key eq "fgfs_SOURCES") {
	    my $group = 'main';
	    print DSP "# Begin Group \"$group\"\r\n";
	    print DSP "\r\n";
	    print DSP "# PROP Default_Filter \"\"\r\n";
	    my @files = split(' ', $contents{$key});
	    foreach (@files) {
		my $file;
		my $src_dir = $base_dir . $relative_dir . '/';
		$src_dir =~ s/\//\\/g; # fixup DOS path separators

		dsp_add_source_rule(\*DSP, $dsp_name, $group, $src_dir . $_);
		if (0) {
		    my $f = $base_dir . $relative_dir . '/' . $_;
		    $f =~ s/\//\\/g;
		    print DSP "# Begin Source File\r\n";
		    print DSP "\r\n";
		    print DSP "SOURCE=$f\r\n";
		    print DSP "\r\n";
		    print DSP "!IF  \"\$(CFG)\" == \"$dsp_name - Win32 Release\"\r\n";
		    print DSP "\r\n";
		    print DSP "# PROP Intermediate_Dir \"Release\\$group\"\r\n";
		    print DSP "\r\n";
		    print DSP "!ELSEIF  \"\$(CFG)\" == \"$dsp_name - Win32 Debug\"\r\n";
		    print DSP "\r\n";
		    print DSP "# PROP Intermediate_Dir \"Debug\\$group\"\r\n";
		    print DSP "\r\n";
		    print DSP "!ENDIF \r\n";
		    print DSP "\r\n";
		    print DSP "# End Source File\r\n";
		}
	    }
	    print DSP "# End Group\r\n";
	}
    }
    close(DSP);
}

sub dsp_finish {
    my $dsp_name = shift;

    open(DSP, ">>$dsp_name" . '.dsp')
	|| die "Can't append to $dsp_name: $!\n";

    foreach my $r (@extra_sources) {
	print DSP "# Begin Source File\r\n\r\n";
	print DSP "$r\r\n";
	print DSP "# End Source File\r\n";
    }
    print DSP "# End Target\r\n";
    print DSP "# End Project\r\n";

    close(DSP);
}

# Return directory name of file.
sub dirname
{
    my ($file) = @_;
    my ($sub);

    ($sub = $file) =~ s,/+[^/]+$,,g;
    $sub = '.' if $sub eq $file;
    return $sub;
}

