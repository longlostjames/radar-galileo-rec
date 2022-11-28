#!/usr/bin/perl -w 
# Owain Davies 08062004
# Darcy Ladd  14012011
# Alan Doo - 20130430 - Update to accept single command line arg
# Mark Fortescue - 20171212 - Updated to take two command line arguments

MAIN:
{

while(1)
{
	# print "Checking Galileo tx power\n";
	# $program = "/usr/local/bin/measure_radar-galileo_transmit_power.pl";
	# system($program);

	print "invoking recording program\n";
#	$program = "./radar-galileo-rec $ARGV[0] $ARGV[1] > /dev/null";
        $program = "./radar-galileo-rec $ARGV[0] $ARGV[1] > /tmp/test.txt";
	system($program);

	print "sleeping for 2 seconds before restart\n";
	sleep 2;
}

}

