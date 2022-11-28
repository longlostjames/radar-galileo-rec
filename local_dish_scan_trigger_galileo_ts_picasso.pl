#!/usr/bin/perl -w 
# Owain Davies 05062003
# look out for an inforec packet
# MOD
# now pass the date/time from shadowfax
# raster numbers no longer exist, the scan number is now the raster number 
# Owain Davies 12022004
# MOD
# Mark Fortescue - 20171212 - Updated to take a command line argument
# MOD
# Mark Fortescue - 20180409 - Updated to add in -tssamples 250 in addition to -tsdump

use Fcntl;
require "asm/ioctls.ph";
use IO::Socket;
use POSIX ":sys_wait_h";


# returns the scan type be it FIXED/RHI/PPI
sub type_of_scan
{
   $ir = $_[0];
   $st = ord (substr($ir,37,1));
   #print $st."\n";
   if ( $st == 0) { $scan_type = "NOT_KNOWN"; }
   if ( $st == 1) { $scan_type = "RHI"; }
   if ( $st == 2) { $scan_type = "PPI"; }
   if ( $st == 3) { $scan_type = "CSP"; }
   if ( $st == 4) { $scan_type = "FIX"; }
   if ( $st == 5) { $scan_type = "C"; }
   if ( $st > 5)  { $scan_type = "NOT_KNOWN"; } 
   return $scan_type;
}

	

sub angles_of_scan
{
   $ir = $_[0];
   $min_angle_lsb = ord(substr($ir,21,1));
   $min_angle_msb = ord(substr($ir,20,1));
   $max_angle_lsb = ord(substr($ir,23,1));
   $max_angle_msb = ord(substr($ir,22,1));
   
   $temp = (($min_angle_msb << 8) + $min_angle_lsb);
   if ( $temp & 0x8000) {
	$temp -= 2**16;
   }
   $min_angle = $temp/60;
   
   $temp = (($max_angle_msb << 8) + $max_angle_lsb);
   if ( $temp & 0x8000) {
        $temp -= 2**16;
   }
   $max_angle = $temp/60;
	

   #print "$min_angle, $max_angle \n";
   return $min_angle, $max_angle;

} 

sub scan_angle
{
   $ir = $_[0];
   $scan_angle_lsb = ord(substr($ir,49,1));
   $scan_angle_msb = ord(substr($ir,48,1));
   
   $scan_angle = (($scan_angle_msb << 8) + $scan_angle_lsb)/60;

   return $scan_angle;
}

sub raster_number 
{
   $ir = $_[0];
   $raster_lsb = ord(substr($ir,40,1));
   $raster_msb = ord(substr($ir,39,1));
   $raster = (($raster_msb << 8) + $raster_lsb);
   return $raster;
}

sub file_number
{
   $ir = $_[0];
   $file_lsb = ord(substr($ir,53,1));
   $file_msb = ord(substr($ir,52,1));
   $file = (($file_msb << 8) + $file_lsb);
   return $file;
}

sub scan_velocity
{
   $ir = $_[0];
   $velocity = ord(substr($ir,36,1))/50;
   return $velocity;
}

MAIN:
{

$sock = IO::Socket::INET->new (
                LocalPort => 50000,
                Type => SOCK_DGRAM,
                Proto => "udp");
die "Could not create socket: $!\n" unless $sock;

$SIG{CHLD} = 'IGNORE';

$yyyymmddhhmmss = "0";
	
system("clear");

$argument = $ARGV[0];

while(1)
{
	print ("Extra Arguments: $argument\n");

	$program = "/home/jif/cl2-test/radar-galileo-rec";

	#system("clear");

	$udp_length = 0;
	
	if ($yyyymmddhhmmss ne "0") 
	{
		print "\nLast scan : $scan_type at $yyyymmddhhmmss\n";
	}

	print("Waiting for a UDP packet from shadowfax (this should indicate the start of a scan)\n\n");
	while ($udp_length != 64) 
	{
		recv( $sock, $ir, 64, 0);
		$udp_length = length($ir);
		print "UDP packet received, length : ".$udp_length."\n";
	}

	#for ($n=0; $n<64; $n++) {
	#	print "$n:".ord(substr($ir,$n,1))." "; 
	#}

	$galileo_on = ord(substr($ir, 55));
	
	if ($galileo_on == 1)
	{

	$scan_type = type_of_scan($ir);
	$experiment_id = ord(substr($ir,35,1));
	$raster = raster_number($ir);
	$file = file_number($ir);
	$operator = substr($ir,43,3);
	$velocity = scan_velocity($ir);
	($min_angle, $max_angle) = angles_of_scan($ir);
	$scan_angle = scan_angle($ir); 
        $psd = ord(substr($ir,38));
	$year 	= ord(substr($ir, 33)) + 2000;
	$month	= ord(substr($ir, 32));
	$day	= ord(substr($ir, 31));
	$hour	= ord(substr($ir, 42));
	$minute = ord(substr($ir, 46));
	$second = ord(substr($ir, 47)); 
	$yyyymmddhhmmss = sprintf("%04d%02d%02d%02d%02d%02d", 	
		$year, $month, $day, $hour, $minute, $second);

	print "\007";
	print "\n";
	print "------------------------------------\n";
	print "     CONTROL BLOCK PARAMETERS\n";
	print "------------------------------------\n";
	print "GALILEO RECORDING   : ON\n";
	print "SCAN TYPE           : $scan_type\n";
	print "FILE NUMBER         : $file\n";
	print "SCAN NUMBER         : $raster\n";
	print "OPERATOR ID         : $operator\n";
	print "EXPERIMENT ID       : $experiment_id\n"; 
	print "PSD                 : $psd\n";
	print "YYYYMMDDHHMMSS      : $yyyymmddhhmmss\n";
	print "MIN,MAX SCAN ANGLES : $min_angle,$max_angle degrees\n"; 
	print "SCAN ANGLE          : $scan_angle degrees\n";
	print "SCAN VELOCITY       : $velocity degrees/s\n";
	print "------------------------------------\n\n";

	$child = fork;

	if ($child != 0)
	{
		# parent
		# print "parent, child $child\n";
		$sister = fork;
		if ($sister == 0)
		{
			# start of sister process
			print "The sister process is looking out for an interrupt UDP packet\n";
			recv( $sock, $ir, 64, 0);
			print "Received a possible interrupt packet, length : ". length($ir)."\n";
			# sister needs to send a signal to signal to the 
			# first obtain pid of the radar recording program
			$a = `pgrep radar-galileo`;
			$a = `pkill -SIGINT radar-galileo`;
			exit(0);		
			print ("Why no exit!\n");
		}
		print "Starting radar recording program using process number $child\n";
		waitpid($child, 0);
		print "Process number $child has exited (radar recording program handler)\n";
		print "The radar recording program must have exited\n";
		print "Waiting for sister to exit, pid $sister\n";
		kill ( "TERM", $sister);
		waitpid($sister, 0);
		print "Sister has exited\n";
		# need to check sister is not active here
	} else {	
		if ($scan_type eq "FIX") 
   		{
			$command = $program." -".lc($scan_type)." 3600"." ".$scan_angle." ".$min_angle;
			$command .= " -position-msg";
   		} elsif ($scan_type ne "NOT_KNOWN") {
   			$command  = $program." -".lc($scan_type)." ".$min_angle." ".$max_angle." -scan_angle ".$scan_angle;
			$command .= " -sv ".$velocity." -position-msg";
   		}
                $more_flags = " -op ".$operator." -tsdump -tssamples 250 -swap ";
 		# if ($psd > 0) { $more_flags .= "-spec $psd "; }
		$more_flags .= "-id $experiment_id -file $file -scan $raster -date $yyyymmddhhmmss ".$argument;
		$log_file = "/var/log/radar-galileo/operation_log.txt"; 
		if ($scan_type ne "NOT_KNOWN")
		{
		#	$command .= " -position-msg";  # CJW 17-01-18
			$actual_command = $command;
			$actual_command .= $more_flags;
   			print $actual_command."\n";
   			$return_value = system ($actual_command." 2>&1 >> ".$log_file);
			print "The exit value of the radar recording program is $return_value\n";
			if ($return_value != 0) { 
				#print "Exiting child process (cohavg)\n";
				exit(0);
			}
		}	
		exit(0);
		print("Why no exit!\n");
	}
	} else {
		print "\007";
        	print "\n";
        	print "------------------------------------\n";
        	print "     CONTROL BLOCK PARAMETERS\n";
        	print "------------------------------------\n";
        	print "GALILEO RECORDING   : OFF\n";
		print "------------------------------------\n";
		print "Do you to GALILEO ON on shadowfax?\n";
	}
	
# end of while loop
}

close($sock);

}

