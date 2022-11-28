#define VERSION_NUMBER "1.5"

/*****************************************************************
 * radar-galileo-rec.c
 * Development Version
 * ---------------------------------------------------------------
 * Acquisition and signal processing software for 35 GHz radar
 *
 * Owain Davies / Ed Pavelin - May 2004
 * Based on radar-acrobat-rec
 *
 * ---------------------------------------------------------------
 * REVISION HISTORY
 * ---------------------------------------------------------------
 *
 * 20061207 OTD copy over of code from radar-copernicus and adapted
 *
 * 20071026 OTD i and q, in addition to the spectra (version 0.5)
 *
 * 20110729 AD  updated for changed in RNC since 2007 plus
 *              modifications for checked-out build location on
 *              NELSON test system
 *
 * 20130429 AD  updated to introduce serial message switch
 *              "--noserial" for fixed position operation
 *              (version 0.6)
 *
 * 20141112 JCN include LDR recording (version 0.7)
 * 20150721 JCN new version for HV pulse pair velocity estimation
 * 20151203 JCN include PHIDP_C recording
 * 20161216 CJW modifications for mode switching
 * 20170815 MTF fix buffer overruns. Update serial message
 *              processing code and change defailt sens/name
 *              of -no-serial -> -position-msg
 * 20180802 MTF Fix Timeseries recording issue
 * 20190912 MTF Fix single polarisation modes.
 *          CJW Fix some incorrectly handled netcdf variable initialisation.
 *          MTF Make changes to include antennae separation in static variables.
 * 20191002 CJW Add in code for SPWVC
 *****************************************************************/

#define NDEBUG

#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>

#include "median.h"
#include <complex.h>
//#define FFTW_NO_Complex
#include <fftw3.h>

#ifdef HAVE_DISLIN
#include <dislin.h>
#endif /* HAVE_DISLIN */

/* PCI DIO24 card stuff */
#include <ctype.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "ixpio.h"

char *device = "/dev/ixpio1";

/* netCDF : netCDF library header file */
#include <netcdf.h>

#include <radar.h> // Master header file for the Universal Radar Code
#include <RSP.h>   // Include file for the RSP package
#include <RDQ.h>   // Include file for the RDQ package
#include <RNC.h>   // Include file for the RNC package
#include <RSM.h>   // Include file for the RSM package
#include <RTS.h>   // Include file for the RTS package

/* header file */
#include "radar-galileo-rec.h"

#define RSP_MOMENTS 3 /* Original is 3. Now 5 is also supported. */

/*---------------------------------------------------------------*
 * below defines the com port that the serial message arrives on *
 * 1 is /dev/ttyS0 or COM1 in MS-DOS langauge                    *
 *---------------------------------------------------------------*/
#define SERIALMESSAGE_PORT "/dev/ttyS1"

typedef struct TimeSeriesObs_st
{
    int azimuthid;
    int elevationid;
    int tsid;
    int dish_tsid;
    int ICOHid;
    int QCOHid;
    int ICXHid;
    int QCXHid;
    int TxPower1id;
    int TxPower2id;
    int VnotHid;
    int RawLogid;

    uint16_t * ICOH;
    uint16_t * QCOH;
    uint16_t * ICXH;
    uint16_t * QCXH;
    uint16_t * TxPower1;
    uint16_t * TxPower2;
    uint16_t * VnotH;
    uint16_t * RawLog;
} TimeSeriesObs_t;

/* function prototype declaration */
static void sig_handler (int sig);
static void SetupTimeSeriesVariables (TimeSeriesObs_t *          obs,
				      int                        ncid,
				      RSP_ParamStruct *          param,
				      URC_ScanStruct *           scan,
				      RNC_DimensionStruct *      dimensions,
				      RSP_ObservablesStruct *    posobs);
static void WriteOutTimeSeriesData (int ncid,
				    const RSP_ParamStruct * param,
				    RSP_ObservablesStruct * posobs,
				    TimeSeriesObs_t * obs,
				    int moment);

#ifdef HAVE_DISLIN
static float 	zmat[256][250];
static int	counter_x, counter_y;
#endif

/* ----------------------------*
 * GLOBAL VARIABLE DEFINITIONS *
 *-----------------------------*/
static int     dmux_table[8];

/*---------------------------------------------------*
 * used for communication between the signal handler *
 * and the main program                              *
 *---------------------------------------------------*/
static bool	exit_now = false;

/* Command line switches */
static bool     debug            = false;
static bool     swap_iq_channels = false;
static bool     tsdump           = false;
static bool     TextTimeSeries   = false;

/* Disable position message for fixed position operation */
static bool positionMessageAct = false;	// default is OFF

// Displays a welcome message with version information
static inline void
disp_welcome_message (void)
{
    printf ("\nradar-galileo-rec: Version %s\n\n", VERSION_NUMBER);
}

/* signal handler */
static void
sig_handler (int sig)
{
    if (!exit_now)
    {
	exit_now = true;
	printf ("***********************************\n");
	printf ("* Received signal %d. Exiting soon *\n", sig);
	printf ("***********************************\n");
    }
}


// Displays help on command-line arguments
static inline void
disp_help (const char * prog)
{
    const char * pt = strrchr (prog, '/');

    if (pt++ != NULL)
	prog = pt;

    printf ("Usage: %s [options]\n\n", prog);
    printf ("Options:\n");
    printf ("--------\n");
    //printf ("\n Configuration options (how to record the data)\n");
    //printf (" ----------------------------------------------\n");
    //printf (" -config filename      Specify alternative config file\n");
    //printf (" -phase                Record absolute phase and std. dev. of phase\n");
    //printf (" -rawzed               Switch off range normalisation and calibration for Z\n");
    //printf (" -spec                 Record power spectra\n");
    printf (" -tsdump               Dump time series to a binary file\n");
    printf (" -tsdump.txt           Dump time series to a text file\n");
    printf (" -tssamples <n>        Only dump first <n> time series samples. 20 < n <= <max-gates>\n");
    printf (" -tsrange <n>          Only dump first <n> km of time series samples. 1.2 < n <= <max-range>\n");
    printf (" -position-msg         Enable 25m Antenna position message reading\n");
    printf (" -long-pulse           Enable single mode long-pulses\n");
    printf (" -alt_modes            Enable alternating mode short-pulses\n");
    printf (" -mode0 <mode>         Set long, short or alternating mode0 mode\n");
    printf (" -mode1 <mode>         Set alternating mode1 mode\n");
    printf (" -nrays_mode0 <rays>   Set number of rays in mode0 for alternating modes\n");
    printf (" -nrays_mode1 <rays>   Set number of rays in mode1 for alternating modes\n");
    printf (" -pulses <num>         Set number of pulses per ray: New not implemented yet\n");
    printf (" -cellwidth <range>    Set cell size (m)           : New not implemented yet\n");
    printf (" -range <min> <max>    Set min/max range (km)\n");
    printf (" -debug                Enable extra output during acquisition\n");
    printf (" -swap                 Swap Co/Cross I/Q ADC data\n");
    //printf (" -quiet                Do not generate lots of text output\n");
    printf ("\n Scanning options (what scan to expect)\n");
    printf (" --------------------------------------\n");
    printf (" -ppi az1 az2          Record PPI scan\n");
    printf (" -rhi el1 el2          Record RHI scan\n");
    printf (" -fix n az1 el1        Fixed dwell for n seconds\n");
    printf (" -csp el1 el2          Record CSP scan\n");
    printf (" -single az1 el1       Record a single ray\n");
    printf (" -man az1 el1          Record Manual/Tracking scan\n");
    printf (" -track az1 el1        Record Manual/Tracking scan\n");
    printf (" -cal az1 el1          Record Calibration scan\n");
    printf (" -c   az1 el1          Record Calibration scan\n");
    printf ("\n Control block options (stored in NetCDF output file)\n");
    printf (" ----------------------------------------------------\n");
    printf (" -sv vel               Specify scan velocity\n");
    printf (" -scan_angle el1       Specify scan angle for PPI scan\n");
    printf (" -scan_angle az1       Specify scan angle for RHI/CSP scan\n");
    printf (" -file nnnn            Specify file (tape) number: Obsolete\n");
    printf (" -scan nnnn            Specify scan number\n");
    printf (" -id nn                Specify experiment ID\n");
    printf (" -date yyyymmddhhmmss  Specify scan date\n");
    printf (" -op xxx               Specify radar operator\n");
    printf ("\n");
}

static inline int
parseargs (int               argc,
	   char *            argv[],
	   RSP_ParamStruct * param,
	   URC_ScanStruct *  scan,
	   int *             start_day)
{
    /* Parse command line args */

    time_t       system_time;
    struct tm    tm;
    const char * operator;
    int          i;

    /*-------------------------------------*
     * Initialise defaults for scan params *
     *-------------------------------------*/
    scan->scanType      = SCAN_FIX;
    scan->file_number   = 0;
    scan->scan_number   = 0;
    scan->experiment_id = 0;
    scan->scan_velocity = -9999;
    scan->dwelltime     = -1;

    param->samples_per_pulse_ts = param->samples_per_pulse;

    operator = getenv ("USER");
    if (operator == NULL) operator = getenv ("USERNAME");
    if (operator == NULL) operator = getenv ("LOGNAME");
    if (operator == NULL) operator = "<unknown>";
    strncpy (scan->operator, operator, sizeof (scan->operator) - 1);
    scan->operator[sizeof (scan->operator) - 1] = '\0';

    system_time = time (NULL);
    gmtime_r (&system_time, &tm);
    strftime (scan->date, sizeof (scan->date),"%Y%m%d%H%M%S", &tm);
    *start_day = tm.tm_mday;

    /* by default real_time_spectra_display is off */
    param->real_time_spectra_display = 0;

    /*--------------------------------------*
     * Now parse any command line arguments *
     *--------------------------------------*/
    for (i = 1; i < argc; i++)
    {
	if (!strcmp (argv[i],"-real_time_spectra_display"))
	{
#ifdef HAVE_DISLIN
	    printf ("Real time spectra display enabled\n");
	    param->real_time_spectra_display = 1;
#else
	    printf ("Real time spectra display not available\n");
#endif /* HAVE_DISLIN */
	}
	else if (!strcmp (argv[i],"-ppi"))
	{
	    /* -------- *
	     * PPI SCAN *
	     * -------- */
	    scan->min_angle = atof (argv[++i]);
	    scan->max_angle = atof (argv[++i]);
	    printf ("PPI Scan %f-%f deg\n", scan->min_angle, scan->max_angle);
	    scan->scanType = SCAN_PPI;
	}
	else if (!strcmp (argv[i],"-rhi"))
	{
	    /* -------- *
	     * RHI SCAN *
	     * -------- */
	    scan->min_angle = atof (argv[++i]);
	    scan->max_angle = atof (argv[++i]);
	    printf ("RHI Scan %f-%f deg\n", scan->min_angle, scan->max_angle);
	    scan->scanType = SCAN_RHI;
	}
	else if (!strcmp (argv[i],"-csp"))
	{
	    /* ---------------- *
	     * SLANT PLANE SCAN *
	     * ---------------- */
	    scan->min_angle = atof (argv[++i]);
	    scan->max_angle = atof (argv[++i]);
	    printf ("CSP Scan %f-%f deg\n", scan->min_angle, scan->max_angle);
	    scan->scanType = SCAN_CSP;
	}
	else if (!strcmp (argv[i],"-fix"))
	{
	    /* ----------- *
	     * FIXED DWELL *
	     * ----------- */
	    scan->dwelltime  = atof (argv[++i]);
	    scan->scan_angle = atof (argv[++i]);
	    scan->min_angle  = atof (argv[++i]);
	    printf ("Fixed dwell for %f seconds\n", scan->dwelltime);
	    printf ("Position Az: %f, El: %f deg\n",
		    scan->scan_angle, scan->min_angle);
	    scan->scanType      = SCAN_FIX;
	    scan->max_angle     = scan->min_angle;
	    scan->scan_velocity = 0;
	}
	else if (!strcmp (argv[i], "-single"))
	{
	    /* ---------------- *
	     * FIXED SINGLE RAY *
	     * ---------------- */
	    scan->scan_angle = atof (argv[++i]);
	    scan->min_angle  = atof (argv[++i]);
	    printf ("Fixed single ray\n");
	    printf ("Position Az: %f, El: %f deg\n", scan->scan_angle, scan->min_angle);
	    scan->scanType      = SCAN_SGL;
	    scan->max_angle     = scan->min_angle; /* Make weird semantics consistant */
	    scan->scan_velocity = 0;
	}
	else if (!strcmp (argv[i], "-man"  ) ||
		 !strcmp (argv[i], "-track"))
	{
	    /* ------------- *
	     * TRACKING SCAN *
	     * ------------- */
	    scan->scan_angle = atof (argv[++i]);
	    scan->min_angle  = atof (argv[++i]);
	    printf ("Track Scan %f-%f deg\n", scan->scan_angle, scan->min_angle);
	    scan->scanType  = SCAN_MAN;
	    scan->max_angle = scan->min_angle; /* Make weird semantics consistant */
	}
	else if (!strcmp (argv[i], "-cal") ||
		 !strcmp (argv[i], "-c"  ))
	{
	    /* ---------------- *
	     * CALIBRATION SCAN *
	     * ---------------- */
	    scan->scan_angle = atof (argv[++i]);
	    scan->min_angle  = atof (argv[++i]);
	    printf ("Calibration Scan %f-%f deg\n", scan->scan_angle, scan->min_angle);
	    scan->scanType  = SCAN_CAL;
	    scan->max_angle = scan->min_angle; /* Make weird semantics consistant */
	}
	else if (!strcmp (argv[i],"-position-msg"))
	{
	    /* ---------------------------- *
	     * 25m Antenna POSITION MESSAGE *
	     * ---------------------------- */
	    positionMessageAct = true;
	    printf ("25m Antenna position message enabled\n");
	}
	else if (!strcmp (argv[i],"-sv"))
	{
	    /* ------------- *
	     * SCAN VELOCITY *
	     * ------------- */
	    scan->scan_velocity = atof (argv[++i]);
	}
	else if (!strcmp (argv[i],"-file"))
	{
	    /* ----------- *
	     * FILE NUMBER *
	     * ----------- */
	    printf ("%s %s is obsolete\n", argv[i], argv[i+1]);
	    scan->file_number = atoi (argv[++i]);
	    printf ("File number   : %04d\n", scan->file_number);
	}
	else if (!strcmp (argv[i],"-scan"))
	{
	    /* ----------- *
	     * SCAN NUMBER *
	     * ----------- */
	    scan->scan_number = atoi (argv[++i]);
	    printf ("Scan number   : %04d\n", scan->scan_number);
	}
	else if (!strcmp (argv[i],"-id"))
	{
	    /* ------------- *
	     * EXPERIMENT ID *
	     * ------------- */
	    scan->experiment_id = atoi (argv[++i]);
	    printf ("Experiment id : %d\n", scan->experiment_id);
	}
	else if (!strcmp (argv[i],"-scan_angle"))
	{
	    /* ---------- *
	     * SCAN ANGLE *
	     * ---------- */
	    scan->scan_angle = atof (argv[++i]);
	    printf ("Scan angle : %f\n", scan->scan_angle);
	}
	else if (!strcmp (argv[i],"-date"))
	{
	    /* ---- *
	     * DATE *
	     * ---- */
	    /* MTF: Fix buffer overrun when date is too big */
	    strncpy (scan->date, argv[++i], sizeof (scan->date) - 1);
	    scan->date[sizeof (scan->date) - 1] = '\0';
	}
	else if (!strcmp (argv[i],"-op"))
	{
	    /* -------- *
	     * OPERATOR *
	     * -------- */
	    /* MTF: Fix buffer overrun when operator id is too big */
	    strncpy (scan->operator, argv[++i], sizeof (scan->operator) - 1);
	    scan->operator[sizeof (scan->operator) - 1] = '\0';
	}
	else if (!strcmp (argv[i],"-long_pulse"))
	{
	    /* -------------------- *
	     * LONG PULSE Selection *
	     * -------------------- */
	    param->long_pulse_mode = 1; /* Changed to boolena arg */
	}
	else if (!strcmp (argv[i],"-alt_modes"))
	{
	    /* --------------------- *
	     * ALTERNATE PULSE modes *
	     * --------------------- */
	    param->alternate_modes = 1; /* Changed to boolena arg */
	}
	else if (!strcmp (argv[i],"-mode0"))
	{
	    /* ----------------- *
	     * MODE 0 Pulse Mode *
	     * ----------------- */
	    param->mode0 = atoi (argv[++i]);
	}
	else if (!strcmp (argv[i],"-mode1"))
	{
	    /* ----------------- *
	     * MODE 1 Pulse Mode *
	     * ----------------- */
	    param->mode1 = atoi (argv[++i]);
	}
	else if (!strcmp (argv[i],"-nrays_mode0"))
	{
	    /* ---------------------- *
	     * MODE 0 Pulse Ray count *
	     * ---------------------- */
	    param->nrays_mode0 = atoi (argv[++i]);
	}
	else if (!strcmp (argv[i],"-nrays_mode1"))
	{
	    /* ---------------------- *
	     * MODE 1 Pulse Ray count *
	     * ---------------------- */
	    param->nrays_mode1 = atoi (argv[++i]);
	}
	else if (!strcmp (argv[i],"-debug"))
	{
	    /* ---------------------------- *
	     * Output Debugging information *
	     * ---------------------------- */
	    debug = true;
	}
	else if (!strcmp (argv[i],"-swap"))
	{
	    /* ---------------------------------- *
	     * Swap Co/Cross polar ADC I/Q inputs *
	     * ---------------------------------- */
	    swap_iq_channels = true;
	}
	else if (!strcmp (argv[i], "-tsdump"))
	{
	    /* ------------------ *
	     * TIME SERIES SWITCH *
	     * ------------------ */
	    tsdump         = true;
	    TextTimeSeries = false;
	    printf ("Time series recording on");
	}
	else if (!strcmp (argv[i], "-tsdump.txt"))
	{
	    /* ------------------ *
	     * TIME SERIES SWITCH *
	     * ------------------ */
	    tsdump         = true;
	    TextTimeSeries = true;
	    printf ("Time series recording on");
	}
	else if (!strcmp (argv[i], "-tssamples"))
	{
	    int samples;

	    samples = atoi (argv[++i]);
	    if (samples < 20)
	    {
		printf ("Invalid number of gates for time series recording\n");
		return -1;
	    }
	    param->samples_per_pulse_ts = samples;
	}
	else if (!strcmp (argv[i], "-tsrange"))
	{
	    const double gate_width = (double)param->clock_divfactor * (SPEED_LIGHT / 2.0) / param->clock;
	    int samples;

	    samples = (int)((atof (argv[++i]) * 1000.0 / gate_width) + 0.5);
	    if (samples < 20)
	    {
		printf ("Invalid range for time series recording\n");
		return -1;
	    }
	    param->samples_per_pulse_ts = samples;
	}
	else if (!strcmp (argv[i], "-range"))
	{
	    const double gate_width = (double)param->clock_divfactor * (SPEED_LIGHT / 2.0) / param->clock;
	    int          min_gate;
	    int          max_gate;

	    /* ------------------------ *
	     * SELECT MIN AND MAX GATES *
	     * ------------------------ */
	    min_gate = (int) (atof (argv[++i]) * 1000.0 / gate_width);
	    max_gate = (int)((atof (argv[++i]) * 1000.0 / gate_width) + 0.5);
	    if (min_gate < 2)
		min_gate = 2;

	    /*
	     * For Galileo this is currently fixed in config file
	     * so override any value entered.
	     */
	    min_gate = (param->delay_clocks / param->clock_divfactor) + 1;

	    if (max_gate <= min_gate)
		max_gate = min_gate + 1;

	    scan->min_gate = min_gate;
	    scan->max_gate = max_gate;
	    printf ("Range: %8.3f to %8.3f km\n",
		    scan->min_gate * gate_width / 1000.0,
		    scan->max_gate * gate_width / 1000.0);

	    param->delay_clocks      = (scan->min_gate - 1) * param->clock_divfactor;
	    param->samples_per_pulse = scan->max_gate - scan->min_gate;
	}
	else if (!strcmp (argv[i], "-help"))
	{
	    disp_help (argv[0]);
	    return 1;
	}
	else
	{
	    /* ------------- *
	     * PARSING ERROR *
	     * ------------- */
	    printf ("** UNKNOWN COMMAND LINE ARGUMENT '%s'!\n",argv[i]);
	    disp_help (argv[0]);
	    return -1;
	}
    }

    /* Make sure time series samples <= samples */
    if (param->samples_per_pulse_ts > param->samples_per_pulse)
    {
	param->samples_per_pulse_ts = param->samples_per_pulse;
    }

    /* -------------------------------------------------------------*
     * This is to remove the effect of Chilbolton azimuths          *
     * care may have to be taken when the radar is not on the dish  *
     * since this may cause an error in the metadata                *
     * -------------------------------------------------------------*/
    if (scan->scanType == SCAN_PPI)
    {
	scan->min_angle -= 90.0;
	scan->max_angle -= 90.0;
    }
    else
    {
	scan->scan_angle -= 90.0;
    }
    return 0;
}


/* ------------------------------------------------------------------*
 * get_config : reads radar config file                              *
 * ------------------------------------------------------------------*/
static inline void
get_config (const char *      filename,
	    RSP_ParamStruct * param,
	    URC_ScanStruct *  scan,
	    int               is_coded)
{
    char codefile[255];

    printf ("Accessing config file: %s\n", filename);

    param->frequency                  = RNC_GetConfigDouble (filename, "radar-frequency");
    param->prf                        = RNC_GetConfigDouble (filename, "prf");
    param->transmit_power             = RNC_GetConfigDouble (filename, "transmit-power");
    param->pulses_per_daq_cycle       = RNC_GetConfigDouble (filename, "pulses");
    param->samples_per_pulse          = RNC_GetConfigDouble (filename, "samples");
    param->ADC_channels               = RNC_GetConfigDouble (filename, "adc-channels");
    param->clock_divfactor            = RNC_GetConfigDouble (filename, "adc-divfactor");
    param->delay_clocks               = RNC_GetConfigDouble (filename, "adc-delayclocks");
    param->pulse_period               = RNC_GetConfigDouble (filename, "chip-length");
    param->pulses_coherently_averaged = RNC_GetConfigDouble (filename, "num-coh-avg");
    param->spectra_averaged           = RNC_GetConfigDouble (filename, "num-spec-avg");
    param->moments_averaged           = RNC_GetConfigDouble (filename, "num-moments-avg");
    param->fft_bins_interpolated      = RNC_GetConfigDouble (filename, "reject-clutter-bins");
    param->clock                      = RNC_GetConfigDouble (filename, "adc-clock");
    param->num_peaks                  = RNC_GetConfigDouble (filename, "num-peaks");
    param->antenna_diameter           = RNC_GetConfigFloat (filename, "antenna_diameter");
    param->beamwidthH                 = RNC_GetConfigFloat (filename, "beamwidthH");
    param->beamwidthV                 = RNC_GetConfigFloat (filename, "beamwidthV");
    param->height                     = RNC_GetConfigFloat (filename, "height");
    param->azimuth_offset             = RNC_GetConfigFloat (filename, "azimuth_offset");
    param->dump_spectra               = RNC_GetConfigFloat (filename, "dump_spectra");
    param->dump_spectra_rapid         = RNC_GetConfigFloat (filename, "dump_spectra_rapid");
    param->num_interleave             = RNC_GetConfigFloat (filename, "num-interleave");
    param->num_tx_pol                 = RNC_GetConfigFloat (filename, "num-tx-pol");
    param->pulse_offset               = RNC_GetConfigFloat (filename, "pulse_offset");
    param->phidp_offset               = RNC_GetConfigFloat (filename, "phidp_offset");
    param->long_pulse_mode            = (int)RNC_GetConfigFloat (filename, "long_pulse_mode");
    param->alternate_modes            = (int)RNC_GetConfigFloat (filename, "alternate_modes");
    param->mode0                      = (int)RNC_GetConfigFloat (filename, "mode0");
    param->mode1                      = (int)RNC_GetConfigFloat (filename, "mode1");
    param->nrays_mode0                = (int)RNC_GetConfigFloat (filename, "nrays_mode0");
    param->nrays_mode1                = (int)RNC_GetConfigFloat (filename, "nrays_mode1");

    /* -----------------------------------------------------------*
     * this is for when the radar is fixed pointing in the cradle *
     * please take care when the radar is tilted                  *
     * -----------------------------------------------------------*/
    scan->min_angle  = RNC_GetConfigFloat (filename, "antenna_elevation");
    scan->max_angle  = RNC_GetConfigFloat (filename, "antenna_elevation");
    scan->scan_angle = RNC_GetConfigFloat (filename, "antenna_azimuth");

    RNC_GetConfig (filename,"code-file", codefile, sizeof (codefile));

    strcpy (param->code_name, "NOT YET IMPLEMENTED\0");

    /* For non-coded pulses */
    param->code_length     = 1;
    param->number_of_codes = 1;

    // param->alternate_modes = 0;
    // param->mode0           = PM_Single_H;
    // param->mode1           = PM_Single_H;
}

/* Loads calibration information from *.cal file */
static inline void
get_cal (RSP_ParamStruct * param,
	 const char *      calfile)
{
    param->ZED_calibration_offset            = RNC_GetConfigDouble (calfile,"z-calibration");
    param->ZDR_calibration_offset            = RNC_GetConfigDouble (calfile,"zdr-calibration");
    param->LDR_calibration_offset            = RNC_GetConfigDouble (calfile,"ldr-calibration");
    param->ZED_incoherent_calibration_offset = RNC_GetConfigDouble (calfile,"z-incoherent-calibration");
    param->ZED_incoherent_noise              = RNC_GetConfigDouble (calfile,"z-incoherent-noise");
    param->range_offset                      = RNC_GetConfigDouble (calfile,"range-offset");
}

/*--------------------------------------------------------------------*
 * make_dmux_table generates the lookup table that is used to extract *
 * channels from the DMA buffer                                       *
 * IN:  channels  the number of channels                              *
 * OUT: nowt                                                          *
 *--------------------------------------------------------------------*/
static inline  void
make_dmux_table (int channels,
		 int dmux_table[8])
{
    if (channels == 4)
    {
	dmux_table[0] = 0;
	dmux_table[1] = 2;
	dmux_table[2] = 1;
	dmux_table[3] = 3;
	/*
	 * Channels 4 to 7 do not exist in the 4-channel system.
	 *
	 * MTF: They need to be allocated to a valid channel as
	 * the usage macros do not check the index returned from
	 * the dmux_table before using it.
	 */
	dmux_table[4] = 3; /* Needs to be in range 0..3 */
	dmux_table[5] = 3; /* Needs to be in range 0..3 */
	dmux_table[6] = 3; /* Needs to be in range 0..3 */
	dmux_table[7] = 3; /* Needs to be in range 0..3 */
    }
    if (channels == 8)
    {
	dmux_table[0] = 0;
	dmux_table[1] = 4;
	dmux_table[2] = 1;
	dmux_table[3] = 5;
	dmux_table[4] = 2;
	dmux_table[5] = 6;
	dmux_table[6] = 3;
	dmux_table[7] = 7;
    }
}


/* Routine to wait for start of scan */
static inline void
wait_scan_start (int                         scantype,
		 RSM_PositionMessageStruct * position_msg,
		 float                       min_angle,
		 float                       max_angle)
{
    float angle;

    switch (scantype)
    {
    case SCAN_PPI:
    case SCAN_RHI:
    case SCAN_CSP:
	break;
    default:
	return;
    }

    printf ("Waiting to get outside scan range...\n");
    do
    {
	RSM_ReadPositionMessage (position_msg);
	switch (scantype)
	{
	case SCAN_PPI: angle = position_msg->az; break;
	case SCAN_RHI: angle = position_msg->el; break;
	case SCAN_CSP: angle = position_msg->el; break;
	default:       angle = 0.0;              break;
	}
    }
    while (angle >= min_angle && angle <= max_angle &&
	   !exit_now);

    if (exit_now)
    {
	return;
    }

    printf ("Waiting to get within scan range...\n");
    printf ("min_angle: %.1f degrees  max_angle: %.1f\n",
	    min_angle, max_angle);
    do
    {
	RSM_ReadPositionMessage (position_msg);
	switch (scantype)
	{
	case SCAN_PPI: angle = position_msg->az; break;
	case SCAN_RHI: angle = position_msg->el; break;
	case SCAN_CSP: angle = position_msg->el; break;
	default:       angle = 0.0;              break;
	}
    }
    while ((angle < min_angle || angle > max_angle) &&
	   !exit_now);
}

/* Routine to wait for end of scan */
static inline bool
scanEnd_test (int                         scantype,
	      RSM_PositionMessageStruct * position_msg,
	      float                       min_angle,
	      float                       max_angle)
{
    float angle;

    switch (scantype)
    {
    case SCAN_PPI: angle = position_msg->az; break;
    case SCAN_RHI: angle = position_msg->el; break;
    case SCAN_CSP: angle = position_msg->el; break;
    case SCAN_SGL: return true;
    default:       return false;
    }

    return (angle > max_angle || angle < min_angle);
}


/*------------------------------------------------------------------------*
 * Routines for mode-switching bitmask                                    *
 * -----------------------------------------------------------------------*/
static uint16_t
dec2bcd_r (uint16_t dec)
{
    return (dec) ? ((dec2bcd_r (dec / 10) << 4) + (dec % 10)) : 0;
}

#if 0
static inline char *
ul16toBinary (uint16_t a)
{
    char bitmask[] = "0000000000000000";
    uint16_t i;

    int j = 0;
    for (i = 0x8000; i != 0; i >>= 1)
    {
        bitmask[j]= (a & i) ? '1' : '0';
        j++;
    }
    return strdup (bitmask);
}

static inline char *
ul8toBinary (uint8_t a)
{
    char bitmask[] = "00000000";
    uint8_t i;

    int j = 0;
    for (i = 0x80; i != 0; i >>= 1)
    {
        bitmask[j]= (a & i) ? '1' : '0';
        j++;
    }
    return strdup (bitmask);
}
#endif

/* Routine to calculate incoherent power */
static inline float
calc_incoherent_power (fftw_complex * in, int nfft)
{
    double power = 0.0;
    register int i;

    for (i = 0; i < nfft; i++)
    {
	/* cabs (in[i]) * cabs (in[i]) only doing it directly avoids 2*sqrt() */
        power += (fftw_real (in[i]) * fftw_real (in[i]) +
		  fftw_imag (in[i]) * fftw_imag (in[i]));
    }
    power /= nfft;
    return power;
}

/*========================= M A I N   C O D E ======================*
 *            [ See disp_help () for command-line options ]          *
 *------------------------------------------------------------------*/
int
main (int    argc,
      char * argv[])
{
#ifdef USE_HOST_EXT
    const char * host_ext = USE_HOST_EXT;
#else  /* USE_HOST_EXT */
    const char * host_ext = NULL;
#endif /* USE_HOST_EXT */

    /* DIO card */
    int fd;
    ixpio_reg_t bank_A, bank_B, bank_C, bank_0;

    int        num_pulses;
    int        amcc_fd = 0;        // file descriptor for the PCICARD
    caddr_t    dma_buffer = NULL;  // size of dma buffer
    uint16_t * dma_banks[2];      // pointers to the dma banks
    int        dma_bank = 0;
    int        proc_bank = 1;
    int        tcount;             // number of bytes to be transferred during the DMA
    uint16_t * data;
    int        count, sample;
    int        start_day = 0;
    int        nspectra;
    int        status;
    long       num_data;
    long       RetriggerDelayTime;
    float *    current_PSD;
    register   int  i, j;
    int        temp_int = 0;
    float      tmpRHO = 0.0;
    float      HH_moments[RSP_MOMENTS];
    float      HV_moments[RSP_MOMENTS];
    float      HH_noise_level;
    float      HV_noise_level;
    float      VV_moments[RSP_MOMENTS];
    float      VH_moments[RSP_MOMENTS];
    float      VV_noise_level;
    float      VH_noise_level;
    int        noisegate1, noisegate2;
    int        gate_offset;
    int        mode_gate_offset;
    time_t     system_time;
    time_t     spectra_time = 0;
    time_t     spectra_rapid_time = 0;
    time_t     temp_time_t;
    char       datestring[25];  /* MTF: 15 -> 25 to fix buffer overrun */
    float * uncoded_mean_vsq; // Used in sigma vbar calculation
    float * uncoded_mean_Zsq; // Used in sigma Zbar calculation

    /* time variables */
    struct timeval tv;
    struct tm      tm;

    PolPSDStruct * PSD;
    URC_ScanStruct scan;
    RNC_DimensionStruct dimensions;

    /* netCDF file pointer */
    int ncid;
    int ncidts             = -1;
    int spectra_ncid       = -1;
    int spectra_rapid_ncid = -1;
    int PSD_varid       [PSD_varidSize];
    int PSD_rapid_varid [RapidPSD_varidSize];
    int file_stateid = 0;
    bool scanEnd = false;
    float norm_uncoded;

    /* Time series file pointer */
    FILE * tsfid = NULL;

    RSM_PositionMessageStruct position_msg;

    /* signal */
    struct      sigaction sig_struct;

    int nm;

    /*--------------------------------------------------------*
     * The following are shortcut pointers to the elements of *
     * the obs structure                                      *
     *--------------------------------------------------------*/
    float * TX_1A;
    float * TX_2A;
    float * TX_1B;
    float * TX_2B;

    float * ZED_HC;
    float * ZED_XHC;
    float * ZED_VC;
    float * ZED_XVC;
    float * SNR_HC;
    float * SNR_XHC;
    float * SNR_VC;
    float * SNR_XVC;
    float * VEL_HC;
    float * VEL_VC;
    float * SPW_HC;
    float * SPW_VC;
    float * VEL_HC_COS, * VEL_HC_SIN;
    float * VEL_VC_COS, * VEL_VC_SIN;
    float * PHIDP_FD_COS, * PHIDP_FD_SIN;
    float * PHIDP_VD_COS, * PHIDP_VD_SIN;
    float * VEL_VD_COS, * VEL_VD_SIN;
    float * VEL_FD_COS, * VEL_FD_SIN;
    float * VEL_VD_COS_even, * VEL_VD_SIN_even;
    float * VEL_FD_COS_even, * VEL_FD_SIN_even;
    float * VEL_VD_COS_odd, * VEL_VD_SIN_odd;
    float * VEL_FD_COS_odd, * VEL_FD_SIN_odd;
    float * LDR_HC, * LDR_VC;
    float * NPC_H, * NPC_V;
    float * VEL_VD, * VEL_FD;
    float * PHIDP_FD, * PHIDP_VD;
    float * ZDR_C;
    float * PH_FD, * PV_FD, * PH_VD, * PV_VD;
    float * PH_FD_even, * PV_FD_even, * PH_VD_even, * PV_VD_even;
    float * PH0_FD_even, * PV0_FD_odd;
    float * PH_FD_odd, * PV_FD_odd, * PH_VD_odd, * PV_VD_odd;
    float * RHO_FD, * RHO_VD;
    float * RHO_FDS, * RHO_VDS;
    float * POW_H, * POW_HX, * POW_V, * POW_VX;
    float wi;     // Individual weighting value
    float * uncoded_sum_wi; // Sum of weighting values
    float tempI_odd, tempI_even, tempQ_odd, tempQ_even;
    float phidp, tempI, tempQ, tempI_vel, tempQ_vel;

    uint16_t * I_uncoded_copolar_H;
    uint16_t * Q_uncoded_copolar_H;
    uint16_t * I_uncoded_crosspolar_H;
    uint16_t * Q_uncoded_crosspolar_H;
    uint16_t * log_raw;
    uint16_t * TX1data;
    uint16_t * TX2data;
    uint16_t * V_not_H;

    int horizontal_first = 1;

    RSP_ParamStruct       param;
    RSP_ComplexType      * timeseries;
    RSP_ObservablesStruct obs;
    RSP_ObservablesStruct PSD_obs;
    RSP_ObservablesStruct PSD_RAPID_obs;
    TimeSeriesObs_t       tsobs;

    RSP_PeakStruct * HH_peaks;
    RSP_PeakStruct * HV_peaks;
    RSP_PeakStruct * VV_peaks;
    RSP_PeakStruct * VH_peaks;

    fftw_complex * in;
    fftw_complex * H_odd;
    fftw_complex * V_odd;
    fftw_complex * H_even;
    fftw_complex * V_even;
    fftw_complex * H0_even;
    fftw_complex * V0_odd;
    fftw_plan    p_uncoded;

    int      collect_spectra_now;
    int      collect_spectra_rapid_now;
    int      obtain_index;
    int      store_index;
    IQStruct IQStruct;

    disp_welcome_message ();

    /*--------------------------------*
     * Set up the signal handler      *
     *--------------------------------*/
    /* Set up the sig_struct variable */
    sig_struct.sa_handler = sig_handler;
    sigemptyset (&sig_struct.sa_mask);
    sig_struct.sa_flags = 0;

    /* Install signal handler and check for error */
    if (sigaction (SIGINT, &sig_struct, NULL) != 0)
    {
	perror ("Error installing signal handler\n");
	return 1;
    }

    /* Read radar config file */
    get_config (CONFIG_FILE, &param, &scan, 0);  // Do it for coded pulses

    /* Parse command line arguments */
    /* overwrite config with command line parameters */
    if (parseargs (argc, argv, &param, &scan, &start_day))
    {
	return 1;
    }

    /* Read calibration file */
    get_cal (&param, CAL_FILE);

    /*-----------------------------------------------------*
     * PCI DIO Card stuff - for setting radar mode       *
     *-----------------------------------------------------*/
    uint16_t pulse_offset_n100ns = (uint16_t) ((param.pulse_offset - 0.2 + 100) * 10 +0.5f); /* 100us added to set bits for correct operation */
    uint16_t pulse_offset_bcd    = dec2bcd_r (pulse_offset_n100ns);
    uint8_t pulse_offset_byte_a  = pulse_offset_bcd & 0xFF;
    uint8_t pulse_offset_byte_b  = pulse_offset_bcd >> 8;

    /* Set bits for chip length (number of 100ns units) */
    uint8_t chip_length_n100ns = (uint8_t) (param.pulse_period / 100 + 0.5f);

    uint8_t mode, mode0, mode1, wivern_mode[2];

    if (param.long_pulse_mode == 0)
    {
	/*----------------------------------------------------------*
	 * Short pulses                                             *
	 * Set mode0 and mode1 - radar will alternate between these *
	 * if param.alternate_modes==1                              *
	 *----------------------------------------------------------*/
	mode0          = (uint8_t) (param.mode0 | 0x08); // Most significant bit 1 to select short pulses
	wivern_mode[0] = (mode0 << 4) | chip_length_n100ns;

	mode1          = (uint8_t) (param.mode1 | 0x08); // Most significant bit 1 to select short pulses
	wivern_mode[1] = (mode1 << 4) | chip_length_n100ns;
    }
    else
    {
	/*--------------*
	 * Long pulses  *
	 *--------------*/
	// Need to double check effect of chip_length parameter here (CJW 20161216)
	mode0          = (uint8_t)param.mode0; // Most significant bit 0
	wivern_mode[0] = (mode0 << 4) | chip_length_n100ns;
	mode1          = (uint8_t)param.mode1; // Most significant bit 0
	wivern_mode[1] = (mode1 << 4) | chip_length_n100ns;
    }
    mode = mode0;

    /*--------------------------------------*
     * Open up the route to the serial port *
     * and apply the right settings         *
     *--------------------------------------*/
    if (positionMessageAct)
    {

	temp_int = RSM_InitialisePositionMessage (SERIALMESSAGE_PORT);
	if (temp_int != 0)
	{
	    printf ("Detected a problem with initialising the position message port\n");
	    return 1;
	}

	/* wait for valid dish position */
	do
	{
	    RSM_ReadPositionMessage (&position_msg);
	    /* MTF: Add in breakout on Ctrl-C */
	}
	while (!exit_now && position_msg.month == 0); // Check msg is valid

	/* Check msg is valid */
	if (exit_now)
	{
	    return 1;
	}

	printf ("Az: %.3f   El: %.3f\n", position_msg.az, position_msg.el);
    }

    /* Fixup param.num_tx_pol for array creation */
    param.mode0 &= 0x07;
    switch (param.mode0)
    {
    case PM_Undefined0:
        param.num_tx_pol = 1;
	break;
    case PM_Single_H:
    case PM_Single_V:
    case PM_Double_H:
    case PM_Double_V:
	param.num_tx_pol = 1;
	break;
    case PM_Single_HV:
    case PM_Double_HV_VH:
    case PM_Double_HV_HV:
	param.num_tx_pol = 2;
	break;
    }
    if (param.alternate_modes != 0)
    {
	param.mode1 &= 0x07;
	switch (param.mode1)
	{
	case PM_Undefined0:
	    break;
	case PM_Single_H:
	case PM_Single_V:
	case PM_Double_H:
	case PM_Double_V:
	    break;
	case PM_Single_HV:
	case PM_Double_HV_VH:
	case PM_Double_HV_HV:
	    param.num_tx_pol = 2;
	    break;
	}
    }
    // /* Hard code to 2 as using 1 breaks ffts */
    // param.num_tx_pol = 2;

    /*------------------------------------*
     * Initialise RSP parameter structure *
     *------------------------------------*/
    RSP_InitialiseParams (&param); // This param is used for uncoded pulses

    gate_offset = (int) (0.5+ (param.pulse_offset * 1e-6) / param.sample_period);

    printf ("Pulse offset: %f\n", param.pulse_offset);
    printf ("Sample_period: %f\n", param.sample_period * 1e9);
    printf ("Gate offset: %d\n",gate_offset);
    printf ("nfft: %d\n", param.nfft);
    printf ("npsd: %d\n", param.npsd);
    printf ("frequency: %f\n", param.frequency);
    printf ("spectra_averaged: %d\n", param.spectra_averaged);
    printf ("Max num tx pol: %d\n", param.num_tx_pol);
    printf ("PRT: %f\n", param.prt);
    printf ("Phidp offset: %f\n", param.phidp_offset);
    printf ("Mode 0: %x\n", wivern_mode[0]);
    printf ("Mode 1: %x\n", wivern_mode[1]);

    printf ("Display parameters:\n");
    RSP_DisplayParams (&param);

    RetriggerDelayTime  = param.prt * 2.0e6;

    /* Remove a suitable amount to ensure good trigger */
    if (RetriggerDelayTime % 500 < 100)
	RetriggerDelayTime -= 500;

    /* Round down to nearest 500us */
    RetriggerDelayTime -= (RetriggerDelayTime % 500);

    /* Sample extra pulses at end so that we have entire code sequence */
    num_pulses = (int) (param.pulses_per_daq_cycle);
    tcount     = num_pulses * param.spectra_averaged * param.samples_per_pulse * param.ADC_channels * sizeof (uint16_t);

    printf ("Num pulses: %d\n",num_pulses);

    // Number of data points to allocate per data stream
    num_data = param.pulses_per_daq_cycle * param.samples_per_pulse;

    // Allocate memory for coded and uncoded data streams
    I_uncoded_copolar_H             = calloc (num_data, sizeof (uint16_t));
    Q_uncoded_copolar_H             = calloc (num_data, sizeof (uint16_t));
    IQStruct.I_uncoded_copolar_H    = calloc (param.samples_per_pulse * param.nfft * param.num_tx_pol * param.spectra_averaged, sizeof (uint16_t));
    IQStruct.Q_uncoded_copolar_H    = calloc (param.samples_per_pulse * param.nfft * param.num_tx_pol * param.spectra_averaged, sizeof (uint16_t));
    I_uncoded_crosspolar_H          = calloc (num_data, sizeof (uint16_t));
    Q_uncoded_crosspolar_H          = calloc (num_data, sizeof (uint16_t));
    IQStruct.I_uncoded_crosspolar_H = calloc (param.samples_per_pulse * param.nfft * param.num_tx_pol * param.spectra_averaged, sizeof (uint16_t));
    IQStruct.Q_uncoded_crosspolar_H = calloc (param.samples_per_pulse * param.nfft * param.num_tx_pol * param.spectra_averaged, sizeof (uint16_t));

    if (I_uncoded_copolar_H == NULL || Q_uncoded_copolar_H == NULL ||
	I_uncoded_crosspolar_H == NULL || Q_uncoded_crosspolar_H == NULL ||
	IQStruct.I_uncoded_copolar_H == NULL || IQStruct.Q_uncoded_copolar_H == NULL ||
	IQStruct.I_uncoded_crosspolar_H == NULL || IQStruct.Q_uncoded_crosspolar_H == NULL)
    {
	fprintf (stderr, "Memory allocation error: %m\n");
	return 3;
    }

    log_raw = calloc (num_data, sizeof (uint16_t));
    TX1data = calloc (num_data, sizeof (uint16_t));
    TX2data = calloc (num_data, sizeof (uint16_t));
    V_not_H = calloc (num_data, sizeof (uint16_t));

    num_data      *= param.spectra_averaged;
    tsobs.ICOH     = malloc (num_data * 8 * sizeof (uint16_t));
    tsobs.QCOH     = tsobs.ICOH     + num_data;
    tsobs.ICXH     = tsobs.QCOH     + num_data;
    tsobs.QCXH     = tsobs.ICXH     + num_data;
    tsobs.TxPower1 = tsobs.QCXH     + num_data;
    tsobs.TxPower2 = tsobs.TxPower1 + num_data;
    tsobs.VnotH    = tsobs.TxPower2 + num_data;
    tsobs.RawLog   = tsobs.VnotH    + num_data;

    if (tsobs.VnotH == NULL ||
	log_raw == NULL ||
	TX1data == NULL ||
	TX2data == NULL ||
	V_not_H == NULL)
    {
	fprintf (stderr, "Memory allocation error: %m\n");
	return 3;
    }

    //printf ("FFTW Version: %s\n", fftw_version);
    //exit (100);

    HH_peaks = calloc (param.num_peaks, sizeof (RSP_PeakStruct));
    HV_peaks = calloc (param.num_peaks, sizeof (RSP_PeakStruct));
    VV_peaks = calloc (param.num_peaks, sizeof (RSP_PeakStruct));
    VH_peaks = calloc (param.num_peaks, sizeof (RSP_PeakStruct));

    if (HH_peaks == NULL || HV_peaks == NULL || VV_peaks == NULL || VH_peaks == NULL)
    {
	fprintf (stderr, "Memory allocation error: %m\n");
	return 3;
    }

    in        = fftw_malloc (sizeof (fftw_complex) * param.nfft);
    H_odd     = fftw_malloc (sizeof (fftw_complex) * (param.nfft * param.num_tx_pol + 1) >> 1);
    V_odd     = fftw_malloc (sizeof (fftw_complex) * (param.nfft * param.num_tx_pol + 1) >> 1);
    H_even    = fftw_malloc (sizeof (fftw_complex) * (param.nfft * param.num_tx_pol + 1) >> 1);
    V_even    = fftw_malloc (sizeof (fftw_complex) * (param.nfft * param.num_tx_pol + 1) >> 1);
    H0_even   = fftw_malloc (sizeof (fftw_complex) * (param.nfft * param.num_tx_pol + 1) >> 1);
    V0_odd    = fftw_malloc (sizeof (fftw_complex) * (param.nfft * param.num_tx_pol + 1) >> 1);
    p_uncoded = fftw_plan_dft_1d (param.nfft, in, in, FFTW_FORWARD, FFTW_ESTIMATE);
    if (in == NULL ||
	H_odd == NULL || V_odd == NULL || H_even == NULL || V_even == NULL ||
	H0_even == NULL || V0_odd == NULL || p_uncoded == NULL)
    {
	fprintf (stderr, "Memory allocation error: %m\n");
	return 3;
    }

    timeseries  = calloc (param.nfft, sizeof (RSP_ComplexType));
    current_PSD = calloc (param.npsd, sizeof (float));
    PSD         = calloc (param.samples_per_pulse, sizeof (PolPSDStruct));

    if (timeseries == NULL || current_PSD == NULL || PSD == NULL)
    {
	fprintf (stderr, "Memory allocation error: %m\n");
	return 3;
    }
	
    for (j = 0; j < param.samples_per_pulse; j++)
    {
	PSD[j].HH = calloc (param.npsd, sizeof (float));   // not coded
	PSD[j].HV = calloc (param.npsd, sizeof (float));   // not coded
	PSD[j].VV = calloc (param.npsd, sizeof (float));   // not coded
	PSD[j].VH = calloc (param.npsd, sizeof (float));   // not coded

	if (PSD[j].HH == NULL || PSD[j].HV == NULL ||
	    PSD[j].VV == NULL || PSD[j].VH == NULL)
	{
	    fprintf (stderr, "Memory allocation error: %m\n");
	    return 3;
	}
    }

    uncoded_mean_vsq = calloc (param.samples_per_pulse, sizeof (float));
    uncoded_mean_Zsq = calloc (param.samples_per_pulse, sizeof (float));
    uncoded_sum_wi   = calloc (param.samples_per_pulse, sizeof (float));

    if (uncoded_mean_vsq == NULL || uncoded_mean_Zsq == NULL ||
	uncoded_sum_wi == NULL)
    {
	fprintf (stderr, "Memory allocation error: %m\n");
	return 3;
    }

    VEL_HC_COS      = calloc (param.samples_per_pulse, sizeof (float));
    VEL_HC_SIN      = calloc (param.samples_per_pulse, sizeof (float));
    VEL_VC_COS      = calloc (param.samples_per_pulse, sizeof (float));
    VEL_VC_SIN      = calloc (param.samples_per_pulse, sizeof (float));
    VEL_VD_COS      = calloc (param.samples_per_pulse, sizeof (float));
    VEL_VD_SIN      = calloc (param.samples_per_pulse, sizeof (float));
    VEL_FD_COS      = calloc (param.samples_per_pulse, sizeof (float));
    VEL_FD_SIN      = calloc (param.samples_per_pulse, sizeof (float));
    VEL_VD_COS_even = calloc (param.samples_per_pulse, sizeof (float));
    VEL_VD_SIN_even = calloc (param.samples_per_pulse, sizeof (float));
    VEL_FD_COS_even = calloc (param.samples_per_pulse, sizeof (float));
    VEL_FD_SIN_even = calloc (param.samples_per_pulse, sizeof (float));
    VEL_VD_COS_odd  = calloc (param.samples_per_pulse, sizeof (float));
    VEL_VD_SIN_odd  = calloc (param.samples_per_pulse, sizeof (float));
    VEL_FD_COS_odd  = calloc (param.samples_per_pulse, sizeof (float));
    VEL_FD_SIN_odd  = calloc (param.samples_per_pulse, sizeof (float));
    PHIDP_FD_COS    = calloc (param.samples_per_pulse, sizeof (float));
    PHIDP_FD_SIN    = calloc (param.samples_per_pulse, sizeof (float));
    PHIDP_VD_COS    = calloc (param.samples_per_pulse, sizeof (float));
    PHIDP_VD_SIN    = calloc (param.samples_per_pulse, sizeof (float));
    PH_FD           = calloc (param.samples_per_pulse, sizeof (float));
    PV_FD           = calloc (param.samples_per_pulse, sizeof (float));
    PH_VD           = calloc (param.samples_per_pulse, sizeof (float));
    PV_VD           = calloc (param.samples_per_pulse, sizeof (float));
    PH_FD_even      = calloc (param.samples_per_pulse, sizeof (float));
    PV_FD_even      = calloc (param.samples_per_pulse, sizeof (float));
    PH_VD_even      = calloc (param.samples_per_pulse, sizeof (float));
    PV_VD_even      = calloc (param.samples_per_pulse, sizeof (float));
    PH_FD_odd       = calloc (param.samples_per_pulse, sizeof (float));
    PV_FD_odd       = calloc (param.samples_per_pulse, sizeof (float));
    PH_VD_odd       = calloc (param.samples_per_pulse, sizeof (float));
    PV_VD_odd       = calloc (param.samples_per_pulse, sizeof (float));
    PH0_FD_even     = calloc (param.samples_per_pulse, sizeof (float));
    PV0_FD_odd      = calloc (param.samples_per_pulse, sizeof (float));

    if (VEL_HC_COS == NULL || VEL_HC_SIN == NULL || VEL_VC_COS == NULL || VEL_VC_SIN == NULL ||
	VEL_VD_COS == NULL || VEL_VD_SIN == NULL || VEL_FD_COS == NULL || VEL_FD_SIN == NULL ||
	VEL_VD_COS_even == NULL || VEL_VD_SIN_even == NULL || VEL_FD_COS_even == NULL || VEL_FD_SIN_even == NULL ||
	VEL_VD_COS_odd == NULL || VEL_VD_SIN_odd == NULL || VEL_FD_COS_odd == NULL || VEL_FD_SIN_odd == NULL ||
	PHIDP_FD_COS == NULL || PHIDP_FD_SIN == NULL ||	PHIDP_VD_COS == NULL ||	PHIDP_VD_SIN == NULL ||
	PH_FD == NULL || PV_FD == NULL || PH_VD == NULL || PV_VD == NULL ||
	PH_FD_even == NULL || PV_FD_even == NULL || PH_VD_even == NULL || PV_VD_even == NULL ||
	PH_FD_odd == NULL || PV_FD_odd == NULL || PH_VD_odd == NULL || PV_VD_odd == NULL ||
	PH0_FD_even == NULL || PV0_FD_odd == NULL)
    {
	fprintf (stderr, "Memory allocation error: %m\n");
	return 3;
    }

    norm_uncoded = 1.0 / param.Wss;

    /*----------------------------*
     * Initialise RSP Observables *
     *----------------------------*/
    RSP_ObsInit (&obs);
    RSP_ObsInit (&PSD_obs);
    /* Last argument determines whether the parameter will be recorded or not */
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"TX_1A");
    TX_1A    = RSP_ObsNew (&obs, "TX_1A", 1, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"TX_2A");
    TX_2A    = RSP_ObsNew (&obs, "TX_2A", 1, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"TX_1B");
    TX_1B    = RSP_ObsNew (&obs, "TX_1B", 1, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"TX_2B");
    TX_2B    = RSP_ObsNew (&obs, "TX_2B", 1, temp_int);

    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"ZED_HC");
    ZED_HC   = RSP_ObsNew (&obs, "ZED_HC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"SNR_HC");
    SNR_HC   = RSP_ObsNew (&obs, "SNR_HC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"POW_H");
    POW_H    = RSP_ObsNew (&obs, "POW_H",    param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"POW_HX");
    POW_HX   = RSP_ObsNew (&obs, "POW_HX",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"POW_V");
    POW_V    = RSP_ObsNew (&obs, "POW_V",    param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"POW_VX");
    POW_VX   = RSP_ObsNew (&obs, "POW_VX",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"ZED_VC");
    ZED_VC   = RSP_ObsNew (&obs, "ZED_VC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"SNR_VC");
    SNR_VC   = RSP_ObsNew (&obs, "SNR_VC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"VEL_HC");
    VEL_HC   = RSP_ObsNew (&obs, "VEL_HC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"VEL_VC");
    VEL_VC   = RSP_ObsNew (&obs, "VEL_VC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"SPW_HC");
    SPW_HC   = RSP_ObsNew (&obs, "SPW_HC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"SPW_VC");
    SPW_VC   = RSP_ObsNew (&obs, "SPW_VC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"SNR_XHC");
    SNR_XHC  = RSP_ObsNew (&obs, "SNR_XHC",  param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"ZED_XHC");
    ZED_XHC  = RSP_ObsNew (&obs, "ZED_XHC",  param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"SNR_XVC");
    SNR_XVC  = RSP_ObsNew (&obs, "SNR_XVC",  param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"ZED_XVC");
    ZED_XVC  = RSP_ObsNew (&obs, "ZED_XVC",  param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"LDR_HC");
    LDR_HC   = RSP_ObsNew (&obs, "LDR_HC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"LDR_VC");
    LDR_VC   = RSP_ObsNew (&obs, "LDR_VC",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"NPC_H");
    NPC_H    = RSP_ObsNew (&obs, "NPC_H",    param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"NPC_V");
    NPC_V    = RSP_ObsNew (&obs, "NPC_V",    param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"VEL_VD");
    VEL_VD   = RSP_ObsNew (&obs, "VEL_VD",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"VEL_FD");
    VEL_FD   = RSP_ObsNew (&obs, "VEL_FD",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"PHIDP_FD");
    PHIDP_FD = RSP_ObsNew (&obs, "PHIDP_FD", param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"PHIDP_VD");
    PHIDP_VD = RSP_ObsNew (&obs, "PHIDP_VD", param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"ZDR_C");
    ZDR_C    = RSP_ObsNew (&obs, "ZDR_C",    param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"RHO_FD");
    RHO_FD   = RSP_ObsNew (&obs, "RHO_FD",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"RHO_VD");
    RHO_VD   = RSP_ObsNew (&obs, "RHO_VD",   param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"RHO_FDS");
    RHO_FDS  = RSP_ObsNew (&obs, "RHO_FDS",  param.samples_per_pulse, temp_int);
    temp_int = (int)RNC_GetConfigDouble (CONFIG_FILE,"RHO_VDS");
    RHO_VDS  = RSP_ObsNew (&obs, "RHO_VDS",  param.samples_per_pulse, temp_int);

    if (TX_1A    == NULL || TX_2A    == NULL || TX_1B    == NULL || TX_2B    == NULL ||
	ZED_HC   == NULL || SNR_HC   == NULL || POW_H    == NULL || POW_HX   == NULL ||
	POW_V    == NULL || POW_VX   == NULL || ZED_VC   == NULL || SNR_VC   == NULL ||
	VEL_HC   == NULL || VEL_VC   == NULL || SPW_HC   == NULL || SNR_XHC  == NULL ||
	ZED_XHC  == NULL || SNR_XVC  == NULL || ZED_XVC  == NULL || LDR_HC   == NULL ||
	LDR_VC   == NULL || NPC_H    == NULL || NPC_V    == NULL || VEL_VD   == NULL ||
	VEL_FD   == NULL || PHIDP_FD == NULL || PHIDP_VD == NULL || ZDR_C    == NULL ||
	RHO_FD   == NULL || RHO_VD   == NULL || RHO_FDS  == NULL || RHO_VDS  == NULL ||
        SPW_VC   == NULL)
    {
	fprintf (stderr, "Memory allocation error: %m\n");
	return 3;
    }

    printf ("Recording observables:");
    for (i = 0; i < obs.n_obs; i++)
    {
	if (obs.record_observable[i] == 1)
	{
	    printf (" %s",obs.name[i]);
	}
    }
    printf ("\n");

#ifdef HAVE_DISLIN
    /* check to see if the real time spectra display option has been selected */
    if (param.real_time_spectra_display == 1)
    {

	metafl ("xwin");
	disini ();
	pagera ();
	hwfont ();

	titlin ("radar-galileo spectra",2);

	name ("velocity", "x");
	name ("height",   "y");
	name ("power",    "z");

	intax ();
	autres (param.npsd,param.samples_per_pulse);
	axspos (300,1850);
	ax3len (2200,1400,1400);

	graf3 (0, param.npsd, 0, 20,
	       0, param.samples_per_pulse, 0, 20,
	       30, 100, 30,10);

    }
#endif /* HAVE_DISLIN */

    /*-----------------------------------------*
     * Set initial mode                        *
     *-----------------------------------------*/

#ifndef NO_DIO
    /* Open PCI DIO card */
    fd = open (device, O_RDWR);
    if (fd < 0)
    {
	perror ("PCI DIO card not present\n");
	printf ("PCI DIO card not present %s: %m\n", device);
	return 2;
    }
#endif /* NO_DIO */

    /* Set config port identifier */
    bank_0.id = IXPIO_PC;

    /* Set all ports for output */
    bank_0.value = 0x07;

#ifndef NO_DIO
    if (ioctl (fd, IXPIO_WRITE_REG, &bank_0))
    {
	perror ("Can't set write mode for bank_0");
	printf ("Can't set write mode for bank_0\n");
	close (fd);
	return 2;
    }
#endif /* NO_DIO */

    /* Set inital mode to mode0 */
    int new_mode = 0;

    /* Set port identifiers for output */
    bank_A.id = IXPIO_P0;
    bank_B.id = IXPIO_P1;
    bank_C.id = IXPIO_P2;

    /* Set bank structure values for writing */
    bank_A.value = pulse_offset_byte_a;
    bank_B.value = pulse_offset_byte_b;
    bank_C.value = wivern_mode[new_mode];

    {
	mode             = (new_mode == 1) ? param.mode1 : param.mode0;
	mode_gate_offset = (mode < PM_Double_H) ? param.samples_per_pulse : gate_offset;

	/* Fixup param.num_tx_pol for mode */
	switch (mode & 0x07)
	{
	case PM_Undefined0:
	    param.num_tx_pol = 1;
	    break;
	case PM_Single_H:
	case PM_Single_V:
	case PM_Double_H:
	case PM_Double_V:
	    param.num_tx_pol = 1;
	    break;
	case PM_Single_HV:
	case PM_Double_HV_VH:
	case PM_Double_HV_HV:
	    param.num_tx_pol = 2;
	    break;
	}
	// /* Hard code to 2 as using 1 breaks ffts */
	// param.num_tx_pol = 2;
    }

#ifndef NO_DIO
    /* Write out values to ports */
    if (ioctl (fd, IXPIO_WRITE_REG, &bank_A))
    {
	perror ("Can't write bank_A");
	printf ("Can't write bank_A value 0x%x\n", bank_A.value);
    }
    else
    {
	printf ("Writing 0x%x to bank_A\n", bank_A.value);
    }

    if (ioctl (fd, IXPIO_WRITE_REG, &bank_B))
    {
	perror ("Can't write bank_B");
	printf ("Can't write bank_B value 0x%x\n", bank_B.value);
    }
    else
    {
	printf ("Writing 0x%x to bank_B\n", bank_B.value);
    }

    if (ioctl (fd, IXPIO_WRITE_REG, &bank_C))
    {
	perror ("Can't write bank_C");
	printf ("Can't write bank_C value 0x%x\n", bank_C.value);
    }
    else
    {
	printf ("Writing 0x%x to bank_C\n", bank_C.value);
    }
#endif /* NO_DIO */

    new_mode = -1;

    /*-------------------------------------------*
     * Set up the data acquisition               *
     *-------------------------------------------*/
    printf ("** Initialising ISACTRL...\n");
    RDQ_InitialiseISACTRL (num_pulses * param.spectra_averaged, param.samples_per_pulse,
			   param.clock_divfactor, param.delay_clocks);

    printf ("** Initialising PCICARD...\n");
    amcc_fd = RDQ_InitialisePCICARD_New (&dma_buffer, DMA_BUFFER_SIZE);

    /* Initialise pointers to DMA banks */
    dma_banks[ 0 ] = (uint16_t *) dma_buffer;
    dma_banks[ 1 ] = (uint16_t *) (dma_buffer + (DMA_BUFFER_SIZE >> 1));

    make_dmux_table (param.ADC_channels, dmux_table);

    printf ("** Starting acquisition...\n");

    /* load in current dish_time */
    if (positionMessageAct)
    {
	RSM_ReadPositionMessage (&position_msg);

	obs.azimuth          = position_msg.az;
	obs.elevation        = position_msg.el;
	obs.dish_year        = position_msg.year;
	obs.dish_month       = position_msg.month;
	obs.dish_day         = position_msg.day;
	obs.dish_hour        = position_msg.hour;
	obs.dish_minute      = position_msg.min;
	obs.dish_second      = position_msg.sec;
	obs.dish_centisecond = position_msg.centi_sec;
    }
    else
    {
	struct tm tm;
	struct timeval tv;

	gettimeofday (&tv, NULL);
	gmtime_r (&tv.tv_sec, &tm);
	obs.dish_year        = tm.tm_year + 1900;
	obs.dish_month       = tm.tm_mon + 1;
	obs.dish_day         = tm.tm_mday;
	obs.dish_hour        = tm.tm_hour;
	obs.dish_minute      = tm.tm_min;
	obs.dish_second      = tm.tm_sec;
	obs.dish_centisecond = tv.tv_usec / 10000U;
	obs.azimuth     = scan.scan_angle;
	obs.elevation   = scan.min_angle;
    }

    /* setup the netCDF file */
    ncid = RNC_OpenNetcdfFile (GetRadarName (GALILEO),
			       GetSpectraName (GALILEO),
			       scan.date, host_ext,
			       GetScanTypeName (scan.scanType),
			       GetSpectraExtension (GALILEO), "raw");
    printf ("Check 2\n");
    RNC_SetupDimensions (ncid, &param, &dimensions);
    printf ("Check 3\n");
    RNC_SetupGlobalAttributes (ncid, GALILEO, &scan, &param, argc, argv);
    file_stateid = RNC_SetupFile_State (ncid);
    RNC_SetupStaticVariables (ncid, GALILEO, &param);
    printf ("Check 4\n");
    RNC_SetupRange (ncid, &param, &dimensions);
    printf ("Check 5\n");
    RNC_SetupDynamicVariables (ncid, GALILEO, &scan, &param, &dimensions, &obs);

    /* change the mode of netCDF from define to data */
    status = nc_enddef (ncid);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    /* Set up spectral dump file */
    if (param.dump_spectra != 0)
    {
	spectra_ncid = RNC_OpenNetcdfFile
	    (GetRadarName (GALILEO_SPECTRA), GetSpectraName (GALILEO_SPECTRA),
	     scan.date, host_ext, GetScanTypeName (scan.scanType),
	     GetSpectraExtension (GALILEO_SPECTRA), "raw");
	RNC_SetupDimensions (spectra_ncid, &param, &dimensions);
	RNC_SetupGlobalAttributes (spectra_ncid, GALILEO, &scan, &param, argc, argv);
	RNC_SetupStaticVariables (spectra_ncid, GALILEO, &param);
	RNC_SetupRange (spectra_ncid, &param, &dimensions);
	RNC_SetupDynamicVariables (spectra_ncid, GALILEO_SPECTRA, &scan, &param, &dimensions, &PSD_obs);
	RNC_SetupLogPSDVariables (spectra_ncid, GALILEO_SPECTRA, &param, &dimensions, PSD_varid);

	/* change the mode of netCDF from define to data */
	status = nc_enddef (spectra_ncid);
	if (status != NC_NOERR) check_netcdf_handle_error (status);

	/* set the time at which a spectra will be dumped */
	time (&temp_time_t);
	spectra_time = param.dump_spectra * (floorl (temp_time_t / param.dump_spectra) + 1.0);
    }

    if (param.dump_spectra_rapid != 0)
    {
	/* make sure bin_ray_number is 0 */
	PSD_RAPID_obs.bin_ray_number = 0;
	PSD_RAPID_obs.ray_number = 0;

	spectra_rapid_ncid = RNC_OpenNetcdfFile
	    (GetRadarName (GALILEO_SPECTRA_RAPID),
	     GetSpectraName (GALILEO_SPECTRA_RAPID),
	     scan.date, host_ext, GetScanTypeName (scan.scanType),
	     GetSpectraExtension (GALILEO_SPECTRA_RAPID), "raw");
	RNC_SetupRapidLogPSDDimensions (spectra_rapid_ncid, GALILEO_SPECTRA_RAPID, &param, &dimensions);
	RNC_SetupGlobalAttributes (spectra_rapid_ncid, GALILEO, &scan, &param, argc, argv);
	RNC_SetupStaticVariables (spectra_rapid_ncid, GALILEO, &param);
	RNC_SetupRange (spectra_rapid_ncid, &param, &dimensions);
	RNC_SetupDynamicVariables (spectra_rapid_ncid, GALILEO_SPECTRA_RAPID, &scan, &param, &dimensions, &PSD_RAPID_obs);
	RNC_SetupLogPSDVariables (spectra_rapid_ncid, GALILEO_SPECTRA_RAPID, &param, &dimensions, PSD_rapid_varid);

	/* change the mode of netCDF from define to data */
	status = nc_enddef (spectra_rapid_ncid);
	if (status != NC_NOERR) check_netcdf_handle_error (status);

	/* set the time at which a spectra will be dumped */
	time (&temp_time_t);
	spectra_rapid_time = param.dump_spectra_rapid * (floorl (temp_time_t / param.dump_spectra_rapid) + 1.0);
    }

    if (tsdump)
    {
	if (TextTimeSeries)
	{
	    /* Setup the time-series dump file */
	    tsfid = RTS_OpenTSFile (GetRadarName (GALILEO), scan.date, host_ext,
				    GetScanTypeName (scan.scanType));

	    if (tsfid == NULL)
	    {
		tsdump = false;
		printf ("**** Can't open time series file: %m ****\n");
		printf ("**** Time series recording off ****\n");
	    }
	    else
	    {
		fprintf (tsfid, "npulse: %d\n", num_pulses * param.spectra_averaged);
		fprintf (tsfid, "nsample: %d\n", param.samples_per_pulse_ts);
		fprintf (tsfid, "divfactor: %d\n", param.clock_divfactor);
		fprintf (tsfid, "delayclocks: %d\n", param.delay_clocks);
		fprintf (tsfid, "ADC_channels: %d\n", param.ADC_channels);
	    }
	}
	else
	{
	    /* NetCDF Time series */
	    RNC_DimensionStruct dimensionsts;

	    memset (&dimensionsts, 0, sizeof (dimensionsts));
	    ncidts = RNC_OpenNetcdfFile (GetRadarName (GALILEO), "ts",
					 scan.date, host_ext,
					 GetScanTypeName (scan.scanType),
					 "", "ts");
	    RNC_SetupDimensions (ncidts, &param, &dimensionsts);
	    RNC_SetupGlobalAttributes (ncidts, GALILEO, &scan, &param, argc, argv);
	    RNC_SetupStaticVariables (ncidts, GALILEO, &param);
	    RNC_SetupRange (ncidts, &param, &dimensionsts);
	    SetupTimeSeriesVariables (&tsobs, ncidts, &param, &scan, &dimensionsts, &obs);

	    status = nc_enddef (ncidts);
	    if (status != NC_NOERR) check_netcdf_handle_error (status);
	}
    }

    /*---------------------*
     * Wait for scan start *
     *---------------------*/
    if (positionMessageAct)
    {
	wait_scan_start (scan.scanType, &position_msg,
			 scan.min_angle, scan.max_angle);
	if (exit_now)
	    goto exit_endacquisition;
    }

    RDQ_StartAcquisition (amcc_fd, dma_bank,
			  (short *)(dma_banks[dma_bank]), tcount);

    int ray_count = 0;
    int remainder = -1;

    /*-----------------------------------------*
     * THIS IS THE START OF THE OUTER RAY LOOP *
     *-----------------------------------------*/
    while (!scanEnd && !exit_now)
    {
	printf ("ray number: %d\n", obs.ray_number);
	printf ("\n<< PRESS CTRL-C TO EXIT >>\n");

	ray_count++;

	/* Initialise observables to zero (Needed for moments averaging) */
	for (i = 0; i < obs.n_obs; i++)
	{
	    printf ("Initialising %s\n", obs.name[i]);
	    for (j = 0; j < obs.n_elements [i]; j++)
	    {
		obs.data[i][j] = 0.0f;
	    }
	}

	for (j = 0; j < param.samples_per_pulse; j++)
	{
	    uncoded_mean_vsq[j] = 0.0f;
	    uncoded_mean_Zsq[j] = 0.0f;
	    uncoded_sum_wi[j]   = 0.0f;
	    VEL_HC_COS[j]       = 0.0f;
	    VEL_HC_SIN[j]       = 0.0f;
	    VEL_VC_COS[j]       = 0.0f;
	    VEL_VC_SIN[j]       = 0.0f;
	    VEL_FD_COS[j]       = 0.0f;
	    VEL_FD_SIN[j]       = 0.0f;
	    VEL_VD_COS[j]       = 0.0f;
	    VEL_VD_SIN[j]       = 0.0f;
	    VEL_FD_COS_even[j]  = 0.0f;
	    VEL_FD_SIN_even[j]  = 0.0f;
	    VEL_VD_COS_even[j]  = 0.0f;
	    VEL_VD_SIN_even[j]  = 0.0f;
	    VEL_FD_COS_odd[j]   = 0.0f;
	    VEL_FD_SIN_odd[j]   = 0.0f;
	    VEL_VD_COS_odd[j]   = 0.0f;
	    VEL_VD_SIN_odd[j]   = 0.0f;
	    PHIDP_FD_COS[j]     = 0.0f;
	    PHIDP_FD_SIN[j]     = 0.0f;
	    PHIDP_VD_COS[j]     = 0.0f;
	    PHIDP_VD_SIN[j]     = 0.0f;
	    PH_FD[j]            = 0.0f;
	    PV_FD[j]            = 0.0f;
	    PH_VD[j]            = 0.0f;
	    PV_VD[j]            = 0.0f;
	    PH_FD_even[j]       = 0.0f;
	    PV_FD_even[j]       = 0.0f;
	    PH_VD_even[j]       = 0.0f;
	    PV_VD_even[j]       = 0.0f;
	    PH_FD_odd[j]        = 0.0f;
	    PV_FD_odd[j]        = 0.0f;
	    PH_VD_odd[j]        = 0.0f;
	    PV_VD_odd[j]        = 0.0f;
	    PH0_FD_even[j]      = 0.0f;
	    PV0_FD_odd[j]       = 0.0f;
	}

	printf ("Done initialising variables...\n");

	/* This step should mean we discard data in proc bank at time of mode change */
	if (new_mode >= 0)
	{
	    /* Wait for acquisition to complete before setting new mode */
	    status = RDQ_WaitForAcquisitionToComplete (amcc_fd);
	    if (status != 0)
		printf ("There was a problem in WaitForAcquisitionToComplete\n");

	    /* Swap around the areas used for storing datq and processing from */
	    dma_bank  = 1 - dma_bank;
	    proc_bank = 1 - proc_bank	;

	    /* Set new mode */
	    bank_C.id    = IXPIO_P2;
	    bank_C.value = wivern_mode[new_mode];

	    {
		mode             = (new_mode == 1) ? param.mode1 : param.mode0;
		mode_gate_offset = (mode < PM_Double_H) ? param.samples_per_pulse : gate_offset;

		/* Fixup param.num_tx_pol for mode */
		switch (mode & 0x07)
		{
		case PM_Undefined0:
		    param.num_tx_pol = 1;
		    break;
		case PM_Single_H:
		case PM_Single_V:
		case PM_Double_H:
		case PM_Double_V:
		    param.num_tx_pol = 1;
		    break;
		case PM_Single_HV:
		case PM_Double_HV_VH:
		case PM_Double_HV_HV:
		    param.num_tx_pol = 2;
		    break;
		}
		// /* Hard code to 2 as using 1 breaks ffts */
		// param.num_tx_pol = 2;
	    }

#ifndef NO_DIO
	    if (ioctl (fd, IXPIO_WRITE_REG, &bank_C))
	    {
		perror ("Can't write bank_C");
		printf ("Can't write bank_C value 0x%x\n", bank_C.value);
	    }
	    else
	    {
		printf ("Writing 0x%x to bank_C\n", bank_C.value);
	    }
#endif /* NO_DIO */
	    new_mode = -1;

	    /*---------------------------------------------------------------------*
	     * Wait untill just before next H pulse to prevent HV timeout.         *
	     *---------------------------------------------------------------------*/
	    usleep (RetriggerDelayTime);

	    data = dma_banks[proc_bank];
	    RDQ_StartAcquisition (amcc_fd, dma_bank,
				  (short *)(dma_banks[dma_bank]), tcount);
	}

        /*----------------------------------------------*
	 * LOOP THROUGH MOMENTS AVERAGING FROM HERE...  *
	 *----------------------------------------------*/
	for (nm = 0; nm < param.moments_averaged; nm++)
	{
	    int idx;

	    for (j = 0; j < param.samples_per_pulse; j++)
	    {
		register int bin_no;
		/* Initialise spectra to zero */
		for (bin_no = 0; bin_no < param.npsd; bin_no++)
		{
		    PSD[j].HH[bin_no] = 0.0f;
		    PSD[j].HV[bin_no] = 0.0f;
		    PSD[j].VV[bin_no] = 0.0f;
		    PSD[j].VH[bin_no] = 0.0f;
		}
	    }

	    /* Wait for data acquisition to complete */
	    status = RDQ_WaitForAcquisitionToComplete (amcc_fd);
	    if (status != 0)
		printf ("There was a problem in WaitForAcquisitionToComplete\n");

	    /* Swap around the areas used for storing daq and processing from */
	    dma_bank  = 1 - dma_bank;
	    proc_bank = 1 - proc_bank;

	    /*---------------------------------------------------------------------*
	     * Wait untill just before next H pulse to prevent HV timeout.         *
	     *---------------------------------------------------------------------*/
	    usleep (RetriggerDelayTime);

	    data = dma_banks[proc_bank];
	    RDQ_StartAcquisition (amcc_fd, dma_bank,
				  (short *)(dma_banks[dma_bank]), tcount);

	    /* get timeofday */
	    gettimeofday (&tv, NULL);
	    gmtime_r (&tv.tv_sec, &tm);
	    obs.year        = tm.tm_year + 1900;
	    obs.month       = tm.tm_mon  + 1;
	    obs.day         = tm.tm_mday;
	    obs.hour        = tm.tm_hour;
	    obs.minute      = tm.tm_min;
	    obs.second      = tm.tm_sec;
	    obs.centisecond = (int)tv.tv_usec/10000;
	    sprintf (datestring,"%04d/%02d/%02d %02d:%02d:%02d.%02d",
		     obs.year, obs.month,  obs.day,
		     obs.hour, obs.minute, obs.second, obs.centisecond);
	    printf ("Date time: %s\n",datestring);

	    /* obtain dish time */
	    if (positionMessageAct)
	    {
		RSM_ReadPositionMessage (&position_msg);
		obs.azimuth          = position_msg.az;
		obs.elevation        = position_msg.el;
		obs.dish_year        = position_msg.year;
		obs.dish_month       = position_msg.month;
		obs.dish_day         = position_msg.day;
		obs.dish_hour        = position_msg.hour;
		obs.dish_minute      = position_msg.min;
		obs.dish_second      = position_msg.sec;
		obs.dish_centisecond = position_msg.centi_sec;
	    }
	    else
	    {
		struct tm tm;
		struct timeval tv;

		gettimeofday (&tv, NULL);
		gmtime_r (&tv.tv_sec, &tm);
		obs.dish_year        = tm.tm_year + 1900;
		obs.dish_month       = tm.tm_mon + 1;
		obs.dish_day         = tm.tm_mday;
		obs.dish_hour        = tm.tm_hour;
		obs.dish_minute      = tm.tm_min;
		obs.dish_second      = tm.tm_sec;
		obs.dish_centisecond = tv.tv_usec / 10000U;

		obs.azimuth     = scan.scan_angle;
		obs.elevation   = scan.min_angle;
	    }

	    PSD_obs.elevation       = obs.elevation;
	    PSD_obs.azimuth         = obs.azimuth;
	    PSD_RAPID_obs.elevation = obs.elevation;
	    PSD_RAPID_obs.azimuth   = obs.azimuth;

	    if (tsfid != NULL)
	    {
		/* time-series ray header */
		fprintf (tsfid, "Ray_number: %d, %d\n", obs.ray_number, nm);
		fprintf (tsfid, "Date_time: %s\n", datestring);
		fprintf (tsfid, "Az: %7.2f, El: %7.2f\n",
			 obs.azimuth, obs.elevation);
	    }

	    /*---------------------------------*
	     * Loop through spectral averaging *
	     *---------------------------------*/
	    /* lets decide if we need to dump some spectra */
	    collect_spectra_rapid_now = 0;
	    collect_spectra_now = 0;
	    system_time = time (NULL);
	    if (param.dump_spectra_rapid != 0)
	    {
		if (spectra_rapid_time <= system_time)
		{
		    collect_spectra_rapid_now = 1;
		}
	    }
	    /* write out spectra to netCDF if required */
	    if (param.dump_spectra != 0)
	    {
		if (spectra_time <= system_time)
		{
		    collect_spectra_now = 1;
		}
	    }

	    for (idx = nspectra = 0; nspectra < param.spectra_averaged; nspectra++)
	    {
		/*----------------------------------------------------------------*
		 * Extract data from DMA memory                                   *
		 *----------------------------------------------------------------*/
		for (i = 0; i < num_pulses; i++)
		{
		    register int count_reg;

		    for (j = 0; j < param.samples_per_pulse; j++)
		    {
			count_reg = (i * param.samples_per_pulse) + j;
			if (swap_iq_channels)
			{
			    tsobs.ICOH[idx]   = I_uncoded_copolar_H[count_reg]    = GET_CHANNEL (data, SWAP_CHAN_Ic);
			    tsobs.QCOH[idx]   = Q_uncoded_copolar_H[count_reg]    = GET_CHANNEL (data, SWAP_CHAN_Qc);
			    tsobs.ICXH[idx]   = I_uncoded_crosspolar_H[count_reg] = GET_CHANNEL (data, SWAP_CHAN_Ix);
			    tsobs.QCXH[idx]   = Q_uncoded_crosspolar_H[count_reg] = GET_CHANNEL (data, SWAP_CHAN_Qx);
			}
			else
			{
			    tsobs.ICOH[idx]   = I_uncoded_copolar_H[count_reg]    = GET_CHANNEL (data, CHAN_Ic);
			    tsobs.QCOH[idx]   = Q_uncoded_copolar_H[count_reg]    = GET_CHANNEL (data, CHAN_Qc);
			    tsobs.ICXH[idx]   = I_uncoded_crosspolar_H[count_reg] = GET_CHANNEL (data, CHAN_Ix);
			    tsobs.QCXH[idx]   = Q_uncoded_crosspolar_H[count_reg] = GET_CHANNEL (data, CHAN_Qx);
			}

			tsobs.TxPower1[idx] = TX1data[count_reg] = GET_CHANNEL (data, CHAN_T1);
			tsobs.TxPower2[idx] = TX2data[count_reg] = GET_CHANNEL (data, CHAN_T2);
			tsobs.VnotH[idx]    = V_not_H[count_reg] = GET_CHANNEL (data, CHAN_V_not_H);
			tsobs.RawLog[idx]   = log_raw[count_reg] = GET_CHANNEL (data, CHAN_INC);

			INC_POINTER (data, param.ADC_channels);
			idx++;

			if (tsfid != NULL && (j < param.samples_per_pulse_ts))
			{
			    /* time-series dump */
			    fprintf (tsfid, "%d %d %hu %hu %hu %hu %hu %hu %hu\n",
				     i + (nspectra * num_pulses), j,
				     I_uncoded_copolar_H[count_reg],
				     Q_uncoded_copolar_H[count_reg],
				     I_uncoded_crosspolar_H[count_reg],
				     Q_uncoded_crosspolar_H[count_reg],
				     TX1data[count_reg],
				     TX2data[count_reg],
				     V_not_H[count_reg]);
			}
		    }
		}

		// Create artificial IQ data to test code
		// art_vel   = -2.0;
		// art_phidp = 0.0 * PI / 180.0;
		//
		// for (i = 0; i < num_pulses; i++)
		// {
		//     register int count_reg;
		//     for (j = 0; j < param.samples_per_pulse; j++)
		//     {
		// 	art_time  = ((float)i * param.prt / 2.0) + ((float)j * param.sample_period);
		// 	art_angle = (4.0 * PI * art_vel * art_time * param.frequency) / 0.299792458;
		// 	count_reg = (i * param.samples_per_pulse) + j;
		// 	if (j < 200)
		// 	{
		// 	    I_uncoded_copolar_H   [count_reg] = 1000 + (int) (100 * cos (art_angle));
		// 	    Q_uncoded_copolar_H   [count_reg] = 1000 + (int) (100 * sin (art_angle));
		// 	    I_uncoded_crosspolar_H[count_reg] = 1000 + (int) (100 * cos (art_angle + art_phidp));
		// 	    Q_uncoded_crosspolar_H[count_reg] = 1000 + (int) (100 * sin (art_angle + art_phidp));
		// 	}
		// 	else
		// 	{
		// 	    I_uncoded_copolar_H   [count_reg] = 1000 ;
		// 	    Q_uncoded_copolar_H   [count_reg] = 1000 ;
		// 	    I_uncoded_crosspolar_H[count_reg] = 1000 ;
		// 	    Q_uncoded_crosspolar_H[count_reg] = 1000 ;
		// 	}
		//     }
		// }
		// End of artificial IQ section

		/* look to see if we are going to dump spectra on the end of this run */
		if (collect_spectra_now == 1)
		{
		    /*-------------------------------------*
		     * Store I and Q for each pulse        *
		     * nspectra defines the spectra number *
		     * lets do the code first              *
		     *-------------------------------------*/
		    for (obtain_index = i = 0; i < param.nfft * param.num_tx_pol; i++)
		    {
			store_index  = (i * param.samples_per_pulse) + (nspectra * param.samples_per_pulse * param.nfft * param.num_tx_pol);
			for (j = 0; j < param.samples_per_pulse; j++)
			{
			    IQStruct.I_uncoded_copolar_H   [store_index] = I_uncoded_copolar_H   [obtain_index];
			    IQStruct.Q_uncoded_copolar_H   [store_index] = Q_uncoded_copolar_H   [obtain_index];
			    IQStruct.I_uncoded_crosspolar_H[store_index] = I_uncoded_crosspolar_H[obtain_index];
			    IQStruct.Q_uncoded_crosspolar_H[store_index] = Q_uncoded_crosspolar_H[obtain_index];
			    store_index++;
			    obtain_index++;
			}
		    }
		}

		if (!nspectra)
		{
		    /* first of all let us work out if a H or V went out first */
		    if (debug)
		    {
			for (j = 0; j < 20; j++)
			{
			    printf ("V_not_H = %d \n",V_not_H[j]);
			}
		    }

		    if (V_not_H[V_not_H_SAMPLE] < 2048)
		    {
			/* this means first pulse is horizontal */
			horizontal_first = 1;
			printf ("The first pulse is horizontal\n");
		    }
		    else
		    {
			/* this means first pulse is vertical */
			horizontal_first = 0;
			printf ("The first pulse is vertical\n");
		    }
		}

		// Average the incoherent channel (uncoded only)
		// for (i = 0; i < param.samples_per_pulse; i++)
		// {
		//     for (j = 0; j < param.pulses_per_daq_cycle; j++)
		//     {
		//	ZED_H[i] += (float)log_raw[ (j * param.samples_per_pulse) + i ];
		//     }
		// }

		/*--------------------------------------------------------------------------------*
		 * Transmit power signals (before and after radome) - A-D values converted to mV  *
		 * Preliminary CJW code                                                           *
		 * Average over samples from istart up to iend-1                                  *
		 *--------------------------------------------------------------------------------*/
		int istart = PowerAvStartSample;
		int iend   = PowerAvEndSample; // param.samples_per_pulse - 14;
                int nsamp;

		/* Ensure averaging is on first pulse only when in dual pulse modes */
		if (iend > mode_gate_offset)
		{
		    iend = mode_gate_offset;
		}
		nsamp = iend - istart;
		for (i = istart; i < iend; i++)
		{
		  for (j = 0; j < param.pulses_per_daq_cycle/2; j++)
		  {
			if (horizontal_first == 0)
			{
			    *TX_1A += (float)TX1data[ ( 2*j    * param.samples_per_pulse) + i ];
			    *TX_1B += (float)TX1data[ ((2*j+1) * param.samples_per_pulse) + i ];
			    *TX_2A += (float)TX2data[ ( 2*j    * param.samples_per_pulse) + i ];
			    *TX_2B += (float)TX2data[ ((2*j+1) * param.samples_per_pulse) + i ];
			}
			else
			{
			    *TX_1B += (float)TX1data[ ( 2*j    * param.samples_per_pulse) + i ];
			    *TX_1A += (float)TX1data[ ((2*j+1) * param.samples_per_pulse) + i ];
			    *TX_2B += (float)TX2data[ ( 2*j    * param.samples_per_pulse) + i ];
			    *TX_2A += (float)TX2data[ ((2*j+1) * param.samples_per_pulse) + i ];
			}
		    }
		}

		//*TX_1A /= (float) (param.samples_per_pulse*param.pulses_per_daq_cycle);
		//*TX_2A /= (float) (param.samples_per_pulse*param.pulses_per_daq_cycle);
		//*TX_1B /= (float) (param.samples_per_pulse*param.pulses_per_daq_cycle);
		//*TX_2B /= (float) (param.samples_per_pulse*param.pulses_per_daq_cycle);

		*TX_1A /= (float) (nsamp*param.pulses_per_daq_cycle);
		*TX_2A /= (float) (nsamp*param.pulses_per_daq_cycle);
		*TX_1B /= (float) (nsamp*param.pulses_per_daq_cycle);
		*TX_2B /= (float) (nsamp*param.pulses_per_daq_cycle);

		*TX_1A *= 2.0;
		*TX_1B *= 2.0;
		*TX_2A *= 2.0;
		*TX_2B *= 2.0;

		/* Convert to mV */
		*TX_1A *= 3000.0 / 4096.0;
		*TX_1B *= 3000.0 / 4096.0;
		*TX_2A *= 3000.0 / 4096.0;
		*TX_2B *= 3000.0 / 4096.0;


		if (mode < PM_Single_HV)
		{
		    for (sample = 0; sample < param.samples_per_pulse - mode_gate_offset; sample++)
		    {
			register int ii, idx;

			for (ii = 0; ii < param.nfft; ii++)
			{
			    idx = (ii * param.samples_per_pulse) + sample;

			    if (mode == PM_Single_H)
			    {
				fftw_real_lv (H_odd[ii])  = I_uncoded_copolar_H   [idx];
				fftw_imag_lv (H_odd[ii])  = Q_uncoded_copolar_H   [idx];
				fftw_real_lv (V_odd[ii])  = I_uncoded_crosspolar_H[idx + mode_gate_offset];
				fftw_imag_lv (V_odd[ii])  = Q_uncoded_crosspolar_H[idx + mode_gate_offset];
				fftw_real_lv (V0_odd[ii]) = I_uncoded_crosspolar_H[idx];
				fftw_imag_lv (V0_odd[ii]) = Q_uncoded_crosspolar_H[idx];
			    }
			    else
			    {
				fftw_real_lv (H_even[ii])  = I_uncoded_copolar_H   [idx + mode_gate_offset];
				fftw_imag_lv (H_even[ii])  = Q_uncoded_copolar_H   [idx + mode_gate_offset];
				fftw_real_lv (V_even[ii])  = I_uncoded_crosspolar_H[idx];
				fftw_imag_lv (V_even[ii])  = Q_uncoded_crosspolar_H[idx];
				fftw_real_lv (H0_even[ii]) = I_uncoded_copolar_H   [idx];
				fftw_imag_lv (H0_even[ii]) = Q_uncoded_copolar_H   [idx];
			    }
			}

			RSP_SubtractOffset_FFTW (H_odd,   param.nfft);
			RSP_SubtractOffset_FFTW (V_odd,   param.nfft);
			RSP_SubtractOffset_FFTW (H_even,  param.nfft);
			RSP_SubtractOffset_FFTW (V_even,  param.nfft);
			RSP_SubtractOffset_FFTW (H0_even, param.nfft);
			RSP_SubtractOffset_FFTW (V0_odd , param.nfft);

			for (ii = 0; ii < param.nfft; ii++)
			{
			    PH_VD[sample]       += (fftw_real (H_odd [ii]) * fftw_real (H_odd [ii]) + fftw_real (H_even[ii]) * fftw_real (H_even[ii]));
			    PH_VD[sample]       += (fftw_imag (H_odd [ii]) * fftw_imag (H_odd [ii]) + fftw_imag (H_even[ii]) * fftw_imag (H_even[ii]));
			    PV_VD[sample]       += (fftw_real (V_odd [ii]) * fftw_real (V_odd [ii]) + fftw_real (V_even[ii]) * fftw_real (V_even[ii]));
			    PV_VD[sample]       += (fftw_imag (V_odd [ii]) * fftw_imag (V_odd [ii]) + fftw_imag (V_even[ii]) * fftw_imag (V_even[ii]));

			    PH_VD_even[sample]  += (fftw_real (H_even[ii]) * fftw_real (H_even[ii]) + fftw_imag (H_even[ii]) * fftw_imag (H_even[ii]));
			    PV_VD_even[sample]  += (fftw_real (V_even[ii]) * fftw_real (V_even[ii]) + fftw_imag (V_even[ii]) * fftw_imag (V_even[ii]));
			    PH0_FD_even[sample] += (fftw_real (H0_even[ii]) * fftw_real (H0_even[ii]) + fftw_imag (H0_even[ii]) * fftw_imag (H0_even[ii]));
			    PV0_FD_odd[sample]  += (fftw_real (V0_odd [ii]) * fftw_real (V0_odd [ii]) + fftw_imag (V0_odd [ii]) * fftw_imag (V0_odd [ii]));
			    PH_VD_odd[sample]   += (fftw_real (H_odd [ii]) * fftw_real (H_odd [ii]) + fftw_imag (H_odd [ii]) * fftw_imag (H_odd [ii]));
			    PV_VD_odd[sample]   += (fftw_real (V_odd [ii]) * fftw_real (V_odd [ii]) + fftw_imag (V_odd [ii]) * fftw_imag (V_odd [ii]));
			}
		    }
		}
		else
		{
		    /* Calculate VEL_VD: velocity from variable delay pulse pair */
		    for (sample = 0; sample < param.samples_per_pulse - mode_gate_offset; sample++)
		    {
			tempI_odd  = 0.0;
			tempQ_odd  = 0.0;
			tempI_even = 0.0;
			tempQ_even = 0.0;
			tempI_vel  = 0.0;
			tempQ_vel  = 0.0;
			register int ii, jj, idx;

			for (ii = 0; ii < param.nfft * param.num_tx_pol; ii++)
			{
			    jj  = (ii / 2);
			    idx = (ii * param.samples_per_pulse) + sample;
			    if ((ii + horizontal_first) % 2 == 1)
			    {
				fftw_real_lv (H_odd[jj]) = I_uncoded_copolar_H   [idx];
				fftw_imag_lv (H_odd[jj]) = Q_uncoded_copolar_H   [idx];
				fftw_real_lv (V_odd[jj]) = I_uncoded_crosspolar_H[idx + mode_gate_offset];
				fftw_imag_lv (V_odd[jj]) = Q_uncoded_crosspolar_H[idx + mode_gate_offset];
			    }
			    else
			    {
				fftw_real_lv (H_even[jj]) = I_uncoded_copolar_H   [idx + mode_gate_offset];
				fftw_imag_lv (H_even[jj]) = Q_uncoded_copolar_H   [idx + mode_gate_offset];
				fftw_real_lv (V_even[jj]) = I_uncoded_crosspolar_H[idx];
				fftw_imag_lv (V_even[jj]) = Q_uncoded_crosspolar_H[idx];
			    }
			}

			RSP_SubtractOffset_FFTW (H_odd,  param.nfft);
			RSP_SubtractOffset_FFTW (V_odd,  param.nfft);
			RSP_SubtractOffset_FFTW (H_even, param.nfft);
			RSP_SubtractOffset_FFTW (V_even, param.nfft);

			for (ii = 0; ii < param.nfft; ii++)
			{
			    // VH is V x conj (H) == 1st pulse H, 2nd pulse V
			    tempI_odd  += fftw_real (V_odd [ii]) * fftw_real (H_odd [ii]) + fftw_imag (V_odd [ii]) * fftw_imag (H_odd [ii]);
			    tempQ_odd  += fftw_imag (V_odd [ii]) * fftw_real (H_odd [ii]) - fftw_real (V_odd [ii]) * fftw_imag (H_odd [ii]);
			    tempI_even += fftw_real (V_even[ii]) * fftw_real (H_even[ii]) + fftw_imag (V_even[ii]) * fftw_imag (H_even[ii]);
			    tempQ_even += fftw_imag (H_even[ii]) * fftw_real (V_even[ii]) - fftw_real (H_even[ii]) * fftw_imag (V_even[ii]);

			    PH_VD[sample]      += (fftw_real (H_odd [ii]) * fftw_real (H_odd [ii]) + fftw_real (H_even[ii]) * fftw_real (H_even[ii]));
			    PH_VD[sample]      += (fftw_imag (H_odd [ii]) * fftw_imag (H_odd [ii]) + fftw_imag (H_even[ii]) * fftw_imag (H_even[ii]));
			    PV_VD[sample]      += (fftw_real (V_odd [ii]) * fftw_real (V_odd [ii]) + fftw_real (V_even[ii]) * fftw_real (V_even[ii]));
			    PV_VD[sample]      += (fftw_imag (V_odd [ii]) * fftw_imag (V_odd [ii]) + fftw_imag (V_even[ii]) * fftw_imag (V_even[ii]));
			    PH_VD_even[sample] += (fftw_real (H_even[ii]) * fftw_real (H_even[ii]) + fftw_imag (H_even[ii]) * fftw_imag (H_even[ii]));
			    PV_VD_even[sample] += (fftw_real (V_even[ii]) * fftw_real (V_even[ii]) + fftw_imag (V_even[ii]) * fftw_imag (V_even[ii]));
			    PH_VD_odd[sample]  += (fftw_real (H_odd [ii]) * fftw_real (H_odd [ii]) + fftw_imag (H_odd [ii]) * fftw_imag (H_odd [ii]));
			    PV_VD_odd[sample]  += (fftw_real (V_odd [ii]) * fftw_real (V_odd [ii]) + fftw_imag (V_odd [ii]) * fftw_imag (V_odd [ii]));
			}

			VEL_VD_COS_even[sample] += tempI_even;
			VEL_VD_SIN_even[sample] += tempQ_even;
			VEL_VD_COS_odd [sample] += tempI_odd;
			VEL_VD_SIN_odd [sample] += tempQ_odd;

			/* Subtract phi_offset from even (HV) and add to odd (VH) */
			tempI      = tempI_even;
			tempQ      = tempQ_even;
			tempI_even = tempI * cos (param.phidp_offset) + tempQ * sin (param.phidp_offset);
			tempQ_even = tempQ * cos (param.phidp_offset) - tempI * sin (param.phidp_offset);
			tempI      = tempI_odd;
			tempQ      = tempQ_odd;
			tempI_odd  = tempI * cos (param.phidp_offset) - tempQ * sin (param.phidp_offset);
			tempQ_odd  = tempI * sin (param.phidp_offset) + tempQ * cos (param.phidp_offset);

			tempQ = -tempI_even * tempQ_odd  + tempI_odd * tempQ_even;
			tempI =  tempI_odd  * tempI_even + tempQ_odd * tempQ_even;
			phidp = atan2 (tempQ, tempI) / 2.0;

			PHIDP_VD_COS[sample] += tempI;
			PHIDP_VD_SIN[sample] += tempQ;

			tempQ      = sin (phidp);
			tempI      = cos (phidp);
			tempQ_vel  = tempI * tempQ_odd + tempI_odd * tempQ;
			tempI_vel  = tempI_odd * tempI - tempQ_odd * tempQ;
			tempQ_vel += tempI * tempQ_even - tempI_even * tempQ;
			tempI_vel += tempI_even * tempI + tempQ_even * tempQ;

			VEL_VD_COS[sample] += tempI_vel;
			VEL_VD_SIN[sample] += tempQ_vel;
		    }


		    /* Calculate VEL_FD: velocity from fixed delay (160 us) pulse pair */
		    for (sample = 0; sample < param.samples_per_pulse; sample++)
		    {
			tempI_odd  = 0.0;
			tempQ_odd  = 0.0;
			tempI_even = 0.0;
			tempQ_even = 0.0;
			tempI_vel  = 0.0;
			tempQ_vel  = 0.0;
			register int ii, jj, idx;

			for (ii = 0; ii < (param.nfft - 1) * param.num_tx_pol; ii++)
			{
			    jj  = (ii / 2);
			    idx = (ii * param.samples_per_pulse) + sample;
			    if ((ii + horizontal_first) % 2 == 1)
			    {
				fftw_real_lv (H_odd [jj]) = I_uncoded_copolar_H   [idx];
				fftw_imag_lv (H_odd [jj]) = Q_uncoded_copolar_H   [idx];
				fftw_real_lv (V_odd [jj]) = I_uncoded_crosspolar_H[idx + param.samples_per_pulse];
				fftw_imag_lv (V_odd [jj]) = Q_uncoded_crosspolar_H[idx + param.samples_per_pulse];
				fftw_real_lv (V0_odd[jj]) = I_uncoded_crosspolar_H[idx];
				fftw_imag_lv (V0_odd[jj]) = Q_uncoded_crosspolar_H[idx];
			    }
			    else
			    {
				fftw_real_lv (H_even [jj]) = I_uncoded_copolar_H   [idx + param.samples_per_pulse];
				fftw_imag_lv (H_even [jj]) = Q_uncoded_copolar_H   [idx + param.samples_per_pulse];
				fftw_real_lv (V_even [jj]) = I_uncoded_crosspolar_H[idx];
				fftw_imag_lv (V_even [jj]) = Q_uncoded_crosspolar_H[idx];
				fftw_real_lv (H0_even[jj]) = I_uncoded_copolar_H   [idx];
				fftw_imag_lv (H0_even[jj]) = Q_uncoded_copolar_H   [idx];
			    }
			}

			RSP_SubtractOffset_FFTW (H_odd,   param.nfft - 1);
			RSP_SubtractOffset_FFTW (V_odd,   param.nfft - 1);
			RSP_SubtractOffset_FFTW (H_even,  param.nfft - 1);
			RSP_SubtractOffset_FFTW (V_even,  param.nfft - 1);
			RSP_SubtractOffset_FFTW (H0_even, param.nfft - 1);
			RSP_SubtractOffset_FFTW (V0_odd , param.nfft - 1);

			for (ii = 0; ii < round (param.nfft - 1); ii++)
			{
			    // VH is V x conj (H) == 1st pulse H, 2nd pulse V
			    tempI_odd  += fftw_real (V_odd [ii]) * fftw_real (H_odd [ii]) + fftw_imag (V_odd [ii]) * fftw_imag (H_odd [ii]);
			    tempQ_odd  += fftw_imag (V_odd [ii]) * fftw_real (H_odd [ii]) - fftw_real (V_odd [ii]) * fftw_imag (H_odd [ii]);
			    tempI_even += fftw_real (V_even[ii]) * fftw_real (H_even[ii]) + fftw_imag (V_even[ii]) * fftw_imag (H_even[ii]);
			    tempQ_even += fftw_imag (H_even[ii]) * fftw_real (V_even[ii]) - fftw_real (H_even[ii]) * fftw_imag (V_even[ii]);

			    PH_FD[sample] += (fftw_real (H_odd[ii]) * fftw_real (H_odd[ii]) + fftw_real (H_even[ii]) * fftw_real (H_even[ii]));
			    PH_FD[sample] += (fftw_imag (H_odd[ii]) * fftw_imag (H_odd[ii]) + fftw_imag (H_even[ii]) * fftw_imag (H_even[ii]));
			    PV_FD[sample] += (fftw_real (V_odd[ii]) * fftw_real (V_odd[ii]) + fftw_real (V_even[ii]) * fftw_real (V_even[ii]));
			    PV_FD[sample] += (fftw_imag (V_odd[ii]) * fftw_imag (V_odd[ii]) + fftw_imag (V_even[ii]) * fftw_imag (V_even[ii]));

			    PH_FD_even[sample]  += (fftw_real (H_even [ii]) * fftw_real (H_even [ii]) + fftw_imag (H_even [ii]) * fftw_imag (H_even [ii]));
			    PV_FD_even[sample]  += (fftw_real (V_even [ii]) * fftw_real (V_even [ii]) + fftw_imag (V_even [ii]) * fftw_imag (V_even [ii]));
			    PH0_FD_even[sample] += (fftw_real (H0_even[ii]) * fftw_real (H0_even[ii]) + fftw_imag (H0_even[ii]) * fftw_imag (H0_even[ii]));
			    PV0_FD_odd[sample]  += (fftw_real (V0_odd [ii]) * fftw_real (V0_odd [ii]) + fftw_imag (V0_odd [ii]) * fftw_imag (V0_odd [ii]));
			    PH_FD_odd[sample]   += (fftw_real (H_odd  [ii]) * fftw_real (H_odd  [ii]) + fftw_imag (H_odd  [ii]) * fftw_imag (H_odd  [ii]));
			    PV_FD_odd[sample]   += (fftw_real (V_odd  [ii]) * fftw_real (V_odd  [ii]) + fftw_imag (V_odd  [ii]) * fftw_imag (V_odd  [ii]));
			}

			VEL_FD_COS_even[sample] += tempI_even;
			VEL_FD_SIN_even[sample] += tempQ_even;
			VEL_FD_COS_odd [sample] += tempI_odd;
			VEL_FD_SIN_odd [sample] += tempQ_odd;

			/* Subtract phi_offset from even (HV) and add to odd (VH) */
			tempI      = tempI_even;
			tempQ      = tempQ_even;
			tempI_even = tempI * cos (param.phidp_offset) + tempQ * sin (param.phidp_offset);
			tempQ_even = tempQ * cos (param.phidp_offset) - tempI * sin (param.phidp_offset);
			tempI      = tempI_odd;
			tempQ      = tempQ_odd;
			tempI_odd  = tempI * cos (param.phidp_offset) - tempQ * sin (param.phidp_offset);
			tempQ_odd  = tempI * sin (param.phidp_offset) + tempQ * cos (param.phidp_offset);


			tempQ = -tempI_even * tempQ_odd  + tempI_odd * tempQ_even;
			tempI =  tempI_odd  * tempI_even + tempQ_odd * tempQ_even;

			phidp = atan2 (tempQ, tempI) / 2.0;
			PHIDP_FD_COS[sample] += tempI;
			PHIDP_FD_SIN[sample] += tempQ;

			tempQ      = sin (phidp);
			tempI      = cos (phidp);
			tempQ_vel  = tempI * tempQ_odd  + tempI_odd  * tempQ;
			tempI_vel  = tempI_odd  * tempI - tempQ_odd  * tempQ;
			tempQ_vel += tempI * tempQ_even - tempI_even * tempQ;
			tempI_vel += tempI_even * tempI + tempQ_even * tempQ;

			VEL_FD_COS[sample] += tempI_vel;
			VEL_FD_SIN[sample] += tempQ_vel;
		    }
		}

		/* Calculate power spectra for each gate */
		//printf ("** Calculating power spectra...\n");
		//printf ("NFFT=%d\n",param.nfft);
		/* Loop through gates */
		for (sample = 0; sample < param.samples_per_pulse; sample++)
		{
		    register int ii, jj, idx;

		    if (mode == PM_Single_H)
		    {
			// 1) UNCODED H-COPOLAR SPECTRUM (HH)
			for (ii = 0; ii < param.nfft; ii++)
			{
			    idx = (ii * param.samples_per_pulse) + sample;
			    fftw_real_lv (in[ii]) = I_uncoded_copolar_H[idx];
			    fftw_imag_lv (in[ii]) = Q_uncoded_copolar_H[idx];
			}
			RSP_SubtractOffset_FFTW (in, param.nfft);
			RSP_CalcPSD_FFTW (in, param.nfft, p_uncoded, param.window, current_PSD, norm_uncoded);
			for (ii = 0; ii < param.npsd; ii++)
			{
			    PSD[sample].HH[ii] += current_PSD[ii] / param.spectra_averaged;
			}

			// 2) UNCODED H-CROSSPOLAR SPECTRUM (HV)
			for (ii = 0; ii < param.nfft; ii++)
			{
			    idx = (ii * param.samples_per_pulse) + sample;
			    fftw_real_lv (in[ii]) = I_uncoded_crosspolar_H[idx];
			    fftw_imag_lv (in[ii]) = Q_uncoded_crosspolar_H[idx];
			}
			RSP_SubtractOffset_FFTW (in, param.nfft);
			RSP_CalcPSD_FFTW (in, param.nfft, p_uncoded, param.window, current_PSD, norm_uncoded);
			for (ii = 0; ii < param.npsd; ii++)
			{
			    PSD[sample].HV[ii] += current_PSD[ii] / param.spectra_averaged;
			}
		    }
		    else if (mode == PM_Single_V)
		    {
			// 3) UNCODED V-COPOLAR SPECTRUM (VV)
			for (ii = 0; ii < param.nfft; ii++)
			{
			    idx = (ii * param.samples_per_pulse) + sample;
			    fftw_real_lv (in[ii]) = I_uncoded_crosspolar_H[idx];
			    fftw_imag_lv (in[ii]) = Q_uncoded_crosspolar_H[idx];
			}
			RSP_SubtractOffset_FFTW (in, param.nfft);
			RSP_CalcPSD_FFTW (in, param.nfft, p_uncoded, param.window, current_PSD, norm_uncoded);
			for (ii = 0; ii < param.npsd; ii++)
			{
			    PSD[sample].VV[ii] += current_PSD[ii] / param.spectra_averaged;
			}

			// 4) UNCODED V-CROSSPOLAR SPECTRUM (VH)
			for (ii = 0; ii < param.nfft; ii++)
			{
				idx = (ii * param.samples_per_pulse) + sample;
				fftw_real_lv (in[ii]) = I_uncoded_copolar_H[idx];
				fftw_imag_lv (in[ii]) = Q_uncoded_copolar_H[idx];
			}
			RSP_SubtractOffset_FFTW (in, param.nfft);
			RSP_CalcPSD_FFTW (in, param.nfft, p_uncoded, param.window, current_PSD, norm_uncoded);
			for (ii = 0; ii < param.npsd; ii++)
			{
			    PSD[sample].VH[ii] += current_PSD[ii] / param.spectra_averaged;
			}
		    }
		    else
		    {
			if (mode != PM_Double_V)
			{
			    // 1) UNCODED H-COPOLAR SPECTRUM (HH)
			    for (ii = 0; ii < param.nfft * param.num_tx_pol; ii++)
			    {
				if ((ii+horizontal_first) % 2 == 1)
				{
				    jj  = (ii / 2);
				    idx = (ii * param.samples_per_pulse) + sample;
				    fftw_real_lv (in[jj]) = I_uncoded_copolar_H[idx];
				    fftw_imag_lv (in[jj]) = Q_uncoded_copolar_H[idx];
				}
			    }
			    RSP_SubtractOffset_FFTW (in, param.nfft);
			    RSP_CalcPSD_FFTW (in, param.nfft, p_uncoded, param.window, current_PSD, norm_uncoded);
			    for (ii = 0; ii < param.npsd; ii++)
			    {
				PSD[sample].HH[ii] += current_PSD[ii] / param.spectra_averaged;
			    }

			    // 2) UNCODED H-CROSSPOLAR SPECTRUM (HV)
			    for (ii = 0; ii < param.nfft * param.num_tx_pol; ii++)
			    {
				if ((ii + horizontal_first) % 2 == 1)
				{
				    jj  = (ii / 2);
				    idx = (ii * param.samples_per_pulse) + sample;
				    fftw_real_lv (in[jj]) = I_uncoded_crosspolar_H[idx];
				    fftw_imag_lv (in[jj]) = Q_uncoded_crosspolar_H[idx];
				}
			    }
			    RSP_SubtractOffset_FFTW (in, param.nfft);
			    RSP_CalcPSD_FFTW (in, param.nfft, p_uncoded, param.window, current_PSD, norm_uncoded);
			    for (ii = 0; ii < param.npsd; ii++)
			    {
				PSD[sample].HV[ii] += current_PSD[ii] / param.spectra_averaged;
			    }
			}
			if (mode != PM_Double_H)
			{
			    // 3) UNCODED V-COPOLAR SPECTRUM (VV)
			    for (ii = 0; ii < param.nfft * param.num_tx_pol; ii++)
			    {
				if ((ii+horizontal_first) % 2 == 0)
				{
				    jj  = (ii / 2);
				    idx = (ii * param.samples_per_pulse) + sample;
				    fftw_real_lv (in[jj]) = I_uncoded_crosspolar_H[idx];
				    fftw_imag_lv (in[jj]) = Q_uncoded_crosspolar_H[idx];
				}
			    }
			    RSP_SubtractOffset_FFTW (in, param.nfft);
			    RSP_CalcPSD_FFTW (in, param.nfft, p_uncoded, param.window, current_PSD, norm_uncoded);
			    for (ii = 0; ii < param.npsd; ii++)
			    {
				PSD[sample].VV[ii] += current_PSD[ii] / param.spectra_averaged;
			    }

			    // 4) UNCODED V-CROSSPOLAR SPECTRUM (VH)
			    for (ii = 0; ii < param.nfft * param.num_tx_pol; ii++)
			    {
				if ((ii+horizontal_first) % 2 == 0)
				{
				    jj  = (ii / 2);
				    idx = (ii * param.samples_per_pulse) + sample;
				    fftw_real_lv (in[jj]) = I_uncoded_copolar_H[idx];
				    fftw_imag_lv (in[jj]) = Q_uncoded_copolar_H[idx];
				}
			    }
			    RSP_SubtractOffset_FFTW (in, param.nfft);
			    RSP_CalcPSD_FFTW (in, param.nfft, p_uncoded, param.window, current_PSD, norm_uncoded);
			    for (ii = 0; ii < param.npsd; ii++)
			    {
				PSD[sample].VH[ii] += current_PSD[ii] / param.spectra_averaged;
			    }
			}
		    }
		}

		if (!exit_now && tsdump && !TextTimeSeries)
		{
		    /* Needs to happen first as WriteDynamicVariables increments ray number */
		    printf ("Writing timeseries variables to NetCDF...\n");
		    WriteOutTimeSeriesData (ncidts, &param, &obs, &tsobs, nm);
		    status = nc_sync (ncidts);
		    if (status != NC_NOERR) check_netcdf_handle_error (status);
		}
	    }
	    /*---------------------------*
	     * END OF SPECTRAL AVERAGING *
	     *---------------------------*/

	    /* update time in spectral information file */
	    PSD_obs.year              = obs.year;
	    PSD_obs.month             = obs.month;
	    PSD_obs.day               = obs.day;
	    PSD_obs.hour              = obs.hour;
	    PSD_obs.minute            = obs.minute;
	    PSD_obs.second            = obs.second;
	    PSD_obs.centisecond       = obs.centisecond;

	    PSD_RAPID_obs.year        = PSD_obs.year;
	    PSD_RAPID_obs.month       = PSD_obs.month;
	    PSD_RAPID_obs.day         = PSD_obs.day;
	    PSD_RAPID_obs.hour        = PSD_obs.hour;
	    PSD_RAPID_obs.minute      = PSD_obs.minute;
	    PSD_RAPID_obs.second      = PSD_obs.second;
	    PSD_RAPID_obs.centisecond = PSD_obs.centisecond;

	    /* writing out rapid spectra */
	    if ((collect_spectra_rapid_now == 1) && !exit_now)
	    {
		printf ("Writing Rapid PSD Variables... ***************************\n");
		RNC_WriteRapidLogPSDVariables (spectra_rapid_ncid, GALILEO_SPECTRA_RAPID, &param, &PSD_RAPID_obs, PSD, PSD_rapid_varid);
		status = nc_sync (spectra_ncid);
		if (status != NC_NOERR) check_netcdf_handle_error (status);
		spectra_rapid_time =  spectra_rapid_time + param.dump_spectra_rapid;
	    }

	    /* write out normal spectra */
	    if ((collect_spectra_now == 1) && !exit_now)
	    {
		printf ("Writing PSD Variables... ***************************\n");
		RNC_WriteLogPSDVariables (spectra_ncid, GALILEO_SPECTRA, &param, &PSD_obs, PSD, &IQStruct, PSD_varid);
		status = nc_sync (spectra_ncid);
		if (status != NC_NOERR) check_netcdf_handle_error (status);
		spectra_time =  spectra_time + param.dump_spectra;
	    }

#ifdef HAVE_DISLIN
	    /* check to see if the real time spectra display option has been selected */
	    if (param.real_time_spectra_display == 1)
	    {
		/* real time display */
		/* write out data */
		for (i = 0; i < param.samples_per_pulse; i++)
		{
		    /* calculate the log10 of the PSD */
		    for (j = 0; j < param.npsd; j++)
		    {
			zmat[j][i] = 10 * log10 (PSD[i].HH[j]);
		    }
		}
		crvmat ((float*)zmat, param.npsd, param.samples_per_pulse, 1, 1);
		height (20);
		title ();
		printf ("x window updated\n");
	    }
#endif /* HAVE_DISLIN */

	    /* Calculate noise from upper range gates */
	    noisegate1 = param.samples_per_pulse - 50;
	    noisegate2 = param.samples_per_pulse - 1;
	    count      = (noisegate2 - noisegate1) + 1;

	    HH_noise_level = 0.0;
	    HV_noise_level = 0.0;
	    VV_noise_level = 0.0;
	    VH_noise_level = 0.0;

	    for (i = noisegate1; i <= noisegate2; i++)
	    {
		if (mode != PM_Single_V && mode != PM_Double_V)
		{ 
		    HH_noise_level += median (PSD[i].HH, param.npsd);
		    HV_noise_level += median (PSD[i].HV, param.npsd);
		}
		if (mode != PM_Single_H && mode != PM_Double_H)
		{ 
		    VV_noise_level += median (PSD[i].VV, param.npsd);
		    VH_noise_level += median (PSD[i].VH, param.npsd);
		}
	    }
	    HH_noise_level /= count;
	    HV_noise_level /= count;
	    VV_noise_level /= count;
	    VH_noise_level /= count;

	    if (mode == PM_Single_V || mode == PM_Double_V)
	    {
		NPC_H[0] += VH_noise_level;
		NPC_V[0] += VV_noise_level;
	    }
	    else
	    {
		NPC_H[0] += HH_noise_level;
		NPC_V[0] += HV_noise_level;
	    }

	    /* Now calculate the Doppler parameters */
	    printf ("** Calculating Doppler parameters...\n");
	    /* Loop through all spectra and get parameters */
	    for (i = 0; i < param.samples_per_pulse; i++)
	    {
		float noise_power, tempPower, tempVel, tempZED;

		// interpolate over clutter
		if (mode != PM_Single_V && mode != PM_Double_V)
		{ 
		    RSP_ClutterInterp (PSD[i].HH, param.npsd, param.fft_bins_interpolated);
		    RSP_ClutterInterp (PSD[i].HV, param.npsd, param.fft_bins_interpolated);

		    /* Find HH peak */
		    RSP_FindPeaksMulti_Destructive (PSD[i].HH, param.npsd, param.num_peaks, HH_noise_level, HH_peaks);
		    /* Calculate HH moments */
		    RSP_CalcSpecMom (PSD[i].HH, param.npsd, HH_peaks, HH_noise_level, HH_moments, RSP_MOMENTS);

		    /* Find HV peak */
		    RSP_FindPeaksMulti_Destructive (PSD[i].HV, param.npsd, param.num_peaks, HV_noise_level, HV_peaks);
		    /* Calculate HV moments */
		    RSP_CalcSpecMom (PSD[i].HV, param.npsd, HV_peaks, HV_noise_level, HV_moments, RSP_MOMENTS);
		}

		if (mode != PM_Single_H && mode != PM_Double_H)
		{ 
		    RSP_ClutterInterp (PSD[i].VV, param.npsd, param.fft_bins_interpolated);
		    RSP_ClutterInterp (PSD[i].VH, param.npsd, param.fft_bins_interpolated);

		    /* Find VV peak */
		    RSP_FindPeaksMulti_Destructive (PSD[i].VV, param.npsd, param.num_peaks, VV_noise_level, VV_peaks);
		    /* Calculate VV moments */
		    RSP_CalcSpecMom (PSD[i].VV, param.npsd, VV_peaks, VV_noise_level, VV_moments, RSP_MOMENTS);

		    /* Find VH peak */
		    RSP_FindPeaksMulti_Destructive (PSD[i].VH, param.npsd, param.num_peaks, VH_noise_level, VH_peaks);
		    /* Calculate VH moments */
		    RSP_CalcSpecMom (PSD[i].VH, param.npsd, VH_peaks, VH_noise_level, VH_moments, RSP_MOMENTS);
		}

		/*----------------------------*
		 * PROCESS UNCODED PARAMETERS *
		 *----------------------------*/
		if (mode == PM_Single_V || mode == PM_Double_V)
		{
		    noise_power = RSP_CalcNoisePower (VV_noise_level, VV_peaks, &param);
		    tempPower   = VV_moments[0] * param.frequency_bin_width;
		}
		else
		{
		    noise_power = RSP_CalcNoisePower (HH_noise_level, HH_peaks, &param);
		    tempPower   = HH_moments[0] * param.frequency_bin_width;
		}

		/* Calculate weighting coefficient */
		//wi = (peaks[0].peakPSD - HH_noise_level);
		//wi = (peaks[0].peakPSD - VV_noise_level);
		wi = 1; // Turn off weighting
		//wi = tempPower/noise_power;
		//if (wi > 1) wi = 1;
		uncoded_sum_wi[i] += wi;

		if (mode != PM_Single_V && mode != PM_Double_V)
		{ 		
		    /* COPOLAR */
		    SNR_HC[i]     += tempPower/noise_power * wi;
		    ZED_HC[i]     += tempPower * wi;
		    tempZED        = 10.0 * log10 (tempPower);
		    tempVel        = RSP_BinToVelocity (HH_moments[1], &param);
		    VEL_HC_COS[i] += cos (tempVel / param.folding_velocity * PI) * wi;
		    VEL_HC_SIN[i] += sin (tempVel / param.folding_velocity * PI) * wi;
		    SPW_HC[i]     += HH_moments[2] * param.frequency_bin_width / param.hz_per_mps * wi;

		    /* CROSSPOLAR */
		    noise_power    = RSP_CalcNoisePower (HV_noise_level, HV_peaks, &param);
		    tempPower      = HV_moments[0] * param.frequency_bin_width;
		    SNR_XHC[i]    += tempPower / noise_power * wi;
		    ZED_XHC[i]    += tempPower * wi;
		}
		if (mode != PM_Single_H && mode != PM_Double_H)
		{ 
		    /* V COPOLAR */
		    noise_power    = RSP_CalcNoisePower (VV_noise_level, VV_peaks, &param);
		    tempPower      = VV_moments[0] * param.frequency_bin_width;
		    SNR_VC[i]     += tempPower/noise_power * wi;
		    ZED_VC[i]     += tempPower * wi;
                    SPW_VC[i]     += VV_moments[2] * param.frequency_bin_width / param.hz_per_mps * wi;

		    tempVel        = RSP_BinToVelocity (VV_moments[1], &param);
		    VEL_VC_COS[i] += cos (tempVel / param.folding_velocity * PI) * wi;
		    VEL_VC_SIN[i] += sin (tempVel / param.folding_velocity * PI) * wi;

		    /* V CROSSPOLAR */
		    noise_power    = RSP_CalcNoisePower (VH_noise_level, VH_peaks, &param);
		    tempPower      = VH_moments[0] * param.frequency_bin_width;
		    SNR_XVC[i]    += tempPower / noise_power * wi;
		    ZED_XVC[i]    += tempPower * wi;
		}
	    }
	} // End of moments averaging loop

	for (i = 0; i < param.samples_per_pulse; i++)
	{
	    /* COMPLETE THE WEIGHTED AVERAGING WITH DIVISION */
	    SNR_HC[i]  /= uncoded_sum_wi[i];
	    ZED_HC[i]  /= uncoded_sum_wi[i];
	    SNR_VC[i]  /= uncoded_sum_wi[i];
	    ZED_VC[i]  /= uncoded_sum_wi[i];
	    SPW_HC[i]  /= uncoded_sum_wi[i];
            SPW_VC[i]  /= uncoded_sum_wi[i];
	    SNR_XHC[i] /= uncoded_sum_wi[i];
	    ZED_XHC[i] /= uncoded_sum_wi[i];
	    SNR_XVC[i] /= uncoded_sum_wi[i];
	    ZED_XVC[i] /= uncoded_sum_wi[i];

	    POW_H[i]   = PH_FD_odd[i];
	    POW_HX[i]  = PV0_FD_odd[i];
	    POW_V[i]   = PV_FD_even[i];
	    POW_VX[i]  = PH0_FD_even[i];
	    POW_H[i]  /= uncoded_sum_wi[i];
	    POW_HX[i] /= uncoded_sum_wi[i];
	    POW_V[i]  /= uncoded_sum_wi[i];
	    POW_VX[i] /= uncoded_sum_wi[i];

	    VEL_VD[i] = atan2 (VEL_VD_SIN[i], VEL_VD_COS[i]) * 0.299792458 / (4.0 * PI * param.pulse_offset * 1e-6 * param.frequency);
	    VEL_FD[i] = atan2 (VEL_FD_SIN[i], VEL_FD_COS[i]) * 0.299792458 * (float)param.num_tx_pol / (4.0 * PI * param.prt * param.frequency);

	    PHIDP_FD[i]  = atan2 (PHIDP_FD_SIN[i], PHIDP_FD_COS[i]) / 2.0;
	    PHIDP_FD[i] *= (180.0 / PI);
	    PHIDP_VD[i]  = atan2 (PHIDP_VD_SIN[i], PHIDP_VD_COS[i]) / 2.0;
	    PHIDP_VD[i] *= (180.0 / PI);

	    RHO_FD[i]  = VEL_FD_COS[i] * VEL_FD_COS[i] + VEL_FD_SIN[i] * VEL_FD_SIN[i];
	    RHO_FD[i] /= PH_FD[i] * PV_FD[i];
	    RHO_FD[i]  = sqrt (RHO_FD[i]);

	    RHO_VD[i]  = VEL_VD_COS[i] * VEL_VD_COS[i] + VEL_VD_SIN[i] * VEL_VD_SIN[i];
	    RHO_VD[i] /= PH_VD[i] * PV_VD[i];
	    RHO_VD[i]  = sqrt (RHO_VD[i]);

	    RHO_FDS[i ] = VEL_FD_COS_even[i] * VEL_FD_COS_even[i] + VEL_FD_SIN_even[i] * VEL_FD_SIN_even[i];
	    RHO_FDS[i] /= PH_FD_even[i] * PV_FD_even[i];
	    tmpRHO      = sqrt (RHO_FDS[i]);
	    RHO_FDS[i]  = VEL_FD_COS_odd[i] * VEL_FD_COS_odd[i] + VEL_FD_SIN_odd[i] * VEL_FD_SIN_odd[i];
	    RHO_FDS[i] /= PH_FD_odd[i] * PV_FD_odd[i];
	    RHO_FDS[i]  = sqrt (RHO_FDS[i]);
	    RHO_FDS[i]  = (RHO_FDS[i] + tmpRHO) / 2.0 ;

	    RHO_VDS[i]  = VEL_VD_COS_even[i] * VEL_VD_COS_even[i] + VEL_VD_SIN_even[i] * VEL_VD_SIN_even[i];
	    RHO_VDS[i] /= PH_VD_even[i] * PV_VD_even[i];
	    tmpRHO      = sqrt (RHO_VDS[i]);
	    RHO_VDS[i]  = VEL_VD_COS_odd[i] * VEL_VD_COS_odd[i] + VEL_VD_SIN_odd[i] * VEL_VD_SIN_odd[i];
	    RHO_VDS[i] /= PH_VD_odd[i] * PV_VD_odd[i];
	    RHO_VDS[i]  = sqrt (RHO_VDS[i]);
	    RHO_VDS[i]  = (RHO_VDS[i] + tmpRHO) / 2.0 ;

	    VEL_HC[i] = atan2 (VEL_HC_SIN[i], VEL_HC_COS[i]) / PI * param.folding_velocity;
	    VEL_VC[i] = atan2 (VEL_VC_SIN[i], VEL_VC_COS[i]) / PI * param.folding_velocity;

	    /* Convert SNRs and ZEDs to dB */
	    SNR_HC[i]  = 10.0 * log10 (SNR_HC[i]);
	    ZED_HC[i]  = 10.0 * log10 (ZED_HC[i]);
	    SNR_XHC[i] = 10.0 * log10 (SNR_XHC[i]);
	    ZED_XHC[i] = 10.0 * log10 (ZED_XHC[i]);
	    SNR_VC[i]  = 10.0 * log10 (SNR_VC[i]);
	    ZED_VC[i]  = 10.0 * log10 (ZED_VC[i]);
	    SNR_XVC[i] = 10.0 * log10 (SNR_XVC[i]);
	    ZED_XVC[i] = 10.0 * log10 (ZED_XVC[i]);
	    POW_H[i]   = 10.0 * log10 (POW_H[i]);
	    POW_HX[i]  = 10.0 * log10 (POW_HX[i]);
	    POW_V[i]   = 10.0 * log10 (POW_V[i]);
	    POW_VX[i]  = 10.0 * log10 (POW_VX[i]);

	    /* Calculate LDR */
	    LDR_HC[i] = ZED_XHC[i] - ZED_HC[i] + param.LDR_calibration_offset;
	    LDR_VC[i] = ZED_XVC[i] - ZED_VC[i] + param.LDR_calibration_offset;

	    /* Do range correction */
	    ZED_HC[i]  += 10.0 * log10 (param.range[i] * param.range[i]) + param.ZED_calibration_offset;
	    ZED_XHC[i] += 10.0 * log10 (param.range[i] * param.range[i]) + param.ZED_calibration_offset;
	    ZED_VC[i]  += 10.0 * log10 (param.range[i] * param.range[i]) + param.ZED_calibration_offset;
	    ZED_XVC[i] += 10.0 * log10 (param.range[i] * param.range[i]) + param.ZED_calibration_offset;

	    /* Calculate ZDR */
	    ZDR_C[i] = ZED_HC[i] - ZED_VC[i];
	}

	/* Range correct 2nd pulse, cross-pol ZDR */
	for (i = mode_gate_offset; i < param.samples_per_pulse; i++)
	{
	    ZED_HC[i]  -= 10.0 * log10 (param.range[i                   ] * param.range[i                   ]);
	    ZED_HC[i]  += 10.0 * log10 (param.range[i - mode_gate_offset] * param.range[i - mode_gate_offset]);
	    ZED_XHC[i] -= 10.0 * log10 (param.range[i                   ] * param.range[i                   ]);
	    ZED_XHC[i] += 10.0 * log10 (param.range[i - mode_gate_offset] * param.range[i - mode_gate_offset]);
	    ZED_VC[i]  -= 10.0 * log10 (param.range[i                   ] * param.range[i                   ]);
	    ZED_VC[i]  += 10.0 * log10 (param.range[i - mode_gate_offset] * param.range[i - mode_gate_offset]);
	    ZED_XVC[i] -= 10.0 * log10 (param.range[i                   ] * param.range[i                   ]);
	    ZED_XVC[i] += 10.0 * log10 (param.range[i - mode_gate_offset] * param.range[i - mode_gate_offset]);
	}

	NPC_H[0] /= uncoded_sum_wi[0];
	NPC_V[0] /= uncoded_sum_wi[0];
	NPC_H[0]  = 10.0 * log10 (NPC_H[0]);
	NPC_V[0]  = 10.0 * log10 (NPC_V[0]);

	/* Only write out variables to netCDF if we are not exiting the program */
	if (!exit_now)
	{
	    printf ("Writing dynamic variables to NetCDF...\n");
	    RNC_WriteDynamicVariables (ncid, &param, &obs);
	    status = nc_sync (ncid);
	    if (status != NC_NOERR) check_netcdf_handle_error (status);
	}

	printf ("I have written dynamic variables to NetCDF\n");
	/*--------------------------------------------------------------------*
	 * check to see if we have started a new day                          *
	 *--------------------------------------------------------------------*/
	system_time = time (NULL);
	gmtime_r (&system_time, &tm);
	if (tm.tm_mday != start_day)
	{
	    printf ("***** New day rollover detected.\n");
	    break; /* Exit loop */
	}

	if (positionMessageAct)
	{
	    /* Read position message again */
	    RSM_ReadPositionMessage (&position_msg);
	}
	else
	{
	    struct tm tm;
	    struct timeval tv;

	    gettimeofday (&tv, NULL);
	    gmtime_r (&tv.tv_sec, &tm);
	    obs.dish_year        = tm.tm_year + 1900;
	    obs.dish_month       = tm.tm_mon + 1;
	    obs.dish_day         = tm.tm_mday;
	    obs.dish_hour        = tm.tm_hour;
	    obs.dish_minute      = tm.tm_min;
	    obs.dish_second      = tm.tm_sec;
	    obs.dish_centisecond = tv.tv_usec / 10000U;
	}

	/* Test for end of scan */
	if (positionMessageAct || scan.scanType == SCAN_SGL)
 	{
	    scanEnd = scanEnd_test (scan.scanType, &position_msg,
				    scan.min_angle, scan.max_angle);
	}

	/* Prepare for possible switch to new mode next ray */
	if ((param.alternate_modes != 0) & (param.long_pulse_mode == 0))
	{
	    remainder = ray_count % (param.nrays_mode0 + param.nrays_mode1);
	    printf ("remainder calculated\n");
	    if (remainder == 0)
	    {
		new_mode  = 0;
		ray_count = 0;
	    }
	    else if (remainder == param.nrays_mode0)
	    {
		new_mode = 1;
	    }
	    else
	    {
		new_mode = -1;
	    }
	}
	else
	{
	    /* Don't alternate modes (remain in mode defined by mode0) */
	    new_mode = -1;
	}
    }

    /*-------------------------------------------------------------------------- *
     * END OF SCAN LOOP -------------------------------------------------------- *
     *========================================================================== */

exit_endacquisition:

    /*------------*
     * Finish off *
     *------------*/
    printf ("*** Closing PCICARD...\n");
    RDQ_ClosePCICARD_New (amcc_fd, &dma_buffer, DMA_BUFFER_SIZE);
#ifndef NO_DIO
    printf ("*** Closing DIO card...\n");
    close (fd);
#endif /* NO_DIO */

    if (tsfid != NULL)
    {
	/* Close time-series file */
	fclose (tsfid);
    }

    /* netCDF : close the netCDF file */
    status = nc_sync (ncid);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
    status = nc_close (ncid);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    if (param.dump_spectra != 0)
    {
	status = nc_sync (spectra_ncid);
	if (status != NC_NOERR) check_netcdf_handle_error (status);
	status = nc_close (spectra_ncid);
	if (status != NC_NOERR) check_netcdf_handle_error (status);
    }

    if (param.dump_spectra_rapid  != 0)
    {
	status = nc_sync (spectra_rapid_ncid);
	if (status != NC_NOERR) check_netcdf_handle_error (status);
	status = nc_close (spectra_rapid_ncid);
	if (status != NC_NOERR) check_netcdf_handle_error (status);
    }

    if (tsdump && !TextTimeSeries)
    {
	printf ("About to sync ts.\n");
	status = nc_sync (ncidts);
	if (status != NC_NOERR) check_netcdf_handle_error (status);
	printf ("About to close ts.\n");
	status = nc_close (ncidts);
	printf ("Status = %d\n", status);
	if (status != NC_NOERR) check_netcdf_handle_error (status);
	free (tsobs.ICOH);
    }

    /*---------------------------*
     * Unallocate all the memory *
     *---------------------------*/
    RSP_FreeMemory (&param);  // Free memory allocated by RSP package
    RSP_ObsFree (&obs);      // Free observables memory
    free (timeseries);
    free (current_PSD);

    for (i = 0; i < param.samples_per_pulse; i++)
    {
	free (PSD[i].HH);
	free (PSD[i].HV);
    }

    free (uncoded_mean_vsq);
    free (uncoded_mean_Zsq);
    free (uncoded_sum_wi);

    free (IQStruct.I_uncoded_copolar_H);
    free (IQStruct.Q_uncoded_copolar_H);
    free (IQStruct.I_uncoded_crosspolar_H);
    free (IQStruct.Q_uncoded_crosspolar_H);

    free (VEL_HC_COS);
    free (VEL_HC_SIN);
    free (VEL_VC_COS);
    free (VEL_VC_SIN);
    free (VEL_VD_COS);
    free (VEL_VD_SIN);
    free (VEL_FD_COS);
    free (VEL_FD_SIN);
    free (VEL_VD_COS_even);
    free (VEL_VD_SIN_even);
    free (VEL_FD_COS_even);
    free (VEL_FD_SIN_even);
    free (VEL_VD_COS_odd);
    free (VEL_VD_SIN_odd);
    free (VEL_FD_COS_odd);
    free (VEL_FD_SIN_odd);
    free (PHIDP_FD_COS);
    free (PHIDP_FD_SIN);
    free (PHIDP_VD_COS);
    free (PHIDP_VD_SIN);
    free (PH_FD);
    free (PV_FD);
    free (PH_VD);
    free (PV_VD);
    free (PH_FD_even);
    free (PV_FD_even);
    free (PH_VD_even);
    free (PV_VD_even);
    free (PH_FD_odd);
    free (PV_FD_odd);
    free (PH_VD_odd);
    free (PV_VD_odd);

    fftw_destroy_plan (p_uncoded);
    fftw_free (V0_odd);
    fftw_free (H0_even);
    fftw_free (V_even);
    fftw_free (H_even);
    fftw_free (V_odd);
    fftw_free (H_odd);
    fftw_free (in);

    if (positionMessageAct)
	RSM_ClosePositionMessage ();

    /*=========*
     * THE END *
     *=========*/
    printf ("All done.\n");
    return 0;
}


/*****************************************************************************/
/*!
 *
 * \fn int TimeSeriesTemplate (int ncid, const char * name,
 *                             const int  * variable_shape,
 *                             const char * std_name,
 *                             const char * long_name,
 *                             const char * units)
 *
 * \breif Helper function for setting up observables.
 *
 * \param [in] ncid            NetCDF file handle.
 * \param [in] name            Observable/variable name.
 * \param [in] variable_shape  Shape array for variable (all have 2 dims)
 * \param [in] std_name        Chilbolton standard name.
 * \param [in] long_name       Long name.
 * \param [in] units           Units string.
 *
 * \return NetCFG veriable id.
 *
 * This function assumes that the variable_shape has been correctly set up
 * for the first three dimensions: time,pulses,unaveraged_range as
 * required.
 *
 *****************************************************************************/
static int
TimeSeriesTemplate (int          ncid,
		    const char * name,
		    const int  * variable_shape,
		    const char * std_name,
		    const char * long_name,
		    const char * units)
{
    int varid;
    int status;

    status = nc_def_var (ncid, name, NC_SHORT, 3, variable_shape, &varid);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    status = nc_put_att_text (ncid, varid, "chilbolton_standard_name",
			      strlen (std_name) + 1, std_name);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    status = nc_put_att_text (ncid, varid, "long_name",
			      strlen (long_name) + 1, long_name);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    status = nc_put_att_text (ncid, varid, "units",
			      strlen (units) + 1, units);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    return varid;
}

static void
SetupTimeSeriesVariables (TimeSeriesObs_t *          obs,
			  int                        ncid,
			  RSP_ParamStruct *          param,
			  URC_ScanStruct *           scan,
			  RNC_DimensionStruct *      dimensions,
			  RSP_ObservablesStruct *    posobs)
{
    short  Bias = 2047;
    double PowerScale = 3000.0 / 4096.0;
    int status;
    int dims[3];
    const char * variable;
    char buffer [1024];

    /*--------------------------------------------------------------------------*
     * define pulses dimension                                                  *
     *--------------------------------------------------------------------------*/
    status = nc_def_dim (ncid, "pulses",
			 param->pulses_per_daq_cycle * param->spectra_averaged,
			 &dims[1]);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    /*--------------------------------------------------------------------------*
     * define samples dimension                                                 *
     *--------------------------------------------------------------------------*/
    status = nc_def_dim (ncid, "samples",
			 param->samples_per_pulse_ts,
			 &dims[2]);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    dims[0] = dimensions->time_dim;
    // dims[2] = dimensions->range_dim;

    /*--------------------------------------------------------------------------*
     * time definition                                                          *
     *--------------------------------------------------------------------------*/
    status = nc_def_var (ncid, "time",
			 NC_FLOAT, 1, dims, &obs->tsid);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "time";
    status = nc_put_att_text (ncid, obs->tsid, "chilbolton_standard_name",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "time";
    status = nc_put_att_text (ncid, obs->tsid, "long_name",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "%.2f";
    status = nc_put_att_text (ncid, obs->tsid, "C_format",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    sprintf (buffer, "seconds since %c%c%c%c-%c%c-%c%c 00:00:00 +00:00",
	     scan->date[0], scan->date[1], scan->date[2], scan->date[3],
	     scan->date[4], scan->date[5],
	     scan->date[6], scan->date[7]);
    status = nc_put_att_text (ncid, obs->tsid, "units",
			      strlen (buffer) + 1, buffer);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    /*--------------------------------------------------------------------------*
     * dish_time definition                                                     *
     *--------------------------------------------------------------------------*/
    status = nc_def_var (ncid, "dish_time",
			 NC_FLOAT, 1, dims, &obs->dish_tsid);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "dish_time";
    status = nc_put_att_text (ncid, obs->dish_tsid, "chilbolton_standard_name",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "dish_time";
    status = nc_put_att_text (ncid, obs->dish_tsid, "long_name",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "%.2f";
    status = nc_put_att_text (ncid, obs->tsid, "C_format",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    sprintf (buffer, "seconds since %04d-%02d-%02d 00:00:00 +00:00",
	     posobs->dish_year,
	     posobs->dish_month,
	     posobs->dish_day);
    status = nc_put_att_text (ncid, obs->dish_tsid, "units",
			      strlen (buffer) + 1, buffer);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    /*--------------------------------------------------------------------------*
     * elevation definition                                                     *
     *--------------------------------------------------------------------------*/
    status = nc_def_var (ncid, "elevation",
			 NC_FLOAT, 1, dims, &obs->elevationid);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "elevation angle above the horizon at the start of the beamwidth";
    status = nc_put_att_text (ncid, obs->elevationid, "long_name",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "%.3f";
    status = nc_put_att_text (ncid, obs->elevationid, "C_format",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "degree";
    status = nc_put_att_text (ncid, obs->elevationid, "units",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);


    /*--------------------------------------------------------------------------*
     * azimuth definition                                                       *
     *--------------------------------------------------------------------------*/
    status = nc_def_var (ncid, "azimuth",
			 NC_FLOAT, 1, dims, &obs->azimuthid);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "azimuth angle clockwise from the grid north at the start of the beamwidth";
    status = nc_put_att_text (ncid, obs->azimuthid, "long_name",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "%.3f";
    status = nc_put_att_text (ncid, obs->azimuthid, "C_format",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    variable = "degree";
    status = nc_put_att_text (ncid, obs->azimuthid, "units",
			      strlen (variable) + 1, variable);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    status = nc_put_att_float (ncid, obs->azimuthid, "azimuth_offset",
			       NC_FLOAT, 1, &param->azimuth_offset);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    obs->ICOHid = TimeSeriesTemplate (
	ncid, "ICOH",
	dims,
	"I_uncoded_copolar_H",
	"I uncoded copolar H",
	"counts");
    status = nc_put_att_short (ncid, obs->ICOHid,
			       "nominal_bias", NC_SHORT, 1,
			       &Bias);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    obs->QCOHid = TimeSeriesTemplate (
	ncid, "QCOH",
	dims,
	"Q_uncoded_copolar_H",
	"Q uncoded copolar H",
	"counts");
    status = nc_put_att_short (ncid, obs->QCOHid,
			       "nominal_bias", NC_SHORT, 1,
			       &Bias);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    obs->ICXHid = TimeSeriesTemplate (
	ncid, "ICXH",
	dims,
	"I_uncoded_crosspolar_H",
	"I uncoded crosspolar H",
	"counts");
    status = nc_put_att_short (ncid, obs->ICXHid,
			       "nominal_bias", NC_SHORT, 1,
			       &Bias);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    obs->QCXHid = TimeSeriesTemplate (
	ncid, "QCXH",
	dims,
	"Q_uncoded_crosspolar_H",
	"Q uncoded crosspolar H",
	"counts");
    status = nc_put_att_short (ncid, obs->QCXHid,
			       "nominal_bias", NC_SHORT, 1,
			       &Bias);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    obs->TxPower1id = TimeSeriesTemplate (
	ncid, "TXP1",
	dims,
	"Internal_Tx_Power",
	"Internal Tx Power",
	"counts");
    status = nc_put_att_double (ncid, obs->TxPower1id,
				"mV scale", NC_DOUBLE, 1,
				&PowerScale);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    obs->TxPower2id = TimeSeriesTemplate (
	ncid, "TXP2",
	dims,
	"External_Tx_Power",
	"External Tx Power",
	"counts");
    status = nc_put_att_double (ncid, obs->TxPower2id,
				"mV scale", NC_DOUBLE, 1,
				&PowerScale);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    obs->VnotHid = TimeSeriesTemplate (
	ncid, "VnotH",
	dims,
	"Pulse_polaration_V_not_H",
	"Pulse polaration V not H",
	"counts");

    obs->RawLogid = TimeSeriesTemplate (
	ncid, "LOG",
	dims,
	"Raw_Log",
	"Raw Log",
	"counts");
    status = nc_put_att_double (ncid, obs->RawLogid,
				"mV scale", NC_DOUBLE, 1,
				&PowerScale);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
}

static void WriteOutTimeSeriesData (int ncid,
				    const RSP_ParamStruct * param,
				    RSP_ObservablesStruct * posobs,
				    TimeSeriesObs_t * obs, int nm)
{
    size_t    variable_count[3];
    size_t    variable_start[3];
    ptrdiff_t variable_stride[3];
    ptrdiff_t variable_imap  [3];
    int       status;
    float     temp_float;

    /*--------------------------------------------------------------------------*
     * write time                                                               *
     *--------------------------------------------------------------------------*/
    variable_start[0] = posobs->ray_number;
    temp_float = (((int)posobs->hour * 3600) + ((int)posobs->minute * 60) +
		  posobs->second + ((float)posobs->centisecond / 100.0));
    status = nc_put_var1_float (ncid, obs->tsid, variable_start, &temp_float);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    /*--------------------------------------------------------------------------*
     * write dish_time                                                          *
     *--------------------------------------------------------------------------*/
    temp_float = (((int)posobs->dish_hour * 3600) +
		  ((int)posobs->dish_minute * 60) + posobs->dish_second +
		  ((float)posobs->dish_centisecond / 100.0));
    status = nc_put_var1_float (ncid, obs->dish_tsid,
				variable_start, &temp_float);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    /*--------------------------------------------------------------------------*
     * write elevation                                                          *
     *--------------------------------------------------------------------------*/
    status = nc_put_var1_float (ncid, obs->elevationid,
				variable_start, &posobs->elevation);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    /*--------------------------------------------------------------------------*
     * write azimuth                                                            *
     *--------------------------------------------------------------------------*/
    temp_float = posobs->azimuth + param->azimuth_offset;
    status = nc_put_var1_float (ncid, obs->azimuthid,
				variable_start, &temp_float);
    if (status != NC_NOERR) check_netcdf_handle_error (status);

    /*--------------------------------------------------------------------------*
     * write radar observables                                                  *
     *--------------------------------------------------------------------------*/
    variable_start[0]  = (posobs->ray_number * param->moments_averaged) + nm;
    variable_start[1]  = 0;
    variable_start[2]  = 0;
    variable_count[0]  = 1;
    variable_count[1]  = param->pulses_per_daq_cycle * param->spectra_averaged;
    variable_count[2]  = param->samples_per_pulse_ts;
    variable_stride[0] = 1;
    variable_stride[1] = 1;
    variable_stride[2] = 1;
    variable_imap[0]   = 1;
    variable_imap[1]   = param->samples_per_pulse;
    variable_imap[2]   = 1;

    status = nc_put_varm_short (ncid, obs->ICOHid, variable_start,
				variable_count, variable_stride,
				variable_imap, (short *)obs->ICOH);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
    status = nc_put_varm_short (ncid, obs->QCOHid, variable_start,
				variable_count, variable_stride,
				variable_imap, (short *)obs->QCOH);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
    status = nc_put_varm_short (ncid, obs->ICXHid, variable_start,
				variable_count, variable_stride,
				variable_imap, (short *)obs->ICXH);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
    status = nc_put_varm_short (ncid, obs->QCXHid, variable_start,
				variable_count, variable_stride,
				variable_imap, (short *)obs->QCXH);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
    status = nc_put_varm_short (ncid, obs->TxPower1id, variable_start,
				variable_count, variable_stride,
				variable_imap, (short *)obs->TxPower1);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
    status = nc_put_varm_short (ncid, obs->TxPower2id, variable_start,
				variable_count, variable_stride,
				variable_imap, (short *)obs->TxPower2);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
    status = nc_put_varm_short (ncid, obs->VnotHid, variable_start,
				variable_count, variable_stride,
				variable_imap, (short *)obs->VnotH);
    if (status != NC_NOERR) check_netcdf_handle_error (status);
    status = nc_put_varm_short (ncid, obs->RawLogid, variable_start,
				variable_count, variable_stride,
				variable_imap, (short *)obs->RawLog);
    if (status != NC_NOERR) check_netcdf_handle_error (status);   
}
