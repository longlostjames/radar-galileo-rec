/*===========================================================================*
 * RTS.c                                                                     *
 * Purpose: 	To help with the generation of time-series files             *
 *---------------------------------------------------------------------------*
 * Author: Chris Walden (CJW)                                                *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * REVISION HISTORY                                                          *
 *---------------------------------------------------------------------------*
 * 20030212 OTD created                                                      *
 * 20070829 OTD modified                                                     *
 * 20070913 OTD added in the ability to dump out IQ data prior to fft        *
 * 20080806 CJW this is now CAMRa-specific to get it to work with disp       *
 * 20090501 CJW modified                                                     *
 *---------------------------------------------------------------------------*
 * TO DO:                                                                    *
 * Think about how to put this into universal radar code                     *
 *===========================================================================*/

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <string.h>
#include <alloca.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <radar.h>
#include <RTS.h>

/*****************************************************************************
 *                                                                           *
 *****************************************************************************/
FILE *
RTS_OpenTSFile (const char * radar_name,
		const char * date,
		const char * host_ext,
		const char * scan_name)
{
    char * ts_pathfile;
    char * pt;
    mode_t mask = 002;
    size_t size;    

    umask (mask);

    size = strlen (RADAR_DATA_PATH) + 23;
    size += strlen (radar_name) << 1;
    if (host_ext != NULL)
	size += strlen (host_ext);
    size += strlen (date);
    size += strlen (scan_name);
    ts_pathfile = alloca (size);

    pt    = stpcpy (ts_pathfile, RADAR_DATA_PATH);
    pt    = stpcpy (pt, radar_name);
    pt    = stpcpy (pt, "/ts/");
    stpncpy (pt, date, 8);
    pt[8] = '\0';
    pt   += strlen (pt);

    if (!mkdir (ts_pathfile, (S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)))
    {
	printf ("directory created : %s\n", ts_pathfile);
    }
    else
    {
	if (errno != EEXIST)
	{
	    printf ("mkdir ERROR, errno : %d (%m), ts_pathfile : %s\n",
		    errno,
		    ts_pathfile);
	}
    }

    *pt++ = '/';
    pt    = stpcpy (pt, radar_name);
    if (host_ext != NULL)
	pt = stpcpy (pt, host_ext);
    *pt++ = '_';
    pt    = stpcpy (pt, date);
    *pt++ = '_';
    pt    = stpcpy (pt, scan_name);
    strcpy (pt, "-ts.dat");

    printf("TS creating : %s\n", ts_pathfile);

    return fopen (ts_pathfile, "w");
}
