/*
 * -----------------------------------------------------------------
 * PCI DIO24 card routines
 * Author: Alan Doo, STFC
 * Modifications: Chris Walden, STFC, 09/12/2016
 * -----------------------------------------------------------------
 */


#include <stdio.h>
#include <stdlib.h>
#include <comedilib.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#include "pci_dio24.h"
#include "radar-galileo-rec.h"

lsampl_t data;
lsampl_t maxdata = 0x01;
lsampl_t mindata = 0x0;


int read_dio(comedi_t *device, int subdev, int start, int end) {

	int chan_count;
	int retval;

	// loop through channels from start to end range
	// set each channel to INPUT/READ mode
	for (chan_count = start; chan_count <= end; chan_count++) {
		retval = comedi_dio_config(device, subdev, chan_count, COMEDI_INPUT);
		if(retval < 0) {
			comedi_perror("set input");
			return -1;
		}
		else {
			//printf("Channel %2d: set for INPUT  -- ", chan_count);
			// read from channel
			// this will read current channel buffer
			retval = comedi_data_read(device, subdev, chan_count, 0, 0, &data);
			if (retval <= 0) {
				comedi_perror("read channel");
				return -1;
			}
			//printf("Channel %2d reads: %d\n", chan_count, data);
		}
	}
	printf("\n");
	return (chan_count-start);	// return channels used
}


int write_dio(comedi_t *device, int subdev, char *bitmask, int start, int end) {

	int chan_count;
	int retval;
	int masklen;
	lsampl_t bit;

	// quality control 24-bit instruction
	masklen = strlen(bitmask);
	if (masklen != 24) {
		printf("bitmask quality: length\n");
		return -1;
	}

	// set each channel to OUTPUT/WRITE mode
	for (chan_count = start; chan_count <= end; chan_count++) {
		retval = comedi_dio_config(device, subdev, chan_count, COMEDI_OUTPUT);
		if(retval < 0) {
			comedi_perror("set output");
			return -1;
		}
	}

	// loop through channels from start to end range
	for (chan_count = start; chan_count <= end; chan_count++) {
		// flip bitmask for significance
		bit = ((lsampl_t)bitmask[((masklen-chan_count)-1)] - 48);
		if ((bit > maxdata) || (bit < mindata)) {
			printf("bitmask quality: value not digital");
			return -2;
		}
		// write data bit to channel buffer
		retval = comedi_data_write(device, subdev, chan_count, 0, 0, bit);
		if (retval != 1) {
			comedi_perror("write channel");
			return -1;
		}
		//printf("Channel %2d sent : %d\n", chan_count, bit);
	}
	//printf("\n");
	return (chan_count-start);	// return channels used
}


