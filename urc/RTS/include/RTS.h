#ifndef _RTS_H
#define _RTS_H

#include <stdio.h>

FILE * RTS_OpenTSFile (const char * radar_name, const char * date,
		       const char * host_ext,   const char * scan_name);

#endif /* _RTS_H */
