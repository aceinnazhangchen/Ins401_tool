#ifndef _LC_GNSS_UPDATE_
#define _LC_GNSS_UPDATE_
#include <stdint.h>
#include "ins_interface_API.h"

int OdoObsUpdata(double systemtime);
int  ADDGNSSDATA(const GnssData msg);
int VirtualObsUpdata(double systemtime);


#endif