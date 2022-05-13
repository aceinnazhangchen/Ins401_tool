#include <math.h>
#include <stdio.h> 
#include <string.h>
#include <stdlib.h>
#include "insoutmsg.h"
#include "lcstruct.h"
#include "earth.h"
#include "orientation.h"
#include "cmatrix.h"

#ifndef CRC32_POLYNOMIAL
#define CRC32_POLYNOMIAL 0xEDB88320L
#endif // !CRC32_POLYNOMIAL


#ifndef RAD2DEG
#define  RAD2DEG (57.295779513082320)
#endif // !RAD2DEG
#ifndef PI
#define PI (3.141592653589793)
#endif // !PI
#ifndef grav_WGS84
#define	grav_WGS84 9.7803267714e0
#endif

OdoSpeed odoStr = { 0 };
RawImu imuStr = { 0 };
Position positionStr = { 0 };
Velocity velocityStr = { 0 };
InsPositionVelocityAttitude inspvastr = { 0 };
INSPVAX inspvaxstr = { 0 };
CORRIMUN  corrimu = { 0 };
CORRIMUN* p_corrimu = &corrimu;
char ggaBuff_bt[120] = { 0 };
char pashrBuff_bt[120] = { 0 };
char rmcBuff[200] = { 0 };
ins_solution_t g_ins_sol = { 0 };
ins_solution_t *g_ptr_ins_sol = &g_ins_sol;
//定义各大洲经纬度范围
/*%Asia[60, 170; -10, 80];
%Europe[-10, 60; 36, 71];
%Oceania[110, 150; -55, -10];
%Africa[-20, 50; -35, 35];
%NorthAmerica[-170, -20; 7, 72];
%SouthAmerica[-80, -40; -54, 12];
%Antarctica[-180, 180; -90, -62];*/
const double areas[28] = { 60, 170, -10, 80 ,\
						  - 10, 60, 36, 71 , \
						  110,150, -55, -10,\
							- 20, 50, -35, 35 \
						   - 170, -20, 7, 72 ,\
							- 80, -40, -54, 12 , \
						   - 180, 180, -90, -62 };

int32_t get_IDCONTINENT_ID(double lat, double lon)
{
	int32_t ret = -1;
	if (lat > 90 || lon > 180 || lat < -90 || lon < -180)
	{
		ret = -1;
	}
	else
	{
		int i = 0;
		for ( i = 0; i < 7; i++)
		{
			double area[4] = { 0.0 };
			memcpy(area, areas + 4 * i, 4 * sizeof(double));
			double lon_min = area[0];
			double lon_max = area[1];
			double lat_min = area[2];
			double lat_max = area[3];
			if (lon < lon_max && lon > lon_min && lat < lat_max && lat > lat_min)
			{
				ret = i + 1;
				break;
			}
		}
		if (i == 7)
		{
			ret = 0;
		}
	}
	return ret;
}
typedef  int64_t  time_t;

typedef struct gtime_t
{        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	double sec;         /* fraction of second under 1 s */
} gtime_t;
#define MAXLEAPS    64                  /* max number of leap seconds table */
const static double gpst0[] = { 1980,1,6,0,0,0 }; /* gps time reference */
static double leaps[MAXLEAPS + 1][7] = { /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	{2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
	{2009,1,1,0,0,0,-15},
	{2006,1,1,0,0,0,-14},
	{1999,1,1,0,0,0,-13},
	{1997,7,1,0,0,0,-12},
	{1996,1,1,0,0,0,-11},
	{1994,7,1,0,0,0,-10},
	{1993,7,1,0,0,0, -9},
	{1992,7,1,0,0,0, -8},
	{1991,1,1,0,0,0, -7},
	{1990,1,1,0,0,0, -6},
	{1988,1,1,0,0,0, -5},
	{1985,7,1,0,0,0, -4},
	{1983,7,1,0,0,0, -3},
	{1982,7,1,0,0,0, -2},
	{1981,7,1,0,0,0, -1},
	{0}
};
static  gtime_t timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec += sec; tt = floor(t.sec); t.time += (int)tt; t.sec -= tt;
	return t;
}
static void time2epoch(gtime_t t, double *ep)
{
	const int mday[] = { /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};
	int days, sec, mon, day;

	/* leap year if year%4==0 in 1901-2099 */
	days = (int)(t.time / 86400);
	sec = (int)(t.time - (time_t)days * 86400);
	for (day = days % 1461, mon = 0; mon < 48; mon++) {
		if (day >= mday[mon]) day -= mday[mon]; else break;
	}
	ep[0] = 1970 + days / 1461 * 4 + mon / 12; ep[1] = mon % 12 + 1; ep[2] = day + 1;
	ep[3] = sec / 3600; ep[4] = sec % 3600 / 60; ep[5] = sec % 60 + t.sec;
}
static gtime_t epoch2time(const double *ep)
{
	const int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
	gtime_t time = { 0 };
	int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

	if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

	/* leap year if year%4==0 in 1901-2099 */
	days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
	sec = (int)floor(ep[5]);
	time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
	time.sec = ep[5] - sec;
	return time;
}
static gtime_t gpst2time(int week, double sec)
{
	gtime_t t = epoch2time(gpst0);

	if (sec < -1E9 || 1E9 < sec) sec = 0.0;
	t.time += 86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}
static gtime_t gpst2utc(gtime_t t)
{
	gtime_t tu;

	////for (i = 0; leaps[i][0] > 0; i++) {
	   //// tu = timeadd(t, leaps[i][6]);
	   //// if (timediff(tu, epoch2time(leaps[i])) >= 0.0) return tu;
	////}
	tu = timeadd(t, leaps[0][6]);
	return tu;
}



static void RealToArray(double value, char *a, unsigned char id, unsigned char dd) //id:integer digits¡ê?dd:decimal digits
{
	uint64_t temp1;
	uint64_t temp2;
	char i;
	if (id == 0)
	{
		if (abs(value) < 10)
		{
			id = 1;
		}
		else if (abs(value) < 100)
		{
			id = 2;
		}
		else if (abs(value) < 1000)
		{
			id = 3;
		}
		else if (abs(value) < 10000)
		{
			id = 4;
		}
		else
		{
			id = 8;
		}
	}

	if (value >= 0)
	{
		temp1 = value * pow(10, dd);
		temp2 = ((int)(value)) * pow(10, dd);
		temp2 = temp1 - temp2;
		temp1 = (int)value;
		for (i = id; i > 0; i--)
		{
			*(a + i - 1) = (int)temp1 % 10 + 0x30;
			temp1 = (int)(temp1 / 10);
		}
		*(a + id) = '.';
		for (i = id + dd; i > id; i--)
		{
			*(a + i) = temp2 % 10 + 0x30;
			temp2 = temp2 / 10;
		}
	}
	else
	{
		value = 0 - value;
		temp1 = value * pow(10, dd);
		temp2 = ((int)(value)) * pow(10, dd);
		temp2 = temp1 - temp2;
		temp1 = (int)value;
		*a = 0x2D;
		for (i = id; i > 0; i--)
		{
			*(a + i) = (int)temp1 % 10 + 0x30;
			temp1 = (int)(temp1 / 10);
		}
		*(a + id + 1) = '.';
		for (i = id + dd + 1; i > id + 1; i--)
		{
			*(a + i) = temp2 % 10 + 0x30;
			temp2 = temp2 / 10;
		}
	}
}

static char *itoa_user(int num, char *str, int radix)
{
	char index[] = "0123456789ABCDEF";
	unsigned unum;
	int i = 0, j, k;

	if (radix == 10 && num < 0)
	{
		unum = (unsigned)-num;
		str[i++] = '-';
	}
	else
		unum = (unsigned)num;
	do
	{
		str[i++] = index[unum % (unsigned)radix];
		unum /= radix;
	} while (unum);
	str[i] = '\0';

	if (str[0] == '-')
		k = 1;
	else
		k = 0;

	for (j = k; j <= (i - 1) / 2; j++)
	{
		char temp;
		temp = str[j];
		str[j] = str[i - 1 + k - j];
		str[i - 1 + k - j] = temp;
	}
	return str;
}

static void deg2dms(double deg, double *dms, int ndec)
{
	double sign = deg < 0.0 ? -1.0 : 1.0, a = fabs(deg);
	double unit = pow(0.1, ndec);
	dms[0] = floor(a);
	a = (a - dms[0]) * 60.0;
	dms[1] = floor(a);
	a = (a - dms[1]) * 60.0;
	dms[2] = floor(a / unit + 0.5) * unit;
	if (dms[2] >= 60.0)
	{
		dms[2] = 0.0;
		dms[1] += 1.0;
		if (dms[1] >= 60.0)
		{
			dms[1] = 0.0;
			dms[0] += 1.0;
		}
	}
	dms[0] *= sign;
}

static  int print_nmea_gga(double *ep, double *pos, int nsat, int type, double dop, double age, char *buff)
{
	double h, dms1[3], dms2[3];
	char *q, sum;
	//char *p = (char *)buff;
	char buf[20] = { 0 };

	if ((pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]) < 1.0)
	{
		strcpy(buff, "$GPGGA,,,,,,,,,,,,,,");
		for (q = (char *)buff + 1, sum = 0; *q; q++)
			sum ^= *q;
		strcat(buff, "*");
		memset(buf, 0, 20);
		itoa_user(sum, buf, 16);
		strcat(buff, buf);
		strcat(buff, "\r\n");

	}
	else
	{
		h = 0.0;
		deg2dms(fabs(pos[0]) * RAD2DEG, dms1, 7);
		deg2dms(fabs(pos[1]) * RAD2DEG, dms2, 7);

		strcpy(buff, "$GPGGA,");

		RealToArray(ep[3] * 10000 + ep[4] * 100 + ep[5] + 0.001, buf, 6, 2);
		strcat(buff, buf);
		strcat(buff, ",");

		memset(buf, 0, 20);
		RealToArray(dms1[0] * 100 + dms1[1] + dms1[2] / 60.0, buf, 4, 7);
		strcat(buff, buf);
		strcat(buff, pos[0] >= 0 ? ",N," : ",S,");

		memset(buf, 0, 20);
		RealToArray(dms2[0] * 100 + dms2[1] + dms2[2] / 60.0, buf, 5, 7);
		strcat(buff, buf);
		strcat(buff, pos[1] >= 0 ? ",E," : ",W,");

		memset(buf, 0, 20);
		// RealToArray(type,buf,1,0);
		itoa_user(type, buf, 10);
		strcat(buff, buf);
		strcat(buff, ",");

		memset(buf, 0, 20);
		RealToArray(nsat, buf, 2, 0);
		buf[2] = 0;
		strcat(buff, buf);
		strcat(buff, ",");

		memset(buf, 0, 20);
		RealToArray(dop, buf, 0, 1);
		strcat(buff, buf);
		strcat(buff, ",");

		memset(buf, 0, 20);
		RealToArray(pos[2] - h, buf, 0, 3);
		strcat(buff, buf);
		strcat(buff, ",M,");

		memset(buf, 0, 20);
		RealToArray(h, buf, 0, 3);
		strcat(buff, buf);
		strcat(buff, ",M,");

		memset(buf, 0, 20);
		RealToArray(age, buf, 0, 1);
		strcat(buff, buf);
		strcat(buff, ",");

		for (q = (char *)buff + 1, sum = 0; *q; q++)
			sum ^= *q; /* check-sum */

		strcat(buff, "*");
		memset(buf, 0, 20);
		itoa_user(sum, buf, 16);
		strcat(buff, buf);

		strcat(buff, "\r\n");
	}
	return strlen(buff);
}

static int print_nmea_pashr_bt(double *ep, char *buff)
{
	char *q, sum;
	char buf[20] = { 0 };

	// $PASHR,hhmmss.ss,HHH.HH,T,RRR.RR,PPP.PP,heave,rr.rrr,pp.ppp,hh.hhh,QF*CC<CR><LF>
	strcpy(buff, "$PASHR,");

	RealToArray(ep[3] * 10000 + ep[4] * 100 + ep[5] + 0.001, buf, 6, 2);
	strcat(buff, buf);
	strcat(buff, ",");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->azimuth, buf, 3, 2);
	strcat(buff, buf);
	strcat(buff, ",T,");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->pitch, buf, 3, 2);
	strcat(buff, buf);
	strcat(buff, ",");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->roll, buf, 3, 2);
	strcat(buff, buf);
	strcat(buff, ",+0.00,");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->azimuth_std, buf, 2, 3);
	strcat(buff, buf);
	strcat(buff, ",");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->pitch_std, buf, 2, 3);
	strcat(buff, buf);
	strcat(buff, ",");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->roll_std, buf, 2, 3);
	strcat(buff, buf);
	strcat(buff, ",");

	strcat(buff, "G");

	for (q = (char *)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */

	strcat(buff, "*");
	memset(buf, 0, 20);
	itoa_user(sum, buf, 16);
	strcat(buff, buf);

	strcat(buff, "\r\n");

	return strlen(buff);
}

extern int ins_print_nmea_pashr(double *ep, char *buff)
{
	char *q, sum;
	char buf[20] = { 0 };

	// $PASHR,hhmmss.ss,HHH.HH,T,RRR.RR,PPP.PP,heave,rr.rrr,pp.ppp,hh.hhh,QF*CC<CR><LF>
	strcpy(buff, "$PASHR,");

	RealToArray(ep[3] * 10000 + ep[4] * 100 + ep[5] + 0.001, buf, 6, 2);
	strcat(buff, buf);
	strcat(buff, ",");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->azimuth, buf, 3, 2);
	strcat(buff, buf);
	strcat(buff, ",T,");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->pitch, buf, 3, 2);
	strcat(buff, buf);
	strcat(buff, ",");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->roll, buf, 3, 2);
	strcat(buff, buf);
	strcat(buff, ",+0.00,");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->azimuth_std, buf, 2, 3);
	strcat(buff, buf);
	strcat(buff, ",");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->pitch_std, buf, 2, 3);
	strcat(buff, buf);
	strcat(buff, ",");

	memset(buf, 0, 20);
	RealToArray(g_ptr_ins_sol->roll_std, buf, 2, 3);
	strcat(buff, buf);
	strcat(buff, ",");

	if (g_ptr_ins_sol->pos_type == 4) {
		strcat(buff, "2,");
	}
	else if (g_ptr_ins_sol->pos_type != 0) {
		strcat(buff, "1,");
	}
	else {
		strcat(buff, "0,");
	}

	if (g_ptr_ins_sol->ins_status > INS_ALIGNING) {
		strcat(buff, "1");
	}
	else {
		strcat(buff, "0");
	}

	for (q = (char *)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */

	strcat(buff, "*");
	memset(buf, 0, 20);
	itoa_user(sum, buf, 16);
	strcat(buff, buf);

	strcat(buff, "\r\n");

	return strlen(buff);
}

extern int ins_print_nmea_ins(double *ep, char *buff)
{
	double dms1[3], dms2[3];
	char *p = buff, *q, sum;
	char str_ins_status[24] = { 0 };
	char str_position_type[16] = { 0 };
	double pos[3], vel[3], att[3];

	pos[0] = g_ptr_ins_sol->latitude;
	pos[1] = g_ptr_ins_sol->longitude;
	pos[2] = g_ptr_ins_sol->height;

	vel[0] = g_ptr_ins_sol->north_velocity;
	vel[1] = g_ptr_ins_sol->east_velocity;
	vel[2] = g_ptr_ins_sol->up_velocity;

	att[0] = g_ptr_ins_sol->roll;
	att[1] = g_ptr_ins_sol->pitch;
	att[2] = g_ptr_ins_sol->azimuth;

	switch (g_ptr_ins_sol->ins_status) {
	case INS_INACTIVE:
		strcpy(str_ins_status, "INS_INACTIVE");
		break;
	case INS_ALIGNING:
		strcpy(str_ins_status, "INS_ALIGNING");
		break;
	case INS_HIGH_VARIANCE:
		strcpy(str_ins_status, "INS_HIGH_VARIANCE");
		break;
	case INS_SOLUTION_GOOD:
		strcpy(str_ins_status, "INS_SOLUTION_GOOD");
		break;
	case INS_SOLUTION_FREE:
		strcpy(str_ins_status, "INS_SOLUTION_FREE");
		break;
	case INS_ALIGNMENT_COMPLETE:
		strcpy(str_ins_status, "INS_ALIGNMENT_COMPLETE");
		break;
	default:
		strcpy(str_ins_status, "NONE");
		break;
	}

	switch (g_ptr_ins_sol->pos_type) {
	case INS_NONE:
		strcpy(str_position_type, "INS_NONE");
		break;
	case INS_PSRSP:
		strcpy(str_position_type, "INS_PSRSP");
		break;
	case INS_PSRDIFF:
		strcpy(str_position_type, "INS_PSRDIFF");
		break;
	case INS_PROPAGATED:
		strcpy(str_position_type, "INS_PROPAGATED");
		break;
	case INS_RTKFIXED:
		strcpy(str_position_type, "INS_RTKFIXED");
		break;
	case INS_RTKFLOAT:
		strcpy(str_position_type, "INS_RTKFLOAT");
		break;
	default:
		strcpy(str_position_type, "NONE");
		break;
	}

	deg2dms(fabs(pos[0]), dms1, 7);
	deg2dms(fabs(pos[1]), dms2, 7);
	p += sprintf(p, "$GNINS,%02.0f%02.0f%05.2f,%s,%s,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
		ep[3], ep[4], ep[5], str_ins_status, str_position_type, dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W", pos[2], vel[0], vel[1], vel[2], att[0], att[1], att[2]);
	for (q = (char *)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return p - (char *)buff;
}

static int print_nmea_rmc(double *ep, double *pos, double heading, double speed, int fixID, char *buff)
{
	double dms1[3], dms2[3], amag = 0.0;
	char *p = buff, *q, sum, *emag = "E";

	if (fixID <= 0)
	{
		p += sprintf(p, "$GPRMC,,,,,,,,,,,,");
		for (q = (char *)buff + 1, sum = 0; *q; q++)
			sum ^= *q;
		p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
		return p - (char *)buff;
	}

	deg2dms(fabs(pos[0]) * RAD2DEG, dms1, 7);
	deg2dms(fabs(pos[1]) * RAD2DEG, dms2, 7);
	p += sprintf(p, "$GPRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W", speed / 0.514444444, heading,
		ep[2], ep[1], (int)ep[0] % 100, amag, emag,
		fixID == 4 || fixID == 5 ? "D" : "A");
	//    sol->stat==SOLQ_DGPS||sol->stat==SOLQ_FLOAT||sol->stat==SOLQ_FIX?"D":"A");
	for (q = (char *)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return p - (char *)buff;
}

extern int ins_print_nmea_vtg(char *buff)
{
	char *p = buff, *q, sum;

	double heading = g_ptr_ins_sol->azimuth;
	double speed = sqrt(g_ptr_ins_sol->east_velocity * g_ptr_ins_sol->east_velocity + g_ptr_ins_sol->north_velocity * g_ptr_ins_sol->north_velocity);

	p += sprintf(p, "$GPVTG,%06.2f,T,,M,%.2f,N,%.2f,K",
		heading, speed / 0.514444444, speed * 3.6);

	for (q = (char *)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return p - (char *)buff;
}

static int print_nmea_vtg(double heading, double speed, char *buff)
{
	char *p = buff, *q, sum;

	p += sprintf(p, "$GPVTG,%06.2f,T,,M,%.2f,N,%.2f,K",
		heading, speed / 0.514444444, speed*3.6);

	for (q = (char *)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return p - (char *)buff;
}

static unsigned long CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for (j = 8; j > 0; j--)
	{
		if (ulCRC & 1)
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
static unsigned long CalculateBlockCRC32(unsigned long ulCount,   /* Number of bytes in the data block */
	unsigned char *ucBuffer) /* Data block */
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while (ulCount-- != 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xff);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return (ulCRC);
}


int writeSAVEStruct(const GnssInsSystem* p_gnssInsSystem, SaveConfig* p_saveconfig)
{
	int ret = 0;
	int32_t n = p_gnssInsSystem->mKalmanStruct.n;

	p_saveconfig->gnss_week = (int16_t)p_gnssInsSystem->mImuData.week;
	p_saveconfig->gnss_second = (int32_t)round(floor(p_gnssInsSystem->mImuData.timestamp));
	p_saveconfig->solution_type = (int8_t)p_gnssInsSystem->ins_status;
	p_saveconfig->position_type = (int8_t)p_gnssInsSystem->ins_positin_type;
	p_saveconfig->latitude = (double)p_gnssInsSystem->mNav.lat * RAD2DEG;
	p_saveconfig->longitude = (double)p_gnssInsSystem->mNav.lon * RAD2DEG;
	p_saveconfig->height = (float)p_gnssInsSystem->mNav.height;
	p_saveconfig->north_velocity = (int16_t)round(p_gnssInsSystem->mNav.vn * 100);
	p_saveconfig->east_velocity = (int16_t)round(p_gnssInsSystem->mNav.ve * 100);
	p_saveconfig->down_velocity = (int16_t)round(p_gnssInsSystem->mNav.vd * 100);
	p_saveconfig->roll = (int16_t)round(p_gnssInsSystem->outNav.roll * RAD2DEG * 100);
	p_saveconfig->pitch = (int16_t)round(p_gnssInsSystem->outNav.pitch * RAD2DEG * 100);
	double heading = p_gnssInsSystem->outNav.heading;
	if (heading < -PI)// heading 0-360
	{
		heading = heading + 2 * PI;
	}
	else if (heading > PI)
	{
		heading = heading - 2 * PI;
	}
	p_saveconfig->azimuth = (int16_t)round(heading* RAD2DEG * 100);
	p_saveconfig->latitude_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[0]) * 100);
	p_saveconfig->longitude_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[n + 1]) * 100);
	p_saveconfig->altitude_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[2 * n + 2]) * 100);
	p_saveconfig->north_velocity_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[3 * n + 3]) * 100);
	p_saveconfig->east_velocity_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[4 * n + 4]) * 100);
	p_saveconfig->down_velocity_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[5 * n + 5]) * 100);
	p_saveconfig->roll_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[6 * n + 6])* RAD2DEG * 100);
	p_saveconfig->pitch_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[7 * n + 7])* RAD2DEG * 100);
	p_saveconfig->azimuth_std = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[8 * n + 8])* RAD2DEG * 100);

	p_saveconfig->gyro_bias_x = (int16_t)round(p_gnssInsSystem->mNav.sensorbias.bias_gyro_x * RAD2DEG * 100);
	p_saveconfig->gyro_bias_y = (int16_t)round(p_gnssInsSystem->mNav.sensorbias.bias_gyro_y * RAD2DEG * 100);
	p_saveconfig->gyro_bias_z = (int16_t)round(p_gnssInsSystem->mNav.sensorbias.bias_gyro_z * RAD2DEG * 100);
	p_saveconfig->acc_bias_x = (int16_t)round(p_gnssInsSystem->mNav.sensorbias.bias_acc_x * 100);
	p_saveconfig->acc_bias_y = (int16_t)round(p_gnssInsSystem->mNav.sensorbias.bias_acc_y * 100);
	p_saveconfig->acc_bias_z = (int16_t)round(p_gnssInsSystem->mNav.sensorbias.bias_acc_z * 100);
	int16_t m = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[9 * n + 9]) * RAD2DEG * 100);
	p_saveconfig->std_gyro_bias_x = m < 1 ? 1 : m;
	m = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[10 * n + 10]) * RAD2DEG * 100);
	p_saveconfig->std_gyro_bias_y = m < 1 ? 1 : m;
	m = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[11 * n + 11]) * RAD2DEG * 100);
	p_saveconfig->std_gyro_bias_z = m < 1 ? 1 : m;
	m = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[12 * n + 13]) * 100);
	p_saveconfig->std_acc_bias_x = m < 1 ? 1 : m;
	m = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[13 * n + 13]) * 100);
	p_saveconfig->std_acc_bias_y = m < 1 ? 1 : m;
	m = (int16_t)round(sqrt(p_gnssInsSystem->mKalmanStruct.P[14 * n + 14]) * 100);
	p_saveconfig->std_acc_bias_z = m < 1 ? 1 : m;
	p_saveconfig->static_type = (int8_t)1;  //comment_1
	p_saveconfig->reserve1 = 0.0;
	p_saveconfig->reserve2 = 0.0;
	ret = 1;
	return ret;
}

int writeSAVEMsg(const SaveConfig* p_saveconfig, SaveMsg* p_savemsg)
{
	int8_t ret = 0;
	long crc;

	p_savemsg->sync1 = 0xAA;
	p_savemsg->sync2 = 0x44;
	p_savemsg->sync3 = 0x12;
	p_savemsg->message_length = 106;
	memcpy(&p_savemsg->saveConfig, p_saveconfig, sizeof(SaveConfig));
	crc = CalculateBlockCRC32(p_savemsg->message_length, (unsigned char *)p_savemsg);
	memcpy((int8_t *)p_savemsg->crc, (int8_t *)&crc, 4);

	return ret;
}

int printasciisavebuf(const SaveConfig msaveconfig, char* buff)
{
	double dms1[3], dms2[3], amag = 0.0;
	char *p = buff, *q, sum, *emag = "E";

	//%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s
	p += sprintf(p, "$developer,%4d,%7d,%d,%d,%14.10f,%14.10f,%9.4f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%10.5f,%10.5f",
		msaveconfig.gnss_week, msaveconfig.gnss_second, msaveconfig.solution_type, msaveconfig.position_type,
		msaveconfig.latitude, msaveconfig.longitude, msaveconfig.height,
		msaveconfig.north_velocity, msaveconfig.east_velocity, msaveconfig.down_velocity,
		msaveconfig.roll, msaveconfig.pitch, msaveconfig.azimuth,
		msaveconfig.longitude_std, msaveconfig.longitude_std, msaveconfig.altitude_std,
		msaveconfig.north_velocity_std, msaveconfig.east_velocity_std, msaveconfig.down_velocity_std,
		msaveconfig.roll_std, msaveconfig.pitch_std, msaveconfig.azimuth_std,
		msaveconfig.gyro_bias_x, msaveconfig.gyro_bias_y, msaveconfig.gyro_bias_z,
		msaveconfig.acc_bias_x, msaveconfig.acc_bias_y, msaveconfig.acc_bias_z,
		msaveconfig.std_gyro_bias_x, msaveconfig.std_gyro_bias_y, msaveconfig.std_gyro_bias_z,
		msaveconfig.std_acc_bias_x, msaveconfig.std_acc_bias_y, msaveconfig.std_acc_bias_z,
		msaveconfig.static_type, msaveconfig.reserve1, msaveconfig.reserve2);
	for (q = (char *)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return p - (char *)buff;
}

int printsavebuf(const int8_t type, const GnssInsSystem* p_gnssInsSystem, char* buf)
{
	int ret = 0;

	SaveConfig msaveconfig;
	if (writeSAVEStruct(p_gnssInsSystem, &msaveconfig) == 1)
	{
		if (type == 0)
		{
			//print ascii buf
			printasciisavebuf(msaveconfig, buf);
			ret = 1;
		}
		else if (type == 1)
		{
			///print binary buf
			SaveMsg savemsg;
			writeSAVEMsg(&msaveconfig, &savemsg);
			memcpy(buf, &savemsg, sizeof(SaveMsg));
			ret = 1;
		}
		else
		{
			//error input type
			ret = -1;
		}
	}
	else
	{
		// error conditon
		ret = -3;
	}
	return ret;
}

int parse_saveconfig(const char* p_savemsg, SaveMsg*  pSaveMsg)
{
	int ret = 0;
	memcpy(pSaveMsg, p_savemsg, sizeof(SaveMsg));
	if (pSaveMsg->sync1 == 0xAA && pSaveMsg->sync2 == 0x44 && pSaveMsg->sync3 == 0x12)
	{
		long crc;
		crc = CalculateBlockCRC32(pSaveMsg->message_length, (unsigned char *)p_savemsg);
		uint8_t crc4[4] = { 0 };
		memcpy((int8_t *)crc4, (int8_t *)&crc, 4);
		if (crc4[0] == pSaveMsg->crc[0] && crc4[1] == pSaveMsg->crc[1] && crc4[2] == pSaveMsg->crc[2] && crc4[3] == pSaveMsg->crc[3])
		{
			ret = 1;
		}
	}
	return ret;
}

extern int writeRawImuMsg(const int gps_update, const int16_t week1, const uint32_t sec1, const int32_t week2, const uint32_t sec2, const ImuData* p_ImuData)
{
	long crc = 0;
	imuStr.header.sync1 = 0xAA;
	imuStr.header.sync2 = 0x44;
	imuStr.header.sync3 = 0x12;
	imuStr.header.header_length = 28;
	imuStr.header.message_id = 268;
	imuStr.header.message_type.format = BINARY;
	imuStr.header.message_type.response = ORIGINAL_MESSAGE;
	imuStr.header.port_address = 0;
	imuStr.header.message_length = 40; //
	imuStr.header.sequence = 0;
	imuStr.header.idle = 0;
	imuStr.header.time_status = 7;
	if (gps_update == 1)
	{
		imuStr.header.gps_week = week2;
		imuStr.header.gps_millisecs = sec2;
	}
	else
	{
		imuStr.header.gps_week = 0;
		imuStr.header.gps_millisecs = 0;
	}

	imuStr.header.status = 0;
	imuStr.header.Reserved = 0;
	imuStr.header.version = 0;

	imuStr.gps_week = week1;
	imuStr.gps_millisecs = sec1;
	//imuStr.imuStatus
	imuStr.z_acceleration = p_ImuData->Accz / grav_WGS84;
	imuStr.y_acceleration_neg = p_ImuData->Accy / grav_WGS84;
	imuStr.x_acceleration = p_ImuData->Accx / grav_WGS84;
	imuStr.z_gyro_rate = p_ImuData->Gyroz * RAD2DEG;
	imuStr.y_gyro_rate_neg = p_ImuData->Gyroy * RAD2DEG;
	imuStr.x_gyro_rate = p_ImuData->Gyrox * RAD2DEG;
	crc = CalculateBlockCRC32(imuStr.header.header_length + imuStr.header.message_length, (unsigned char *)&imuStr);
	memcpy((int8_t *)imuStr.crc, (int8_t *)&crc, 4);
	return 1;
}

static int writePositionMsg(const int week, const uint32_t gps_millisecs, const GnssData* p_GnssData)
{
	long crc;
	positionStr.header.sync1 = 0xAA;
	positionStr.header.sync2 = 0x44;
	positionStr.header.sync3 = 0x12;
	positionStr.header.header_length = 28;
	positionStr.header.message_id = 42;
	positionStr.header.message_type.format = BINARY;
	positionStr.header.message_type.response = ORIGINAL_MESSAGE;
	positionStr.header.port_address = 0;
	positionStr.header.message_length = 72; //
	positionStr.header.sequence = 0;
	positionStr.header.idle = 0;
	positionStr.header.time_status = 0;
	positionStr.header.gps_week = week;
	positionStr.header.gps_millisecs = gps_millisecs;
	positionStr.header.status = 0;
	positionStr.header.Reserved = 0;
	positionStr.header.version = 0;

	positionStr.solution_status = 0;
	positionStr.position_type = p_GnssData->gpsFixType;


	positionStr.latitude = p_GnssData->latitude * RAD2DEG;
	positionStr.longitude = p_GnssData->longitude * RAD2DEG;
	positionStr.height = p_GnssData->altitude;
	positionStr.undulation = 0;
	positionStr.datum_id = 0;
	positionStr.latitude_standard_deviation = p_GnssData->latitude_std;
	positionStr.longitude_standard_deviation = p_GnssData->longitude_std;
	positionStr.height_standard_deviation = p_GnssData->altitude_std;
	// positionStr.base_station_id[4];RAD2DEG
	positionStr.differential_age = 0;
	positionStr.solution_age = p_GnssData->sol_age;
	positionStr.number_of_satellites = 0;
	positionStr.number_of_satellites_in_solution = p_GnssData->Num_track;
	positionStr.num_gps_plus_glonass_l1 = 0;
	positionStr.num_gps_plus_glonass_l2 = 0;
	positionStr.reserved = 0;
	positionStr.extended_solution_status = 0;
	positionStr.reserved2 = 0;
	positionStr.signals_used_mask = 0;


	crc = CalculateBlockCRC32(positionStr.header.header_length + positionStr.header.message_length, (unsigned char *)&positionStr);
	memcpy((int8_t *)positionStr.crc, (int8_t *)&crc, 4);
	return 1;

}

static int writeVelocityMsg(const int week, const uint32_t gps_millisecs, const GnssData* p_GnssData)
{
	long crc;
	velocityStr.header.sync1 = 0xAA;
	velocityStr.header.sync2 = 0x44;
	velocityStr.header.sync3 = 0x12;
	velocityStr.header.header_length = 28;
	velocityStr.header.message_id = 99;
	velocityStr.header.message_type.format = BINARY;
	velocityStr.header.message_type.response = ORIGINAL_MESSAGE;
	velocityStr.header.port_address = 0;
	velocityStr.header.message_length = 44; //
	velocityStr.header.sequence = 0;
	velocityStr.header.idle = 0;
	velocityStr.header.time_status = 0;
	velocityStr.header.gps_week = week;
	velocityStr.header.gps_millisecs = gps_millisecs;
	velocityStr.header.status = 0;
	velocityStr.header.Reserved = 0;
	velocityStr.header.version = 0;

	velocityStr.latency = 0;
	velocityStr.age = 0;
	velocityStr.horizontal_speed = sqrt(p_GnssData->north_velocity * p_GnssData->north_velocity + p_GnssData->east_velocity * p_GnssData->east_velocity);
	velocityStr.track_over_ground = atan2(p_GnssData->east_velocity, p_GnssData->north_velocity) * RAD2DEG;
	velocityStr.vertical_speed = -p_GnssData->up_velocity;
	crc = CalculateBlockCRC32(velocityStr.header.header_length + velocityStr.header.message_length, (unsigned char *)&velocityStr);
	memcpy((int8_t *)velocityStr.crc, (int8_t *)&crc, 4);
	return 1;
}

extern int writeGnssRawMsg(const int week, const uint32_t gps_millisecs, const GnssData* p_GnssData)
{
	writePositionMsg(week, gps_millisecs, p_GnssData);
	writeVelocityMsg(week, gps_millisecs, p_GnssData);
	return 1;
}



extern int writeGGAMsg(int week, double time, const GnssInsSystem* p_gnssInsSystem, char* ggaBuff, char*rmcBuff, char* vtgBuff)
{
	double ep[6];
	double horizontal_speed;
	uint8_t type = 0;
	double blh[3];
	gtime_t gpstime = gpst2time(week, time);
	gtime_t utctime = gpst2utc(gpstime);
	time2epoch(utctime, ep);


	blh[0] = p_gnssInsSystem->outNav.lat;
	blh[1] = p_gnssInsSystem->outNav.lon;
	blh[2] = p_gnssInsSystem->outNav.height;

	// gga type need to be changed
	if (p_gnssInsSystem->mlc_STATUS == 4)
	{

		if (g_ptr_ins_sol->pos_type == 1 || g_ptr_ins_sol->pos_type == 4 || g_ptr_ins_sol->pos_type == 5) {
			type = g_ptr_ins_sol->pos_type + 5;
		}
		else {
			type = 7;
		}
	}
	else
	{
		type = g_ptr_ins_sol->pos_type;
	}

	print_nmea_gga(ep, blh, p_gnssInsSystem->mGnssData.Num_track, type, p_gnssInsSystem->mGnssData.HDOP, p_gnssInsSystem->mGnssData.sol_age, ggaBuff);
	
	print_nmea_pashr_bt(ep, pashrBuff_bt);

	horizontal_speed = sqrt(p_gnssInsSystem->outNav.vn * p_gnssInsSystem->outNav.vn + p_gnssInsSystem->outNav.ve * p_gnssInsSystem->outNav.ve);

	print_nmea_rmc(ep, blh, p_gnssInsSystem->outNav.heading * RAD2DEG, horizontal_speed, g_ptr_ins_sol->pos_type, rmcBuff);

	print_nmea_vtg(p_gnssInsSystem->outNav.heading * RAD2DEG, horizontal_speed, vtgBuff);


	//outnmea_gga(ggaBuff_bt, ep, 2, blh, p_gnssInsSystem->mGnssData.numSatellites, p_gnssInsSystem->mGnssData.HDOP, p_gnssInsSystem->mGnssData.sol_age);

	return 1;
}

extern int writeOdoDataMsg(const OdoData* p_OdoData)
{
	long crc = 0;
	odoStr.header.sync1 = 0xAA;
	odoStr.header.sync2 = 0x44;
	odoStr.header.sync3 = 0x12;
	odoStr.header.header_length = 28;
	odoStr.header.message_id = 177;
	odoStr.header.message_type.format = BINARY;
	odoStr.header.message_type.response = ORIGINAL_MESSAGE;
	odoStr.header.port_address = 0;
	odoStr.header.message_length = 30; //16
	odoStr.header.sequence = 0;
	odoStr.header.idle = 0;
	odoStr.header.time_status = 7;
	odoStr.header.gps_week = p_OdoData->week;
	odoStr.header.gps_millisecs = (int)(p_OdoData->reserve * 1000);

	odoStr.header.status = 0;
	odoStr.header.Reserved = 0;
	odoStr.header.version = 0;

	odoStr.gps_millisecs = (int)(p_OdoData->timestamp * 1000);
	odoStr.speed = p_OdoData->vehicle_speed;
	odoStr.mode = p_OdoData->mode;
	odoStr.week = p_OdoData->week;
	odoStr.fwd = p_OdoData->fwd;
	odoStr.wheel_tick = p_OdoData->wheel_tick;
	crc = CalculateBlockCRC32(odoStr.header.header_length + odoStr.header.message_length, (unsigned char *)&odoStr);
	memcpy((int8_t *)odoStr.crc, (int8_t *)&crc, 4);
	return 1;
}
extern int writeINSPVAXMsg(int32_t week, uint32_t itow, const GnssInsSystem* p_gnssInsSystem)
{
	long crc;
	int32_t n = p_gnssInsSystem->mKalmanStruct.n;
	inspvaxstr.header.sync1 = 0xAA;
	inspvaxstr.header.sync2 = 0x44;
	inspvaxstr.header.sync3 = 0x12;
	inspvaxstr.header.header_length = 28;
	inspvaxstr.header.message_id = 1465;
	inspvaxstr.header.message_type.format = BINARY;
	inspvaxstr.header.message_type.response = ORIGINAL_MESSAGE;
	inspvaxstr.header.port_address = 0;
	inspvaxstr.header.message_length = 126; //
	inspvaxstr.header.sequence = 0;
	inspvaxstr.header.idle = 0;
	inspvaxstr.header.time_status = 0;
	inspvaxstr.header.gps_week = week;
	inspvaxstr.header.gps_millisecs = itow;
	inspvaxstr.ins_status = (int)p_gnssInsSystem->ins_status;
	inspvaxstr.pos_type = (int)p_gnssInsSystem->ins_positin_type;
	inspvaxstr.latitude = p_gnssInsSystem->outNav.lat * RAD2DEG;
	inspvaxstr.longitude = p_gnssInsSystem->outNav.lon * RAD2DEG;
	inspvaxstr.height = p_gnssInsSystem->outNav.height;
	inspvaxstr.north_velocity = p_gnssInsSystem->outNav.vn;
	inspvaxstr.east_velocity = p_gnssInsSystem->outNav.ve;
	inspvaxstr.up_velocity = -p_gnssInsSystem->outNav.vd;
	inspvaxstr.roll = p_gnssInsSystem->outNav.roll * RAD2DEG;
	inspvaxstr.pitch = p_gnssInsSystem->outNav.pitch * RAD2DEG;
	inspvaxstr.azimuth = p_gnssInsSystem->outNav.heading * RAD2DEG;
	inspvaxstr.latitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[0]);
	inspvaxstr.longitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[n + 1]);
	inspvaxstr.altitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[2 * n + 2]);
	inspvaxstr.north_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[3 * n + 3]);
	inspvaxstr.east_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[4 * n + 4]);
	inspvaxstr.up_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[5 * n + 5]);
	inspvaxstr.roll_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[6 * n + 6])* RAD2DEG;
	inspvaxstr.pitch_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[7 * n + 7])* RAD2DEG;
	inspvaxstr.azimuth_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[8 * n + 8])* RAD2DEG;
	inspvaxstr.Ext_sol_stat = 0;
	inspvaxstr.time_since_update = (int16_t)p_gnssInsSystem->GNSSLOSETIME1;
	crc = CalculateBlockCRC32(inspvaxstr.header.header_length + inspvaxstr.header.message_length, (unsigned char *)&inspvaxstr);
	memcpy((int8_t *)inspvaxstr.crc, (int8_t *)&crc, 4);
	return 1;
}

extern int writeINSPVAMsg(int32_t week, uint32_t itow, const GnssInsSystem* p_gnssInsSystem)
{
	long crc;
	inspvastr.header.sync1 = 0xAA;
	inspvastr.header.sync2 = 0x44;
	inspvastr.header.sync3 = 0x12;
	inspvastr.header.header_length = 28;
	inspvastr.header.message_id = 507;
	inspvastr.header.message_type.format = BINARY;
	inspvastr.header.message_type.response = ORIGINAL_MESSAGE;
	inspvastr.header.port_address = 0;
	inspvastr.header.message_length = 88; //
	inspvastr.header.sequence = 0;
	inspvastr.header.idle = 0;
	inspvastr.header.time_status = 0;
	inspvastr.header.gps_week = week;
	inspvastr.header.gps_millisecs = itow;
	inspvastr.gps_week = week;
	inspvastr.gps_millisecs = itow;
	inspvastr.latitude = p_gnssInsSystem->outNav.lat * RAD2DEG;
	inspvastr.longitude = p_gnssInsSystem->outNav.lon * RAD2DEG;
	inspvastr.height = p_gnssInsSystem->outNav.height;
	inspvastr.north_velocity = p_gnssInsSystem->outNav.vn;
	inspvastr.east_velocity = p_gnssInsSystem->outNav.ve;
	inspvastr.up_velocity = -p_gnssInsSystem->outNav.vd;
	inspvastr.roll = p_gnssInsSystem->outNav.roll * RAD2DEG;
	inspvastr.pitch = p_gnssInsSystem->outNav.pitch * RAD2DEG;
	inspvastr.azimuth = p_gnssInsSystem->outNav.heading * RAD2DEG;
	inspvastr.status = p_gnssInsSystem->mlc_STATUS;
	crc = CalculateBlockCRC32(inspvastr.header.header_length + inspvastr.header.message_length, (unsigned char *)&inspvastr);
	memcpy((int8_t *)inspvastr.crc, (int8_t *)&crc, 4);
	return 1;
}

/*output MISRVB on time*/
int writeMISALIGNMsg(int8_t mode, double * origin_cbv, int8_t* MisAlignmentAiax, double* misAlignment, MISRVB* misrvb)
{
	int ret = 0;
	//double eular[3] ={ 0.0};
	double C_bv[3][3] = { {0.0} };
	double C1_temp1[3][3] = { {0.0} }, C1_temp2[3][3] = { {0.0} };
	double C_tb[3][3] = { {0.0,0.0,0.0}, {0.0,0.0,0.0},{0.0,0.0,0.0} }, C_vt[3][3] = { {0.0,0.0,0.0}, {0.0,0.0,0.0},{0.0,0.0,0.0} };

	if (mode == 1 || mode == 2 || mode == 3 || mode == 4) //curent est good    ///4 est but not use
	{
		C1_temp1[0][abs(MisAlignmentAiax[0]) - 1] = abs(MisAlignmentAiax[0]) / MisAlignmentAiax[0];
		C1_temp1[1][abs(MisAlignmentAiax[1]) - 1] = abs(MisAlignmentAiax[1]) / MisAlignmentAiax[1];
		C1_temp1[2][abs(MisAlignmentAiax[2]) - 1] = abs(MisAlignmentAiax[2]) / MisAlignmentAiax[2];

		euler2dcm(misAlignment, C1_temp2);
		//	MatrixMutiply(*C1_temp2, *C1_temp1, 3, 3, 3, *C_bv);?
		MatrixMutiply(*C1_temp2, *C1_temp1, 3, 3, 3, *C_bv);
		MatrixTranspose(&C_bv[0][0], 3, 3, *C_vt);

		/*如果存在RBV，将RBV再放上去*/
		MatrixTranspose(origin_cbv, 3, 3, *C_tb);
		MatrixMutiply(*C_tb, *C_vt, 3, 3, 3, *(misrvb->CVB));

		dcm2euler(misrvb->CVB, misrvb->RVB);
		if (misrvb->RVB[2] < -PI)// heading 0-360
		{
			misrvb->RVB[2] += 2 * PI ;
		}
		else if (misrvb->RVB[2] >  PI) misrvb->RVB[2] -= 2 * PI;
		misrvb->flag = mode;

		misrvb->output_flag = 1;
	}
	ret = 1;
	return ret;
}

void _copy_ins_result(const int32_t week, const double timestamp, const GnssInsSystem* p_gnssInsSystem, ins_solution_t* p_ins_sol)
{
	int32_t n = p_gnssInsSystem->mKalmanStruct.n;
	p_ins_sol->gps_week = week;
	p_ins_sol->gps_millisecs = (uint32_t)((timestamp + 0.0001) * 1000);
	p_ins_sol->ins_status = (uint32_t)p_gnssInsSystem->ins_status;
	p_ins_sol->pos_type = (uint32_t)p_gnssInsSystem->ins_positin_type;
	p_ins_sol->latitude = p_gnssInsSystem->outNav.lat * RAD2DEG;
	p_ins_sol->longitude = p_gnssInsSystem->outNav.lon * RAD2DEG;
	p_ins_sol->height = p_gnssInsSystem->outNav.height;
	p_ins_sol->north_velocity = p_gnssInsSystem->outNav.vn;
	p_ins_sol->east_velocity = p_gnssInsSystem->outNav.ve;
	p_ins_sol->up_velocity = -p_gnssInsSystem->outNav.vd;
	p_ins_sol->roll = p_gnssInsSystem->outNav.roll * RAD2DEG;
	p_ins_sol->pitch = p_gnssInsSystem->outNav.pitch * RAD2DEG;
	p_ins_sol->azimuth = p_gnssInsSystem->outNav.heading * RAD2DEG;
	p_ins_sol->latitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[0]);
	p_ins_sol->longitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[n + 1]);
	p_ins_sol->altitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[2 * n + 2]);
	p_ins_sol->north_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[3 * n + 3]);
	p_ins_sol->east_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[4 * n + 4]);
	p_ins_sol->up_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[5 * n + 5]);
	p_ins_sol->roll_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[6 * n + 6])* RAD2DEG;
	p_ins_sol->pitch_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[7 * n + 7])* RAD2DEG;
	p_ins_sol->azimuth_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[8 * n + 8])* RAD2DEG;
	p_ins_sol->time_since_update = (uint16_t)p_gnssInsSystem->GNSSLOSETIME1;
	p_ins_sol->mlc_status = p_gnssInsSystem->mlc_STATUS;
	if (p_ins_sol->mlc_status == INS_FUSING || p_gnssInsSystem->mGnssData.gpsFixType != INS_NONE)
	{
		p_ins_sol->id_continent = get_IDCONTINENT_ID(p_ins_sol->latitude , p_ins_sol->longitude);
	}
	else
	{
		p_ins_sol->id_continent = -2;  //未进入GNSS解状态
	}
	p_ins_sol->isUseOdo = (int8_t)p_gnssInsSystem->isUseOdo & p_gnssInsSystem->isOdoInterfaceGOOD;

	p_ins_sol->Ext_sol_stat = p_gnssInsSystem->Ext_sol_stat;
	p_ins_sol->time_since_update = p_gnssInsSystem->time_since_update;
}

extern ins_solution_t* getinssolutionmsg()
{
	return g_ptr_ins_sol;
}

int8_t ins_set_rawimuoutput(const int32_t cur_week, const double timestamp, const ImuData* p_imu_data, const imu_interface_status* p_odometer_interface_status, ImuData* p_rawimu)
{
	int8_t ret = 0;
	/*拷贝*/
	p_rawimu->week = cur_week;
	p_rawimu->timestamp = timestamp;
	p_rawimu->Gyrox = p_imu_data->Gyrox;
	p_rawimu->Gyroy = p_imu_data->Gyroy;
	p_rawimu->Gyroz = p_imu_data->Gyroz;
	p_rawimu->Accx = p_imu_data->Accx;
	p_rawimu->Accy = p_imu_data->Accy;
	p_rawimu->Accz = p_imu_data->Accz;
	/*结合接受到的数据 imu_interface_status*/
	p_rawimu->IMUstatus = p_imu_data->IMUstatus;
	return ret;
}

int8_t ins_set_corrimuoutput(const int32_t week, const double timestamp, const GnssInsSystem* p_gnssInsSystem, CORRIMUN* p_corrimu)
{
	int8_t ret = 0;
	/*拷贝*/
	p_corrimu->week = week;
	p_corrimu->timestamp = timestamp;
	p_corrimu->RollRate = p_gnssInsSystem->outNav.w_b[0] * RAD2DEG;
	p_corrimu->PitchRate = p_gnssInsSystem->outNav.w_b[1] * RAD2DEG;
	p_corrimu->YawRate = p_gnssInsSystem->outNav.w_b[2] * RAD2DEG;
	p_corrimu->LateralAcc = p_gnssInsSystem->outNav.a_b[0];
	p_corrimu->LongitudinalAcc = p_gnssInsSystem->outNav.a_b[1];
	p_corrimu->VerticalAcc = p_gnssInsSystem->outNav.a_b[2];
	p_corrimu->CORIMUstatus = p_gnssInsSystem->ins_status;
	ret = 1;
	return ret;
}

int8_t ins_set_inspvaxoutput(const int32_t week, const double timestamp, const GnssInsSystem* p_gnssInsSystem, INSPVAXN* g_ins_sol)
{
	int8_t ret = 0;
	int32_t n = p_gnssInsSystem->mKalmanStruct.n;

	g_ins_sol->gps_week = week;
	g_ins_sol->gps_millisecs = (uint32_t)((timestamp + 0.0001) * 1000);
	g_ins_sol->ins_status = (uint32_t)p_gnssInsSystem->ins_status;
	g_ins_sol->pos_type = (uint32_t)p_gnssInsSystem->ins_positin_type;
	g_ins_sol->latitude = p_gnssInsSystem->outNav.lat * RAD2DEG;
	g_ins_sol->longitude = p_gnssInsSystem->outNav.lon * RAD2DEG;
	g_ins_sol->height = p_gnssInsSystem->outNav.height;
	g_ins_sol->north_velocity = p_gnssInsSystem->outNav.vn;
	g_ins_sol->east_velocity = p_gnssInsSystem->outNav.ve;
	g_ins_sol->up_velocity = -p_gnssInsSystem->outNav.vd;
	g_ins_sol->roll = p_gnssInsSystem->outNav.roll * RAD2DEG;
	g_ins_sol->pitch = p_gnssInsSystem->outNav.pitch * RAD2DEG;
	g_ins_sol->azimuth = p_gnssInsSystem->outNav.heading * RAD2DEG;
	g_ins_sol->latitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[0]);
	g_ins_sol->longitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[n + 1]);
	g_ins_sol->altitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[2 * n + 2]);
	g_ins_sol->north_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[3 * n + 3]);
	g_ins_sol->east_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[4 * n + 4]);
	g_ins_sol->up_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[5 * n + 5]);
	g_ins_sol->roll_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[6 * n + 6])* RAD2DEG;
	g_ins_sol->pitch_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[7 * n + 7])* RAD2DEG;
	g_ins_sol->azimuth_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[8 * n + 8])* RAD2DEG;
	g_ins_sol->Ext_sol_stat = p_gnssInsSystem->Ext_sol_stat;
	g_ins_sol->time_since_update = p_gnssInsSystem->time_since_update;
	g_ins_sol->mlc_status = p_gnssInsSystem->mlc_STATUS;
	g_ins_sol->isUseOdo = (int8_t)p_gnssInsSystem->isUseOdo & p_gnssInsSystem->isOdoInterfaceGOOD;
	g_ins_sol->odoscale = (float)p_gnssInsSystem->mNav.Odo_scale;
	g_ins_sol->curIsUseZupt = (int8_t)p_gnssInsSystem->CurIsUseZupt;//p_gnssInsSystem->CurIsUseZupt;

	if (g_ins_sol->mlc_status == INS_FUSING || p_gnssInsSystem->mGnssData.gpsFixType != INS_NONE)
	{
		g_ins_sol->id_continent = get_IDCONTINENT_ID(g_ins_sol->latitude, g_ins_sol->longitude);
	}
	else
	{
		g_ins_sol->id_continent = -2;  //未进入GNSS解状态
	}
	return ret;
}

int8_t ins_set_biasoutput(const int32_t week, const double timestamp, const GnssInsSystem* p_gnssInsSystem, INSBIASN* insbias)
{
	int8_t ret = 0;
	int32_t n = p_gnssInsSystem->mKalmanStruct.n;
	insbias->week = week;
	insbias->timestamp = timestamp;
	insbias->ins_status = (uint32_t)p_gnssInsSystem->ins_status;
	insbias->pos_type = (uint32_t)p_gnssInsSystem->ins_positin_type;
	insbias->gyro_bias_x = p_gnssInsSystem->mNav.sensorbias.bias_gyro_x * RAD2DEG;
	insbias->gyro_bias_y = p_gnssInsSystem->mNav.sensorbias.bias_gyro_y * RAD2DEG;
	insbias->gyro_bias_z = p_gnssInsSystem->mNav.sensorbias.bias_gyro_z * RAD2DEG;
	insbias->acc_bias_x = p_gnssInsSystem->mNav.sensorbias.bias_acc_x;
	insbias->acc_bias_y = p_gnssInsSystem->mNav.sensorbias.bias_acc_y;
	insbias->acc_bias_z = p_gnssInsSystem->mNav.sensorbias.bias_acc_z;


	insbias->std_gyro_bias_x = sqrt(p_gnssInsSystem->mKalmanStruct.P[9 * n + 9]) * RAD2DEG;
	insbias->std_gyro_bias_y = sqrt(p_gnssInsSystem->mKalmanStruct.P[10 * n + 10]) * RAD2DEG;
	insbias->std_gyro_bias_z = sqrt(p_gnssInsSystem->mKalmanStruct.P[11 * n + 11]) * RAD2DEG;
	insbias->std_acc_bias_x = sqrt(p_gnssInsSystem->mKalmanStruct.P[12 * n + 12]);
	insbias->std_acc_bias_y = sqrt(p_gnssInsSystem->mKalmanStruct.P[13 * n + 13]);
	insbias->std_acc_bias_z = sqrt(p_gnssInsSystem->mKalmanStruct.P[14 * n + 14]);
	ret = 1;
	return ret;
}

int8_t ins_set_rawgnssoutput(const int32_t week, const double timestamp, const GnssData* p_gnss_data, const GnssData_interface_status* p_gnss_interface_status, GnssData* p_out_gnss_data)
{
	int8_t ret = 0;
	/*拷贝*/
	memcpy(p_out_gnss_data, p_gnss_data, sizeof(GnssData));
	p_out_gnss_data->week = week;
	p_out_gnss_data->timestamp = timestamp;
	return ret;
}

int8_t ins_set_rawodooutput(const int32_t week, const double timestamp, const OdoData* p_odo_data, const odometer_interface_status* p_odometer_interface_status, OdoData* p_out_odo_data)
{
	int8_t ret = 0;
	/*拷贝*/
	memcpy(p_out_odo_data, p_odo_data, sizeof(OdoData));
	p_out_odo_data->week = week;
	p_out_odo_data->timestamp = timestamp;
	ret = 1;
	return ret;
}
