#include "ins_interface_API.h"
#include <math.h>
#include <stdlib.h>
#include "lcsystem.h"
#include "lcgnssupdate.h"
#include "lcinsupdate.h"
#include "loosecoupleset.h"
#include "insoutmsg.h"
#include "version.h"


#ifndef SCALEFACTOR_ODO 
#define SCALEFACTOR_ODO (0.97 * 10 * 0.640 * 3.1415926/2000)
#endif
#ifndef PI 
#define PI ( 3.1415926)
#endif
#ifndef DEG2RAD
#define  DEG2RAD (PI/180)
#endif // !DEG2RAD
#ifndef RAD2DEG
#define  RAD2DEG (57.295779513082320)
#endif // !RAD2DEG
#ifndef SECONDS_IN_WEEK
#define  SECONDS_IN_WEEK (7*24*60*60)
#endif // !SECONDS_IN_WEEK

/*输出控制器，由于输出在IMU线程，set统一放在IMU线程，故*/

static OutputMsg g_0utputMsg;
static OutputControlOntime m_OutputControlOntime_outputs;
static globalvariable_data m_globalvariable_data =
{
	.g_gnss_start_week = -1,
	.m_imu_msg.use_flag = 0,
	.m_gnss_msg.use_flag = 0,
	.m_odo_msg.use_flag = 0,
};

static int8_t set_gnss_start_week(int32_t week)
{
	int8_t ret = 0;
	if (week > 1024 && week < 3072)
	{
		if (m_globalvariable_data.g_gnss_start_week == -1)
		{
			m_globalvariable_data.g_gnss_start_week = week;
			ret = 1;
		}
		else
		{
			ret = 2;
		}
	}
	return ret;
}

static int8_t is_gnss_start_week_valid(void)
{
	int8_t ret = 0;
	if (m_globalvariable_data.g_gnss_start_week == -1) {
		ret = 0;
	}
	else {
		ret = 1;
	}
	return ret;
}

static int32_t get_gnss_start_week(void)
{
	return m_globalvariable_data.g_gnss_start_week;
}

static int8_t ins_get_lcsetisuseodo_API(void)
{
	return p_prcopt.isUseOdo;
}

static int8_t convert_systime2caltime(const int32_t week, const double time, int32_t* p_week, double *p_time)
{
	int8_t ret = 0;
	if (week > 1024 && week < 3072)
	{
		if (is_gnss_start_week_valid() == 1)
		{

			//lock_on 4 系统时间
			*p_time += (week - get_gnss_start_week()) * SECONDS_IN_WEEK;
			*p_week = get_gnss_start_week();
			ret = 1;
		}
		else
		{
			if (1 == set_gnss_start_week(week))
			{
				*p_time += (week - get_gnss_start_week()) * SECONDS_IN_WEEK;
				*p_week = get_gnss_start_week();
				ret = 1;
			}
			else
			{
				*p_week = week;
				*p_time = time;
			}
		}
	}
	else
	{
		*p_week = week;
		*p_time = time;
	}
	return ret;
}

extern int8_t ins_set_lcsettingMODE_API(const int32_t type)
{
	int8_t ret = 0;
	if (type == 1001)
	{
		p_prcopt.isOnlineMisAlignmentEst = 1;
		p_prcopt.insinitfromprev = 0;
		ret = 1;
	}
	else if (type == 1002)
	{
		p_prcopt.isUseOdo = 1;
		p_prcopt.isInsFreInit = 0;
		ret = 1;
	}
	else if (type == 1012)
	{
		p_prcopt.isUseOdo = 0;
		p_prcopt.isInsFreInit = 0;
		ret = 1;
	}
	else if (type == 3004)
	{
		p_prcopt.Initialcondition = 4;
		ret = 1;
	}
	else if (type == 4001)
	{
		p_prcopt.initfmpre = 1;
		p_prcopt.savesavetype = 0;
		ret = 1;
	}
	else if (type == 4002)
	{
		p_prcopt.initfmpre = 1;
		p_prcopt.savesavetype = 1;
	}

	return ret;
}

extern int8_t ins_init_OUTMESSAGE_API(const uint32_t type)
{
	int8_t ret = 0;
	if (type > 200000)
	{
		ret = -1;
	}
	else
	{
		p_prcopt.OutputMessageType = type;
		ret = 1;
	}
	return ret;
}


extern int8_t ins_set_lcsettingDataRata_API(const int32_t type)
{
	int8_t ret = 0;
	if (type == 2003)
	{
		p_prcopt.imuDataRate = 100;
		ret = 1;
	}
	return ret;
}

extern int8_t ins_get_inslibdescrption_API(char* str, int32_t length)
{
	int8_t ret = 0;
	ret = get_versionstr(str, length);
	return ret;
};

extern int8_t ins_get_savebuf_API(const int8_t type, char* buf)
{
	int8_t ret = 0;
	if (printsavebuf(type, &mGnssInsSystem, buf) == 1)
	{
		ret = 1;
	}
	return ret;
}

void  convertSaveconfig2devset(const SaveConfig* p_saveconfig, developerSetting* p_developerSetting)
{
	p_developerSetting->insinitfromprev = 1;
	//需要进行转化，得到自己想要的格式，这里统一转化为double，方便一点点，后续会按照结构体存放
	p_developerSetting->initpar[0] = (double)p_saveconfig->gnss_week;  //GNSS week()(int16_t) - > GNSS week()(float64_t)  
	p_developerSetting->initpar[1] = (double)p_saveconfig->gnss_second;  //GNSS time of week(second)(int32_t) ->GNSS time of week(second)(float64_t)
	p_developerSetting->initpar[2] = p_saveconfig->solution_type;  //solution_type()(int8_t)  -> solution_type()(float64_t)
	p_developerSetting->initpar[3] = p_saveconfig->position_type;  //position_type()(int8_t)  -> position_type()(float64_t)
	p_developerSetting->initpar[4] = p_saveconfig->latitude * DEG2RAD;  //latitude(deg)(float64_t) -> latitude(rad)(float64_t)
	p_developerSetting->initpar[5] = p_saveconfig->longitude * DEG2RAD;  //lontitde(deg)(float64_t) -> lontitde(rad)(float64_t)
	p_developerSetting->initpar[6] = p_saveconfig->height;  //height(m)(float32_t) -> height(m)(float64_t)
	p_developerSetting->initpar[7] = (double)p_saveconfig->north_velocity / 100.0;  //v_n(cm/s)(in16_t) -> v_n(m/s)(float64_t)
	p_developerSetting->initpar[8] = (double)p_saveconfig->east_velocity / 100.0;  //v_e(cm/s)(in16_t) -> v_n(m/s)(float64_t)
	p_developerSetting->initpar[9] = (double)p_saveconfig->down_velocity / 100.0;  //v_d(cm/s)(in16_t) -> v_n(m/s)(float64_t)
	p_developerSetting->initpar[10] = (double)p_saveconfig->roll / 100.0 * DEG2RAD; //roll(10 ^ -2deg)(int16_t)-> roll(rad)(float64_t)
	p_developerSetting->initpar[11] = (double)p_saveconfig->pitch / 100.0 * DEG2RAD; //pitch(10 ^ -2deg)(int16_t)-> pitch(rad)(float64_t)
	p_developerSetting->initpar[12] = (double)p_saveconfig->azimuth / 100.0 * DEG2RAD; //yaw(10 ^ -2deg)(int16_t)-> yaw(rad)(float64_t)
	p_developerSetting->initpar[13] = (double)p_saveconfig->latitude_std / 100.0; //std_lat(cm)(int16_t) -> std_lat(m)(float64_t)
	p_developerSetting->initpar[14] = (double)p_saveconfig->longitude_std / 100.0; //std_lat(cm)(int16_t) -> std_lon(m)(float64_t)
	p_developerSetting->initpar[15] = (double)p_saveconfig->altitude_std / 100.0; //std_lat(cm)(int16_t) -> std_height(m)(float64_t)
	p_developerSetting->initpar[16] = (double)p_saveconfig->north_velocity_std / 100.0; //std_v_n(cm/s)(int16_t) -> std_v_n(m/s)(float64_t)
	p_developerSetting->initpar[17] = (double)p_saveconfig->east_velocity_std / 100.0; //std_v_e(cm/s)(int16_t) -> std_v_e(m/s)(float64_t)
	p_developerSetting->initpar[18] = (double)p_saveconfig->down_velocity_std / 100.0; //std_v_d(cm/s)(int16_t) -> std_v_d(m/s)(float64_t)
	p_developerSetting->initpar[19] = (double)p_saveconfig->roll_std / 100.0 * DEG2RAD; //std_roll(10 ^ -2deg)(int16_t)-> std_roll(rad)(float64_t)
	p_developerSetting->initpar[20] = (double)p_saveconfig->pitch_std / 100.0 * DEG2RAD; //std_pitch(10 ^ -2deg)(int16_t)-> std_pitch(rad)(float64_t)
	p_developerSetting->initpar[21] = (double)p_saveconfig->azimuth_std / 100.0 * DEG2RAD; //std_yaw(10 ^ -2deg)(int16_t)-> std_yaw(rad)(float64_t)
	p_developerSetting->initpar[22] = (double)p_saveconfig->gyro_bias_x / 100.0 * DEG2RAD; //gyro_bias_x(10 ^ -2deg/s)(int16_t)-> gyro_bias_x(rad/s)(float64_t)
	p_developerSetting->initpar[23] = (double)p_saveconfig->gyro_bias_y / 100.0 * DEG2RAD; //gyro_bias_y(10 ^ -2deg/s)(int16_t)-> gyro_bias_y(rad/s)(float64_t)
	p_developerSetting->initpar[24] = (double)p_saveconfig->gyro_bias_z / 100.0 * DEG2RAD; //gyro_bias_z(10 ^ -2deg/s)(int16_t)-> gyro_bias_z(rad/s)(float64_t)
	p_developerSetting->initpar[25] = (double)p_saveconfig->acc_bias_x / 100.0; //acc_bias_x(10^-2m/s^2)(int16_t)-> acc_bias_x(m/s^2)(float64_t)
	p_developerSetting->initpar[26] = (double)p_saveconfig->acc_bias_y / 100.0; //acc_bias_y(10^-2m/s^2)(int16_t)-> acc_bias_y(m/s^2)(float64_t)
	p_developerSetting->initpar[27] = (double)p_saveconfig->acc_bias_z / 100.0; //acc_bias_z(10^-2m/s^2)(int16_t)-> acc_bias_z(m/s^2)(float64_t)
	p_developerSetting->initpar[28] = (double)p_saveconfig->std_gyro_bias_x / 100.0 * DEG2RAD; //std_gyro_bias_x(10 ^ -2deg/s)(int16_t)-> std_gyro_bias_x(rad/s)(float64_t)
	p_developerSetting->initpar[29] = (double)p_saveconfig->std_gyro_bias_y / 100.0 * DEG2RAD; //std_gyro_bias_y(10 ^ -2deg/s)(int16_t)-> std_gyro_bias_y(rad/s)(float64_t)
	p_developerSetting->initpar[30] = (double)p_saveconfig->std_gyro_bias_z / 100.0 * DEG2RAD; //std_gyro_bias_z(10 ^ -2deg/s)(int16_t)-> std_gyro_bias_z(rad/s)(float64_t)
	p_developerSetting->initpar[31] = (double)p_saveconfig->std_acc_bias_x / 100.0; //std_acc_bias_x(10^-2m/s^2)(int16_t)-> std_acc_bias_x(m/s^2)(float64_t)
	p_developerSetting->initpar[32] = (double)p_saveconfig->std_acc_bias_y / 100.0; //std_acc_bias_x(10^-2m/s^2)(int16_t)-> std_acc_bias_x(m/s^2)(float64_t)
	p_developerSetting->initpar[33] = (double)p_saveconfig->std_acc_bias_z / 100.0; //std_acc_bias_x(10^-2m/s^2)(int16_t)-> std_acc_bias_x(m/s^2)(float64_t)
	p_developerSetting->initpar[34] = (double)p_saveconfig->static_type; //static_type()(int8_t)-> static_type()(double)
}

static int8_t setdeveloperSettingB(const char* p_savemsg, developerSetting* p_developerSetting)
{
	int8_t ret = 0;
	SaveMsg msavemsg;
	if (parse_saveconfig(p_savemsg, &msavemsg) == 1)
	{
		convertSaveconfig2devset(&msavemsg.saveConfig, p_developerSetting);
		ret = 1;
	}
	else
	{
		ret = -1;
	}
	return ret;
}
#ifndef MAXFIELD
#define MAXFIELD 100
#endif
#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif
static int8_t  parse_fields(char* const buffer, char** val, int32_t* size)
{
	int8_t ret = -1;
	char* p, *q;
	int32_t n = 0;

	/* parse fields */
	for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
		if ((q = strchr(p, ',')) || (q = strchr(p, '*'))) {
			val[n++] = p;
			*q = '\0';
		}
		else
		{
			val[n++] = p;
			break;
		}

	}
	*size = n;
	ret = 1;
	return ret;
}

static int8_t setdeveloperSettingA(const char* p_savemsg, developerSetting* p_developerSetting)
{
	int8_t ret = 0;
	char  *val[40];
	int32_t sentencesize;
	parse_fields((char* const)p_savemsg, val, &sentencesize);
	//后续转为读取二进制文件 文件头 文件尾巴 语句类型 字节长度 CRC校验位均需要考虑，可以参考NOVATEL的格式 developer
	if (strstr(val[0], "$developer") != NULL)
	{
		p_developerSetting->insinitfromprev = 1;
		//需要进行转化，得到自己想要的格式，这里统一转化为double，方便一点点，后续会按照结构体存放
		p_developerSetting->initpar[0] = atoi(val[1]);  //GNSS week()(int16_t) - > GNSS week()(float64_t)  
		p_developerSetting->initpar[1] = atoi(val[2]);  //GNSS time of week(second)(int32_t) ->GNSS time of week(second)(float64_t)
		p_developerSetting->initpar[2] = atoi(val[3]);  //solution_type()(int8_t)  -> solution_type()(float64_t)
		p_developerSetting->initpar[3] = atoi(val[4]);  //position_type()(int8_t)  -> position_type()(float64_t)
		p_developerSetting->initpar[4] = atof(val[5]) * PI / 180;  //latitude(deg)(float64_t) -> latitude(rad)(float64_t)
		p_developerSetting->initpar[5] = atof(val[6]) * PI / 180;  //lontitde(deg)(float64_t) -> lontitde(rad)(float64_t)
		p_developerSetting->initpar[6] = atof(val[7]);  //height(m)(float32_t) -> height(m)(float64_t)
		p_developerSetting->initpar[7] = (double)atoi(val[8]) / 100.0;  //v_n(cm/s)(in16_t) -> v_n(m/s)(float64_t)
		p_developerSetting->initpar[8] = (double)atoi(val[9]) / 100.0;  //v_e(cm/s)(in16_t) -> v_n(m/s)(float64_t)
		p_developerSetting->initpar[9] = (double)atoi(val[10]) / 100.0;  //v_d(cm/s)(in16_t) -> v_n(m/s)(float64_t)
		p_developerSetting->initpar[10] = (double)atoi(val[11]) / 100.0 * PI / 180; //roll(10 ^ -2deg)(int16_t)-> roll(rad)(float64_t)
		p_developerSetting->initpar[11] = (double)atoi(val[12]) / 100.0 * PI / 180; //pitch(10 ^ -2deg)(int16_t)-> pitch(rad)(float64_t)
		p_developerSetting->initpar[12] = (double)atoi(val[13]) / 100.0 * PI / 180; //yaw(10 ^ -2deg)(int16_t)-> yaw(rad)(float64_t)
		p_developerSetting->initpar[13] = (double)atoi(val[14]) / 100.0; //std_lat(cm)(int16_t) -> std_lat(m)(float64_t)
		p_developerSetting->initpar[14] = (double)atoi(val[15]) / 100.0; //std_lat(cm)(int16_t) -> std_lon(m)(float64_t)
		p_developerSetting->initpar[15] = (double)atoi(val[16]) / 100.0; //std_lat(cm)(int16_t) -> std_height(m)(float64_t)
		p_developerSetting->initpar[16] = (double)atoi(val[17]) / 100.0; //std_v_n(cm/s)(int16_t) -> std_v_n(m/s)(float64_t)
		p_developerSetting->initpar[17] = (double)atoi(val[18]) / 100.0; //std_v_e(cm/s)(int16_t) -> std_v_e(m/s)(float64_t)
		p_developerSetting->initpar[18] = (double)atoi(val[19]) / 100.0; //std_v_d(cm/s)(int16_t) -> std_v_d(m/s)(float64_t)
		p_developerSetting->initpar[19] = (double)atoi(val[20]) / 100.0 * PI / 180; //std_roll(10 ^ -2deg)(int16_t)-> std_roll(rad)(float64_t)
		p_developerSetting->initpar[20] = (double)atoi(val[21]) / 100.0 * PI / 180; //std_pitch(10 ^ -2deg)(int16_t)-> std_pitch(rad)(float64_t)
		p_developerSetting->initpar[21] = (double)atoi(val[22]) / 100.0 * PI / 180; //std_yaw(10 ^ -2deg)(int16_t)-> std_yaw(rad)(float64_t)
		p_developerSetting->initpar[22] = (double)atoi(val[23]) / 100.0 * PI / 180; //gyro_bias_x(10 ^ -2deg/s)(int16_t)-> gyro_bias_x(rad/s)(float64_t)
		p_developerSetting->initpar[23] = (double)atoi(val[24]) / 100.0 * PI / 180; //gyro_bias_y(10 ^ -2deg/s)(int16_t)-> gyro_bias_y(rad/s)(float64_t)
		p_developerSetting->initpar[24] = (double)atoi(val[25]) / 100.0 * PI / 180; //gyro_bias_z(10 ^ -2deg/s)(int16_t)-> gyro_bias_z(rad/s)(float64_t)
		p_developerSetting->initpar[25] = (double)atoi(val[26]) / 100.0; //acc_bias_x(10^-2m/s^2)(int16_t)-> acc_bias_x(m/s^2)(float64_t)
		p_developerSetting->initpar[26] = (double)atoi(val[27]) / 100.0; //acc_bias_y(10^-2m/s^2)(int16_t)-> acc_bias_y(m/s^2)(float64_t)
		p_developerSetting->initpar[27] = (double)atoi(val[28]) / 100.0; //acc_bias_z(10^-2m/s^2)(int16_t)-> acc_bias_z(m/s^2)(float64_t)
		p_developerSetting->initpar[28] = (double)atoi(val[29]) / 100.0 * PI / 180; //std_gyro_bias_x(10 ^ -2deg/s)(int16_t)-> std_gyro_bias_x(rad/s)(float64_t)
		p_developerSetting->initpar[29] = (double)atoi(val[30]) / 100.0 * PI / 180; //std_gyro_bias_y(10 ^ -2deg/s)(int16_t)-> std_gyro_bias_y(rad/s)(float64_t)
		p_developerSetting->initpar[30] = (double)atoi(val[31]) / 100.0 * PI / 180; //std_gyro_bias_z(10 ^ -2deg/s)(int16_t)-> std_gyro_bias_z(rad/s)(float64_t)
		p_developerSetting->initpar[31] = (double)atoi(val[32]) / 100.0; //std_acc_bias_x(10^-2m/s^2)(int16_t)-> std_acc_bias_x(m/s^2)(float64_t)
		p_developerSetting->initpar[32] = (double)atoi(val[33]) / 100.0; //std_acc_bias_x(10^-2m/s^2)(int16_t)-> std_acc_bias_x(m/s^2)(float64_t)
		p_developerSetting->initpar[33] = (double)atoi(val[34]) / 100.0; //std_acc_bias_x(10^-2m/s^2)(int16_t)-> std_acc_bias_x(m/s^2)(float64_t)
		p_developerSetting->initpar[34] = (double)atoi(val[35]); //static_type()(int8_t)-> static_type()(double)
		ret = 1;

	}
	return ret;
}

static int8_t reinitLCsetting(const int8_t  type, const char* config_s, LCSetting* plcSetting)
{
	int8_t ret = 0;
	developerSetting mdeveloperSetting;
	if (type == 0)
	{
		if (setdeveloperSettingA(config_s, &mdeveloperSetting) == 1)
		{
			ret = 1;
		}
		else
		{
			ret = -1;
		}
	}
	else if (type == 4)
	{
		if (setdeveloperSettingB(config_s, &mdeveloperSetting) == 1)
		{
			ret = 1;
		}
		else
		{
			ret = 1;
		}
	}

	if (ret == 1)
	{
		if (initsysfromdeveloper(mdeveloperSetting, plcSetting) == 1)
		{
			ret = 1;
		}
		else
		{
			ret = -2;
		}
	}
	return ret;
}

extern int8_t ins_init_API(const uint8_t type, const char* userconfig_s, const char* developerconfig_s)
{
	//type ascii or bin 
	int8_t ret = 0;
	if (type & INITVAR_USECONFIG << 0)
	{
		userSetting muserSetting;
		memcpy(&muserSetting, userconfig_s, sizeof(muserSetting));
		if (initsysfromUser(&muserSetting, &p_prcopt) == 1)
		{

			ret = 1;
		}
		else
		{
			ret = -1;
		}
	}
	if (ret == 1 && (type & INITVAR_DEVELOPERCONFIG))
	{
		if (reinitLCsetting(type & 1 << 2, developerconfig_s, &p_prcopt) == 1)
		{

			ret = 1;
		}
		else
		{
			ret = 2;
			//error_msg wrong INITVAR_DEVELOPERCONFIG
		}

	}

	if (ret == 1 || ret == 2 )
	{
		if (initsystemfromcfg(&p_prcopt) != 1)
		{
			ret = -3;
		}
	}
	return ret;
}

static uint8_t get_gnss_obs_valid(void)
{
	return m_OutputControlOntime_outputs.GNSS_OBS_output;
}

static void set_gnss_obs_valid(uint8_t value)
{
	m_OutputControlOntime_outputs.GNSS_OBS_output = value;
}

static uint8_t get_imu_obs_valid(void)
{
	return m_OutputControlOntime_outputs.IMU_OBS_output;
}

static void set_imu_obs_valid(uint8_t value)
{
	m_OutputControlOntime_outputs.IMU_OBS_output = value;
}

static uint8_t get_odo_obs_valid(void)
{
	return m_OutputControlOntime_outputs.ODO_OBS_output;
}

static void set_odo_obs_valid(uint8_t value)
{
	m_OutputControlOntime_outputs.ODO_OBS_output = value;
}

extern uint8_t ins_get_gnssoutputflag_API(void)
{
	uint8_t ret = 0;
	ret = get_gnss_obs_valid();
	set_gnss_obs_valid(0);
	return ret;
}

extern int32_t ins_get_inslcSTATUS_API(void)
{
	return mGnssInsSystem.mlc_STATUS;
}

int8_t ins_set_missetoutput(MISRVB* output)
{
	int8_t ret = 0;
	if (mGnssInsSystem.outputRVB.output_flag == 1)
	{
		ret = 1;
		memcpy(output, &mGnssInsSystem.outputRVB, sizeof(MISRVB));
		mGnssInsSystem.outputRVB.output_flag = 0;
	}
	return ret;
}

/*  ADD origin GNSS data and Updata process
	args:  GnssData *mGnssData              origin gnss data struct
	return:status ()

*/
static int8_t INSADDGNSSDATA(const GnssData mGnssData)
{
	int8_t ret = -1;
	mGnssInsSystem.GNSSflag = 1;
	if (mGnssInsSystem.mImuData.timestamp + 0.2 / p_prcopt.imuDataRate >= mGnssData.timestamp)
	{
		KFStatus = 1;
		ret = ADDGNSSDATA(mGnssData);
		//avoid unexpected  return
		mGnssInsSystem.GNSSflag = 0;
		if (isEFKFinished)
		{
			KFStatus = 2;
		}
		else
		{
			KFStatus = 0;
		}
		set_gnss_obs_valid(1);
	}
	else
	{
		//unexpected GNSS
		ret = 0;
	}
	return ret;
}

GnssData*  ins_get_gnss_cal_data()
{
	return  &m_globalvariable_data.m_gnss_msg.m_gnss_data;
}

static int8_t INSAddIMUData(const ImuData mImuData)
{
	int8_t ret = -1;
	if (1 == mGnssInsSystem.GNSSflag && 0 == KFStatus)
	{
		GnssData* p_cal_gnss = ins_get_gnss_cal_data();

		if (mImuData.timestamp + 0.2 / p_prcopt.imuDataRate >= p_cal_gnss->timestamp)
		{
			KFStatus = 1;
			ADDGNSSDATA(*p_cal_gnss);
			if (isEFKFinished)
			{
				KFStatus = 2;
			}
			else
			{
				KFStatus = 0;
			}
			set_gnss_obs_valid(1);
		}
	}
	else if (1 == mGnssInsSystem.GNSSflag && KFStatus != 0) // 
	{
		mGnssInsSystem.GNSSflag = 0;
	}
	ret = AddIMUData(mImuData);
	return ret;
}

static int8_t INSAddODOData(const OdoData odo_data)
{
	int8_t ret = -1;

	double curent_vehicle_speed = 0.0;

	switch (odo_data.mode)
	{
	case 0:
	{
		ret = 1;
		mGnssInsSystem.Odomode = 1;
		curent_vehicle_speed = odo_data.vehicle_speed;
	}break;
	case 1:
	{
		if (odo_data.timestamp - mGnssInsSystem.mOdoData.timestamp > 0 && mGnssInsSystem.mOdoData.timestamp > 0)
		{
			double vehicle_speed;
			if (odo_data.timestamp - mGnssInsSystem.mOdoData.timestamp > 0.99 && odo_data.timestamp - mGnssInsSystem.mOdoData.timestamp < 1.01)
			{
				vehicle_speed = SCALEFACTOR_ODO * (odo_data.wheel_tick - mGnssInsSystem.mOdoData.wheel_tick);
			}
			else
			{
				vehicle_speed = SCALEFACTOR_ODO * (odo_data.wheel_tick - mGnssInsSystem.mOdoData.wheel_tick) * 0.1 / (odo_data.timestamp - mGnssInsSystem.mOdoData.timestamp);
			}
			vehicle_speed = mGnssInsSystem.mOdoData.fwd == 1 ? vehicle_speed : -vehicle_speed;
			if (fabs(vehicle_speed) < 0.01)
			{
				curent_vehicle_speed = 0;
			}
			else
			{
				curent_vehicle_speed = vehicle_speed;
				// curent_vehicle_speed = 0.10 * mGnssInsSystem.mOdoData.vehicle_speed + 0.90 * vehicle_speed;
			}
			ret = 1;
		}
		else
		{
			ret = 0;
			mGnssInsSystem.IsUseOdo = 0;
		}
	}break;
	default:
		break;
	}


	mGnssInsSystem.mOdoData.week = odo_data.week;
	mGnssInsSystem.mOdoData.timestamp = odo_data.timestamp;
	mGnssInsSystem.mOdoData.vehicle_speed = curent_vehicle_speed;
	mGnssInsSystem.mOdoData.wheel_tick = odo_data.wheel_tick;
	mGnssInsSystem.mOdoData.fwd = odo_data.fwd;
	return ret;
}

int8_t ins_add_imudata(const ImuData input_imu_data, ImuDataMsg* p_imu_msg)	//TODO:
{
	int8_t ret = 0;
	//lock_on 1 最原始的imu_data
	{
		p_imu_msg->gnss_week = input_imu_data.week;
		p_imu_msg->gnss_timeofweek = input_imu_data.timestamp;

		ImuData *p_ImuData = &p_imu_msg->m_imu_data;
		memcpy(p_ImuData, &input_imu_data, sizeof(ImuData));
		convert_systime2caltime(p_imu_msg->gnss_week, p_imu_msg->gnss_timeofweek, &p_ImuData->week, &p_ImuData->timestamp);
		p_imu_msg->use_flag = 1;
	}
	ret = 1;
	return ret;
}

int8_t INSAddODOData_CAL(int32_t week, double timestamp);


static int8_t ins_set_result_INSIDE(const int mode, int32_t recweek, double rectimestamp, OutputMsg* p_outputMsg);

extern int8_t INSADDIMUData_API(const ImuData mImuData)
{
	int8_t ret = 0;

	/*------------数据接口层-----------------------------------*/
	ImuDataMsg* p_imu_msg = &m_globalvariable_data.m_imu_msg;

	ImuData* p_cal_imu = &m_globalvariable_data.m_imu_msg.m_imu_data;

	ins_add_imudata(mImuData, p_imu_msg);

	INSAddODOData_CAL(p_cal_imu->week, p_cal_imu->timestamp);

	ret = INSAddIMUData(*p_cal_imu);

	if (ret != 1)
	{
		//throw message error
		int error_out = 1;
	}
	//lc output message
	ins_set_result_INSIDE(p_prcopt.OutputMessageType, mImuData.week, mImuData.timestamp, &g_0utputMsg);

	return ret;
}

int8_t ins_add_gnssdata(int32_t recweek, double rectimestamp, const GnssData gnss, GnssDataMsg* p_gnss_msg)
{
	int8_t ret = 0;
	//lock_on 3 最原始的gnss_data
	p_gnss_msg->gnss_week = gnss.week;
	p_gnss_msg->gnss_timeofweek = gnss.timestamp;

	GnssData *p_gnssData = &p_gnss_msg->m_gnss_data;
	memcpy(p_gnssData, &gnss, sizeof(GnssData));
	convert_systime2caltime(p_gnss_msg->gnss_week, p_gnss_msg->gnss_timeofweek, &p_gnssData->week, &p_gnssData->timestamp);
	convert_systime2caltime(recweek, rectimestamp, &p_gnssData->week, &p_gnssData->timestampd);
	p_gnss_msg->use_flag = 1;
	ret = 1;
	return ret;
}

extern int8_t INSADDGNSSDATA_API(int32_t recweek, double rectimestamp, const GnssData mGnssData)
{
	int8_t ret = -1;
	static double lastcalgnsstime = 0;

	GnssDataMsg* p_gnss_msg = &m_globalvariable_data.m_gnss_msg;
	GnssData* p_cal_gnss = &p_gnss_msg->m_gnss_data;

	ins_add_gnssdata(recweek, rectimestamp, mGnssData, p_gnss_msg);

	if (KFStatus == 0 && p_cal_gnss->timestamp - lastcalgnsstime > 0)
	{
		ret = INSADDGNSSDATA(*p_cal_gnss);
		lastcalgnsstime = p_cal_gnss->timestamp;
	}
	else
	{
		//worning:something is wrong
		//unexpected msg output_gnss_mesg_error
	}
	return ret;
}


int8_t INSAddODOData_sample(const OdoData odo_data)
{
	int8_t ret = 0;
	OdoData calmOdoData;
	memcpy(&calmOdoData, &odo_data, sizeof(GnssData));
	convert_systime2caltime(odo_data.week, odo_data.timestamp, &calmOdoData.week, &calmOdoData.timestamp);
	ret = INSAddODOData(calmOdoData);
	writeOdoDataMsg(&odo_data);
	return ret;
}


#define ODODataINPUTDF (10)

static double lastOdoAddTime = 0.0;
int32_t ins_get_systemODOinterface_STATUS(void)
{
	return mGnssInsSystem.isOdoInterfaceGOOD;
}
int32_t ins_set_systemODOinterface_STATUS(int32_t value)
{
	int32_t ret = 0;
	mGnssInsSystem.isOdoInterfaceGOOD = value;
	ret = value;
	return ret;
}
int8_t INSAddODOData_CAL(int32_t week, double timestamp)
{
	int8_t ret = 0;
	OdoDataMsg* p_odo_msg = &m_globalvariable_data.m_odo_msg;
	odometer_interface_status* p_odometer_interface_status = &p_odo_msg->m_odometer_interface_status;
	double timestampfloor = floor((timestamp - 0.00001) * ODODataINPUTDF) / ODODataINPUTDF;
	double dt = timestamp - (timestampfloor);
	if (timestamp < lastOdoAddTime)
	{
		lastOdoAddTime = timestamp;
	}
	/*should do something avoid time from no to yes*/
	if (week > 1024 && week < 3072)
	{
		if (mGnssInsSystem.systemtime_ready == 0)
		{
			lastOdoAddTime = timestamp;
		}
	}

	if (timestamp < lastOdoAddTime - 0.5)
	{
		lastOdoAddTime = timestamp;
	}
    // (dt <= 0.005 || dt > 0.095) && timestamp > lastOdoAddTime + 0.099
    // 根据IMU时间和上一次用odo的时间来控制10Hz
    // imu 10Hz处丢失，就会导致判断不进，若IMU时间10Hz间隔，相差超过1ms，也会导致判断进不去，一般不会有问题
    // 若odo数据为20Hz,则会只用到10Hz
    // 若odo数据为10Hz,浮动超过1ms，会导致丢弃
    // 若odo数据为5Hz,则会导致odo不使用
	if ((dt <= 0.5 / p_prcopt.imuDataRate || dt > 1.0 / ODODataINPUTDF - 0.5 / p_prcopt.imuDataRate) && timestamp > lastOdoAddTime + 0.99 / ODODataINPUTDF)
	{
		lastOdoAddTime = timestamp;
		m_OutputControlOntime_outputs.ODO_OBS_output = 1;
		double odotimestamp = p_odo_msg->m_odo_data.timestamp;

		if (p_odo_msg->use_flag == 1)
		{
			p_odo_msg->use_flag = 0;
			if (fabs(timestamp - odotimestamp)< 1.01 / ODODataINPUTDF)
			{
				p_odometer_interface_status->wheel_tick_normal_time++;
				p_odometer_interface_status->wheel_tick_normal_time = p_odometer_interface_status->wheel_tick_normal_time > 50 ? 50 : p_odometer_interface_status->wheel_tick_normal_time;
				p_odometer_interface_status->wheel_tick_lose_time = 0;
				p_odometer_interface_status->wheel_tick_lose_cur = 0;

			}
			else
			{
				p_odometer_interface_status->wheel_tick_lose_time++;
				p_odometer_interface_status->wheel_tick_lose_cur = 1;
				p_odometer_interface_status->wheel_tick_normal_time = 0;
			}

		}
		else
		{
			p_odometer_interface_status->wheel_tick_lose_time++;
			p_odometer_interface_status->wheel_tick_lose_cur = 1;
			p_odometer_interface_status->wheel_tick_normal_time = 0;
		}

		if (p_odometer_interface_status->wheel_tick_lose_time > 50)
		{
			p_odometer_interface_status->wheel_tick_lose_long = 1;
		}
		if (p_odometer_interface_status->wheel_tick_normal_time > 40)
		{
			p_odometer_interface_status->wheel_nomal_flag = 1;
		}

		if (ins_get_systemODOinterface_STATUS() != 1)
		{
			if (p_odometer_interface_status->wheel_tick_normal_time == 50)
			{
				ins_set_systemODOinterface_STATUS(1);
			}
		}
		else if (p_odometer_interface_status->wheel_tick_lose_long == 1 && ins_get_systemODOinterface_STATUS() == 1)
		{
			ins_set_systemODOinterface_STATUS(0);
		}


		if (!p_odometer_interface_status->wheel_tick_lose_cur && ins_get_systemODOinterface_STATUS() == 1)
		{
			OdoData m_cal_odo;
			memcpy(&m_cal_odo, &p_odo_msg->m_odo_data, sizeof(OdoData));
			INSAddODOData(m_cal_odo);
		}
		ret = 1;
	}
	return ret;
}

int8_t ins_add_ododata(const OdoData odo, OdoDataMsg* p_odo_msg)
{
	int8_t ret = 0;
	// LOCK 6

	p_odo_msg->gnss_week = odo.week;
	p_odo_msg->gnss_timeofweek = odo.timestamp;

	OdoData *p_odoData = &p_odo_msg->m_odo_data;
	memcpy(p_odoData, &odo, sizeof(OdoData));
	convert_systime2caltime(p_odo_msg->gnss_week, p_odo_msg->gnss_timeofweek, &p_odoData->week, &p_odoData->timestamp);
	
	if (p_odoData->vehicle_speed >= 0)
	{
		p_odoData->vehicle_speed = p_odoData->vehicle_speed;
		p_odoData->fwd = 1;
	}
	else
	{
		p_odoData->vehicle_speed = -p_odoData->vehicle_speed;
		p_odoData->fwd = 0;
	}
	
	p_odo_msg->use_flag = 1;
	ret = 1;
	return ret;
}

extern int8_t INSAddODOData_API(const OdoData odo_data)
{
	int8_t ret = 0;
	OdoDataMsg* p_odo_msg = &m_globalvariable_data.m_odo_msg;
	odometer_interface_status* p_odometer_interface_status = &p_odo_msg->m_odometer_interface_status;
	ins_add_ododata(odo_data, p_odo_msg);
	return ret;
}

extern ins_solution_t* ins_get_Soltuion_API(void)
{
	return  getinssolutionmsg();
}

extern int ins_get_printnmeains_API(double *ep, char *buff)
{
	int ret = 0;
	ret = ins_print_nmea_ins(ep, buff);
	return ret;
}

extern int ins_get_printnmeavtg_API(char *buff)
{
	int ret = 0;
	ret = ins_print_nmea_vtg(buff);
	return ret;
}

extern int ins_get_printnmeapashr_API(double *ep, char *buff)
{
	int ret = 0;
	ret = ins_print_nmea_pashr(ep, buff);
	return ret;
}



extern int8_t ins_init_post_API(char* poc_set)
{
	int8_t ret = 0;
	memcpy(&p_prcopt, poc_set, sizeof(LCSetting));
	return ret;
}

static int8_t ins_set_rawgnssoutput(const int32_t week, const double timestamp, const GnssData* p_gnss_data, const GnssData_interface_status* p_gnss_interface_status, GnssData* p_out_gnss_data)
{
	int8_t ret = 0;
	/*拷贝*/
	memcpy(p_out_gnss_data, p_gnss_data, sizeof(GnssData));
	p_out_gnss_data->week = week;
	p_out_gnss_data->timestamp = timestamp;
	return ret;
}

static int8_t ins_set_result_INSIDE(const int mode, int32_t recweek, double rectimestamp, OutputMsg* p_outputMsg)
{
	int ret = 0;
	p_outputMsg->update = Output_NONE;

	p_outputMsg->mlc_STATUS = (uint32_t)mGnssInsSystem.mlc_STATUS;
	/*on change*/
	if (mode & Output_RAWGNSS && m_OutputControlOntime_outputs.GNSS_OBS_output == 1)
	{
		//lock
		GnssDataMsg* p_gnss_msg = &m_globalvariable_data.m_gnss_msg;
		GnssData* p_rawgnss = &p_outputMsg->rawgnss;
		ins_set_rawgnssoutput(p_gnss_msg->gnss_week, p_gnss_msg->gnss_timeofweek, &p_gnss_msg->m_gnss_data, &p_gnss_msg->m_gnss_interface_status, p_rawgnss);
		m_OutputControlOntime_outputs.GNSS_OBS_output = 0;
		p_outputMsg->update |= Output_RAWGNSS;
	}

	/*According to frequency*/
	if (mode & Output_INSGGA)
	{
		GnssInsSystem* p_GnssInsSystem = &mGnssInsSystem;
		writeGGAMsg(recweek, rectimestamp, p_GnssInsSystem, p_outputMsg->ggaBuff, p_outputMsg->rmcBuff, p_outputMsg->vtgBuff);
		p_outputMsg->update |= Output_INSGGA;
		p_outputMsg->update |= Output_INSRMC;
		p_outputMsg->update |= Output_INSVTG;
	}

	/*According to frequency*/
	if (mode & Output_INSPVAX)
	{
		GnssInsSystem* p_GnssInsSystem = &mGnssInsSystem;
		INSPVAXN* p_inspvax = &p_outputMsg->inspvax;
		ins_set_inspvaxoutput(recweek, rectimestamp, p_GnssInsSystem, p_inspvax);
		p_outputMsg->update |= Output_INSPVAX;
	}

	/*According to frequency*/
	if (mode & Output_CORRIMU)
	{
		GnssInsSystem* p_GnssInsSystem = &mGnssInsSystem;
		CORRIMUN* p_corrimu = &p_outputMsg->corrimu;
		ins_set_corrimuoutput(recweek, rectimestamp, p_GnssInsSystem, p_corrimu);
		p_outputMsg->update |= Output_CORRIMU;
	}

	/*According to frequency*/
	if (mode &Output_BIAS)
	{
		GnssInsSystem* p_GnssInsSystem = &mGnssInsSystem;
		ins_set_biasoutput(recweek, rectimestamp, p_GnssInsSystem, &p_outputMsg->insbias);
		p_outputMsg->update |= Output_BIAS;
	}

	/*According to frequency*/
	if (mode &Output_SOL)
	{
		GnssInsSystem* p_GnssInsSystem = &mGnssInsSystem;
		_copy_ins_result(recweek, rectimestamp, p_GnssInsSystem, &p_outputMsg->ins_solution);
		p_outputMsg->update |= Output_SOL;
	}
	
	/*According to frequency*/
	if (mode & Output_RAWIMU)
	{
		ImuDataMsg *p_imudatamsg = &m_globalvariable_data.m_imu_msg;
		ImuData* p_rawimu = &p_outputMsg->rawimu;
		ins_set_rawimuoutput(p_imudatamsg->gnss_week, p_imudatamsg->gnss_timeofweek, &p_imudatamsg->m_imu_data, &p_imudatamsg->m_imu_interface_status, p_rawimu);
		p_outputMsg->update |= Output_RAWIMU;
	}

	/*on change*/
	if (mode & Output_RAWODO&& m_OutputControlOntime_outputs.ODO_OBS_output == 1)
	{
		OdoDataMsg *p_ododatamsg = &m_globalvariable_data.m_odo_msg;
		OdoData* p_rawodo = &p_outputMsg->rawodo;

		ins_set_rawodooutput(p_ododatamsg->gnss_week, p_ododatamsg->gnss_timeofweek, &p_ododatamsg->m_odo_data, &p_ododatamsg->m_odometer_interface_status, p_rawodo);

		m_OutputControlOntime_outputs.ODO_OBS_output == 0;
		p_outputMsg->update |= Output_RAWODO;
	}

	if (mode & Output_SAVEMSG && fmod(rectimestamp + 0.5 / p_prcopt.imuDataRate, 5) < 1.0 / p_prcopt.imuDataRate)
	{
		OdoData* p_rawodo = &p_outputMsg->rawodo;
		char* p_saveMsg = p_outputMsg->saveMsg;
		printsavebuf(p_prcopt.savesavetype, &mGnssInsSystem, p_saveMsg);
		p_outputMsg->update |= Output_SAVEMSG;
	}

	if (mode & Output_ONLINEMISEST)
	{
		MISRVB* p_misrbv = &p_outputMsg->misrbv;
		if (ins_set_missetoutput(p_misrbv) == 1)
		{
			p_outputMsg->update |= Output_ONLINEMISEST;
		}
	}

	return ret;
}

extern int8_t ins_get_result_API(OutputMsg* p_outputMsg)
{
	int8_t ret = 0;
	memcpy(p_outputMsg, &g_0utputMsg, sizeof(OutputMsg));
	ret = g_0utputMsg.update;
	g_0utputMsg.update = 0;
	return ret;
}