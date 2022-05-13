#include <math.h>
#include <string.h>
#include "lcinsupdate.h"
#include "lcsystem.h"
#include "insmech.h"
#include "earth.h"
#include "orientation.h"
#include "cmatrix.h"
#include "zuptdetect.h"
#include "lcgnssupdate.h"
#include "positionfilter.h"
#include "ins_interface_API.h"
#include "loosecoupleset.h"

#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif
#ifndef DEG2RAD
#define DEG2RAD (0.017453292519943)
#endif // !DEG2RAD
#ifndef RAD2DEG
#define RAD2DEG (57.295779513082323)
#endif // !DEG2RAD
#ifndef LIMITACCNORMAL
#define  LIMITACCNORMAL (30)
#endif // !LIMITACCNORMAL
#ifndef LIMITGYROZNORMAL
#define  LIMITGYROZNORMAL (100 * 0.017453292519943)
#endif // !LIMITACCNORMAL
#ifndef LIMITGYROXYNORMAL
#define  LIMITGYROXYNORMAL (20 * 0.017453292519943)
#endif // !LIMITACCNORMAL
#ifndef NHCSTARTTIME
#define  NHCSTARTTIME (20)
#endif // !LIMITACCNORMAL
#ifndef NHCTIMEINTERVAL1
#define  NHCTIMEINTERVAL1 (0.5) //0.3
#endif // !LIMITACCNORMAL
#ifndef NHCTIMEINTERVAL2
#define  NHCTIMEINTERVAL2 (0.8)
#endif // !LIMITACCNORMAL
#ifndef NHCTIME
#define  NHCTIME (0.5) //0.34
#endif // !LIMITACCNORMAL

extern GnssInsSystem mGnssInsSystem;
extern int8_t isEFKFinished;
extern int8_t PositionInitFlag;

extern UpdataStruct mUpdataStruct;
extern LCSetting p_prcopt;
Nav lastzuptnav;
double lastzupttime;

// 已分析
static int8_t IsNeedLC(double lastinstime, double nowinstime, double NEXTLCTIME)
{
	int8_t ret = 0;
    double timestampfloor = floor(mGnssInsSystem.mInsData.timestamp - 0.00001);
	if (1 == p_prcopt.useGNSSRate ) // 1Hz gnss更新
	{
		if (mGnssInsSystem.mInsData.timestamp - timestampfloor > 0.6) // 当前时刻0.6~1s (左边不一定闭合，右边闭合，影响不大)
		{
			if (fabs(timestampfloor + 1 - NEXTLCTIME) > 0.001)
			{
				mGnssInsSystem.nextLCTime = timestampfloor + 1; // 更新下一个LC更新的时刻
			}
		}
	}
	else if (10 == p_prcopt.useGNSSRate)
	{
		timestampfloor = floor((mGnssInsSystem.mInsData.timestamp - 0.00001)*10)/10;
		if (mGnssInsSystem.mInsData.timestamp - timestampfloor > 0.6/p_prcopt.useGNSSRate)
		{
			if (fabs(timestampfloor + 1.0 / p_prcopt.useGNSSRate - NEXTLCTIME) > 0.001)
			{
				mGnssInsSystem.nextLCTime = floor(timestampfloor * 10 + 1 + 0.00001) / 10;
				NEXTLCTIME = mGnssInsSystem.nextLCTime;
			}
		}
	}

	if (lastinstime < NEXTLCTIME && nowinstime >= NEXTLCTIME)
	{
		ret = NEXTLCTIME - lastinstime > nowinstime - NEXTLCTIME ? 1 : 2; // 接近lastinstime, ret=2; 接近nowinstime, ret=1
		if (!((ret == 1 && fabs(nowinstime - NEXTLCTIME) < 0.0001) || (ret == 2 && fabs(NEXTLCTIME - lastinstime) < 0.0001)))
		{
			ret = 3; // NEXTLCTIME不是lastinstime或者nowinstime时刻
		}
	}

	if (p_prcopt.isUseNHC && 10 == p_prcopt.gnssDataRate && 1 == p_prcopt.useGNSSRate) // 不执行
	{
		if (mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.firstGNSSUseTime > NHCSTARTTIME
			&&mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastGNSSLCTime > NHCTIMEINTERVAL1
			&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastNHCLCTime > NHCTIMEINTERVAL1
			&& mGnssInsSystem.mImuData.timestamp - floor(mGnssInsSystem.mImuData.timestamp) > NHCTIME - 0.02
			&&mGnssInsSystem.lastGNSSLCTime > 0
			&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.nextNHCLCTime > NHCTIMEINTERVAL2
			&& mGnssInsSystem.mImuData.flag == 1
			)
			//&&fabs(mGnssInsSystem.mNav.wnb_n[2]) < 0.05)     //�˴�NHC/ODO ���� //NHC����ʱ���ODOһ��
		{
			mGnssInsSystem.nextNHCLCTime = mGnssInsSystem.mImuData.timestamp;
			ret = 4;
			return ret;
		}
	}

	else if (p_prcopt.isUseNHC && 1 == p_prcopt.gnssDataRate) // 只用1Hz的
	{
		if (mGnssInsSystem.insinitfromprev != 0) // 开机定位模式，先不看
		{
			if (/*mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.firstGNSSUseTime > NHCSTARTTIME*/
				/*&&*/mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastGNSSLCTime > NHCTIMEINTERVAL1
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastNHCLCTime > NHCTIMEINTERVAL2
				&& mGnssInsSystem.mImuData.timestamp - floor(mGnssInsSystem.mImuData.timestamp) > NHCTIME
				//&&mGnssInsSystem.lastGNSSLCTime > 0
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.nextNHCLCTime > NHCTIMEINTERVAL2)
				//&&fabs(mGnssInsSystem.mNav.wnb_n[2]) < 0.05)     //�˴�NHC/ODO ���� //NHC����ʱ���ODOһ��
			{
				mGnssInsSystem.nextNHCLCTime = mGnssInsSystem.mImuData.timestamp;
				ret = 4;
				return ret;
			}
		}
		else
		{
			if (mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.firstGNSSUseTime > NHCSTARTTIME
				&&mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastGNSSLCTime > NHCTIMEINTERVAL1
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastNHCLCTime > NHCTIMEINTERVAL2
				&& mGnssInsSystem.mImuData.timestamp - floor(mGnssInsSystem.mImuData.timestamp) > NHCTIME
				&&mGnssInsSystem.lastGNSSLCTime > 0
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.nextNHCLCTime > NHCTIMEINTERVAL2)
				//&&fabs(mGnssInsSystem.mNav.wnb_n[2]) < 0.05)     //�˴�NHC/ODO ���� //NHC����ʱ���ODOһ��
			{
				mGnssInsSystem.nextNHCLCTime = mGnssInsSystem.mImuData.timestamp; // 更新nextNHCLCTime
				ret = 4;
				return ret;
			}
		}
	}
	else if (p_prcopt.isUseNHC && 10 == p_prcopt.gnssDataRate) // 不执行
	{
		if (p_prcopt.isUseOdo)
		{
			if (mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.firstGNSSUseTime > NHCSTARTTIME
				//&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastGNSSLCTime > NHCTIMEINTERVAL1 / p_prcopt.useOdoRate
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastNHCLCTime > NHCTIMEINTERVAL1 / p_prcopt.useOdoRate
				&& mGnssInsSystem.mImuData.timestamp - floor(mGnssInsSystem.mImuData.timestamp) > NHCTIME / p_prcopt.useOdoRate - 0.02
				&& mGnssInsSystem.lastGNSSLCTime > 0
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.nextNHCLCTime > NHCTIMEINTERVAL2 / p_prcopt.useOdoRate)
				//&&fabs(mGnssInsSystem.mNav.wnb_n[2]) < 0.05)     //�˴�NHC/ODO ���� //NHC����ʱ���ODOһ��
			{
				mGnssInsSystem.nextNHCLCTime = mGnssInsSystem.mImuData.timestamp;
				ret = 4;
				return ret;
			}
		}
		else
		{
			if (mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.firstGNSSUseTime > NHCSTARTTIME
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastGNSSLCTime > NHCTIMEINTERVAL1 / p_prcopt.useNHCRate
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastNHCLCTime > NHCTIMEINTERVAL1 / p_prcopt.useNHCRate
				&& mGnssInsSystem.mImuData.timestamp - floor(mGnssInsSystem.mImuData.timestamp) > NHCTIME / p_prcopt.useNHCRate
				&& mGnssInsSystem.lastGNSSLCTime > 0
				&& mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.nextNHCLCTime > NHCTIMEINTERVAL2 / p_prcopt.useNHCRate)
				//&&fabs(mGnssInsSystem.mNav.wnb_n[2]) < 0.05)     //�˴�NHC/ODO ���� //NHC����ʱ���ODOһ��
			{
				mGnssInsSystem.nextNHCLCTime = mGnssInsSystem.mImuData.timestamp;
				ret = 4;
				return ret;
			}
		}
	}
	return ret;

};
int8_t GetLCData(Nav* mNav, double LCTime, int type)
{
	int8_t ret = 0;

		if (1 == type)
		{
			if (1 == p_prcopt.useGNSSRate)
			{
				mGnssInsSystem.nextLCTime = mGnssInsSystem.nextLCTime + 1;
			}
			else if (10 == p_prcopt.useGNSSRate)
			{
				mGnssInsSystem.nextLCTime = floor (mGnssInsSystem.nextLCTime * 10 + 1 + 0.00001)/10;
			}
		}
		else if (2 == type)
		{
			if (10 == p_prcopt.useGNSSRate)
			{
				mGnssInsSystem.nextLCTime = floor(mGnssInsSystem.nextLCTime * 10 + 1 + 0.00001) / 10;
			}
		}
	
	mUpdataStruct.LCTime = LCTime;
	mUpdataStruct.IsUseZUPT = 0;
	mUpdataStruct.IsUseZUPTA = 0;
	mUpdataStruct.Zuptlockflag = 1;


	if (p_prcopt.isUseOdo)
	{
		if (fabs(mGnssInsSystem.mOdoData.vehicle_speed) < 0.05)
		{
			mUpdataStruct.IsUseZUPT = GetZuptVal(); // ZUPT 更新开始
			if (mUpdataStruct.IsUseZUPT == 0)
			{
				mGnssInsSystem.mOdoData.fwd = -1;
			}
		}
	}
	else if (1)//sqrt(mGnssInsSystem.mNav.vn * mGnssInsSystem.mNav.vn + mGnssInsSystem.mNav.ve * mGnssInsSystem.mNav.ve) < 1.5)
	{
		if (p_prcopt.isUseZUPT == 1)
		{
			mUpdataStruct.IsUseZUPT = GetZuptVal();
		}
	}
	if (mUpdataStruct.IsUseZUPT)
	{
		mUpdataStruct.zuptdifftime = mGnssInsSystem.mImuData.timestamp - lastzupttime;
		if (fabs(mUpdataStruct.zuptdifftime) < 0.6)
		{
			mUpdataStruct.IsUseZUPTA = 1;
			mUpdataStruct.DiffHEAD = mGnssInsSystem.mNav.heading - lastzuptnav.heading;
			if (mUpdataStruct.DiffHEAD > PI)mUpdataStruct.DiffHEAD -= 2 * PI;
			if (mUpdataStruct.DiffHEAD < -PI)mUpdataStruct.DiffHEAD += 2 * PI;

		}
	}
	mGnssInsSystem.CurIsUseZupt = mUpdataStruct.IsUseZUPT;

	mUpdataStruct.mPVA.latitude = mNav->lat;
	mUpdataStruct.mPVA.longitude = mNav->lon;
	mUpdataStruct.mPVA.altitude = mNav->height;
	mUpdataStruct.mPVA.northVelocity = mNav->vn;
	mUpdataStruct.mPVA.eastVelocity = mNav->ve;
	mUpdataStruct.mPVA.downVelocity = mNav->vd;
	mUpdataStruct.mPVA.roll = mNav->roll;
	mUpdataStruct.mPVA.pitch = mNav->pitch;
	mUpdataStruct.mPVA.yaw = mNav->heading;

	memcpy(mUpdataStruct.mPVA.wnb_n, mNav->wnb_n, 3 * sizeof(double));
	memcpy(mUpdataStruct.mPVA.wnb_b, mNav->wnb_b, 3 * sizeof(double));
	memcpy(mUpdataStruct.mPVA.a_n, mNav->a_n, 3 * sizeof(double));

	memcpy(mUpdataStruct.mPVA.w_ie, mGnssInsSystem.mPar.w_ie, 3 * sizeof(double));
	memcpy(mUpdataStruct.mPVA.w_en, mGnssInsSystem.mPar.w_en, 3 * sizeof(double));
	memcpy(mUpdataStruct.mPVA.w_ib, mGnssInsSystem.mPar.w_b, 3 * sizeof(double));


	memcpy(mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.P, mGnssInsSystem.mKalmanStruct.n * mGnssInsSystem.mKalmanStruct.n * sizeof(float));
	memcpy(mUpdataStruct.P_r, mGnssInsSystem.mKalmanStruct.P, mGnssInsSystem.mKalmanStruct.n * mGnssInsSystem.mKalmanStruct.n * sizeof(float));

	memcpy(mUpdataStruct.C_bn, mGnssInsSystem.mNav.c_bn, 3 * 3 * sizeof(double));


	memset(mUpdataStruct.Q, 0, mGnssInsSystem.mKalmanStruct.n * mGnssInsSystem.mKalmanStruct.n  * sizeof(float));
	memset(mUpdataStruct.PHI, 0, mGnssInsSystem.mKalmanStruct.n * mGnssInsSystem.mKalmanStruct.n *  sizeof(float));

	memset(mUpdataStruct.X, 0, 9 * sizeof(float)); 
	memset(mUpdataStruct.X + 9, 0, (mGnssInsSystem.mKalmanStruct.n-9) * sizeof(float));
	

	for (int8_t i = 0; i < mGnssInsSystem.mKalmanStruct.n; i++)
	{
		for (int8_t j = 0; j < mGnssInsSystem.mKalmanStruct.n; j++)
		{
			if (i == j)
			{
				mUpdataStruct.PHI[i*mGnssInsSystem.mKalmanStruct.n + j] = 1;
				continue;
			}
		}
	}
	ret = 1;
	return ret;
}
/*  Check origin imu data and convert it to INS process struct  
    args:  ImuData *imudata              origin imu data struct
	       ImuData *SySimudata           INS process struct 
	return:status ( -1: loss imu data too much and reset
	                1: ok;
	                2: right IMU data interval is too low                 
					3: loss imu data and not reset
					4: Data overrun                    
					5: Vehicle threshold exceeded)
    此函数已分析完
*/
static int8_t ConvertIMUData(const ImuData *imudata, ImuData *SySimudata)  
{
	int8_t ret = -1;
	SySimudata->week = imudata->week;
	SySimudata->flag = imudata->flag;
	float dt = 0.0;
	if (fabs(imudata->timestamp - mGnssInsSystem.firstimutime) < 0.00001  || mGnssInsSystem.mpreImuData.timestamp  < 0.0001)
	{
		dt = 1.0/p_prcopt.imuDataRate;
	}
	else
	{
		dt = (float)(imudata->timestamp - mGnssInsSystem.mpreImuData.timestamp);
	}
	if (dt < 0.1 / p_prcopt.imuDataRate) // imu时间过快，或者时间异常，出现回退
	{
		ret = 2;
		return ret;
	}
	else if (dt > 5) // 丢数超过5s，重置
	{
		mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
		mGnssInsSystem.InitNavi = 0;
		mGnssInsSystem.mpreImuData.timestamp = 0;
		mGnssInsSystem.Isfirstfusionimu = 1;
		mGnssInsSystem.mErrorType = IMUInterrupt5S;
		ErrorReset(mGnssInsSystem.mErrorType);
		//hardreset
#ifdef DEBUGMESSAGE
		printf("IMU Data:interrupt error3, gnss_week: %d, gnss_time0fweek: %.4f, interruption time: %.2f\n", SySimudata->week, SySimudata->timestamp,dt);
#endif // DEBUGMESSAGE
		ret = -1;
		return ret;
	}
	else if (dt > 0.2) // 丢数超过200ms，重置
	{
		mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
		mGnssInsSystem.InitNavi = 0;
		mGnssInsSystem.mpreImuData.timestamp = 0;
		mGnssInsSystem.Isfirstfusionimu = 1;
		//softreset
		mGnssInsSystem.mErrorType = IMUInterruptS;
		ErrorReset(mGnssInsSystem.mErrorType);
#ifdef DEBUGMESSAGE
		printf("IMU Data:interrupt error2, gnss_week: %d, gnss_time0fweek: %.4f, interruption time: %.2f\n", SySimudata->week, SySimudata->timestamp, dt);
#endif // DEBUGMESSAGE
		ret = -1;
		return ret;
	}
	else if (dt > 2.0 / p_prcopt.imuDataRate && dt < 0.2 ) // 普通丢数
	{
		mGnssInsSystem.mErrorType = IMUInterruptWarning;
#ifdef DEBUGMESSAGE
		printf("IMU Data:interrupt error1, gnss_week: %d, gnss_time0fweek: %.4f, interruption time: %.2f\n", SySimudata->week, SySimudata->timestamp, dt);
#endif // DEBUGMESSAGE
		ret = 3;      
	}

	SySimudata->timestamp = imudata->timestamp;
	SySimudata->timestamped = imudata->timestamped;
	
	double acc[3] = { imudata->Accx ,imudata->Accy,imudata->Accz};
	double gyro[3] = { imudata->Gyrox ,imudata->Gyroy,imudata->Gyroz};

	double acc_1[3], gyro_1[3];
	MatrixMutiply(*mGnssInsSystem.RotationCBV, acc, 3, 3, 1, acc_1);
	MatrixMutiply(*mGnssInsSystem.RotationCBV, gyro, 3, 3, 1, gyro_1);

	SySimudata->Accx = acc_1[0];
	SySimudata->Accy = acc_1[1];
	SySimudata->Accz = acc_1[2];
	SySimudata->Gyrox = gyro_1[0];
	SySimudata->Gyroy = gyro_1[1];
	SySimudata->Gyroz = gyro_1[2];
	/*IMU Data overrun and  Data elimination*/
	if (fabs(SySimudata->Accx) > 2 * LIMITACCNORMAL || fabs(SySimudata->Accy > 2 * LIMITACCNORMAL) || fabs(SySimudata->Accz) > 2 * LIMITACCNORMAL
		|| fabs(SySimudata->Gyrox) > 3 * LIMITGYROXYNORMAL
		|| fabs(SySimudata->Gyroy) > 3 * LIMITGYROXYNORMAL
		|| fabs(SySimudata->Gyroz) > 3 * LIMITGYROZNORMAL)
	{
		mGnssInsSystem.mErrorType = IMUDataExp;
#ifdef DEBUGMESSAGE
		printf("IMU Data:data error, gnss_week: %d, gnss_time0fweek: %.4f\n", SySimudata->week, SySimudata->timestamp);
#endif // DEBUGMESSAGE
		ret = 4;
		return ret;
	}
	/*Vehicle threshold exceeded*/
	else if (fabs(SySimudata->Accx) > LIMITACCNORMAL || fabs(SySimudata->Accy > LIMITACCNORMAL)|| fabs(SySimudata->Accz) > LIMITACCNORMAL
		|| fabs(SySimudata->Gyrox) > LIMITGYROXYNORMAL
		|| fabs(SySimudata->Gyroy) > LIMITGYROXYNORMAL
		|| fabs(SySimudata->Gyroz) > LIMITGYROZNORMAL)
	{
		mGnssInsSystem.mErrorType = IMUDataExp;
#ifdef DEBUGMESSAGE
		printf("IMU Data:data error, gnss_week: %d, gnss_time0fweek: %.4f\n", SySimudata->week, SySimudata->timestamp);
#endif // DEBUGMESSAGE
		ret = 5;
		return ret;
	}

	ret = 1;	
	return ret;
}
static int BiasDateLength = 0;
static int BiasCheckDataLegth = 20;
static float BiasdetectData[20][6] = { {0.0} };
static float BiasMax[6] = { 0.0 };
static float BiasMean[6] = { 0.0 };
static float BiasVar[6] = { 0.0 };
static float BiasStd[6] = { 0.0 };
static float BiasThreShold[6] = { 20 * PI / 180 / 3600,20 * PI / 180 / 3600,20 * PI / 180 / 3600,
	0.016,0.016 ,0.016 };
// 检查IMU BIAS是否稳定
// 已分析
static int8_t CheckImuBiasEstStb(float* bias)
{
	int8_t ret = -1;
    // 数据放入BiasdetectData数组
	if (BiasDateLength == BiasCheckDataLegth)
	{
		for (int16_t i = 0; i < BiasDateLength - 1; i++)
		{
			for (int16_t j = 0; j < 6; j++)
			{
				BiasdetectData[i][j] = BiasdetectData[i + 1][j];
			}
		}
		BiasdetectData[BiasDateLength - 1][0] = *bias;
		BiasdetectData[BiasDateLength - 1][1] = *(bias+1);
		BiasdetectData[BiasDateLength - 1][2] = *(bias + 2);
		BiasdetectData[BiasDateLength - 1][3] = *(bias + 3);
		BiasdetectData[BiasDateLength - 1][4] = *(bias + 4);
		BiasdetectData[BiasDateLength - 1][5] = *(bias + 5);
	}
	if (BiasDateLength < BiasCheckDataLegth)
	{
		BiasdetectData[BiasDateLength][0] = *(bias);
		BiasdetectData[BiasDateLength][1] = *(bias + 1);
		BiasdetectData[BiasDateLength][2] = *(bias + 2);
		BiasdetectData[BiasDateLength][3] = *(bias + 3);
		BiasdetectData[BiasDateLength][4] = *(bias + 4);
		BiasdetectData[BiasDateLength][5] = *(bias + 5);
		BiasDateLength += 1;
		ret = -1;
		return ret;
	}
	memset(BiasMax, 0, 6 * sizeof(float));
	memset(BiasMean, 0, 6 * sizeof(float));
	memset(BiasVar, 0, 6 * sizeof(float));
	memset(BiasStd, 0, 6 * sizeof(float));
	for (int16_t i = 0; i < BiasDateLength; i++)
	{
		for (int16_t j = 0; j < 6; j++)
		{
			BiasMax[j] += BiasdetectData[i][j];
		}
	}
	for (int16_t j = 0; j < 6; j++)
	{
		BiasMean[j] = BiasMax[j] / BiasDateLength;
	}
	for (int16_t i = 0; i < BiasDateLength; i++)
	{
		for (int16_t j = 0; j < 6; j++)
		{
			BiasVar[j] += (BiasdetectData[i][j] - BiasMean[j])*(BiasdetectData[i][j] - BiasMean[j]);
		}
	}
	for (int16_t j = 0; j < 6; j++)
	{
		BiasStd[j] = sqrt(BiasVar[j] / BiasDateLength);
	}
	if (BiasStd[0] < BiasThreShold[0] && BiasStd[1] < BiasThreShold[1] && BiasStd[2] < BiasThreShold[2]
		&& BiasStd[3] < BiasThreShold[3] && BiasStd[4] < BiasThreShold[4] && BiasStd[5] < BiasThreShold[5])
	{
		ret = 1;
	}
	else
	{
		ret = 0;
	}
	return ret;
}
static int8_t MechanizationINT(const ImuData* imudata)
{
	int8_t ret = 0;
	float DeltaT = 0.0;
	DeltaT = (float)(imudata->timestamp - mGnssInsSystem.mpreImuData.timestamp);

	if (DeltaT < 0.00001)
	{
		ret = -1;
#ifdef DEBUGMESSAGE
		printf("IMU Data:interrupt error1, gnss_week: %d, gnss_time0fweek: %.4f\n", imudata->week, imudata->timestamp);
#endif // DEBUGMESSAGE
		return ret;
	}
	DataChangeLC(imudata, &mGnssInsSystem.mInsData); // 赋当前INS所需的IMU数据
	compensate(&mGnssInsSystem.mNav.sensorbias, &mGnssInsSystem.mInsData); // 减去bias

	if (PositionInitFlag) // 不会执行到
	{
		initsystemfromGNSS_STATUS1();
		PositionInitFlag = 0;
	}
	if (isEFKFinished) // EKF 结束标志
	{
		/*feed back time for output*/
        // 观测反馈的时间
		if (mGnssInsSystem.outPerMeasUpdata&GNSSPoitionUpdate) //&& (mGnssInsSystem.PreGNSSQUALITYMode == 4 || mGnssInsSystem.PreGNSSQUALITYMode == 5)
		{
			mGnssInsSystem.lastgnssfeedbacktime = imudata->timestamp;
		}
		if (mGnssInsSystem.outPerMeasUpdata&ZuptUpdate)
		{
			mGnssInsSystem.lastzuptfeedbacktime = imudata->timestamp;
		}
		if (mGnssInsSystem.outPerMeasUpdata&OdoUpdate)
		{
			mGnssInsSystem.lastodofeedbacktime = imudata->timestamp;
		}
		if (mGnssInsSystem.outPerMeasUpdata&NHCUpdate)
		{
			mGnssInsSystem.lastnhcfeedbacktime = imudata->timestamp;
		}




		KFStatus = 2;
        // 状态预测
        // X_pred = PHI * X,  mGnssInsSystem.mKalmanStruct.X = mUpdataStruct.PHI * mUpdataStruct.X
		MatrixMutiplyfloat(mUpdataStruct.PHI, mUpdataStruct.X, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.n, 1, mGnssInsSystem.mKalmanStruct.X);
		//memcpy(mUpdataStruct.X, mGnssInsSystem.mKalmanStruct.X, 16 * sizeof(float));
		float PHIP[StateX2], PHIt[StateX2];
        // 状态预测均方差
        // PHIP = PHI * P,  PHIP = mUpdataStruct.PHI * mUpdataStruct.P
		MatrixMutiplyfloat(mUpdataStruct.PHI, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.n, PHIP);
        // PHIt = PHI',  PHIt = mUpdataStruct.PHI'
		MatrixTransposefloat(mUpdataStruct.PHI, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.n, PHIt);
        // P主 = PHIP * PHIt, mGnssInsSystem.mKalmanStruct.P = PHIP * PHIt
		MatrixMutiplyfloat(PHIP, PHIt, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.P);
        // P = P主 + Q, mGnssInsSystem.mKalmanStruct.P = mGnssInsSystem.mKalmanStruct.P + mUpdataStruct.Q
		MatrixAddfloat(mGnssInsSystem.mKalmanStruct.P, mUpdataStruct.Q, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.P);

		int16_t feedback = 1;
		{
			//判断零偏估计是否稳定，不稳定返回0
			mGnssInsSystem.BiasEstStab = CheckImuBiasEstStb(&(mGnssInsSystem.mNav.sensorbias.bias_gyro_x));
		}
        // 反馈
		KF_feedback(&(mGnssInsSystem.mPar), mGnssInsSystem.mKalmanStruct.X, &(mGnssInsSystem.mNav), feedback, &mGnssInsSystem);
		isEFKFinished = 0;
		KFStatus = 0;
		IsFeedback = 1;
		ret = 2;
	}

    // 把当前的解算结果mNav拷贝到前一解算结果mpreNav
	memcpy(&(mGnssInsSystem.mpreNav), &(mGnssInsSystem.mNav), sizeof(Nav));

    // 0,默认 1,2,3 gnss, 4 nhc
	int8_t type = IsNeedLC(mGnssInsSystem.mpreInsData.timestamp, mGnssInsSystem.mInsData.timestamp, mGnssInsSystem.nextLCTime);

	float phi[StateX2];

	if (type == 1)
	{
		INS_MECH(&mGnssInsSystem.mpreInsData, &mGnssInsSystem.mInsData, &mGnssInsSystem.mpreNav, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar);

		KF_predict_16PHI(DeltaT, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar, &mGnssInsSystem.mIMUSensor,mGnssInsSystem.mKalmanStruct.n, phi);
		KF_predict(mGnssInsSystem.mKalmanStruct.n, phi, mGnssInsSystem.Q, DeltaT, mGnssInsSystem.mKalmanStruct.X, mGnssInsSystem.mKalmanStruct.P , &mUpdataStruct);

		GetLCData(&mGnssInsSystem.mNav, mGnssInsSystem.nextLCTime,1); // 数据更新到mUpdataStruct
	}
	else if (type == 2)
	{
		GetLCData(&mGnssInsSystem.mNav, mGnssInsSystem.nextLCTime,1);
		INS_MECH(&mGnssInsSystem.mpreInsData, &mGnssInsSystem.mInsData, &mGnssInsSystem.mpreNav, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar);

		KF_predict_16PHI(DeltaT, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar, &mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, phi);
		KF_predict(mGnssInsSystem.mKalmanStruct.n, phi, mGnssInsSystem.Q, DeltaT, mGnssInsSystem.mKalmanStruct.X, mGnssInsSystem.mKalmanStruct.P, &mUpdataStruct);
	}
	else if (type == 3)
	{
		INS_MECH(&mGnssInsSystem.mpreInsData, &mGnssInsSystem.mInsData, &mGnssInsSystem.mpreNav, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar);
		KF_predict_16PHI(DeltaT, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar, &mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, phi);
		KF_predict(mGnssInsSystem.mKalmanStruct.n, phi, mGnssInsSystem.Q, DeltaT, mGnssInsSystem.mKalmanStruct.X, mGnssInsSystem.mKalmanStruct.P, &mUpdataStruct);

		double dt = (imudata->timestamp - mGnssInsSystem.nextLCTime) / (double)(DeltaT);
		mGnssInsSystem.mpreNav.lat = (1 - dt)*mGnssInsSystem.mNav.lat + mGnssInsSystem.mpreNav.lat *dt;
		mGnssInsSystem.mpreNav.lon = (1 - dt)*mGnssInsSystem.mNav.lon + mGnssInsSystem.mpreNav.lon *dt;
		mGnssInsSystem.mpreNav.height = (1 - dt)*mGnssInsSystem.mNav.height + mGnssInsSystem.mpreNav.height *dt;
		GetLCData(&mGnssInsSystem.mpreNav, mGnssInsSystem.nextLCTime,1);
	}
	else if (type == 4)
	{
		INS_MECH(&mGnssInsSystem.mpreInsData, &mGnssInsSystem.mInsData, &mGnssInsSystem.mpreNav, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar);
		KF_predict_16PHI(DeltaT, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar, &mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, phi);
		KF_predict(mGnssInsSystem.mKalmanStruct.n, phi, mGnssInsSystem.Q, DeltaT, mGnssInsSystem.mKalmanStruct.X, mGnssInsSystem.mKalmanStruct.P, &mUpdataStruct);
		GetLCData(&mGnssInsSystem.mNav, mGnssInsSystem.nextNHCLCTime, 2);
	}
	else
	{
		INS_MECH(&mGnssInsSystem.mpreInsData, &mGnssInsSystem.mInsData, &mGnssInsSystem.mpreNav, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar);
		KF_predict_16PHI(DeltaT, &mGnssInsSystem.mNav, &mGnssInsSystem.mPar, &mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, phi);
		KF_predict(mGnssInsSystem.mKalmanStruct.n, phi, mGnssInsSystem.Q, DeltaT, mGnssInsSystem.mKalmanStruct.X, mGnssInsSystem.mKalmanStruct.P, &mUpdataStruct);
	}

    // 把当前的IMU数据拷贝到前一个IMU数据
	memcpy(&mGnssInsSystem.mpreInsData, &mGnssInsSystem.mInsData, sizeof(INS));
	ret |= 1;
	return ret;
}
void ConvertIMU2USER(Nav* p_Nav,double UserLeverarm[3],double Rvb[3])
{
//	double C_nb[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
	double C_nv[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
	double C_vn[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

	if(p_prcopt.isUseMisAlignment)
	{
		double eular[3] = { p_Nav->roll, p_Nav->pitch,p_Nav->heading };
		//double C_bn[3][3];
		MatrixMutiply(*p_Nav->c_bn, *mGnssInsSystem.C_InstallationAngle, 3, 3, 3, *C_vn);
		MatrixTranspose(*C_vn, 3, 3, *C_nv);
		dcm2euler(C_vn, eular);
		p_Nav->roll = (float)eular[0];
		p_Nav->pitch = (float)eular[1];
		p_Nav->heading = (float)eular[2];
	}
	else
	{
		MatrixTranspose(*p_Nav->c_bn, 3, 3, *C_nv);
		memcpy(*C_vn, *p_Nav->c_bn, 9 * sizeof(double));
	}

	MatrixMutiply(*C_nv, p_Nav->a_n, 3, 3, 1, p_Nav->a_b);
	MatrixMutiply(*C_nv, p_Nav->wnb_n, 3, 3, 1, p_Nav->w_b);

	if (p_prcopt.isUserOutPut)
	{
		double v[3] = { p_Nav->vn, p_Nav->ve, p_Nav->vd };
		double v_out[3];
		//double C_bv[3][3];
		double temp1[3], temp2[3];
		double C1[3][3];
		GetSkewSymmetricMatrixOfVector(p_Nav->wnb_b, *C1);   //C1 Mwnbn
		if (p_prcopt.isUseMisAlignment)
		{
			double leverarm_b[3];
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, UserLeverarm, 3, 3, 1, leverarm_b);  //temp1 Mbv
			MatrixMutiply(*C1, leverarm_b, 3, 3, 1, temp2);  //temp1 Mbv
			MatrixMutiply(*p_Nav->c_bn, temp2, 3, 3, 1, temp1);
			MatrixAdd(v, temp1, 3, 1, v_out);
		}
		else
		{
			MatrixMutiply(*C1, UserLeverarm, 3, 3, 1, temp2);  //temp1 Mbv
			MatrixMutiply(*p_Nav->c_bn, temp2, 3, 3, 1, temp1);
			MatrixAdd(v, temp1, 3, 1, v_out);
		}

		p_Nav->vn = (float)v_out[0];
		p_Nav->ve = (float)v_out[1];
		p_Nav->vd = (float)v_out[2];
	}

	if (p_prcopt.isUserOutPut)
	{
		double leverarm_n[3];
		MatrixMutiply(*C_vn, UserLeverarm, 3, 3, 1, leverarm_n);
		double pos[3] = { p_Nav->lat, p_Nav->lon, p_Nav->height };
		double M, N;
		UpdateMN(pos, &M, &N);
		double d_leverarm[3];
		d_leverarm[0] = leverarm_n[0] / (M + pos[2]);
		d_leverarm[1] = leverarm_n[1] / ((N + pos[2])*cos(pos[0]));
		d_leverarm[2] = -leverarm_n[2];

		MatrixAdd(pos, d_leverarm, 3, 1, pos);
		p_Nav->lat = pos[0];
		p_Nav->lon = pos[1];
		p_Nav->height = pos[2];
	}
}
static int8_t zuptlock()
{
	int8_t ret = -1;
	if (mUpdataStruct.Zuptlockflag == 1 && IsFeedback == 1)
	{
		if (mUpdataStruct.IsUseZUPT )// && mGnssInsSystem.mImuData.timestamp- mGnssInsSystem.firstGNSSUseTime/*- mGnssInsSystem.zupttime2*/>115200)
		{
			memcpy(&lastzuptnav, &mGnssInsSystem.mNav, sizeof(Nav));
    		lastzupttime = mGnssInsSystem.mImuData.timestamp;
			if (mGnssInsSystem.BiasEstStab == 1)
			{
				mGnssInsSystem.zupttime2 += mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastzuptdetcttime;
				if (mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastzuptdetcttime > 1.5
					&& mGnssInsSystem.firstzupt
					&& mGnssInsSystem.lastzuptdetcttime > 0.001)
				{
					mGnssInsSystem.firstzupt = 0;
				}
				mGnssInsSystem.lastzuptdetcttime = mGnssInsSystem.mImuData.timestamp;
				int8_t curzupt = 0;
				if (p_prcopt.isUseOdo)
				{
					if (fabs(mGnssInsSystem.mOdoData.vehicle_speed) < 0.05 && GetZuptVal())
					{
						curzupt = 1;
					}
				}
				else
				{
					if (GetZuptVal())
					{
						curzupt = 1;
					}
				}


		if (curzupt)
		{
			mGnssInsSystem.mNav.vn = 0.0;
			mGnssInsSystem.mNav.ve = 0.0;
			mGnssInsSystem.mNav.vd = 0.0;
		}
		if (!curzupt)
		{
			mGnssInsSystem.lastzuptdetcttime = mGnssInsSystem.mImuData.timestamp;
			mGnssInsSystem.firstzupt = 0;
			mUpdataStruct.Zuptlockflag = 0;
			IsFeedback = 0;
		}

				if (!mGnssInsSystem.firstzupt &&curzupt)
				{
					ret = 2;
					mGnssInsSystem.firstzupt = 1;
					memcpy(&mGnssInsSystem.zuptNav, &mGnssInsSystem.mNav, sizeof(Nav));
				}
				else if (mGnssInsSystem.firstzupt)
				{
					if (curzupt)
					{
						ret = 1;
						mGnssInsSystem.mNav.roll = mGnssInsSystem.zuptNav.roll;
						mGnssInsSystem.mNav.pitch = mGnssInsSystem.zuptNav.pitch;
						mGnssInsSystem.mNav.heading = mGnssInsSystem.zuptNav.heading;
						memcpy(mGnssInsSystem.mNav.c_bn, mGnssInsSystem.zuptNav.c_bn, 9 * sizeof(double));
						memcpy(mGnssInsSystem.mNav.q_bn, mGnssInsSystem.zuptNav.q_bn, 4 * sizeof(double));
						memcpy(&lastzuptnav, &mGnssInsSystem.mNav, sizeof(Nav));
						lastzupttime = mGnssInsSystem.mImuData.timestamp;
					}
				}
				mUpdataStruct.Zuptlockflag = 0;
				IsFeedback = 0;
			}

		}
		else if ( mUpdataStruct.IsUseZUPT != 1 )
		{
			mGnssInsSystem.lastzuptdetcttime = mGnssInsSystem.mImuData.timestamp;
			mGnssInsSystem.firstzupt = 0;
		}
		mUpdataStruct.Zuptlockflag = 0;
		IsFeedback = 0;
	}
	return ret;
}
/*  ADD origin imu data and INS process 
	args:  ImuData *imudata              origin imu data struct
	return:status ( -3: System crash
	                -2: Incorrect ins time
	                -1: loss imu data too much and reset
					 1: ok;
					 2: right IMU data interval is too low
					 3: loss imu data and not reset
					 4: Data overrun
					 5: Vehicle threshold exceeded)

*/
int8_t AddIMUData(const ImuData mImudata)
{
	int8_t retal = -1;
	mGnssInsSystem.outPerMeasUpdata = mGnssInsSystem.PerMeasUpdata;
    mGnssInsSystem.PerMeasUpdata = 0;
	mGnssInsSystem.INSPROCESSMODE = 0;

	if (p_prcopt.insinitfromprev == 1) { // 开机定位模式, 先不看
        if (mGnssInsSystem.systemtime_ready == 0) {
            if (mGnssInsSystem.firstimutime < 0.0) {
                mGnssInsSystem.firstimutime = mImudata.timestamp;
            }
        }

        if (mImudata.week > 1024 && mImudata.week < 3072)
		{
			if (mGnssInsSystem.systemtime_ready == 0)
			{
				mGnssInsSystem.systemtime_ready = 1;
				mGnssInsSystem.firstimutime = mImudata.timestamp + mGnssInsSystem.mpreImuData.timestamp - mGnssInsSystem.firstimutime + 1.0 / p_prcopt.imuDataRate;
				mGnssInsSystem.mpreImuData.timestamp = mImudata.timestamp - 1.0 / p_prcopt.imuDataRate;
				DataChangeLC(&mGnssInsSystem.mpreImuData, &mGnssInsSystem.mpreInsData);
            }
		}

		if (mGnssInsSystem.systemtime_ready == 1)
		{
			if (mImudata.week > 1024 && mImudata.week < 3072)
			{
			}
			else
			{
				retal = -2;
				return retal;
			}
		}

	} else {
        if (mImudata.week > 1024 && mImudata.week < 3072) {
            if (mGnssInsSystem.firstimutime < 0.0) {    // 只进一次，初始化第一个IMU时间信息
                mGnssInsSystem.systemtime_ready = 1;    // 时间就绪标志
                mGnssInsSystem.firstimutime = mImudata.timestamp;   // 初始化第一个IMU时间
                mGnssInsSystem.mpreImuData.timestamp = mImudata.timestamp - 1.0 / p_prcopt.imuDataRate; // 初始化前一个IMU时间
            }
        } else {
            retal = -2;
            return retal;
        }
    }



    //print
	retal = ConvertIMUData(&mImudata, &mGnssInsSystem.mImuData); // 转换成算法所需的IMU数据

	if (-1 == retal || 2 == retal || 4 == retal)
	{
		return retal;
	}

	if (p_prcopt.insinitfromprev == 1) // 开机定位模式, 先不看
	{
		SetZuptDetectData(mGnssInsSystem.mImuData, p_prcopt.imuDataRate);
		int8_t IMUMoveMode = GetIMUMoveMode();
	}
	else if (mGnssInsSystem.mlc_STATUS != INSDATA_NREADY) //
	{
		SetZuptDetectData(mGnssInsSystem.mImuData, p_prcopt.imuDataRate); // 填入Zuptimu数据
		//int8_t IMUMoveMode = GetIMUMoveMode();  
		// if (IMUMoveMode != 0)
		// {
		// 	mGnssInsSystem.mErrorType = MisAligmentChange;
		// 	InitHardModeAxix(p_prcopt.MisAlignmentAiax);
		// 	ErrorReset(mGnssInsSystem.mErrorType);
		// 	mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
		// }
	}
	mGnssInsSystem.ins_status = INS_INACTIVE;
    // UART7_Printf("timestamp:%f,firstimutime:%f\r\n",mGnssInsSystem.mImuData.timestamp,mGnssInsSystem.firstimutime);
    switch (mGnssInsSystem.mlc_STATUS) {
        case INSDATA_NREADY:  // INSDATA_NREADY为初始化模式
        {
            int8_t IsImuNormal = 0;  // CheckImuDataBase(mGnssInsSystem.mImuData);
            // 数据持续5s后进入 INSDATA_READY
            if (mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.firstimutime > 5 && mGnssInsSystem.firstimutime > 0.0) {
                IsImuNormal = 1;
            }
            if (IsImuNormal) {
                mGnssInsSystem.mlc_STATUS = INSDATA_READY;
                mGnssInsSystem.ins_status = INS_INACTIVE;
#ifdef DEBUGMESSAGE
                printf("INS FUSION :imu data ready, gnss_week: %d, gnss_time0fweek: %.4f\n", mGnssInsSystem.mImuData.week, mGnssInsSystem.mImuData.timestamp);
#endif  // DEBUGMESSAGE
            }
        } break;
        case INSDATA_READY: 
        {
            if (!p_prcopt.isOnlineMisAlignmentEst) { // 不使用安装角在线估计，直接跳过
                mGnssInsSystem.mlc_STATUS = MISALIGNMENT_COMPLETE;
            }
        } break;
        case MISALIGNMENT_ING: // 正在进行安装角估计
        {
        } break;
        case MISALIGNMENT_COMPLETE: // 安装角估计完成
        {
            if (mGnssInsSystem.insinitfromprev == 1) { // 开机定位模式, 先不看
                // zupt and bias check
                double Abias[6] = {0.0};
                if (ismeetstartcondition(Abias) == 1) {
                    // check bias is appropriate
                    double eular[3] = {p_prcopt.initpar[10], p_prcopt.initpar[11], p_prcopt.initpar[12]};
                    euler2dcm(eular, mGnssInsSystem.mNav.c_bn);
                    double c_nb[3][3] = {0.0};
                    double g[3] = {0.0, 0.0, -9.8};
                    double g1[3] = {0.0, 0.0, -9.8};
                    MatrixTranspose(*mGnssInsSystem.mNav.c_bn, 3, 3, *c_nb);
                    MatrixMutiply(*c_nb, g, 3, 3, 1, g1);
                    double accbias[3] = {0.0};
                    MatrixSub(&(Abias[3]), g1, 3, 1, accbias);

                    double gyro_1[3] = {0.0, 0.0, 0.0};  // ignore earth rotation
                    double error_bias[6] = {0.0};
                    for (int i = 0; i < 3; i++) {
                        error_bias[i] = p_prcopt.gyroBias[i] - Abias[i];
                    }
                    for (int i = 0; i < 3; i++) {
                        error_bias[i + 3] = p_prcopt.accBias[i] - accbias[i];
                    }

                    int flag = 1;
                    for (int i = 0; i < 3; i++) {
                        if (fabs(error_bias[i]) > 1 * PI / 180 || fabs(error_bias[i + 3]) > 0.2) {
                            flag &= 0;
                        } else {
                            flag = 1;
                        }
                    }

                    if (flag == 1) {
                        initsystemSoftreset();
                        initsystemfromPre(error_bias);
                        mGnssInsSystem.AlignCTime = mGnssInsSystem.mImuData.timestamp;
                        mGnssInsSystem.mlc_STATUS = INS_FUSING;
                    } else {
                        mGnssInsSystem.mErrorType = BiasChangeError;
                        ErrorReset(mGnssInsSystem.mErrorType);
                        mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
                    }
                    // cal acc BIAS
                } else {
                }
                mGnssInsSystem.insinitfromprev = 2;
            }
        } break;
        case ALIGNMENT_COMPLETE:
        {

        }break;
        case INS_FUSING: // 初始对准后
        {
            // 初始化3分钟内，出现连续3个GNSS丢失/SPP/RTD后，停车直接重新初始化，float不恢复，连续的3个FIX才恢复
            if (mImudata.timestamp - mGnssInsSystem.AlignCTime < 180) {
                double todsec = mImudata.timestamp - floor(mImudata.timestamp + 0.5);
                if (fabs(todsec) < 0.005) {
                    uint8_t pregnss_status = 0;
                    if (mImudata.timestamp - mGnssInsSystem.premGnssData.timestamp > 1.01) { // 丢失gnss
                        pregnss_status = 0;
                    } else {
                        if (mGnssInsSystem.premGnssData.gpsFixType <= 2) { // SPP/RTD
                            pregnss_status = 1;
                        } else if (mGnssInsSystem.premGnssData.gpsFixType == 5) {
                            pregnss_status = 2; // float
                        } else if (mGnssInsSystem.premGnssData.gpsFixType == 4) {
                            pregnss_status = 3; // fix
                        }
                    }

                    if (pregnss_status <= 1) {
                        mGnssInsSystem.system_init_gnssbadflag++;
                        if (mGnssInsSystem.system_init_gnssbadflag >= 3) { // 连续3个GNSS丢失/SPP/RTD，中间没FIX
                            mGnssInsSystem.system_init_status = 0x00;
                        }
                        mGnssInsSystem.system_init_gnssgoodflag = 0;

                    } else if (pregnss_status == 2) {
                        mGnssInsSystem.system_init_gnssgoodflag = 0;
                        
                    } else if (pregnss_status == 3) {
                        mGnssInsSystem.system_init_gnssgoodflag++;
                        if (mGnssInsSystem.system_init_gnssbadflag <= 2) {
                            mGnssInsSystem.system_init_gnssbadflag = 0;
                        }
                        if (mGnssInsSystem.system_init_gnssgoodflag >= 3) { // 连续3个FIX
                            mGnssInsSystem.system_init_status = 0x01;
                        }
                    }
                }
            } else {
                mGnssInsSystem.system_init_status = 0x01; // 超过时间认为正常
            }
            

            if (1 == mGnssInsSystem.Isfirstfusionimu) // 初始化是1，相当于只执行1次
            {
                DataChangeLC(&mGnssInsSystem.mpreImuData, &mGnssInsSystem.mpreInsData); // 初始化前一个INS的IMU数据
                mGnssInsSystem.Isfirstfusionimu = 0;
            }
            if (mGnssInsSystem.mImuData.timestamped > 1 && p_prcopt.gnssDataRate == 0) //unkonwn gnssDataRate
            {
                SetLCtime(mGnssInsSystem.mImuData.timestamped);
            }
            /*ZUPTLOCK */
            if (p_prcopt.isUseZUPTLOCK) // 使用
            {
                if (p_prcopt.gnssDataRate == 1) // 为1
                {
                    if (1 == zuptlock()) // 要先执行MechanizationINT，才可能返回1
                    {
                        mGnssInsSystem.outPerMeasUpdata |= ZuptLock;
                        mGnssInsSystem.lastzuptlocktime = mGnssInsSystem.mInsData.timestamp;
                    }
                }
            }

            //// 1 INS MECHINE 2 EKF FEED BACK 3 OBS UPDATA NO FEED BACK
            mGnssInsSystem.INSPROCESSMODE = MechanizationINT(&mGnssInsSystem.mImuData);

            if (mGnssInsSystem.INSPROCESSMODE == 3)
            {
                /*ins ekf feedback*/
                if (1 == p_prcopt.isusefliter) // 不使用
                {
                    int8_t isgnssupdate = 0;
                    if (mGnssInsSystem.mMeasUpdataType == GNSSPoitionUpdate)
                    {
                        isgnssupdate = 1;
                    }
                    positionfilter_set(mUpdataStruct.X, &(mGnssInsSystem.posfliter), isgnssupdate, mUpdataStruct.IsUseZUPT,
                        mGnssInsSystem.mPar.Rm, mGnssInsSystem.mPar.Rn, mGnssInsSystem.outNav.height, mGnssInsSystem.outNav.lat);
                }
                memset(mUpdataStruct.X, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));

            }
            /*VirtualObsUpdata NHC ZUPT*/
            if (p_prcopt.isUseOdo && mGnssInsSystem.isOdoInterfaceGOOD
                && fabs(mGnssInsSystem.mInsData.timestamp - mGnssInsSystem.nextNHCLCTime) < 0.00001)
            {
                if (OdoObsUpdata(mGnssInsSystem.mInsData.timestamp) > 0)
                {
                    mGnssInsSystem.INSPROCESSMODE |= 8;
                }
            }
            else if (fabs(mGnssInsSystem.mInsData.timestamp - mGnssInsSystem.nextNHCLCTime) < 0.00001)
            {
                if(VirtualObsUpdata(mGnssInsSystem.mInsData.timestamp)==1)mGnssInsSystem.INSPROCESSMODE |= 4;
            }

        }break;
        default:
            retal = -3;
            return  retal;
    }
        /* OutPut*/
	mGnssInsSystem.ins_positin_type = INS_NONE;
	if (mGnssInsSystem.mlc_STATUS == INS_FUSING)
	{
		if (fabs(mGnssInsSystem.mInsData.timestamp - mGnssInsSystem.lastGNSSLCTime) < 3 && mGnssInsSystem.firstGNSSTime != -1)
				//&& fabs(mGnssInsSystem.lastGNSSLCTime - mGnssInsSystem.mGnssData.timestamp) < 0.01)
		{
			mGnssInsSystem.ins_positin_type = mGnssInsSystem.lastGNSSLCType;
		}
		else
		{
			mGnssInsSystem.ins_positin_type = INS_PROPAGATED;
		}
	}
    // 拷贝结果到输出结构体
	memcpy(&mGnssInsSystem.outNav, &mGnssInsSystem.mNav, sizeof(Nav));
	if (mGnssInsSystem.mlc_STATUS == INS_FUSING)
	{
		mGnssInsSystem.ins_status = INS_HIGH_VARIANCE;
		double standarddeviation = sqrt(mGnssInsSystem.mKalmanStruct.P[0]  + mGnssInsSystem.mKalmanStruct.P[1+ mGnssInsSystem.mKalmanStruct.n]);
		
		if (standarddeviation < 0.5 && mGnssInsSystem.BiasEstStab == 1)// && mGnssInsSystem.BiasEstStab == 1)
		{
			mGnssInsSystem.ins_status = INS_SOLUTION_GOOD;
		}
		if (mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastGNSSLCTime > 10 && mGnssInsSystem.lastGNSSLCTime > 0)
		{
			mGnssInsSystem.ins_status = INS_SOLUTION_FREE;
		}
		if (1)//p_prcopt.isUserOutPut Output result vechile system 
		{
			if (1 == p_prcopt.isusefliter) // 不使用
			{
				positionfilter_compensent((double*)&mGnssInsSystem.outNav, &mGnssInsSystem.posfliter, mGnssInsSystem.mPar.Rm, mGnssInsSystem.mPar.Rn, mGnssInsSystem.outNav.height, mGnssInsSystem.outNav.lat);
			}
            // 转到用户输出投影点
			ConvertIMU2USER(&mGnssInsSystem.outNav, p_prcopt.userLeverArm, p_prcopt.misAlignment);
		}
	}
	else
	{
		if (mGnssInsSystem.insinitfromprev == 1 && mGnssInsSystem.mlc_STATUS != INS_FUSING)
		{
			bootpositioninit();
			memcpy(&mGnssInsSystem.outNav, &mGnssInsSystem.mNav, sizeof(Nav));
			ConvertIMU2USER(&mGnssInsSystem.outNav, p_prcopt.userLeverArm, p_prcopt.misAlignment);

		}
		else if (fabs(mGnssInsSystem.outNav.lat) < 0.01 &&
			fabs(mGnssInsSystem.outNav.lon) < 0.01 &&
			fabs(mGnssInsSystem.outNav.height) < 0.01
			&& mGnssInsSystem.firstGNSSTime != -1)
		{
			mGnssInsSystem.outNav.lat = mGnssInsSystem.mGnssData.latitude;
			mGnssInsSystem.outNav.lon = mGnssInsSystem.mGnssData.longitude;
			mGnssInsSystem.outNav.height = mGnssInsSystem.mGnssData.altitude;
			mGnssInsSystem.outNav.vn = mGnssInsSystem.mGnssData.north_velocity;
			mGnssInsSystem.outNav.ve = mGnssInsSystem.mGnssData.east_velocity;
			mGnssInsSystem.outNav.vd = -mGnssInsSystem.mGnssData.up_velocity;
			mGnssInsSystem.outNav.roll = 0;
			mGnssInsSystem.outNav.pitch = 0;
			mGnssInsSystem.outNav.heading = mGnssInsSystem.mGnssData.heading;
		}
		else if (mGnssInsSystem.firstGNSSTime != -1)
		{
			mGnssInsSystem.outNav.lat = mGnssInsSystem.mGnssData.latitude;
			mGnssInsSystem.outNav.lon = mGnssInsSystem.mGnssData.longitude;
			mGnssInsSystem.outNav.height = mGnssInsSystem.mGnssData.altitude;
			mGnssInsSystem.outNav.vn = mGnssInsSystem.mGnssData.north_velocity;
			mGnssInsSystem.outNav.ve = mGnssInsSystem.mGnssData.east_velocity;
			mGnssInsSystem.outNav.vd = -mGnssInsSystem.mGnssData.up_velocity;
			mGnssInsSystem.outNav.roll = 0;
			mGnssInsSystem.outNav.pitch = 0;
			mGnssInsSystem.outNav.heading = mGnssInsSystem.mGnssData.heading;
		}
	}

	/*	int16_t id_continent;
       int32_t Ext_sol_stat;			    //!< Extended solution status
	    uint16_t time_since_update;         //  Elapsed time since the last ZUPT or NHC or positionupdate (seconds)*/
	if (mGnssInsSystem.mlc_STATUS == INS_FUSING)
	{
		if (mGnssInsSystem.outPerMeasUpdata & GNSSPoitionUpdate
			|| mGnssInsSystem.outPerMeasUpdata & ZuptUpdate
			|| mGnssInsSystem.outPerMeasUpdata & OdoUpdate
			|| mGnssInsSystem.outPerMeasUpdata & NHCUpdate
			|| mGnssInsSystem.outPerMeasUpdata & ZuptLock)
		{
			mGnssInsSystem.Ext_sol_stat = mGnssInsSystem.Ext_sol_stat | (uint16_t)mGnssInsSystem.outPerMeasUpdata;
		}
		/*			double lastgnssfeedbacktime;
				double lastodofeedbacktime;
				double lastnhcfeedbacktime;
				double lastzuptfeedbacktime;
				double lastzuptlocktime;*/
		double* p_curtime = &(mGnssInsSystem.mImuData.timestamp);
		if (*p_curtime - mGnssInsSystem.lastgnssfeedbacktime > 1)
		{
			mGnssInsSystem.Ext_sol_stat = mGnssInsSystem.Ext_sol_stat & 0xfe; // 0b11111110
		}
		if (*p_curtime - mGnssInsSystem.lastodofeedbacktime > 1)
		{
			mGnssInsSystem.Ext_sol_stat = mGnssInsSystem.Ext_sol_stat & 0xef; // 0b11101111
		}
		if (*p_curtime - mGnssInsSystem.lastnhcfeedbacktime > 1)
		{
			mGnssInsSystem.Ext_sol_stat = mGnssInsSystem.Ext_sol_stat & 0xf7; // 0b11110111
		}
		if (*p_curtime - mGnssInsSystem.lastzuptfeedbacktime > 1)
		{
			mGnssInsSystem.Ext_sol_stat = mGnssInsSystem.Ext_sol_stat & 0xbf; // 0b10111111
		}
		if (*p_curtime - mGnssInsSystem.lastzuptlocktime > 1)
		{
			mGnssInsSystem.Ext_sol_stat = mGnssInsSystem.Ext_sol_stat & 0xbb; // 0b10111011
		}

		double maxtime = mGnssInsSystem.lastgnssfeedbacktime;
		if (maxtime < mGnssInsSystem.lastodofeedbacktime)
		{
			maxtime = mGnssInsSystem.lastodofeedbacktime;
		}
		if (maxtime < mGnssInsSystem.lastnhcfeedbacktime)
		{
			maxtime = mGnssInsSystem.lastnhcfeedbacktime;
		}
		if (maxtime < mGnssInsSystem.lastzuptfeedbacktime)
		{
			maxtime = mGnssInsSystem.lastzuptfeedbacktime;
		}
		if (maxtime < mGnssInsSystem.lastzuptlocktime)
		{
			maxtime = mGnssInsSystem.lastzuptlocktime;
		}
		if (maxtime != -1)
		{
			mGnssInsSystem.time_since_update = 1000*(*p_curtime - maxtime);
		}
		else
		{
			mGnssInsSystem.time_since_update = 0;
		}
	}
	else
	{
		mGnssInsSystem.Ext_sol_stat = 0;
		mGnssInsSystem.time_since_update = 0;
	}

    if (mGnssInsSystem.mlc_STATUS == INS_FUSING) {
        if ((mGnssInsSystem.outPerMeasUpdata & ZuptUpdate) != 0) {
            if (mGnssInsSystem.system_init_status == 0x00) {
                // reset
                mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
                mGnssInsSystem.InitNavi = 0;
                mGnssInsSystem.mpreImuData.timestamp = 0;
                mGnssInsSystem.Isfirstfusionimu = 1;
                //softreset
                mGnssInsSystem.mErrorType = INITPOOR;
                ErrorReset(mGnssInsSystem.mErrorType);
#ifdef DEBUGMESSAGE
    printf("SYSTEM zupt error reset: gnss_week: %d, gnss_time0fweek: %.4f\n", mImudata.week, mImudata.timestamp);
#endif // DEBUGMESSAGE
            }
        }
    }


	mGnssInsSystem.mMeasUpdataType = MeasNone;
	//mGnssInsSystem.PerMeasUpdata = 0;
	//mGnssInsSystem.CurIsUseZupt = 0;
	mGnssInsSystem.mErrorType = 0;
	memcpy(&(mGnssInsSystem.mpreImuData), &(mGnssInsSystem.mImuData), sizeof(mGnssInsSystem.mImuData));
	retal = 1;
	return  retal;
}
