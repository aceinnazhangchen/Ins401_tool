#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "lcsystem.h"
#include "define.h"
#include "earth.h"
#include "orientation.h"
#include "cmatrix.h"
#include "zuptdetect.h"
#include "positionfilter.h"

#ifndef DEG2RAD
#define DEG2RAD (0.017453292519943)
#endif

#define MOTIONINIT_ROLLSTABILTIY   2         //deg
#define MOTIONINIT_PITCHSTABILTIY  2         //deg
#define MOTIONINIT_YAWSTABILTIY    4         //deg

#define MACRO_TO_STRING_BUILD(x)    #x
#define MACRO_TO_STRING(x)          MACRO_TO_STRING_BUILD(x)
uint8_t ins_version[] = "INS_LIB_v_" MACRO_TO_STRING(VERSION_INS_LIB);

extern LCSetting p_prcopt;
UpdataStruct mUpdataStruct;
GnssInsSystem mGnssInsSystem;
int8_t isEFKFinished = 0;
int8_t PositionInitFlag = 0;
int8_t IsFeedback = 0;
int KFStatus = 0; //  Kalman filter 0��oempty 1��oUpdata 2��ofeedback



int8_t SetImuSensorParameter(ImuSensor* imusensor, const int16_t mImuSensorType, const int16_t IMUDataRate)
{
	int8_t retal = -1;
	uint8_t i;
	enum ImuSensorType
	{
		SPAN_CPT = 0,
		ACEINNA,
		UBLOX_M8P
	};
	switch (mImuSensorType)
	{
	case SPAN_CPT:
	{
		for (i = 0; i < 3; ++i)
		{
			imusensor->Gyr_bias_std[i] = 0.45 * DEG2RAD / 3600;    //rad/s
			imusensor->Acc_bias_std[i] = 75 * 1.0e-5;               //m/s^2
		}
		imusensor->Gyr_noise_arw = 0.06 / 60 * DEG2RAD;              //rad/sqrt(sec)
		imusensor->Acc_noise_vrw = 0.06 / 60;                      //m/s/sqrt(sec)     
		imusensor->Gyr_bias_CorTime = 1.0 * 3600;                  //sec
		imusensor->Acc_bias_CorTime = 1.0 * 3600;                  //sec
		imusensor->Gyr_bias_rept = 20 * DEG2RAD / 3600;         //rad/s
		imusensor->Acc_bias_rept = 3 * 1.0e-2;                      //m/s^2

	}break;
	case ACEINNA:
	{
		if (p_prcopt.gnssDataRate == 0)
		{
			for (i = 0; i < 3; ++i)
			{
				imusensor->Gyr_bias_std[i] = 2 * DEG2RAD / 3600;
				imusensor->Acc_bias_std[i] = 100 * 1.0e-5;
			}
			imusensor->Gyr_noise_arw = 0.06 * DEG2RAD;
			imusensor->Acc_noise_vrw = 0.06 / 60;
			imusensor->Gyr_bias_CorTime = 0.2 * 3600;
			imusensor->Acc_bias_CorTime = 0.2 * 3600;

			imusensor->Gyr_bias_rept = 72 * DEG2RAD / 3600;         //rad/s
			imusensor->Acc_bias_rept = 30 * 1.0e-2;                      //m/s^2
		}
		else
		{
			//for (i = 0; i < 3; ++i)
			//{
			//	imusensor->Gyr_bias_std[i] = 5 * DEG2RAD / 3600;
			//	imusensor->Acc_bias_std[i] = 800 * 1.0e-5;
			//}
			////imusensor->Gyr_noise_arw = 0.06 / 60 * DEG2RAD;
			////imusensor->Acc_noise_vrw = 0.4 / 60;
			//imusensor->Gyr_noise_arw = 0.06 / 60 * DEG2RAD;
			//imusensor->Acc_noise_vrw = 0.4 / 60;
/*fisst*/
			//for (i = 0; i < 3; ++i)
			//{
			//	imusensor->Gyr_bias_std[i] = 10 * DEG2RAD / 3600;
			//	imusensor->Acc_bias_std[i] = 800 * 1.0e-5;
			//}
			////imusensor->Gyr_noise_arw = 0.06 / 60 * DEG2RAD;
			////imusensor->Acc_noise_vrw = 0.4 / 60;
			//imusensor->Gyr_noise_arw = 0.06 / 60 * DEG2RAD;
			//imusensor->Acc_noise_vrw = 0.4 / 60;
			//imusensor->Gyr_bias_CorTime = 0.2 * 3600;
			//imusensor->Acc_bias_CorTime = 0.2 * 3600;

			//imusensor->Gyr_bias_rept = 72 * DEG2RAD / 3600;         //rad/s
			//imusensor->Acc_bias_rept = 30 * 1.0e-2;                      //m/s^2

			for (i = 0; i < 3; ++i)
			{
				imusensor->Gyr_bias_std[i] = 10.0 * DEG2RAD / 3600;
				imusensor->Acc_bias_std[i] = 200.0 * 1.0e-5;
			}
			//imusensor->Gyr_noise_arw = 0.06 / 60 * DEG2RAD;
			//imusensor->Acc_noise_vrw = 0.4 / 60;
			imusensor->Gyr_noise_arw = 0.13 / 60 * DEG2RAD;
			imusensor->Acc_noise_vrw = 0.5 / 60;
			imusensor->Gyr_bias_CorTime = 0.2 * 3600;
			imusensor->Acc_bias_CorTime = 0.2 * 3600;

			imusensor->Gyr_bias_rept = 72 * DEG2RAD / 3600;         //rad/s
			imusensor->Acc_bias_rept = 30 * 1.0e-2;                      //m/s^2
		}

		//for (i = 0; i < 3; ++i)
		//{
		//	imusensor->Gyr_bias_std[i] = 2 * PI / 180 / 3600;
		//	imusensor->Acc_bias_std[i] = 1 * 1.0e-5;
		//}
		//imusensor->Gyr_noise_arw = 0.06 * (PI / 180);
		//imusensor->Acc_noise_vrw = 0.06 / 60;
		//imusensor->Gyr_bias_CorTime = 2 * 3600;
		//imusensor->Acc_bias_CorTime = 2 * 3600;

		//imusensor->Gyr_bias_rept = 72 * (PI / 180) / 3600;         //rad/s
		//imusensor->Acc_bias_rept = 0.01;                      //m/s^2
	}break;
	case UBLOX_M8P:
	{
		if (p_prcopt.imuDataRate == 10)
		{
			for (i = 0; i < 3; ++i)
			{
				imusensor->Gyr_bias_std[i] = 10 * DEG2RAD / 3600;
				imusensor->Acc_bias_std[i] = 200 * 1.0e-5;
			}
			imusensor->Gyr_noise_arw = 2.4 / 60 * DEG2RAD;
			imusensor->Acc_noise_vrw = 2.4 / 60;
			imusensor->Gyr_bias_CorTime = 0.2 * 3600;
			imusensor->Acc_bias_CorTime = 0.2 * 3600;

			imusensor->Gyr_bias_rept = 72 * DEG2RAD / 3600;         //rad/s
			imusensor->Acc_bias_rept = 10 * 1.0e-2;                      //m/s^2
		}
		else if (p_prcopt.imuDataRate == 100)
		{
			for (i = 0; i < 3; ++i)
			{
				imusensor->Gyr_bias_std[i] = 20 * DEG2RAD / 3600;
				imusensor->Acc_bias_std[i] = 800 * 1.0e-5;
			}
			//imusensor->Gyr_noise_arw = 4.8 / 60 * DEG2RAD;
			//imusensor->Acc_noise_vrw = 20 / 60;
			imusensor->Gyr_noise_arw = 4.8 / 60 * DEG2RAD;
			imusensor->Acc_noise_vrw = 20 / 60;
			imusensor->Gyr_bias_CorTime = 0.2 * 3600;
			imusensor->Acc_bias_CorTime = 0.2 * 3600;

			imusensor->Gyr_bias_rept = 720 * DEG2RAD / 3600;         //rad/s
			imusensor->Acc_bias_rept = 100 * 1.0e-2;                      //m/s^2
		}

	}break;
	default:
		return retal;
	}
	imusensor->bg_model[0] = exp(-1.0 / imusensor->Gyr_bias_CorTime / IMUDataRate);
	imusensor->bg_model[1] = exp(-1.0 / imusensor->Gyr_bias_CorTime / IMUDataRate);
	imusensor->bg_model[2] = exp(-1.0 / imusensor->Gyr_bias_CorTime / IMUDataRate);
	imusensor->ba_model[0] = exp(-1.0 / imusensor->Acc_bias_CorTime / IMUDataRate);
	imusensor->ba_model[1] = exp(-1.0 / imusensor->Acc_bias_CorTime / IMUDataRate);
	imusensor->ba_model[2] = exp(-1.0 / imusensor->Acc_bias_CorTime / IMUDataRate);
	retal = 1;
	return  retal;
}

int8_t InitQ(const ImuSensor imusensor, const int16_t n, float* Q)
{
	int8_t ret = -1;
	memset(Q, 0, n * sizeof(float));
	Q[0] = 0.0;
	Q[1] = Q[0];
	Q[2] = Q[0];

	Q[3] = imusensor.Acc_noise_vrw*imusensor.Acc_noise_vrw;
	Q[4] = Q[3];
	Q[5] = Q[3];

	Q[6] = imusensor.Gyr_noise_arw*imusensor.Gyr_noise_arw;
	Q[7] = Q[6];
	Q[8] = Q[6];

	Q[9] = 2 * imusensor.Gyr_bias_std[0] * imusensor.Gyr_bias_std[0] / imusensor.Gyr_bias_CorTime;
	Q[10] = 2 * imusensor.Gyr_bias_std[1] * imusensor.Gyr_bias_std[1] / imusensor.Gyr_bias_CorTime;
	Q[11] = 2 * imusensor.Gyr_bias_std[2] * imusensor.Gyr_bias_std[2] / imusensor.Gyr_bias_CorTime;

	Q[12] = 2 * imusensor.Acc_bias_std[0] * imusensor.Acc_bias_std[0] / imusensor.Acc_bias_CorTime;
	Q[13] = 2 * imusensor.Acc_bias_std[1] * imusensor.Acc_bias_std[1] / imusensor.Acc_bias_CorTime;
	Q[14] = 2 * imusensor.Acc_bias_std[2] * imusensor.Acc_bias_std[2] / imusensor.Acc_bias_CorTime;
	if (16 == n)
	{
		Q[15] = 0.000001;
	}
	ret = 1;
	return ret;
}

int8_t InitP(const GnssData mGnssData, const float delay, const ImuSensor mIMUSensor, const int16_t n, float* P)
{
	int8_t ret = -1;
	double P_s[6] = { 0.0 };
	if (!mGnssInsSystem.InitSensor)
	{
		P_s[0] = P[9 * n + 9];
		P_s[1] = P[10 * n + 10];
		P_s[2] = P[11 * n + 11];
		P_s[3] = P[12 * n + 12];
		P_s[4] = P[13 * n + 13];
		P_s[5] = P[14 * n + 14];
	}

	memset(P, 0, n * n * sizeof(float));
	float roll_std = MOTIONINIT_ROLLSTABILTIY * PI / 180;
	float pitch_std = MOTIONINIT_PITCHSTABILTIY * PI / 180;
	float heading_std = MOTIONINIT_YAWSTABILTIY * PI / 180;

	P[0] = 100 * mGnssData.latitude_std * mGnssData.latitude_std + delay * 5 * 5;
	P[1 * n + 1] = 100 * mGnssData.longitude_std * mGnssData.longitude_std + delay * 5 * 5;
	P[2 * n + 2] = 100 * mGnssData.altitude_std * mGnssData.altitude_std + delay * 2 * 2;

	P[3 * n + 3] = 0.5;
	P[4 * n + 4] = 0.5;
	P[5 * n + 5] = 0.5;

	P[6 * n + 6] = roll_std * roll_std;
	P[7 * n + 7] = pitch_std * pitch_std;
	P[8 * n + 8] = heading_std * heading_std;

	if (mGnssInsSystem.InitSensor)
	{
		P[9 * n + 9] = mIMUSensor.Gyr_bias_rept * mIMUSensor.Gyr_bias_rept;
		P[10 * n + 10] = P[9 * n + 9];
		P[11 * n + 11] = P[9 * n + 9];

		P[12 * n + 12] = mIMUSensor.Acc_bias_rept * mIMUSensor.Acc_bias_rept;
		P[13 * n + 13] = P[12 * n + 12];
		P[14 * n + 14] = P[12 * n + 12];
		mGnssInsSystem.InitSensor = 0;
	}
	else
	{
		P[9 * n + 9] = P_s[0];

		P[10 * n + 10] = P_s[1];
		P[11 * n + 11] = P_s[2];

		P[12 * n + 12] = P_s[3];
		P[13 * n + 13] = P_s[4];
		P[14 * n + 14] = P_s[5];
	}
	if (16 == n)
	{
		P[15 * n + 15] = 0.0001;
	}

	ret = 1;
	return ret;

}

int8_t initsystemfromcfg(const LCSetting* p_lcSetting)
{
	int ret = -1;

	mGnssInsSystem.useNativeGNSSPOS = 0;

	mGnssInsSystem.mNav.sensorbias.bias_acc_x = p_lcSetting->accBias[0];
	mGnssInsSystem.mNav.sensorbias.bias_acc_y = p_lcSetting->accBias[1];
	mGnssInsSystem.mNav.sensorbias.bias_acc_z = p_lcSetting->accBias[2];
	mGnssInsSystem.mNav.sensorbias.bias_gyro_x = p_lcSetting->gyroBias[0] * PI / 180;
	mGnssInsSystem.mNav.sensorbias.bias_gyro_y = p_lcSetting->gyroBias[1] * PI / 180;
	mGnssInsSystem.mNav.sensorbias.bias_gyro_z = p_lcSetting->gyroBias[2] * PI / 180;
	mGnssInsSystem.mNav.Odo_scale = 1;

	ret = SetImuSensorParameter(&mGnssInsSystem.mIMUSensor, p_lcSetting->imuSensorType, p_lcSetting->imuDataRate);
	ret = SetZUPTThreShold(p_lcSetting->imuSensorType, p_lcSetting->imuDataRate);
	double C_vb[3][3];
	euler2dcm(p_lcSetting->rotationRBV, C_vb);
	MatrixTranspose(*C_vb, 3, 3, *mGnssInsSystem.RotationCBV);

	// if (p_prcopt.isUseOdo == 1)
	// {
	mGnssInsSystem.mKalmanStruct.n = 16;
	// }
	// else
	// {
	// 	mGnssInsSystem.mKalmanStruct.n = 15;
	// }

	InitQ(mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.Q);

	mGnssInsSystem.premGnssData.timestamp = -999;
	mGnssInsSystem.lastObstime = -999;
	mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
	mGnssInsSystem.Isfirstfusionimu = 1;
	mGnssInsSystem.firstGNSSTime = -1;
	mGnssInsSystem.InitNavi = 0;
	mGnssInsSystem.InitSensor = 1;
	mGnssInsSystem.firstimutime = -1;
	mGnssInsSystem.systemtime_ready = 0;

	mGnssInsSystem.firstzupt = 0;
	mGnssInsSystem.lastzuptdetcttime = -1;

	/*GNSS QC*/
	mGnssInsSystem.cnt_bad_gnss = 0;
	mGnssInsSystem.isheadonline = 1;

	mGnssInsSystem.error_odocout = 0;
	mGnssInsSystem.right_odocout = 0;
	mGnssInsSystem.IsUseOdo = 0;
	mGnssInsSystem.zupttime1 = 0;
	mGnssInsSystem.zupttime2 = 0;

	mGnssInsSystem.BiasEstStab = -1;
	mGnssInsSystem.IsGnssGood = -1;
	mGnssInsSystem.SystemStab = -1;

	if (p_lcSetting->isUseOdo == 1)
	{
		mGnssInsSystem.isUseOdo = 1;
		mGnssInsSystem.isOdoInterfaceGOOD = 1;
	}
	else
	{
		mGnssInsSystem.isUseOdo = 0;
		mGnssInsSystem.isOdoInterfaceGOOD = 0;

	}

	if (p_lcSetting->insinitfromprev == 1)
	{
		mGnssInsSystem.insinitfromprev = 1;
	}
	else
	{
		mGnssInsSystem.insinitfromprev = 0;
	}
	if (p_lcSetting->isOnlineMisAlignmentEst == 1)
	{
		mGnssInsSystem.insinitfromprev = 0;
		p_prcopt.insinitfromprev = 0;
	}
	if (p_prcopt.Initialcondition == 4)
	{
		mGnssInsSystem.initspeed = 0.0;
	}
	else
	{
		mGnssInsSystem.initspeed = 2.0;
	}

	mGnssInsSystem.lastgnssfeedbacktime = -1;
	mGnssInsSystem.lastodofeedbacktime = -1;
	mGnssInsSystem.lastnhcfeedbacktime = -1;
	mGnssInsSystem.lastzuptfeedbacktime = -1;
	mGnssInsSystem.lastzuptlocktime = -1;

    mGnssInsSystem.system_init_status = 0;

	positionfilter_init(&(mGnssInsSystem.posfliter));
	InitHardModeAxix(p_prcopt.MisAlignmentAiax);
	ret = 1;
	return ret;
}

int8_t initsystemfromGNSS()
{
	int8_t ret = -1;
	mGnssInsSystem.GNSSflag = 0;

	if (p_prcopt.gnssDataRate == 1)
	{
		mGnssInsSystem.nextLCTime = floor(mGnssInsSystem.mGnssData.timestamp) + 1;
	}
	else if (p_prcopt.gnssDataRate == 10)
	{
		mGnssInsSystem.nextLCTime = floor(mGnssInsSystem.mGnssData.timestamp * 10 + 1 + 0.05) / 10;
	}

	double eular[3] = { mGnssInsSystem.mNav.roll, mGnssInsSystem.mNav.pitch,mGnssInsSystem.mNav.heading };
	//double v_v[3] = { mGnssInsSystem.mNav.vn, mGnssInsSystem.mNav.ve,mGnssInsSystem.mNav.vd };

	if (p_prcopt.isUseMisAlignment)
	{
		double C_bv[3][3];
		double C1_temp1[3][3] = { {0.0} }, C1_temp2[3][3] = { {0.0} };
		C1_temp1[0][abs(p_prcopt.MisAlignmentAiax[0]) - 1] = abs(p_prcopt.MisAlignmentAiax[0]) / p_prcopt.MisAlignmentAiax[0];
		C1_temp1[1][abs(p_prcopt.MisAlignmentAiax[1]) - 1] = abs(p_prcopt.MisAlignmentAiax[1]) / p_prcopt.MisAlignmentAiax[1];
		C1_temp1[2][abs(p_prcopt.MisAlignmentAiax[2]) - 1] = abs(p_prcopt.MisAlignmentAiax[2]) / p_prcopt.MisAlignmentAiax[2];

		euler2dcm(p_prcopt.misAlignment, C1_temp2);
		//	MatrixMutiply(*C1_temp2, *C1_temp1, 3, 3, 3, *C_bv);?
		MatrixMutiply(*C1_temp2, *C1_temp1, 3, 3, 3, *C_bv);

		//	double axixchange[3][3];
		MatrixTranspose(&C_bv[0][0], 3, 3, &mGnssInsSystem.C_InstallationAngle[0][0]);
		//dcm2euler(*C_bv, eular);


		double C_vn[3][3];
		euler2dcm(eular, C_vn);
		MatrixMutiply(*C_vn, *C_bv, 3, 3, 3, *mGnssInsSystem.mNav.c_bn);
		dcm2euler(mGnssInsSystem.mNav.c_bn, eular);
		mGnssInsSystem.mNav.roll = (float)eular[0];
		mGnssInsSystem.mNav.pitch = (float)eular[1];
		mGnssInsSystem.mNav.heading = (float)eular[2];
		//double v_b[3];


		MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.priLeverArm, 3, 3, 1, mGnssInsSystem.leverarm_b);

		if (p_prcopt.isUserOutPut)
		{
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.userLeverArm, 3, 3, 1, mGnssInsSystem.userleverarm_b);
		}
		if (p_prcopt.isUseNHC)
		{
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.odoLeverArm, 3, 3, 1, mGnssInsSystem.odoleverarm_b);

		}
	}
	else
	{
		euler2dcm(eular, mGnssInsSystem.mNav.c_bn);
		memcpy(mGnssInsSystem.leverarm_b, p_prcopt.priLeverArm, 3 * sizeof(double));
		if (p_prcopt.isUserOutPut)
		{
			memcpy(mGnssInsSystem.userleverarm_b, p_prcopt.userLeverArm, 3 * sizeof(double));
		}
		if (p_prcopt.isUseNHC)
		{
			memcpy(mGnssInsSystem.odoleverarm_b, p_prcopt.odoLeverArm, 3 * sizeof(double));

		}
	}
	euler2quat(eular, mGnssInsSystem.mNav.q_bn);
	double leverarm_n[3];
	MatrixMutiply(*mGnssInsSystem.mNav.c_bn, mGnssInsSystem.leverarm_b, 3, 3, 1, leverarm_n);

	double pos[3] = { mGnssInsSystem.mGnssData.latitude, mGnssInsSystem.mGnssData.longitude, mGnssInsSystem.mGnssData.altitude };
	double M, N;
	UpdateMN(pos, &M, &N);

	double d_leverarm[3];
	d_leverarm[0] = leverarm_n[0] / (M + pos[2]);
	d_leverarm[1] = leverarm_n[1] / ((N + pos[2])*cos(pos[0]));
	d_leverarm[2] = -leverarm_n[2];

	MatrixSub(pos, d_leverarm, 3, 1, pos);

	float dt_dely = (float)(mGnssInsSystem.mGnssData.timestampd - mGnssInsSystem.mGnssData.timestamp);
	mGnssInsSystem.mNav.lat = pos[0] + (double)(dt_dely * mGnssInsSystem.mNav.vn) / (M + pos[2]);
	mGnssInsSystem.mNav.lon = pos[1] + (double)(dt_dely * mGnssInsSystem.mNav.ve) / ((N + pos[2])*cos(pos[0]));
	mGnssInsSystem.mNav.height = pos[2];

	pos2quat(mGnssInsSystem.mNav.lat, mGnssInsSystem.mNav.lon, mGnssInsSystem.mNav.q_ne);

	ret = InitP(mGnssInsSystem.mGnssData, dt_dely, mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, mGnssInsSystem.mKalmanStruct.P);

	ret = 1;
	return ret;
}

int8_t initsystemfromGNSS_STATUS1()
{
	int8_t ret = -1;
	mGnssInsSystem.GNSSflag = 0;

	if (p_prcopt.gnssDataRate == 1)
	{
		mGnssInsSystem.nextLCTime = floor(mGnssInsSystem.mGnssData.timestamp) + 1;
	}
	else if (p_prcopt.gnssDataRate == 10)
	{
		mGnssInsSystem.nextLCTime = floor(mGnssInsSystem.mGnssData.timestamp * 10 + 1 + 0.05) / 10;
	}

    // 需要弄清楚姿态从哪里来的
	double eular[3] = { mGnssInsSystem.mNav.roll, mGnssInsSystem.mNav.pitch,mGnssInsSystem.mNav.heading };

	if (p_prcopt.isUseMisAlignment) // 0
	{
		double C_bv[3][3];
		euler2dcm(p_prcopt.misAlignment, C_bv);
		MatrixTranspose(*C_bv, 3, 3, *mGnssInsSystem.C_InstallationAngle);

		double C_vn[3][3];
		euler2dcm(eular, C_vn);
		MatrixMutiply(*C_vn, *C_bv, 3, 3, 3, *mGnssInsSystem.mNav.c_bn);
		dcm2euler(mGnssInsSystem.mNav.c_bn, eular);
		mGnssInsSystem.mNav.roll = (float)eular[0];
		mGnssInsSystem.mNav.pitch = (float)eular[1];
		mGnssInsSystem.mNav.heading = (float)eular[2];
		MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.priLeverArm, 3, 3, 1, mGnssInsSystem.leverarm_b);

		if (p_prcopt.isUserOutPut)
		{
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.userLeverArm, 3, 3, 1, mGnssInsSystem.userleverarm_b);
		}
		if (p_prcopt.isUseNHC)
		{
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.odoLeverArm, 3, 3, 1, mGnssInsSystem.odoleverarm_b);

		}
	}
	else
	{
		euler2dcm(eular, mGnssInsSystem.mNav.c_bn);// 得到c_bn
		memcpy(mGnssInsSystem.leverarm_b, p_prcopt.priLeverArm, 3 * sizeof(double));
		if (p_prcopt.isUserOutPut)
		{
			memcpy(mGnssInsSystem.userleverarm_b, p_prcopt.userLeverArm, 3 * sizeof(double));
		}
		if (p_prcopt.isUseNHC)
		{
			memcpy(mGnssInsSystem.odoleverarm_b, p_prcopt.odoLeverArm, 3 * sizeof(double));

		}
	}
	euler2quat(eular, mGnssInsSystem.mNav.q_bn); // 得到q_bn
	double leverarm_n[3];
	MatrixMutiply(*mGnssInsSystem.mNav.c_bn, mGnssInsSystem.leverarm_b, 3, 3, 1, leverarm_n);

	double pos[3] = { mGnssInsSystem.mGnssData.latitude, mGnssInsSystem.mGnssData.longitude, mGnssInsSystem.mGnssData.altitude };
	double M, N;
	UpdateMN(pos, &M, &N);
    // 杆臂补偿
	double d_leverarm[3];
	d_leverarm[0] = leverarm_n[0] / (M + pos[2]);
	d_leverarm[1] = leverarm_n[1] / ((N + pos[2])*cos(pos[0]));
	d_leverarm[2] = -leverarm_n[2];

	MatrixSub(pos, d_leverarm, 3, 1, pos);
    // 预测的下一个gnss位置
	float dt_dely = (float)(mGnssInsSystem.mGnssData.timestampd - mGnssInsSystem.mGnssData.timestamp);
	mGnssInsSystem.mNav.lat = pos[0] + (double)(dt_dely * mGnssInsSystem.mNav.vn) / (M + pos[2]);
	mGnssInsSystem.mNav.lon = pos[1] + (double)(dt_dely * mGnssInsSystem.mNav.ve) / ((N + pos[2])*cos(pos[0]));
	mGnssInsSystem.mNav.height = pos[2];

	pos2quat(mGnssInsSystem.mNav.lat, mGnssInsSystem.mNav.lon, mGnssInsSystem.mNav.q_ne); // 得到q_ne

	//ret = InitP(mGnssInsSystem.mGnssData, dt_dely, mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.P);
	//mGnssInsSystem.mKalmanStruct.n = 15;

	ret = 1;
	return ret;
}

int8_t initsystemSoftreset()

{
	int ret = 0;
	isEFKFinished = 0;
	IsFeedback = 0;
	KFStatus = 0;
	//double DeltaT = 0;
	mGnssInsSystem.CurIsUseZupt = 0;
	mGnssInsSystem.firstzupt = 0;
	mGnssInsSystem.lastzuptdetcttime = -1;
	positionfilter_init(&(mGnssInsSystem.posfliter));
	/*GNSS QC*/
	mGnssInsSystem.cnt_bad_gnss = 0;
	mGnssInsSystem.isheadonline = 1;

	if (mGnssInsSystem.InitNavi != 1)
	{
		mGnssInsSystem.InitNavi = 1;
		mGnssInsSystem.firstGNSSUseTime = -1;
		mGnssInsSystem.lastObstime = -1;
		mGnssInsSystem.lastGNSSLCTime = -1;
		mGnssInsSystem.lastNHCLCTime = -1;
		mGnssInsSystem.lastZuptTime = -1;
		mGnssInsSystem.GNSSLOSETIME1 = -1;   //NOT DELETE ZUPT TIME
		mGnssInsSystem.GNSSLOSETIME2 = -1;
		mGnssInsSystem.nextLCTime = -1;
	}
	ret = 1;
	return ret;
}



int8_t PostionInit()
{
	int8_t ret = 0;
	return ret;
}
int8_t VelocityInit()
{
	int8_t ret = 0;
	return ret;
}
int8_t AttitudeInit()
{
	int8_t ret = 0;
	return ret;
}

int8_t BiasInit()
{
	int8_t ret = 0;
	mGnssInsSystem.mNav.sensorbias.bias_acc_x = 0.0;
	mGnssInsSystem.mNav.sensorbias.bias_acc_y = 0.0;
	mGnssInsSystem.mNav.sensorbias.bias_acc_z = 0.0;
	mGnssInsSystem.mNav.sensorbias.bias_gyro_x = 0.0;
	mGnssInsSystem.mNav.sensorbias.bias_gyro_y = 0.0;
	mGnssInsSystem.mNav.sensorbias.bias_gyro_z = 0.0;
	ret = 1;
	return ret;
}

int8_t SystemNavPostionPInit()
{
	int8_t ret = 0;
	return ret;
}

int8_t SystemNavVelocityPInit()
{
	int8_t ret = 0;
	return ret;
}

int8_t SystemNavAttitudePInit()
{
	int8_t ret = 0;
	return ret;
}

int8_t SystemBiasPInit()
{
	int8_t ret = 0;
	return ret;
}

int8_t InternalRecordsInit()
{
	int8_t ret = 0;
	mGnssInsSystem.premGnssData.timestamp = -1;
	mGnssInsSystem.lastObstime = -1;
	mGnssInsSystem.Isfirstfusionimu = 1;
	mGnssInsSystem.firstGNSSTime = -1;
	mGnssInsSystem.InitNavi = 0;
	mGnssInsSystem.InitSensor = 1;
	mGnssInsSystem.CurIsUseZupt = 0;

	/*GNSS QC*/
	mGnssInsSystem.cnt_bad_gnss = 0;
	mGnssInsSystem.isheadonline = 1;

	if (p_prcopt.isUseOdo == 1)
	{
		mGnssInsSystem.isUseOdo = 1;
		mGnssInsSystem.isOdoInterfaceGOOD = 1;
	}
	else
	{
		mGnssInsSystem.isUseOdo = 0;
		mGnssInsSystem.isOdoInterfaceGOOD = 0;
	}
	mGnssInsSystem.insinitfromprev = 0;

	mGnssInsSystem.lastgnssfeedbacktime = -1;
	mGnssInsSystem.lastodofeedbacktime = -1;
	mGnssInsSystem.lastnhcfeedbacktime = -1;
	mGnssInsSystem.lastzuptfeedbacktime = -1;
	mGnssInsSystem.lastzuptlocktime = -1;

	if (mGnssInsSystem.InitNavi != 1)
	{
		mGnssInsSystem.InitNavi = 1;
		mGnssInsSystem.firstGNSSUseTime = -1;
		mGnssInsSystem.lastObstime = -1;
		mGnssInsSystem.lastGNSSLCTime = -1;
		mGnssInsSystem.lastNHCLCTime = -1;
		mGnssInsSystem.lastZuptTime = -1;
		mGnssInsSystem.GNSSLOSETIME1 = -1;   //NOT DELETE ZUPT TIME
		mGnssInsSystem.GNSSLOSETIME2 = -1;
		mGnssInsSystem.nextLCTime = -1;
		mGnssInsSystem.BiasEstStab = -1;
		mGnssInsSystem.IsGnssGood = -1;
		mGnssInsSystem.SystemStab = -1;
	}
	return ret;
}

int8_t StaticvariableInit()
{
	int8_t ret = 0;
	return ret;
}

int8_t SoftReset()
{
	/* System x,p reset*/
	int8_t ret = 0;
	PostionInit();
	VelocityInit();
	SystemNavPostionPInit();
	SystemNavAttitudePInit();
	return ret;
}

int8_t HardReset()
{
	/* System  x,p reset bias reset */
	int8_t ret = 0;
	PostionInit();
	VelocityInit();
	AttitudeInit();
	BiasInit();
	SystemNavPostionPInit();
	SystemNavAttitudePInit();
	SystemNavAttitudePInit();
	SystemBiasPInit();
	return ret;
}

int8_t ErrorReset(int32_t error_type)
{
	int8_t ret = 0;
	switch (error_type)
	{
    case(INITPOOR):
	{
		mGnssInsSystem.mlc_STATUS = INSDATA_READY;
		SoftReset();
		InternalRecordsInit();
		StaticvariableInit();
	}break;
	case(IMUInterruptS):
	{
		mGnssInsSystem.mlc_STATUS = INSDATA_READY;
		SoftReset();
		InternalRecordsInit();
		StaticvariableInit();
	}break;
	case(IMUInterrupt5S):
	{
		mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
		HardReset();
		InternalRecordsInit();
		StaticvariableInit();
	}break;
	case(INSSystemExp1):
	{
		mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
		HardReset();
		InternalRecordsInit();
		StaticvariableInit();
	}break;
	case(MisAligmentChange):
	{
		mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
		HardReset();
		InternalRecordsInit();
		StaticvariableInit();
	}break;
	case(GnssInnoError):
	{
		mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
		HardReset();
		InternalRecordsInit();
		StaticvariableInit();
		InitInnoErrorDect();
	}break;
	default:
		break;
	}

	return ret;
}

int8_t SetLCtime(double lctime)
{
	mGnssInsSystem.nextLCTime = lctime;
	return 1;
}

int8_t initsystemfromPre(double* error_bias)
{
	int8_t ret = -1;
	mGnssInsSystem.GNSSflag = 0;
	mGnssInsSystem.nextLCTime = floor(mGnssInsSystem.mImuData.timestamp) + 1;


	if (p_prcopt.isUseMisAlignment)
	{
		double C_bv[3][3];
		euler2dcm(p_prcopt.misAlignment, C_bv);
		MatrixTranspose(*C_bv, 3, 3, *mGnssInsSystem.C_InstallationAngle);

		MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.priLeverArm, 3, 3, 1, mGnssInsSystem.leverarm_b);

		if (p_prcopt.isUserOutPut)
		{
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.userLeverArm, 3, 3, 1, mGnssInsSystem.userleverarm_b);
		}
		if (p_prcopt.isUseNHC)
		{
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.odoLeverArm, 3, 3, 1, mGnssInsSystem.odoleverarm_b);
		}
	}
	else
	{
		memcpy(mGnssInsSystem.leverarm_b, p_prcopt.priLeverArm, 3 * sizeof(double));
		if (p_prcopt.isUserOutPut)
		{
			memcpy(mGnssInsSystem.userleverarm_b, p_prcopt.userLeverArm, 3 * sizeof(double));
		}
		if (p_prcopt.isUseNHC)
		{
			memcpy(mGnssInsSystem.odoleverarm_b, p_prcopt.odoLeverArm, 3 * sizeof(double));
		}
	}
	//attitude
	double eular[3] = { p_prcopt.initpar[10], p_prcopt.initpar[11],p_prcopt.initpar[12] };
	euler2dcm(eular, mGnssInsSystem.mNav.c_bn);
	mGnssInsSystem.mNav.roll = (float)eular[0];
	mGnssInsSystem.mNav.pitch = (float)eular[1];
	mGnssInsSystem.mNav.heading = (float)eular[2];
	euler2quat(eular, mGnssInsSystem.mNav.q_bn);

	//position
	double pos[3] = { p_prcopt.initpar[4], p_prcopt.initpar[5], p_prcopt.initpar[6] };
	double M, N;
	UpdateMN(pos, &M, &N);
	mGnssInsSystem.mNav.lat = pos[0];
	mGnssInsSystem.mNav.lon = pos[1];
	mGnssInsSystem.mNav.height = pos[2];
	pos2quat(mGnssInsSystem.mNav.lat, mGnssInsSystem.mNav.lon, mGnssInsSystem.mNav.q_ne);

	//speed
	mGnssInsSystem.mNav.vn = p_prcopt.initpar[7];
	mGnssInsSystem.mNav.ve = p_prcopt.initpar[8];
	mGnssInsSystem.mNav.vd = p_prcopt.initpar[9];

	//BIAS STD EST 


	int n = mGnssInsSystem.mKalmanStruct.n;
	memset(mGnssInsSystem.mKalmanStruct.P, 0, n * n * sizeof(float));
	mGnssInsSystem.mKalmanStruct.P[0] = p_prcopt.initpar[13] * p_prcopt.initpar[13];
	mGnssInsSystem.mKalmanStruct.P[1 * n + 1] = p_prcopt.initpar[14] * p_prcopt.initpar[14];
	mGnssInsSystem.mKalmanStruct.P[2 * n + 2] = p_prcopt.initpar[15] * p_prcopt.initpar[15];
	mGnssInsSystem.mKalmanStruct.P[3 * n + 3] = p_prcopt.initpar[16] * p_prcopt.initpar[16];
	mGnssInsSystem.mKalmanStruct.P[4 * n + 4] = p_prcopt.initpar[17] * p_prcopt.initpar[17];
	mGnssInsSystem.mKalmanStruct.P[5 * n + 5] = p_prcopt.initpar[18] * p_prcopt.initpar[18];
	mGnssInsSystem.mKalmanStruct.P[6 * n + 6] = p_prcopt.initpar[19] * p_prcopt.initpar[19];
	mGnssInsSystem.mKalmanStruct.P[7 * n + 7] = p_prcopt.initpar[20] * p_prcopt.initpar[20];
	mGnssInsSystem.mKalmanStruct.P[8 * n + 8] = p_prcopt.initpar[21] * p_prcopt.initpar[21];

	mGnssInsSystem.mKalmanStruct.P[9 * n + 9] = p_prcopt.initpar[19] * p_prcopt.initpar[19];
	mGnssInsSystem.mKalmanStruct.P[10 * n + 10] = p_prcopt.initpar[20] * p_prcopt.initpar[20];
	mGnssInsSystem.mKalmanStruct.P[11 * n + 11] = p_prcopt.initpar[21] * p_prcopt.initpar[21];
	mGnssInsSystem.mKalmanStruct.P[12 * n + 12] = p_prcopt.initpar[19] * p_prcopt.initpar[19];
	mGnssInsSystem.mKalmanStruct.P[13 * n + 13] = p_prcopt.initpar[20] * p_prcopt.initpar[20];
	mGnssInsSystem.mKalmanStruct.P[14 * n + 14] = p_prcopt.initpar[21] * p_prcopt.initpar[21];

	mGnssInsSystem.mKalmanStruct.P[9 * n + 9] = mGnssInsSystem.mIMUSensor.Gyr_bias_rept * mGnssInsSystem.mIMUSensor.Gyr_bias_rept;
	mGnssInsSystem.mKalmanStruct.P[10 * n + 10] = mGnssInsSystem.mKalmanStruct.P[9 * n + 9];
	mGnssInsSystem.mKalmanStruct.P[11 * n + 11] = mGnssInsSystem.mKalmanStruct.P[9 * n + 9];

	mGnssInsSystem.mKalmanStruct.P[12 * n + 12] = mGnssInsSystem.mIMUSensor.Acc_bias_rept * mGnssInsSystem.mIMUSensor.Acc_bias_rept;
	mGnssInsSystem.mKalmanStruct.P[13 * n + 13] = mGnssInsSystem.mKalmanStruct.P[12 * n + 12];
	mGnssInsSystem.mKalmanStruct.P[14 * n + 14] = mGnssInsSystem.mKalmanStruct.P[12 * n + 12];

	for (int i = 0; i < 3; i++)
	{
		if (error_bias[i] < mGnssInsSystem.mIMUSensor.Gyr_bias_rept)
		{
			mGnssInsSystem.mKalmanStruct.P[(9 + i) * n + 9 + i] = error_bias[i] * error_bias[i];
		}
	}
	for (int i = 0; i < 3; i++)
	{
		if (error_bias[3 + i] < mGnssInsSystem.mIMUSensor.Acc_bias_rept)
		{
			mGnssInsSystem.mKalmanStruct.P[(12 + i) * n + 12 + i] = error_bias[3 + i] * error_bias[3 + i];
		}
	}

	ret = 1;
	return ret;
}


int8_t bootpositioninit(void)
{
	if (p_prcopt.isUseMisAlignment)
	{
		double C_bv[3][3];
		euler2dcm(p_prcopt.misAlignment, C_bv);
		MatrixTranspose(*C_bv, 3, 3, *mGnssInsSystem.C_InstallationAngle);

		MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.priLeverArm, 3, 3, 1, mGnssInsSystem.leverarm_b);

		if (p_prcopt.isUserOutPut)
		{
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.userLeverArm, 3, 3, 1, mGnssInsSystem.userleverarm_b);
		}
		if (p_prcopt.isUseNHC)
		{
			MatrixMutiply(*mGnssInsSystem.C_InstallationAngle, p_prcopt.odoLeverArm, 3, 3, 1, mGnssInsSystem.odoleverarm_b);
		}
	}
	else
	{
		memcpy(mGnssInsSystem.leverarm_b, p_prcopt.priLeverArm, 3 * sizeof(double));
		if (p_prcopt.isUserOutPut)
		{
			memcpy(mGnssInsSystem.userleverarm_b, p_prcopt.userLeverArm, 3 * sizeof(double));
		}
		if (p_prcopt.isUseNHC)
		{
			memcpy(mGnssInsSystem.odoleverarm_b, p_prcopt.odoLeverArm, 3 * sizeof(double));
		}
	}
	//attitude
	double eular[3] = { p_prcopt.initpar[10], p_prcopt.initpar[11],p_prcopt.initpar[12] };
	euler2dcm(eular, mGnssInsSystem.mNav.c_bn);
	mGnssInsSystem.mNav.roll = (float)eular[0];
	mGnssInsSystem.mNav.pitch = (float)eular[1];
	mGnssInsSystem.mNav.heading = (float)eular[2];
	euler2quat(eular, mGnssInsSystem.mNav.q_bn);

	//position
	double pos[3] = { p_prcopt.initpar[4], p_prcopt.initpar[5], p_prcopt.initpar[6] };
	double M, N;
	UpdateMN(pos, &M, &N);
	mGnssInsSystem.mNav.lat = pos[0];
	mGnssInsSystem.mNav.lon = pos[1];
	mGnssInsSystem.mNav.height = pos[2];
	pos2quat(mGnssInsSystem.mNav.lat, mGnssInsSystem.mNav.lon, mGnssInsSystem.mNav.q_ne);

	//speed
	mGnssInsSystem.mNav.vn = p_prcopt.initpar[7];
	mGnssInsSystem.mNav.ve = p_prcopt.initpar[8];
	mGnssInsSystem.mNav.vd = p_prcopt.initpar[9];

	//BIAS STD EST 


	int n = mGnssInsSystem.mKalmanStruct.n;
	memset(mGnssInsSystem.mKalmanStruct.P, 0, n * n * sizeof(float));
	mGnssInsSystem.mKalmanStruct.P[0] = p_prcopt.initpar[13] * p_prcopt.initpar[13];
	mGnssInsSystem.mKalmanStruct.P[1 * n + 1] = p_prcopt.initpar[14] * p_prcopt.initpar[14];
	mGnssInsSystem.mKalmanStruct.P[2 * n + 2] = p_prcopt.initpar[15] * p_prcopt.initpar[15];
	mGnssInsSystem.mKalmanStruct.P[3 * n + 3] = p_prcopt.initpar[16] * p_prcopt.initpar[16];
	mGnssInsSystem.mKalmanStruct.P[4 * n + 4] = p_prcopt.initpar[17] * p_prcopt.initpar[17];
	mGnssInsSystem.mKalmanStruct.P[5 * n + 5] = p_prcopt.initpar[18] * p_prcopt.initpar[18];
	mGnssInsSystem.mKalmanStruct.P[6 * n + 6] = p_prcopt.initpar[19] * p_prcopt.initpar[19];
	mGnssInsSystem.mKalmanStruct.P[7 * n + 7] = p_prcopt.initpar[20] * p_prcopt.initpar[20];
	mGnssInsSystem.mKalmanStruct.P[8 * n + 8] = p_prcopt.initpar[21] * p_prcopt.initpar[21];

	mGnssInsSystem.mKalmanStruct.P[9 * n + 9] = p_prcopt.initpar[19] * p_prcopt.initpar[19];
	mGnssInsSystem.mKalmanStruct.P[10 * n + 10] = p_prcopt.initpar[20] * p_prcopt.initpar[20];
	mGnssInsSystem.mKalmanStruct.P[11 * n + 11] = p_prcopt.initpar[21] * p_prcopt.initpar[21];
	mGnssInsSystem.mKalmanStruct.P[12 * n + 12] = p_prcopt.initpar[19] * p_prcopt.initpar[19];
	mGnssInsSystem.mKalmanStruct.P[13 * n + 13] = p_prcopt.initpar[20] * p_prcopt.initpar[20];
	mGnssInsSystem.mKalmanStruct.P[14 * n + 14] = p_prcopt.initpar[21] * p_prcopt.initpar[21];
}