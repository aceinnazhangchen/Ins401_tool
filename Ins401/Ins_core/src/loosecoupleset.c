#include "loosecoupleset.h"
#ifdef __cplusplus
extern "C" {
#endif
#ifdef  __cplusplus

	LCSetting p_prcopt =
	{
		p_prcopt.initialAttitudeMode = 0,
		p_prcopt.attitueRPH[0] = 0,
		p_prcopt.attitueRPH[1] = 0,
		p_prcopt.attitueRPH[2] = 0,

		p_prcopt.imuSensorType = 1,                                         /*IMU type*/
		p_prcopt.imuDataRate = 100,
		p_prcopt.gnssSensorType = 0,
		p_prcopt.gnssDataType = 3,                                          /*0 none ;1 << 0: gnss position; 1 << 1: gnss vel; 1 << 2: gnss heading*/
		p_prcopt.gnssDataRate = 1,
		p_prcopt.odoDataRate = 0,

		p_prcopt.priLeverArm[0] = 0,                                   /*Offset from the IMU center of navigation to the phase center of the primary GNSS antenna.*/
		p_prcopt.priLeverArm[1] = 0,
		p_prcopt.priLeverArm[2] = 0,
		p_prcopt.isUseDuaAnt = 0,
		p_prcopt.secLeverArm[0] = 0,                                       /*Offset from the IMU center of navigation to the phase center of the secondary GNSS antenna.*/
		p_prcopt.secLeverArm[1] = 0,
		p_prcopt.secLeverArm[2] = 0,

		p_prcopt.rotationRBV[0] = 0,
		p_prcopt.rotationRBV[1] = 0,
		p_prcopt.rotationRBV[2] = 0,

		p_prcopt.isUseMisAlignment = 0,
		p_prcopt.MisAlignmentAiax[0] = 1,
		p_prcopt.MisAlignmentAiax[1] = 2,
		p_prcopt.MisAlignmentAiax[2] = 3,
		p_prcopt.misAlignment[0] = 0,                                     /* Rotation from the vehicle frame to the IMU body frame.*/
		p_prcopt.misAlignment[1] = 0,
		p_prcopt.misAlignment[2] = 0,
		p_prcopt.isOnlineMisAlignmentEst = 0,

		p_prcopt.useGNSSRate = 1,
		p_prcopt.isUseNHC = 1,
		p_prcopt.isUseNHCRES = 0,
		p_prcopt.useNHCRate = 1,
		p_prcopt.odoLeverArm[0] = 0,                                          /*Offset from the IMU center of navigation to the  center of the Odometer.*/
		p_prcopt.odoLeverArm[1] = 0,
		p_prcopt.odoLeverArm[2] = 0,
		p_prcopt.isUseGNSSVel = 0,
		p_prcopt.isUseOdo = 0,
		p_prcopt.useOdoRate = 1,
		p_prcopt.odoScale = 1,

		p_prcopt.GNSSScale = 1,
		p_prcopt.isUseHeadOnline = 0,
		p_prcopt.isUseExpQC = 0,
		p_prcopt.isInsFreInit = 1,
		p_prcopt.insFreTre = 300,

		p_prcopt.isUseZUPT = 1,
		p_prcopt.isUseZUPTLOCK = 1,
		p_prcopt.isUseGNSSZupt = 0,

		p_prcopt.accBias[0] = 0,                                            /*mGal*/
		p_prcopt.accBias[1] = 0,
		p_prcopt.accBias[2] = 0,
		p_prcopt.gyroBias[0] = 0,                                          /*deg/h*/
		p_prcopt.gyroBias[1] = 0,
		p_prcopt.gyroBias[2] = 0,
		p_prcopt.accScale[0] = 0,                                           /*ppm*/
		p_prcopt.accScale[1] = 0,
		p_prcopt.accScale[2] = 0,
		p_prcopt.gyroScale[0] = 0,                                         /*ppm*/
		p_prcopt.gyroScale[1] = 0,
		p_prcopt.gyroScale[2] = 0,

		p_prcopt.isUserOutPut = 1,
		p_prcopt.userLeverArm[0] = 0, /*Offset from the IMU center of navigation to the  center of the output user.*/
		p_prcopt.userLeverArm[1] = 0,
		p_prcopt.userLeverArm[2] = 0,

		p_prcopt.isusefliter = 0,
		p_prcopt.OutputMessageType = 2047,   // current all
		p_prcopt.Initialcondition = 1,

	};
#else
	LCSetting p_prcopt =
	{
	   .initialAttitudeMode = 0,
	   .attitueRPH = {0, 0, 0},

	   .imuSensorType = 1,                                         /*IMU type*/
	   .imuDataRate = 100,
	   .gnssSensorType = 0,
	   .gnssDataType = 3,                                          /*0 none ;1 << 0: gnss position; 1 << 1: gnss vel; 1 << 2: gnss heading*/
	   .gnssDataRate = 1,
	   .odoDataRate = 0,

	   .priLeverArm = {0, 0, 0},                                   /*Offset from the IMU center of navigation to the phase center of the primary GNSS antenna.*/
	   .isUseDuaAnt = 0,
	   .secLeverArm = {0, 0, 0},                                       /*Offset from the IMU center of navigation to the phase center of the secondary GNSS antenna.*/

	   .rotationRBV = {0, 0, 0},

	   .isUseMisAlignment = 0,
	   .MisAlignmentAiax = {1,2,3},
	   .misAlignment = {0, 0, 0},                                      /* Rotation from the vehicle frame to the IMU body frame.*/
	   .isOnlineMisAlignmentEst = 0,

	   .useGNSSRate = 1,
	   .isUseNHC = 1,
	   .useNHCRate = 1,
	   .odoLeverArm = {0, 0, 0},                                          /*Offset from the IMU center of navigation to the  center of the Odometer.*/
	   .isUseGNSSVel = 0,
	   .isUseOdo = 0,
	   .useOdoRate = 1,
	   .odoScale = 1,
	   .isUseNHCRES = 0,
	   .GNSSScale = 1,
	   .isUseHeadOnline = 0,  //0
	   .isUseExpQC = 0,   //0
	   .isInsFreInit = 0,
	   .insFreTre = 300,

	   .isUseZUPT = 1,
	   .isUseZUPTLOCK = 1,
	   .isUseGNSSZupt = 0,

	   .accBias = {0, 0, 0},                                            /*mGal*/
	   .gyroBias = {0, 0, 0},                                          /*deg/h*/
	   .accScale = {0, 0, 0},                                           /*ppm*/
	   .gyroScale = {0, 0, 0},                                         /*ppm*/

	   .isUserOutPut = 1,
	   .userLeverArm = {0, 0, 0}, /*Offset from the IMU center of navigation to the  center of the output user.*/
	   .isusefliter = 0,
	   .OutputMessageType = 4095,   // current all
	   .Initialcondition = 1,
	};
#endif

#ifndef DEG2RAD
#define  DEG2RAD (3.1415926/180)
#endif // !
	extern int8_t initsysfromdeveloper(const developerSetting mdeveloperSetting, LCSetting* p_LCSetting)
	{
		int8_t ret = 0;

		p_LCSetting->insinitfromprev = 0;
		if (mdeveloperSetting.insinitfromprev == 1)
		{
			//check init parameters
			if (mdeveloperSetting.initpar[0] > 2014                                                                         // GNSS week
				&& mdeveloperSetting.initpar[1] >= 0 && mdeveloperSetting.initpar[1] <= 7 * 24 * 60 * 60                        // GNSS time of week
				&& mdeveloperSetting.initpar[2] >= 2                                                                         // ins solution type
				&& mdeveloperSetting.initpar[4] > -90 * 3.14 / 180 && mdeveloperSetting.initpar[4] < 90 * 3.14 / 180         //latitude
				&& mdeveloperSetting.initpar[5] > -180 * 3.14 / 180 && mdeveloperSetting.initpar[5] < 180 * 3.14 / 180        //longitude
				&& mdeveloperSetting.initpar[6] > -1000 && mdeveloperSetting.initpar[6] < 1000                                //height
				&& mdeveloperSetting.initpar[7] > -0.1 && mdeveloperSetting.initpar[7] < 0.1                                  //v_n
				&& mdeveloperSetting.initpar[8] > -0.1 && mdeveloperSetting.initpar[8] < 0.1                                   //v_e
				&& mdeveloperSetting.initpar[9] > -0.1 && mdeveloperSetting.initpar[9] < 0.1                                   //v_d
				&& mdeveloperSetting.initpar[10] > -90 * 3.14 / 180 && mdeveloperSetting.initpar[10] < 90 * 3.14 / 180          //roll
				&& mdeveloperSetting.initpar[11] > -90 * 3.14 / 180 && mdeveloperSetting.initpar[11] < 90 * 3.14 / 180          //pitch
				&& mdeveloperSetting.initpar[12] > -180 * 3.14 / 180 && mdeveloperSetting.initpar[12] < 180 * 3.14 / 180        //heading
				&& mdeveloperSetting.initpar[13] > 0 && mdeveloperSetting.initpar[13] < 10                                      //lat_std
				&& mdeveloperSetting.initpar[14] > 0 && mdeveloperSetting.initpar[14] < 10                                      //lon_std
				&& mdeveloperSetting.initpar[15] > 0 && mdeveloperSetting.initpar[15] < 10                                      //height_std
				&& mdeveloperSetting.initpar[16] > 0 && mdeveloperSetting.initpar[16] < 0.5                                      //vn_std
				&& mdeveloperSetting.initpar[17] > 0 && mdeveloperSetting.initpar[17] < 0.5                                     //ve_std
				&& mdeveloperSetting.initpar[18] > 0 && mdeveloperSetting.initpar[18] < 0.5                                     //vd_std
				&& mdeveloperSetting.initpar[19] > 0 && mdeveloperSetting.initpar[19] < 5 * 3.14 / 180                          //roll_std
				&& mdeveloperSetting.initpar[20] > 0 && mdeveloperSetting.initpar[20] < 5 * 3.14 / 180                          //pitch_std
				&& mdeveloperSetting.initpar[21] > 0 && mdeveloperSetting.initpar[21] < 10 * 3.14 / 180                         //heading_std
				&& mdeveloperSetting.initpar[22] > -2.0 && mdeveloperSetting.initpar[22] < 2.0                                   //Gyro_x_bias
				&& mdeveloperSetting.initpar[23] > -2.0 && mdeveloperSetting.initpar[23] < 2.0                                   //Gyro_y_bias
				&& mdeveloperSetting.initpar[24] > -2.0 && mdeveloperSetting.initpar[24] < 2.0                                   //Gyrp_z_bias
				&& mdeveloperSetting.initpar[25] > -2.0 && mdeveloperSetting.initpar[25] < 2.0                                   //Gyro_x_bias
				&& mdeveloperSetting.initpar[26] > -2.0 && mdeveloperSetting.initpar[26] < 2.0                                   //Gyro_y_bias
				&& mdeveloperSetting.initpar[27] > -2.0 && mdeveloperSetting.initpar[27] < 2.0                                   //Gyrp_z_bias
				&& mdeveloperSetting.initpar[28] > 0.0 && mdeveloperSetting.initpar[28] < 2.0                                   //Gyro_x_bias_std
				&& mdeveloperSetting.initpar[29] > 0.0 && mdeveloperSetting.initpar[29] < 2.0                                   //Gyro_y_bias_std
				&& mdeveloperSetting.initpar[30] > 0.0 && mdeveloperSetting.initpar[30] < 2.0                                   //Gyrp_z_bias_std
				&& mdeveloperSetting.initpar[31] > 0.0 && mdeveloperSetting.initpar[31] < 2.0                                   //Gyro_x_bias_std
				&& mdeveloperSetting.initpar[32] > 0.0 && mdeveloperSetting.initpar[32] < 2.0                                   //Gyro_y_bias_std
				&& mdeveloperSetting.initpar[33] > 0.0 && mdeveloperSetting.initpar[33] < 2.0                                   //Gyrp_z_bias_std
				&& mdeveloperSetting.initpar[34] > 0.9999 && mdeveloperSetting.initpar[34] < 1.0001                           //isstatic

				)
			{
				p_LCSetting->insinitfromprev = 1;
				memcpy(p_LCSetting->initpar, mdeveloperSetting.initpar, 40 * sizeof(double));
				p_LCSetting->gyroBias[0] = mdeveloperSetting.initpar[22];
				p_LCSetting->gyroBias[1] = mdeveloperSetting.initpar[23];
				p_LCSetting->gyroBias[2] = mdeveloperSetting.initpar[24];
				p_LCSetting->accBias[0] = mdeveloperSetting.initpar[25];
				p_LCSetting->accBias[1] = mdeveloperSetting.initpar[26];
				p_LCSetting->accBias[2] = mdeveloperSetting.initpar[27];
				ret = 1;
			}
			else
			{
				p_LCSetting->insinitfromprev = 0;
				ret = -1;
			}

		}
		if (p_LCSetting->initfmpre == 0)
		{
			p_LCSetting->insinitfromprev = 0;
			ret = -1; //unexpected input
		}
		return ret;
	}

	extern int8_t initsysfromUser(const userSetting* p_userSetting, LCSetting* p_LCSetting)
	{
		int8_t ret = 1;
		if (p_userSetting->rotation_rbv[0] >= -360 && p_userSetting->rotation_rbv[0] <= 360
			&& p_userSetting->rotation_rbv[1] >= -360 && p_userSetting->rotation_rbv[1] <= 360
			&& p_userSetting->rotation_rbv[2] >= -360 && p_userSetting->rotation_rbv[2] <= 360
			)
		{
			p_LCSetting->rotationRBV[0] = p_userSetting->rotation_rbv[0] * DEG2RAD;
			p_LCSetting->rotationRBV[1] = p_userSetting->rotation_rbv[1] * DEG2RAD;
			p_LCSetting->rotationRBV[2] = p_userSetting->rotation_rbv[2] * DEG2RAD;
		}
		else
		{
			p_LCSetting->rotationRBV[0] = 0.0;
			p_LCSetting->rotationRBV[1] = 0.0;
			p_LCSetting->rotationRBV[2] = 0.0;
			ret = -1;
		}
		if (p_userSetting->pri_lever_arm[0] >= -10 && p_userSetting->pri_lever_arm[0] <= 10
			&& p_userSetting->pri_lever_arm[1] >= -10 && p_userSetting->pri_lever_arm[1] <= 10
			&& p_userSetting->pri_lever_arm[2] >= -10 && p_userSetting->pri_lever_arm[2] <= 10
			)
		{
			p_LCSetting->priLeverArm[0] = p_userSetting->pri_lever_arm[0];
			p_LCSetting->priLeverArm[1] = p_userSetting->pri_lever_arm[1];
			p_LCSetting->priLeverArm[2] = p_userSetting->pri_lever_arm[2];
		}
		else
		{
			p_LCSetting->priLeverArm[0] = 0.0;
			p_LCSetting->priLeverArm[1] = 0.0;
			p_LCSetting->priLeverArm[2] = 0.0;
			ret = -2;
		}
		if (p_userSetting->vrp_lever_arm[0] >= -10 && p_userSetting->vrp_lever_arm[0] <= 10
			&& p_userSetting->vrp_lever_arm[1] >= -10 && p_userSetting->vrp_lever_arm[1] <= 10
			&& p_userSetting->vrp_lever_arm[2] >= -10 && p_userSetting->vrp_lever_arm[2] <= 10
			)
		{
			p_LCSetting->odoLeverArm[0] = p_userSetting->vrp_lever_arm[0];
			p_LCSetting->odoLeverArm[1] = p_userSetting->vrp_lever_arm[1];
			p_LCSetting->odoLeverArm[2] = p_userSetting->vrp_lever_arm[2];
		}
		else
		{
			p_LCSetting->odoLeverArm[0] = 0.0;
			p_LCSetting->odoLeverArm[1] = 0.0;
			p_LCSetting->odoLeverArm[2] = 0.0;
			ret = -3;
		}
		if (p_userSetting->user_lever_arm[0] >= -10 && p_userSetting->user_lever_arm[0] <= 10
			&& p_userSetting->user_lever_arm[1] >= -10 && p_userSetting->user_lever_arm[1] <= 10
			&& p_userSetting->user_lever_arm[2] >= -10 && p_userSetting->user_lever_arm[2] <= 10
			)
		{
			p_LCSetting->userLeverArm[0] = p_userSetting->user_lever_arm[0];
			p_LCSetting->userLeverArm[1] = p_userSetting->user_lever_arm[1];
			p_LCSetting->userLeverArm[2] = p_userSetting->user_lever_arm[2];
		}
		else
		{
			p_LCSetting->userLeverArm[0] = 0.0;
			p_LCSetting->userLeverArm[1] = 0.0;
			p_LCSetting->userLeverArm[2] = 0.0;
		}
		return ret;
	}

#ifdef __cplusplus
}
#endif