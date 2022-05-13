#ifndef LOOSECOUPLESET_H_
#define LOOSECOUPLESET_H_
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// #pragma pack(1)
	typedef struct LCSetting
	{
		int16_t initialAttitudeMode;                                   //0-MOTION 1-GIVEN  2 STATIC			
		double attitueRPH[3];                                          // SystemInit by Given attitue (deg)

		int16_t imuSensorType;                                         //IMU type
		int16_t imuDataRate;
		int16_t gnssSensorType;
		int16_t gnssDataType;                                          // 0 none ;1 << 0: gnss position; 1 << 1: gnss vel; 1 << 2: gnss heading
		int16_t gnssDataRate;
		int16_t odoDataRate;

		double priLeverArm[3];                                          //Offset from the IMU center of navigation to the phase center of the primary GNSS antenna.
		int8_t isUseDuaAnt;
		double secLeverArm[3];                                          //Offset from the IMU center of navigation to the phase center of the secondary GNSS antenna.

		double rotationRBV[3];

		int8_t isUseMisAlignment;
		int8_t MisAlignmentAiax[3];
		double misAlignment[3];                                          //Rotation from the vehicle frame to the IMU body frame.
		int8_t isOnlineMisAlignmentEst;

		int8_t useGNSSRate;
		int8_t isUseNHC;
		int8_t isUseNHCRES;
		int8_t useNHCRate;
		double odoLeverArm[3];                                          //Offset from the IMU center of navigation to the  center of the Odometer.
		int8_t isUseGNSSVel;
		int8_t isUseOdo;                                               //设置死的
		int8_t useOdoRate;
		double odoScale;

		int8_t isUseZUPT;
		int8_t isUseZUPTLOCK;

		double GNSSScale;
		int8_t isUseHeadOnline;
		int8_t isUseExpQC;
		int8_t isInsFreInit;
		double insFreTre;
		int8_t isUseGNSSZupt;

		double accBias[3];                                             //mGal
		double gyroBias[3];                                            //deg/h
		double accScale[3];                                            //ppm
		double gyroScale[3];                                           //ppm

		int8_t  isUserOutPut;
		double  userLeverArm[3];//Offset from the IMU center of navigation to the  center of the output user.
		int8_t isusefliter;

		int8_t initfmpre;        // 0: no start position 1: Use start position  程序设置死的
		int8_t readsavetype;     //
		int8_t savesavetype;     // 0:保存ASCII 数据 1：保存二进制数据  实时是二进制 时候配置成ASCII
		//start position mode
		int8_t insinitfromprev;    //start position moving 
		double initpar[40];

		uint32_t OutputMessageType;   // current all
		uint32_t Initialcondition;    //初始化条件 1:正常 2:float 3 spp 4:速度为零 
	}LCSetting;
	enum INITVAR
	{
		INITVAR_NONE = 0,
		INITVAR_USECONFIG = 1 << 0,
		INITVAR_DEVELOPERCONFIG = 1 << 1,
		INITVAR_DEVELOPERCONFIGBIN = 1 << 2,
	};
	typedef struct userSetting
	{
		float pri_lever_arm[3];   //meter
		float vrp_lever_arm[3];   //meter
		float rotation_rbv[3];    //deg
		float user_lever_arm[3];  //meter
	}userSetting;

	typedef struct developerSetting
	{
		int8_t insinitfromprev;
		double initpar[40];
	}developerSetting;
// #pragma pack()

	extern LCSetting p_prcopt;

	extern int8_t initsysfromdeveloper(const developerSetting mdeveloperSetting, LCSetting* p_prcopt);
	extern int8_t initsysfromUser(const userSetting* muserSetting, LCSetting* p_prcopt);
#ifdef __cplusplus
}
#endif

#endif