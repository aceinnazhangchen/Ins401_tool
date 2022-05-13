#pragma once
#include <stdint.h>

enum emPackageType {
	em_RAW_IMU = 0x0a01,
	em_GNSS_SOL = 0x0a02,
	em_INS_SOL = 0x0a03,
	em_RAW_ODO = 0x0a04,
	em_DIAGNOSTIC_MSG = 0x0a05,
	em_ROVER_RTCM = 0x0a06,
	em_MISALIGN = 0x0a07,
	PowerUpDR_MES = 0x0a09,
	em_CHECK = 0x4D44,
	em_GNSS_SOL_INTEGEITY = 0x6749,
	em_RTK_DEBUG1 = 0xD101,
	em_PACKAGE_FD = 0x6664,
};
#pragma pack(push, 1)

struct raw_imu_t {
	uint16_t	gps_week;
	uint32_t	gps_millisecs;
	float		accel_x;
	float		accel_y;
	float		accel_z;
	float		gyro_x;
	float		gyro_y;
	float		gyro_z;
};

struct gnss_sol_t {
	uint16_t	gps_week;
	uint32_t	gps_millisecs;
	/*
	0: INVALID
	1: Single-point positioning (SPP)
	2: Real time differential GNSS (RTD)
	4: Real time kinematic (RTK), ambiguity fixed (RTK_FIXED)
	5: RTK with ambiguity float (RTK_FLOAT)
	*/
	uint8_t		position_type;
	double		latitude;
	double		longitude;
	double		height;
	float		latitude_std;
	float		longitude_std;
	float		height_std;
	uint8_t		numberOfSVs;
	uint8_t		numberOfSVs_in_solution;
	float		hdop;
	float		diffage;
	float		north_vel;
	float		east_vel;
	float		up_vel;
	float		north_vel_std;
	float		east_vel_std;
	float		up_vel_std;
};

struct ins_sol_t {
	uint16_t	gps_week;
	uint32_t	gps_millisecs;
	/*
	0:INVALID
	1:INS_ALIGNING
	2:INS_HIGH_VARIANCE
	3:INS_SOLUTION_GOOD
	4:INS_SOLUTION_FREE
	5:INS_ALIGNMENT_COMPLETE
	*/
	uint8_t		ins_status;
	/*
	0:INVALID
	1:SPP/INS
	2:RTD/INS
	3:INS_PROPAGATE
	4:RTK_FIXED/INS
	5:RTK_FLOAT/INS
	*/
	uint8_t		ins_position_type;
	double		latitude;
	double		longitude;
	double		height;
	float		north_velocity;
	float		east_velocity;
	float		up_velocity;
	float		longitudinal_velocity;
	float		lateral_velocity;
	float		roll;
	float		pitch;
	float		heading;
	float		latitude_std;
	float		longitude_std;
	float		height_std;
	float		north_velocity_std;
	float		east_velocity_std;
	float		up_velocity_std;
	float		long_vel_std;
	float		lat_vel_std;
	float		roll_std;
	float		pitch_std;
	float		heading_std;
	int16_t		id_contient;
};

#pragma pack(pop)