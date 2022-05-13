/** ***************************************************************************
 * @file int_interface_API.h 
 * @brief API functions between GNSS and INS
 * 
 *
 *****************************************************************************/

#ifndef __INS_INTERFACE_API_
#define __INS_INTERFACE_API_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WIN32
#define ARM_MCU
#endif


#ifndef SECONDSOFWEEK
#define SECONDSOFWEEK (604800)
#endif // !SECONDSOFWEEK

// #pragma pack(1)
enum IDCONTINENT
{
	ID_NONE = -2,
	ID_ERROR = -1,
	ID_UNKNOWN = 0,
	ID_ASIA = 1,
	ID_EUROPE = 2,
	ID_OCEANIA = 3,
	ID_AFRICA = 4,
	ID_NORTHAMERICA = 5,
	ID_SOUTHAMERICA = 6,
	ID_ANTARCTICA = 7,
};
typedef struct  ImuData//ǰ����
{
	int32_t week;
	double timestamp;  //GPS time

	double timestamped; //received time 

	double Gyrox;	// angle velocity along x axis  rad/s
	double Gyroy;	// angle velocity along y axis  rad/s
	double Gyroz;	// angle velocity along z axis  rad/s

	double Accx;	// line acceleration along x axis  m/s^2
	double Accy;	// line acceleration along y axis  m/s^2
	double Accz;	// line acceleration along z axis  m/s^2

	uint32_t IMUstatus;
	double flag;    // The first data in a package
}ImuData;

typedef struct OdoData
{
	int32_t week;             //GPS week 
	double timestamp;         //GPS time
	int8_t mode;              //mode 0: CAN BUS 1:wheel tick
	double vehicle_speed;     //CAN BUS: speed 
	int8_t fwd;               //1:forward;0:parking;-1:Back-up (CAN from Gear information,wheel_tick from Voltage level)
	uint64_t wheel_tick;      //Cumulative count
	uint32_t Odostatus;       //input origin Odostatus use low bit16 and high bit16 detect  of the output calculatted by interface and  algorithm
	double reserve;           //reserved :0 //
}OdoData;

typedef struct  GnssData
{
	int32_t week;
	double timestamp;  //GPS time 

	double timestampd; //received time
	uint16_t gpsFixType;//0:NGNSS  1:spp, 2:PSR,  4:fixed,5:float

	double longitude;	// degrees
	double latitude;	// degrees
	double altitude;	// ellipsoidal height - WGS84 (m)

	float north_velocity;	// m/s
	float east_velocity;	// m/s
	float up_velocity;		// m/s

	float longitude_std;	// longitude standard deviation
	float latitude_std;	// latitude standard deviation
	float altitude_std;	// altitude standard deviation

	float north_velocity_std;	// velocity standard deviation
	float east_velocity_std;
	float up_velocity_std;

	float length; //Dual antenna baseline length
	float pitch;
	float heading; //Dual antenna

	float pitch_std;
	float heading_std; //Dual antenna

	uint16_t Num_track; //number of stars visible
	uint16_t Num_sol; //number of stars visible

	uint16_t average_snr;

	float HDOP;
	float PDOP;
	float TDOP;
	float sol_age;

	uint32_t Gnssstatus;       //input origin GNSSstatus use low bit16 and high bit16 detect  of the output calculatted by interface and  algorithm
} GnssData;

typedef struct MISRVB
{
	int8_t flag;
	double RVB[3];
	double CVB[3][3];
	int8_t output_flag;
}MISRVB; //flag == 1|| 2||3 should be write

typedef struct ins_solution_t
{
	uint16_t gps_week;          // GPS Week number
    uint32_t gps_millisecs;     // Milliseconds into week
	uint32_t ins_status;        //!< Solution status
	uint32_t pos_type;          //!< Position type
	double latitude;			//!< latitude - WGS84 (deg)
	double longitude;			//!< longitude - WGS84 (deg)
	double height;				//!< Height above mean sea level  - WGS84 (m)
	double north_velocity;		//!< velocity in a northerly direction (m/s)
	double east_velocity;		//!< velocity in an easterly direction (m/s)
	double up_velocity;			//!< velocity in an up direction
	double roll;				//!< right handed rotation around y-axis (degrees)
	double pitch;				//!< right handed rotation aruond x-axis (degrees)
	double azimuth;				//!< right handed rotation around z-axis (degrees)
	float latitude_std;
	float longitude_std;
	float altitude_std;
	float north_velocity_std;
	float east_velocity_std;
	float up_velocity_std;
	float roll_std;
	float pitch_std;
	float azimuth_std;
	int32_t Ext_sol_stat;			    //!< Extended solution status
	uint16_t time_since_update;         //  Elapsed time since the last ZUPT or NHC or positionupdate (seconds)
	uint16_t mlc_status;
	int16_t id_continent;
	int8_t isUseOdo;
} ins_solution_t;

//corrimu output,use this unit
typedef struct CORRIMUN
{
	int32_t week;
	double timestamp;     
	double PitchRate;	    // About x-axis rotation (right-handed) (deg / s)
	double RollRate;	    // About y-axis rotation (right-handed) (deg / s)
	double YawRate;      	// About z-axis rotation (right-handed) (deg / s)
	double LateralAcc;	    // line acceleration along x axis  m/s^2
	double LongitudinalAcc;	// line acceleration along y axis  m/s^2
	double VerticalAcc;	    // line acceleration along z axis  m/s^2
	uint32_t CORIMUstatus;  // IMU STATUS + INS STATUS 
}CORRIMUN;

typedef struct INSPVAXN
{
	int32_t gps_week;             //!< Message header
	uint32_t gps_millisecs;
	int32_t ins_status;         //!< Solution status
	int32_t pos_type;           //!< Position type
	double latitude;			//!< latitude - WGS84 (deg)
	double longitude;			//!< longitude - WGS84 (deg)
	double height;				//!< Height above mean sea level  - WGS84 (m)
	float undulation;          //!< Undulation (m)
	double north_velocity;		//!< velocity in a northerly direction (m/s)
	double east_velocity;		//!< velocity in an easterly direction (m/s)
	double up_velocity;			//!< velocity in an up direction
	double roll;				//!< right handed rotation around y-axis (degrees)
	double pitch;				//!< right handed rotation aruond x-axis (degrees)
	double azimuth;				//!< right handed rotation around z-axis (degrees)
	float latitude_std;
	float longitude_std;
	float altitude_std;
	float north_velocity_std;
	float east_velocity_std;
	float up_velocity_std;
	float roll_std;
	float pitch_std;
	float azimuth_std;
	int32_t Ext_sol_stat;			    //!< Extended solution status
	uint16_t time_since_update;         //  Elapsed time since the last ZUPT or NHC or positionupdate (seconds)

	int8_t isUseOdo;
	float odoscale;
	int8_t curIsUseZupt;
	int16_t mlc_status;
	int16_t id_continent;
}INSPVAXN;

typedef struct INSBIASN
{
	int32_t week;             //!< Message header
	double timestamp;
	int32_t ins_status;         //!< Solution status
	int32_t pos_type;
	float gyro_bias_x;
	float gyro_bias_y;
	float gyro_bias_z;
	float acc_bias_x;
	float acc_bias_y;
	float acc_bias_z;
	float std_gyro_bias_x;
	float std_gyro_bias_y;
	float std_gyro_bias_z;
	float std_acc_bias_x;
	float std_acc_bias_y;
	float std_acc_bias_z;
}INSBIASN;
enum OutputType
{
	Output_NONE = 0,
	/*According to frequency*/
	Output_INSGGA = 1 << 0,
	Output_INSRMC = 1 << 1,
	Output_INSVTG = 1 << 2,
	Output_RAWGNSS = 1 << 3,
	Output_RAWIMU = 1 << 4,
	Output_RAWODO = 1 << 5,
	Output_INSPVAX = 1 << 6,
	Output_CORRIMU = 1 << 7,
	Output_BIAS = 1 << 8,
	Output_DEBUG = 1 << 9,
	Output_SOL = 1 << 10,
	/*According to frequency*/
	Output_SAVEMSG = 1 << 11,
	/*onchange*/
	Output_ONLINEMISEST = 1 << 12,
	/*once*/
};
typedef struct OutputMsg
{
	int64_t update;

	/*i dont know*/
	uint32_t mlc_STATUS;

	/*According to frequency*/
	/*nema*/
	char ggaBuff[120];
	char rmcBuff[200];
	char vtgBuff[120];
	/*raw struct*/
	ImuData rawimu;
	OdoData rawodo;
	GnssData rawgnss;
	/*result*/
	CORRIMUN corrimu;
	INSPVAXN inspvax;
	INSBIASN insbias;
	ins_solution_t ins_solution;
    /*onchange*/
	MISRVB misrbv;
	char saveMsg[512];
}OutputMsg;
// #pragma pack()


// #pragma pack(1)
// typedef struct OutputMsg
// {
// 	int64_t update;

// 	/*i dont know*/
// 	uint32_t mlc_STATUS;

// 	/*According to frequency*/
// 	/*nema*/
// 	char ggaBuff[120];
// 	char rmcBuff[200];
// 	char vtgBuff[120];
// 	/*raw struct*/
// 	ImuData rawimu;
// 	OdoData rawodo;
// 	GnssData rawgnss;
// 	/*result*/
// 	CORRIMUN corrimu;
// 	INSPVAXN inspvax;
// 	INSBIASN insbias;
// 	ins_solution_t ins_solution;
//     /*onchange*/
// 	MISRVB misrbv;
// 	char saveMsg[110];
// }OutputMsg;
// #pragma pack()


/*************************ins system set******************************/ 
//type == 1001 use online est
extern int8_t ins_set_lcsettingMODE_API(const int32_t type);

extern int8_t ins_init_API(const uint8_t type,const char* userconfig_s,const char* developerconfig_s);

extern int8_t ins_init_OUTMESSAGE_API(const uint32_t type);


/************************add data**************************************/
extern int8_t INSAddODOData_API(const OdoData odo_data);

extern int8_t INSADDGNSSDATA_API(int32_t recweek, double rectimestamp,const GnssData mGnssData);

extern int8_t INSADDIMUData_API(const ImuData mImuData);

/************************get result************************************/
//show if you get
extern int8_t  ins_get_inslibdescrption_API(char* str, int32_t length);

extern int8_t ins_get_savebuf_API(const int8_t type, char* buf);


extern int8_t ins_get_result_API(OutputMsg* p_outputMsg);


/*需要讨论*/
//extern int ins_get_printnmeains_API(double *ep, char *buff);
//
//extern int ins_get_printnmeavtg_API(char *buff);
//
//extern int ins_get_printnmeapashr_API(double *ep, char *buff);

//extern int8_t ins_get_lcsetisuseodo_API(void);

//extern uint8_t ins_get_gnssoutputflag_API(void);


/*only show post*/ 
extern int8_t ins_init_post_API(char* poc_set);
uint8_t* get_ins_compile_commit();

#ifdef __cplusplus
}
#endif
#endif /* __INS_INTERFACE_API_ */
