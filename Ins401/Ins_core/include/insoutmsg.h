#ifndef _INS_OUTMESGS_
#define _INS_OUTMESGS_
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lcstruct.h"
#include "ins_interface_API.h"
enum IMUSTATUS
{
	IMUSTATUS_ERRORAL = 0x00000001,  //0 = Normal, 1 = Sensor Failure
	IMUSTATUS_GyroXStatusOK = 0x00000002,  //1 = Valid, 0 = Invalid
	IMUSTATUS_GyroYStatusOK = 0x00000004,  //1 = Valid, 0 = Invalid
	IMUSTATUS_GyroZStatusOK = 0x00000008,  //1 = Valid, 0 = Invalid
	IMUSTATUS_AccelerometerXStatusOK = 0x00000010,  //1 = Valid, 0 = Invalid
	IMUSTATUS_AccelerometerYStatusOK = 0x00000020,  //1 = Valid, 0 = Invalid
	IMUSTATUS_AccelerometerZStatusOK = 0x00000040,  //1 = Valid, 0 = Invalid
	IMUSTATUS_TemperatureStatusOK = 0x00000080,  //1 = Valid, 0 = Invalid
	IMUSTATUS_TemperatureCOUNT = 0x0000FF00,  //IMU temperature reading as follows :Signed 2 - byte value(SHORT) 25ºC = 0x0000 1 LSB = 0.1ºC

	IMUSTATUS_REPETITIVE = 0x00010000,  //0 = Normal, 1 = Sensor Failure  
	IMUSTATUS_LOSE_CUREENT = 0x00020000,  //0 = Normal, 1 = Sensor Failure
	IMUSTATUS_LOSE_SHORT = 0x00040000,   //0 = Normal, 1 = Sensor Failure
	IMUSTATUS_LOSE_LONG = 0x00080000,   //0 = Normal, 1 = Sensor Failure  
	IMUSTATUS_LOSECOUNT = 0x00F00000,   //0 ~ 15

	IMUSTATUS_Outliers = 0x01000000,   //0 = Normal, 1 = Sensor Failure  
	IMUSTATUS_OutMOTIONMODE = 0x02000000,   //0 = Normal, 1 = Sensor Failure  
};
enum ODOSTATUS
{
	/*RAW ODO form CAN detect*/
	ODOSTATUS_ERRORAL = 0x00000001,                //0 = Normal, 1 = Sensor Failure
	GearidentifierSTATUS = 0x00000002,                //0 = Normal, 1 = Sensor Failure
	SPEEDidentifierSTATUS = 0x00000004,                //0 = Normal, 1 = Sensor Failure

	/*ODO interface*/ /*16~23*/
	ODOSTATUS_REPETITIVE = 0x00010000,     //0 = Normal, 1 = Sensor Failure  
	ODOSTATUS_LOSE_CUREENT = 0x00020000,   //0 = Normal, 1 = Sensor Failure
	ODOSTATUS_LOSE_SHORT = 0x00040000,     //0 = Normal, 1 = Sensor Failure
	ODOSTATUS_LOSE_LONG = 0x00080000,      //0 = Normal, 1 = Sensor Failure  
	ODOSTATUS_LOSECOUNT = 0x00F00000,      //0 ~ 15
	/*24~27 detect by car move*/
	ODOSTATUS_Outliers = 0x01000000,       //0 = Normal, 1 = Sensor Failure  
	ODOSTATUS_OutMOTIONMODE = 0x02000000,       //0 = Normal, 1 = Sensor Failure
	/*28~32 detect by ins fusion*/
	ODOSTATUS_LongitSpeedWRONGCUR = 0x10000000,        //0 = Normal, 1 = Sensor Failure
	ODOSTATUS_LongitSpeedWRONGLONG = 0x20000000,        //0 = Normal, 1 = Sensor Failure
	ODOSTATUS_LargeScale = 0x40000000,        //0 = Normal, 1 = Sensor Failure
};
enum GnssSTATUS
{
	/*RAW GNSS form CAN detect*/
	GNSSSTATUS_ERRORAL = 0x00000001,                //0 = Normal, 1 = Sensor Failure


	/*ODO interface*/ /*16~23*/
	GNSSSTATUS_REPETITIVE = 0x00010000,         //0 = Normal, 1 = Sensor Failure  
	GNSSSTATUS_LOSE_CUREENT = 0x00020000,       //0 = Normal, 1 = Sensor Failure
	GNSSSTATUS_LOSE_SHORT = 0x00040000,         //0 = Normal, 1 = Sensor Failure
	GNSSSTATUS_LOSE_LONG = 0x00080000,          //0 = Normal, 1 = Sensor Failure  
	GNSSSTATUS_LOSECOUNT = 0x00F00000,          //0 ~ 15
	/*24~27 detect by experience car or STD*/
	GNSSSTATUS_Outliers = 0x01000000,            //0 = Normal, 1 = Sensor Failure 
	GNSSSTATUS_OutMOTIONMODE = 0x02000000,       //0 = Normal, 1 = Sensor Failure
	GNSSSTATUS_Reinitialize = 0x04000000,        //0 = Normal, 1 = Tunnel or underground garage
	/*28~32 detect by ins fusion*/
	ODOSTATUS_HPDWRONGCUR = 0x10000000,        //0 = Normal, 1 = Sensor Failure
	ODOSTATUS_HPDWRONGLONG = 0x20000000,        //0 = Normal, 1 = Sensor Failure
	ODOSTATUS_VPDWRONGCUR = 0x40000000,        //0 = Normal, 1 = Sensor Failure
	ODOSTATUS_VPDWRONGLONG = 0x80000000,        //0 = Normal, 1 = Sensor Failure
};
enum CORRIMUSTATUS
{
	/*RAW GNSS form CAN detect*/
	CORRIMUSTATUS_ERRORAL = 0x00000001,                //0 = Normal, 1 = Sensor Failure


	/*ODO interface*/ /*16~23*/
	CORRIMUSTATUS_REPETITIVE = 0x00010000,         //0 = Normal, 1 = Sensor Failure  
	CORRIMUSTATUS_LOSE_CUREENT = 0x00020000,       //0 = Normal, 1 = Sensor Failure
	CORRIMUSTATUS_LOSE_SHORT = 0x00040000,         //0 = Normal, 1 = Sensor Failure
	CORRIMUSTATUS_LOSE_LONG = 0x00080000,          //0 = Normal, 1 = Sensor Failure  
	CORRIMUSTATUS_LOSECOUNT = 0x00F00000,          //0 ~ 15
	/*24~27 detect by experience car or STD*/
	CORRIMUSTATUS_Outliers = 0x01000000,            //0 = Normal, 1 = Sensor Failure 
	CORRIMUSTATUS_OutMOTIONMODE = 0x02000000,       //0 = Normal, 1 = Sensor Failure
	CORRIMUSTATUS_Reinitialize = 0x04000000,        //0 = Normal, 1 = Tunnel or underground garage
	/*28~32 detect by ins fusion*/
	CORRIMUSTATUS_HPDWRONGCUR = 0x10000000,        //0 = Normal, 1 = Sensor Failure
	CORRIMUSTATUS_HPDWRONGLONG = 0x20000000,        //0 = Normal, 1 = Sensor Failure
	CORRIMUSTATUS_VPDWRONGCUR = 0x40000000,        //0 = Normal, 1 = Sensor Failure
	CORRIMUSTATUS_VPDWRONGLONG = 0x80000000,        //0 = Normal, 1 = Sensor Failure
};
typedef enum MessageFormat //!< Bits 5-6 of MessageType struct
{
    BINARY = 0,
    ASCII = 1,
    ABREVIATED_ASCII = 2,
    NMEA = 3,
} MessageFormat;
typedef enum ResponseBit //!< Last bit (7) of MessageType struct
{
    ORIGINAL_MESSAGE = 0,
    RESPONSE_MESSAGE = 1,
} ResponseBit;
#pragma pack(1)
typedef struct  SaveConfig
{
	int16_t gnss_week;
	int32_t gnss_second;
	int8_t solution_type;
	int8_t position_type;
	double latitude;
	double longitude;
	float height;
	int16_t north_velocity;
	int16_t east_velocity;
	int16_t down_velocity;
	int16_t roll;
	int16_t pitch;
	int16_t azimuth;
	int16_t latitude_std;
	int16_t longitude_std;
	int16_t altitude_std;
	int16_t north_velocity_std;
	int16_t east_velocity_std;
	int16_t down_velocity_std;
	int16_t roll_std;
	int16_t pitch_std;
	int16_t azimuth_std;
	int16_t gyro_bias_x;
	int16_t gyro_bias_y;
	int16_t gyro_bias_z;
	int16_t acc_bias_x;
	int16_t acc_bias_y;
	int16_t acc_bias_z;
	int16_t std_gyro_bias_x;
	int16_t std_gyro_bias_y;
	int16_t std_gyro_bias_z;
	int16_t std_acc_bias_x;
	int16_t std_acc_bias_y;
	int16_t std_acc_bias_z;
	int8_t static_type;
	double reserve1;
	double reserve2;
}SaveConfig;

typedef struct SaveMsg
{
	uint8_t sync1;            //!< start of packet first byte (0xAA)
	uint8_t sync2;            //!< start of packet second byte (0x44)
	uint8_t sync3;            //!< start of packet third  byte (0x12)
	uint16_t message_length;  //!< Message length (Not including header or CRC)
	uint16_t message_id;      //!< Message ID number
	SaveConfig saveConfig;
	uint8_t crc[4];                           //!< 32-bit cyclic redundancy check (CRC)
}SaveMsg;
#pragma pack()

#if 0
typedef union MessageType{
  struct {
    unsigned char reserved : 5;
    MessageFormat format : 2;
    ResponseBit response : 1;
  } mess_bit;
  uint8_t    mess;
} MessageType;
#endif

typedef struct MessageType{
    unsigned char reserved : 5;
    MessageFormat format : 2;
    ResponseBit response : 1;
}MessageType;
// __attribute__ ((__aligned__(1)))
typedef struct Oem4BinaryHeader
{
    uint8_t sync1;            //!< start of packet first byte (0xAA)
    uint8_t sync2;            //!< start of packet second byte (0x44)
    uint8_t sync3;            //!< start of packet third  byte (0x12)
    uint8_t header_length;    //!< Length of the header in bytes ( From start of packet )
    uint16_t message_id;      //!< Message ID number
    MessageType message_type; //!< Message type - binary, ascii, nmea, etc...
    uint8_t port_address;     //!< Address of the data port the log was received on
    uint16_t message_length;  //!< Message length (Not including header or CRC)
    uint16_t sequence;        //!< Counts down from N-1 to 0 for multiple related logs
    uint8_t idle;             //!< Time the processor was idle in last sec between logs with same ID
    uint8_t time_status;      //!< Indicates the quality of the GPS time
    uint16_t gps_week;        //!< GPS Week number
    uint32_t gps_millisecs;   //!< Milliseconds into week
    uint32_t status;          //!< Receiver status word
    uint16_t Reserved;        //!< Reserved for internal use
    uint16_t version;         //!< Receiver software build number (0-65535)
} Oem4BinaryHeader;
typedef struct Position
{
    Oem4BinaryHeader header;                  //!< Message header
    uint32_t solution_status;                 //!< Solution status
    uint32_t position_type;                   //!< Position type
    double latitude;                          //!< latitude (deg)
    double longitude;                         //!< longitude (deg)
    double height;                            //!< height above mean sea level (m)
    float undulation;                         //!< Undulation - the relationship between the geoid and the ellipsoid (m)
    uint32_t datum_id;                        //!< datum id number
    float latitude_standard_deviation;        //!< latitude standard deviation (m)
    float longitude_standard_deviation;       //!< longitude standard deviation (m)
    float height_standard_deviation;          //!< height standard deviation (m)
    int8_t base_station_id[4];                //!< base station id
    float differential_age;                   //!< differential position age (sec)
    float solution_age;                       //!< solution age (sec)
    uint8_t number_of_satellites;             //!< number of satellites tracked
    uint8_t number_of_satellites_in_solution; //!< number of satellites used in solution
    uint8_t num_gps_plus_glonass_l1;          //!< number of GPS plus GLONASS L1 satellites used in solution
    uint8_t num_gps_plus_glonass_l2;          //!< number of GPS plus GLONASS L2 satellites used in solution
    uint8_t reserved;                         //!< reserved
    uint8_t extended_solution_status;         //!< extended solution status - OEMV and greater only
    uint8_t reserved2;                        //!< reserved
    uint8_t signals_used_mask;                //!< signals used mask - OEMV and greater only
    uint8_t crc[4];                           //!< 32-bit cyclic redundancy check (CRC)
} Position;
/*!
 * Velocity Message Structure
 * This log contains the best available velocity
 * information computed by the receiver. In addition,
 * it reports a velocity status indicator, which is
 * useful in indicating whether or not the corresponding
 * data is valid. The velocity measurements sometimes
 * have a latency associated with them. The time of validity
 * is the time tag in the log minus the latency value.
 *
 * This structure represents the format of the following messages:
 *  - BESTVEL
 *  - RTKVEL
 *  - PSRVEL
 */
typedef struct Velocity
{
	Oem4BinaryHeader header;			//!< Message header
	uint32_t solution_status;		//!< Solution status
	uint32_t position_type;			//!< Position type
	float latency;						//!< measure of the latency of the velocity time tag in seconds
	float age;							//!< differential age in seconds
	double horizontal_speed;			//!< horizontal speed in m/s
	double track_over_ground;			//!< direction of travel in degrees
	double vertical_speed; 				//!< vertical speed in m/s
	float reserved;
	int8_t crc[4];
}Velocity;
/*!
 * INSPVA Message Structure
 * This log allows INS position, velocity and
 * attitude to be collected in one log, instead
 * of using three separate logs.
 */
 typedef struct InsPositionVelocityAttitude
{
	Oem4BinaryHeader header;	//!< Message header
	uint32_t gps_week;			//!< GPS week number
	double gps_millisecs;		//!< Milliseconds into GPS week
	double latitude;			//!< latitude - WGS84 (deg)
	double longitude;			//!< longitude - WGS84 (deg)
	double height;				//!< Ellipsoidal height - WGS84 (m)
	double north_velocity;		//!< velocity in a northerly direction (m/s)
	double east_velocity;		//!< velocity in an easterly direction (m/s)
	double up_velocity;			//!< velocity in an up direction
	double roll;				//!< right handed rotation around y-axis (degrees)
	double pitch;				//!< right handed rotation aruond x-axis (degrees)
	double azimuth;				//!< right handed rotation around z-axis (degrees)
	int32_t status;			//!< status of the INS system
	int8_t crc[4];
}InsPositionVelocityAttitude;
typedef struct INSPVAX
{
	Oem4BinaryHeader header;	//!< Message header
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
	int32_t Ext_sol_stat;			//!< Extended solution status
	int16_t time_since_update;      //!< Elapsed time since the last ZUPT or positionupdate (seconds)
	int8_t crc[4];
}INSPVAX;
typedef struct ImuStatus
{
    unsigned counter : 4;                    //!< 4 byte counter
    unsigned imu_test : 1;                   //!< IMU test: Passed=0, Failed=1
    unsigned z_axis_path_length_control : 1; //!< Z-axis path length control: Good=0, Reset=1
    unsigned y_axis_path_length_control : 1; //!< Y-axis path length control: Good=0, Reset=1
    unsigned x_axis_path_length_control : 1; //!< X-axis path length control: Good=0, Reset=1
    unsigned accelerometer_temperature : 8;  //!< Accelerometer temperature
    unsigned software_version : 8;           //!< IMU software version number
    unsigned reserved : 3;                   //!< Reserved
    unsigned gyro_test : 1;                  //!< Gyro tests: Passed=0, Failed=1
    unsigned accel_test : 1;                 //!< Accelerometer tests: Passed=0, Failed=1
    unsigned other_tests : 1;                //!< Other tests: Passed=0, Failed=1
    unsigned memory_tests : 1;               //!< Memory tests: Passed=0, Failed=1
    unsigned processor_tests : 1;            //!< Processor tests: Passed=0, Failed=1
} ImuStatus;
typedef struct RawImu
{
    Oem4BinaryHeader header;  //!< Message header
    uint32_t gps_week;        //!< GPS week number
    double gps_millisecs;     //!< Milliseconds into GPS week
    ImuStatus imuStatus;      //!< Status of the IMU
    float z_acceleration;     //!< change in velocity along z axis in scaled m/s
    float y_acceleration_neg; //!< -change in velocity along y axis in scaled m/s
    float x_acceleration;     //!< change in velocity along x axis in scaled m/s
    float z_gyro_rate;        //!< change in angle around z axis in radians
    float y_gyro_rate_neg;    //!< -(change in angle around y axis) in radians
    float x_gyro_rate;        //!< change in angle around x axis in radians
    int8_t crc[4];
} RawImu;
typedef struct OdoSpeed
{
    Oem4BinaryHeader header;  //!< Message header
    int week;
    double gps_millisecs;     //!< Milliseconds into GPS week
    char mode;
    double speed;
    char fwd;
    uint64_t wheel_tick;
    int8_t crc[4];
} OdoSpeed;

typedef struct OutputControlOntime
{
	uint8_t ODO_OBS_output;
	uint8_t GNSS_OBS_output;
	uint8_t IMU_OBS_output;
}OutputControlOntime;

typedef struct  odometer_interface_status
{
	uint8_t wheel_tick_lose_cur;  // 0: 数据正常          1：当前数据丢失（数据本身会延续使用上一历元）
	uint8_t wheel_tick_lose_long; // 0: 数据未长时间丢失  1：程序启动后2s无数据接入 2：运行过程中数据丢失超过五秒
	uint16_t wheel_tick_lose_time;  //0.1second
	uint16_t wheel_tick_normal_time;  //0.1sec
	uint8_t wheel_nomal_flag; // 0：无数据解入        1：有正常数据接入正常
}odometer_interface_status;

typedef	struct OdoDataMsg
{
	// LOCK 6
	uint8_t use_flag;  //0：数据已经塞入算法 1：新数据获取
 //通过塞入时间获取的GNSS时间
	int32_t gnss_week;
	double gnss_timeofweek;
	OdoData m_odo_data;
	odometer_interface_status m_odometer_interface_status;
}OdoDataMsg;

typedef struct  imu_interface_status
{
	uint8_t  imu_tick_lose_cur;    // 0: 数据正常          1：当前数据丢失（数据本身会延续使用上一历元）
	uint8_t  imu_tick_lose_long;   // 0: 数据未长时间丢失  1：程序启动后2s无数据接入 2：运行过程中数据丢失超过五秒
	uint16_t imu_tick_lose_time;  //msecond
	uint16_t imu_tick_normal_time;  //0.1sec
	uint8_t  imu_normal_flag;      // 0：无数据解入        1：有正常数据接入正常
}imu_interface_status;

typedef struct ImuDataMsg
{
	uint8_t use_flag;  //0：数据已经塞入算法 1：新数据获取
	//通过塞入时间获取的GNSS时间
	int32_t gnss_week;
	double gnss_timeofweek;
	ImuData m_imu_data; //数据经过转化后塞入该变量，并统计数据完整性
	imu_interface_status m_imu_interface_status;//用来说明收到数据的是否完整
}ImuDataMsg;

typedef struct  GnssData_interface_status
{
	uint8_t gnss_tick_lose_cur;     //0: 数据正常          1：当前数据丢失（数据本身会延续使用上一历元）
	uint8_t gnss_tick_lose_long;    //0: 数据未长时间丢失  1：程序启动后2s无数据接入 2：运行过程中数据丢失超过五秒
	uint16_t gnss_tick_lose_time;   //0.1second
	uint16_t gnss_tick_normal_time; //0.1sec
	uint8_t gnss_nomal_flag;        //0：无数据解入        1：有正常数据接入正常
	//------
}GnssData_interface_status;

typedef	struct GnssDataMsg
{
	uint8_t use_flag;  //0：数据已经塞入算法 1：新数据获取
	//通过塞入时间获取的GNSS时间
	int32_t gnss_week;
	double gnss_timeofweek;
	//----------------------------
	//lock_on 5 计算采用的GNSS 数据
	GnssData m_gnss_data;

	GnssData_interface_status m_gnss_interface_status;
}GnssDataMsg;

typedef struct globalvariable_data
{
	//lock_on 1 最原始的imu_data 太多数据线程需要使用，建议不要直接访问
	int32_t g_gnss_start_week;
	//------------------------
	ImuDataMsg m_imu_msg;
	OdoDataMsg m_odo_msg;
	GnssDataMsg m_gnss_msg;
}globalvariable_data;





int writeRawImuMsg(const int gps_update,const int16_t week1,const uint32_t sec1, const int32_t week2, const uint32_t sec2,const ImuData* p_ImuData);

int writeGnssRawMsg(const int week, const uint32_t gps_millisecs, const GnssData* p_GnssData);

int writeOdoDataMsg(const OdoData* p_OdoData);

extern int writeGGAMsg(int week, double time, const GnssInsSystem* p_gnssInsSystem, char* ggaBuff, char*rmcBuff, char* vtgBuff);

int writeINSPVAXMsg(int32_t week, uint32_t itow, const GnssInsSystem* p_gnssInsSystem);

int writeINSPVAMsg(int32_t week, uint32_t itow, const GnssInsSystem* p_gnssInsSystem);

int writeMISALIGNMsg(int8_t mode, double * origin_cbv, int8_t* MisAlignmentAiax, double* misAlignment, MISRVB* misrvb);

void _copy_ins_result(const int32_t week, const double timestamp, const GnssInsSystem* p_gnssInsSystem, ins_solution_t* p_ins_sol);

extern ins_solution_t* getinssolutionmsg(void);


extern int ins_print_nmea_pashr(double *ep, char *buff);

extern int ins_print_nmea_vtg(char *buff);

extern int ins_print_nmea_ins(double *ep, char *buff);

int printsavebuf(const int8_t type, const GnssInsSystem* p_gnssInsSystem, char* buf);

int parse_saveconfig(const char* p_savemsg, SaveMsg*  pSaveMsg);

int8_t ins_set_corrimuoutput(const int32_t week, const double timestamp, const GnssInsSystem* p_gnssInsSystem,CORRIMUN* p_corrimu);

int8_t ins_set_rawimuoutput(const int32_t cur_week, const double timestamp, const ImuData* p_imu_data, const imu_interface_status* p_odometer_interface_status, ImuData* p_rawimu);

int8_t ins_set_rawodooutput(const int32_t week, const double timestamp, const OdoData* p_odo_data, const odometer_interface_status* p_odometer_interface_status, OdoData* p_out_odo_data);

extern RawImu imuStr;
extern Position positionStr ;
extern Velocity velocityStr;
extern InsPositionVelocityAttitude inspvastr;
extern INSPVAX inspvaxstr;
extern char ggaBuff_bt[120];
extern char pashrBuff_bt[120];
extern char rmcBuff[200];

#endif // !_INS_OUTMESGS_
