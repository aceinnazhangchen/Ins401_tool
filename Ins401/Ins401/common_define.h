#pragma once

#define		PACKET_MSG_HEADER	0x5555
#define		PACKET_MAC_LEN		14						/* dest_mac(6)+ src_mac(6) + load_len(2) */
#define		PACKET_MAX_SIZE		1500
#define		MAC_ADDRESS_LEN		6
#define		FILE_PACKET_LEN		960
#define		IMU_PACKET_LEN		192
#define		SDK_PACKET_LEN		1024

/*******************USER CMD****************************/
#define     IAP_CMD_GV              0xcc01              /** IAP Get BootLoader Version **/
#define     USER_CMD_GET_CONFIG     0xcc02              /** Get User Configuration **/
#define     USER_CMD_SET_CONFIG     0xcc03              /** Set User Configuration **/
#define     USER_CMD_RTCM_DATA		0x0b02
/*******************IAP CMD****************************/
#define     IAP_CMD_JI              0xaa01              /** IAP Jump to Boot     **/
#define     IAP_CMD_JA              0xaa02              /** IAP Jump to APP     **/
#define     IAP_CMD_WA              0xaa03              /** IAP Write APP       **/
#define     IAP_CMD_CS              0xaa04              /** IAP Set Write Core  **/
/*******************IMU CMD****************************/
#define     IMU_CMD_JI              0x4a49              /** IMU Jump to IAP **/
#define     IMU_CMD_JI_RET          0x494a
#define     IMU_CMD_JA              0x4a41              /** IMU Jump to APP **/
#define     IMU_CMD_JA_RET			0x414a
#define     IMU_CMD_WA              0x5741              /** IMU Write APP   **/
#define     IMU_CMD_WA_RET			0x4157              
/*******************STA9100 CMD************************/
#define     SDK_CMD_JS				0xaa05              /** St9100 Jump Boot **/
#define     SDK_CMD_JG				0xaa06              /** St9100 Jump GNSS **/
#define		SDK_CMD_SENDSDK			0xaa07              /** St9100 Send SDK **/
#define     SDK_CMD_SENDJL			0xaa08              /** St9100 Send Loader **/

#define     ST9100_ETH_SYNC         {0xfd, 0xc6, 0x49, 0x28}            /** St9100 Sync **/
#define     ST9100_ETH_CHGBUAD		0x71                /** St9100 Change BaudRate **/
#define     ST9100_ETH_CHKBAUD		0x38                /** St9100 Check BaudRate **/
#define     ST9100_ETH_CHKREADY		0x5A                /** St9100 Check Ready **/
#define     ST9100_BAUDRATE			{0x00, 0x84, 0x03, 0x00}
#define     ST9100_ETH_BOOTPREINFO  {0xf4, 0x01, 0xd5, 0xbc, 0x73, 0x40, 0x98, 0x83, \
                                     0x04, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00}
#define     ST9100_BOOTPREINFO_LEN	16
#define     ST9100_ETH_EFLASH       0x4A
#define     ST9100_ETH_WFLASH       0x64

struct net_card_t {
	char show_name[256];
	char dev_name[256];
};

enum emUpgradeStep {
	upgrade_rtk,
	upgrade_ins,
	upgrade_sdk,//sta9100
	upgrade_imu,
	upgrade_all
};

#pragma pack(push, 1)

struct msg_packet_t {
	uint16_t    msg_head;
	uint16_t    msg_type;
	uint32_t    msg_len;
	uint8_t     msg_data[PACKET_MAX_SIZE];
};

struct app_packet_t {
	uint8_t			dest_mac[MAC_ADDRESS_LEN];
	uint8_t			src_mac[MAC_ADDRESS_LEN];
	uint16_t		load_len;
	msg_packet_t	msg_load;
};

typedef struct {
	uint32_t binSize;
	uint32_t bootMode;
	uint32_t crc;
	uint32_t destinationAddress;
	uint32_t entryPoint;
	uint8_t  erase_nvm_u8;
	uint8_t  eraseOnly_u8;
	uint8_t  programOnly_u8;
	uint8_t  subSector_u8;
	uint8_t  sta8090fg_u8;
	uint8_t  res1_8;
	uint8_t  res2_8;
	uint8_t  res3_8;
	uint32_t nvmoffset;
	uint32_t nvmEraseSize;
	uint32_t debug;
	uint32_t debugAction;
	uint32_t debugAddress;
	uint32_t debugSize;
	uint32_t debugData;
}stSTA9100BinInfo;

struct config_parameter_t {
	uint32_t sequence_id;
	float value;
};

#pragma pack(pop)