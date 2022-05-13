#include "QCommonFuction.h"
#include <QtEndian>
#include "common_function.h"
#include "Ins401_Message.h"

#define KB 1024

QCommonFuction::QCommonFuction(QObject *parent)
	: QObject(parent)
{
}

QCommonFuction::~QCommonFuction()
{
}

QString QCommonFuction::FormatBytes(int byte) {
	QString number_str;
	double number = 0;;
	if (byte < KB) {
		number_str = QString::number(byte) + "B";
	}
	else if (byte >= KB && byte <= KB * KB) {
		number = double(byte) / KB;
		number_str = QString::number(number, 'f', 2) + "K";
	}
	else if (byte >= KB * KB && byte <= KB * KB * KB) {
		number = double(byte) / (KB * KB);
		number_str = QString::number(number, 'f', 4) + "M";
	}
	return number_str;
}

void QCommonFuction::pack_to_little_endian(app_packet_t & pak)
{
	pak.load_len = qToLittleEndian<uint16_t>(pak.load_len);
	pak.msg_load.msg_type = qToLittleEndian<uint16_t>(pak.msg_load.msg_type);
	pak.msg_load.msg_len = qToLittleEndian<uint32_t>(pak.msg_load.msg_len);
}

void QCommonFuction::pack_from_little_endian(app_packet_t & pak)
{
	pak.load_len = qFromBigEndian<uint32_t>(pak.load_len);
	pak.msg_load.msg_type = qFromLittleEndian<uint32_t>(pak.msg_load.msg_type);
	pak.msg_load.msg_len = qFromLittleEndian<uint32_t>(pak.msg_load.msg_len);
}

bool QCommonFuction::check_pak_crc(msg_packet_t & pak)
{
	uint16_t crc = calc_crc((uint8_t*)&pak.msg_type, pak.msg_len + 6);
	uint16_t read_crc = qFromBigEndian<uint16_t>(&pak.msg_data[pak.msg_len]);
	return (crc == read_crc);
}

bool QCommonFuction::is_package_valid(app_packet_t & pak) {
	if (pak.load_len >= PACKET_MAX_SIZE) return false;
	if (pak.msg_load.msg_head != PACKET_MSG_HEADER) return false;
	if (pak.msg_load.msg_len >= PACKET_MAX_SIZE) return false;
	return true;
}

bool QCommonFuction::is_user_message(int type)
{
	bool ret = false;
	switch (type) {
	case IAP_CMD_GV:
	case USER_CMD_SET_CONFIG:
	case USER_CMD_GET_CONFIG:
		ret = true;
		break;
	default:
		break;
	}
	return ret;
}

bool QCommonFuction::is_log_message(int type) {
	bool ret = false;
	switch (type) {
	case em_RAW_IMU:
	case em_GNSS_SOL:
	case em_INS_SOL:
	case em_RAW_ODO:
	case em_DIAGNOSTIC_MSG:
	case em_ROVER_RTCM:
	case em_MISALIGN:
	case PowerUpDR_MES:
	case em_CHECK:
	case em_GNSS_SOL_INTEGEITY:
	case em_RTK_DEBUG1:
	case em_PACKAGE_FD:
		//case USER_CMD_SET_CONFIG:
		//case USER_CMD_GET_CONFIG:
	case USER_CMD_RTCM_DATA:
		ret = true;
		break;
	default:
		break;
	}
	return ret;
}

bool QCommonFuction::is_upgrade_message(int type) {
	bool ret = false;
	switch (type) {
		//case IAP_CMD_GV:
	case IAP_CMD_JI:
	case IAP_CMD_JA:
	case IAP_CMD_WA:
	case IAP_CMD_CS:
	case IMU_CMD_JI:
	case IMU_CMD_JI_RET:
	case IMU_CMD_JA:
	case IMU_CMD_JA_RET:
	case IMU_CMD_WA:
	case IMU_CMD_WA_RET:
	case SDK_CMD_JS:
	case SDK_CMD_JG:
	case SDK_CMD_SENDSDK:
	case SDK_CMD_SENDJL:
		ret = true;
		break;
	default:
		break;
	}
	return ret;
}