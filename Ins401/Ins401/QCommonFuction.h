#pragma once

#include <QObject>
#include "common_define.h"

class QCommonFuction : public QObject
{
	Q_OBJECT

public:
	QCommonFuction(QObject *parent);
	~QCommonFuction();
	static QString FormatBytes(int byte);
	static void pack_to_little_endian(app_packet_t & pak);
	static void pack_from_little_endian(app_packet_t & pak);
	static bool check_pak_crc(msg_packet_t & pak);
	static bool is_package_valid(app_packet_t & pak);
	static bool is_user_message(int type);
	static bool is_log_message(int type);
	static bool is_upgrade_message(int type);
};
