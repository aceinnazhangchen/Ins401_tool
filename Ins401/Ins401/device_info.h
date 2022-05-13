#pragma once

#include <QObject>
#include <QDir>
#include "common_define.h"

class device_info : public QObject
{
	Q_OBJECT

public:
	device_info(QObject *parent);
	~device_info();
	void set_dest_mac(uint8_t mac[MAC_ADDRESS_LEN]);
	void set_dev_info(const char* info);
	void set_ui_record_table_row(const int32_t row);
	QString get_mac_str();
	uint8_t * get_mac();
	QString get_dev_info();
	QString get_SN();
	QString get_PN();
	QString get_firmware_version();
	QString get_app_version();
	int32_t get_ui_record_table_row();
	void reg_exp_device_info();
	void make_record_path();
	QDir get_record_path();
private:
	QString sn;
	QString pn;
	QString firmware_version;
	QString app_version;
	uint64_t dest_mac_num;
	uint8_t dest_mac[MAC_ADDRESS_LEN];
	QString dev_info;
	int32_t ui_record_table_row;
	QDir record_path;
};

typedef std::shared_ptr<device_info> device_info_ptr;