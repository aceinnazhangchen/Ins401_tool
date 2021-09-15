#pragma once

#include <QThread>
#include <QFile>
#include <QWaitCondition>
#include <QMutex>
#include "common_define.h"

class device_log_thread : public QThread
{
	Q_OBJECT

public:
	device_log_thread(QObject *parent);
	~device_log_thread();
	void run();
	void stop();
	void set_dest_mac(uint8_t mac[MAC_ADDRESS_LEN]);
	void set_dev_info(const char* info);
	void set_ui_table_row(const int32_t row);
	QString get_mac_str();
	QString get_dev_info();
	int32_t get_ui_table_row();
	void log_data(const uint8_t * data, uint32_t len);
	void reg_exp_sn();
protected:
	void set_file_name();
private:
	bool m_isStop;
	uint64_t dest_mac_num;
	uint8_t dest_mac[MAC_ADDRESS_LEN];
	QString dev_info;
	QString sn;
	int32_t ui_table_row;
	QByteArray log_byte_aray;
	uint32_t log_size;
	QFile log_file;
	QMutex mutex;
	QWaitCondition cond_wait;
};
