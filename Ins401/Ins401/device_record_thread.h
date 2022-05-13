#pragma once
#include <QThread>
#include <QFile>
#include <QMutex>
#include <QWaitCondition>
#include <QList>
#include "common_define.h"
#include "Ins401_Message.h"
#include "device_info.h"

class device_record_thread : public QThread
{
	Q_OBJECT

public:
	device_record_thread(QObject *parent, device_info_ptr info);
	~device_record_thread();
	void run();
	void stop();
	void log_data(const uint8_t * data, uint32_t len);
protected:
	bool open_record_files();
private:
	bool m_isStop;
	QByteArray log_byte_aray;
	QList<int> m_pak_pos_list;
	uint32_t log_size;
	QFile user_log_file;
	QFile rover_log_file;
	QMutex mutex;
	QWaitCondition cond_wait;
	device_info_ptr m_info_ptr;
signals:
	void sgnSendGGA(QByteArray gga);
	void sgnReceiveLogPak(int type);
public:
	ins_sol_t m_ins_pak;
	gnss_sol_t m_gnss_pak;
	raw_imu_t m_raw_imu;
};

typedef std::shared_ptr<device_record_thread> device_record_thread_ptr;