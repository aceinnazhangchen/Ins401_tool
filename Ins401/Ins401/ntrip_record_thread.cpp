#include "ntrip_record_thread.h"
#include <QDateTime>

ntrip_record_thread::ntrip_record_thread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, log_size(0)
{
}

ntrip_record_thread::~ntrip_record_thread()
{
}

void ntrip_record_thread::run()
{
	m_isStop = false;
	log_size = 0;
	log_byte_aray.clear();
	if (!open_record_files()) {
		return;
	}
	QByteArray data_array;
	data_array.clear();
	while (!m_isStop)
	{
		mutex.lock();
		if (log_byte_aray.size() == 0) {
			cond_wait.wait(&mutex);
		}
		data_array.swap(log_byte_aray);
		mutex.unlock();
		base_log_file.write(data_array);
		log_size += data_array.size();
		data_array.clear();
	}
	base_log_file.close();
}

void ntrip_record_thread::stop()
{
	m_isStop = true;
	cond_wait.wakeAll();
}

void ntrip_record_thread::setTargetPath(QDir path)
{
	targetDir = path;
}

bool ntrip_record_thread::open_record_files()
{
	QDateTime curDateTime = QDateTime::currentDateTime();
	QString time_str = curDateTime.toString("yyyy_MM_dd_hh_mm_ss");
	base_log_file.setFileName(targetDir.absolutePath() + QDir::separator() + "rtcm_base_" + time_str + ".bin");
	if(base_log_file.open(QIODevice::WriteOnly)){
		return true;
	}else {
		base_log_file.close();
		return false;
	}
}

void ntrip_record_thread::onReceiveData(QByteArray buffer)
{
	if (base_log_file.isOpen()) {
		mutex.lock();
		log_byte_aray.append(buffer);
		cond_wait.wakeAll();
		mutex.unlock();
	}
}
