#pragma once

#include <QThread>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QWaitCondition>

class ntrip_record_thread : public QThread
{
	Q_OBJECT

public:
	ntrip_record_thread(QObject *parent);
	~ntrip_record_thread();
	void run();
	void stop();
	void setTargetPath(QDir path);
protected:
	bool open_record_files();
public slots:
	void onReceiveData(QByteArray buffer);
private:
	bool m_isStop;
	uint32_t log_size;
	QByteArray log_byte_aray;
	QFile base_log_file;
	QMutex mutex;
	QWaitCondition cond_wait;
	QDir targetDir;
};

typedef std::shared_ptr<ntrip_record_thread> ntrip_record_thread_ptr;