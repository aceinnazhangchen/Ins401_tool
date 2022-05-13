#include "device_obj.h"
#include "devices_manager.h"
#include "ConfigFile.h"
#include <QDateTime.h>

device_obj::device_obj(QObject *parent)
	: QObject(parent)
{
	m_device_info_ptr = device_info_ptr(new device_info(this));
	m_record_thread_ptr = device_record_thread_ptr(new device_record_thread(this, m_device_info_ptr));
	m_base_ntrip_ptr = ntrip_client_ptr(new NtripClient(this));
	m_ntrip_record_thread_ptr = ntrip_record_thread_ptr(new ntrip_record_thread(this));

	connect(m_record_thread_ptr.get(), SIGNAL(sgnSendGGA(QByteArray)), m_base_ntrip_ptr.get(), SLOT(onSendGGA(QByteArray)), Qt::QueuedConnection);
	connect(m_base_ntrip_ptr.get(), SIGNAL(sgnData(QByteArray)), m_ntrip_record_thread_ptr.get(), SLOT(onReceiveData(QByteArray)), Qt::QueuedConnection);
}

device_obj::~device_obj()
{
	disconnect(m_record_thread_ptr.get(), SIGNAL(sgnSendGGA(QByteArray)), m_base_ntrip_ptr.get(), SLOT(onSendGGA(QByteArray)));
	disconnect(m_base_ntrip_ptr.get(), SIGNAL(sgnData(QByteArray)), m_ntrip_record_thread_ptr.get(), SLOT(onReceiveData(QByteArray)));
	m_device_info_ptr = NULL;
	m_record_thread_ptr = NULL;
	m_base_ntrip_ptr = NULL;
	m_ntrip_record_thread_ptr = NULL;
}

void device_obj::start_record()
{
	if (!m_record_thread_ptr->isRunning()) {
		m_record_thread_ptr->start();
	}
	if (!m_base_ntrip_ptr->isOpen()) {
		m_base_ntrip_ptr->open();
	}
	if (!m_ntrip_record_thread_ptr->isRunning()) {
		m_ntrip_record_thread_ptr->setTargetPath(m_device_info_ptr->get_record_path());
		m_ntrip_record_thread_ptr->start();
	}
}

void device_obj::stop_record()
{
	if (m_base_ntrip_ptr->isOpen()) {
		m_base_ntrip_ptr->close();
	}
	if (m_record_thread_ptr->isRunning()) {
		m_record_thread_ptr->stop();
		m_record_thread_ptr->wait();
	}	
	if (m_ntrip_record_thread_ptr->isRunning()) {
		m_ntrip_record_thread_ptr->stop();
		m_ntrip_record_thread_ptr->wait();
	}
}

bool device_obj::is_recording()
{
	return m_record_thread_ptr->isRunning();
}

void device_obj::set_dest_mac(uint8_t mac[MAC_ADDRESS_LEN])
{
	m_device_info_ptr->set_dest_mac(mac);
	m_base_ntrip_ptr->setDestMac(mac);
}

void device_obj::set_dev_info(const char * info)
{
	m_device_info_ptr->set_dev_info(info);
}

void device_obj::makeRecordPath()
{
	m_device_info_ptr->make_record_path();
}

void device_obj::setNtripConfig(stNtripConfig & config)
{
	m_base_ntrip_ptr->setConfig(config);
}

void device_obj::recordConfig(QJsonObject & config)
{
	QJsonArray root_array;
	QJsonObject app;
	app.insert("app_name", "RTK_INS");
	app.insert("version", m_device_info_ptr->get_app_version());
	QJsonObject device;
	device.insert("name", "INS401");
	device.insert("imu", "INS401");
	device.insert("pn", m_device_info_ptr->get_PN());
	device.insert("firmware_version", m_device_info_ptr->get_firmware_version());
	device.insert("sn", m_device_info_ptr->get_SN());
	QJsonObject root;
	root.insert("time", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
	root.insert("app", app);
	root.insert("device", device);
	root.insert("interface", "100base-t1");
	root.insert("parameters", config);
	root_array.append(root);

	QJsonDocument document;
	document.setArray(root_array);
	QByteArray byteArray = document.toJson(QJsonDocument::Indented);
	QString jsonStr = (byteArray);
	QFile config_file;
	QDir targetDir = m_device_info_ptr->get_record_path();
	config_file.setFileName(targetDir.absolutePath() + QDir::separator() + "configuration.json");
	if (!config_file.open(QIODevice::ReadWrite | QIODevice::Text | QIODevice::Truncate))
	{
		qDebug() << "file error";
	}
	QTextStream in(&config_file);
	in << jsonStr;
	config_file.close();
}

device_info_ptr device_obj::get_info()
{
	return m_device_info_ptr;
}

device_record_thread_ptr device_obj::get_recorder()
{
	return m_record_thread_ptr;
}

ntrip_client_ptr device_obj::get_ntrip()
{
	return m_base_ntrip_ptr;
}
