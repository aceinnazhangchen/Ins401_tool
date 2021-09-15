#include "device_log_thread.h"
#include <QDateTime>
#include "devices_manager.h"

device_log_thread::device_log_thread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, dest_mac_num(0)
	, ui_table_row(0)
	, log_size(0)
{
}

device_log_thread::~device_log_thread()
{
}

void device_log_thread::run()
{
	m_isStop = false;
	log_size = 0;
	log_byte_aray.clear();
	set_file_name();
	if (!log_file.open(QIODevice::WriteOnly)) {
		return;
	}
	while (!m_isStop)
	{
		mutex.lock();
		if (log_byte_aray.size() == 0) {
			cond_wait.wait(&mutex);
		}
		log_file.write(log_byte_aray);
		log_size += log_file.size();
		emit devices_manager::Instance().sgnLogSize(ui_table_row, log_size);
		log_byte_aray.clear();
		mutex.unlock();
	}
	log_file.close();
}

void device_log_thread::stop()
{
	m_isStop = true;
	cond_wait.wakeAll();
}

void device_log_thread::set_dest_mac(uint8_t mac[MAC_ADDRESS_LEN])
{
	memcpy(dest_mac, mac, MAC_ADDRESS_LEN);
	memcpy(&dest_mac_num, mac, MAC_ADDRESS_LEN);
}

void device_log_thread::set_dev_info(const char * info)
{
	dev_info = info;
	reg_exp_sn();
}

void device_log_thread::set_ui_table_row(const int32_t row)
{
	ui_table_row = row;
}

QString device_log_thread::get_mac_str()
{
	return QString::asprintf("%.2X:%.2X:%.2X:%.2X:%.2X:%.2X", dest_mac[0], dest_mac[1], dest_mac[2], dest_mac[3], dest_mac[4], dest_mac[5]);
}

QString device_log_thread::get_dev_info()
{
	return dev_info;
}

int32_t device_log_thread::get_ui_table_row()
{
	return ui_table_row;
}

void device_log_thread::log_data(const uint8_t * data, uint32_t len)
{
	if (log_file.isOpen()) {
		mutex.lock();
		log_byte_aray.append((const char*)data, len);
		cond_wait.wakeAll();
		mutex.unlock();
	}
}

void device_log_thread::reg_exp_sn()
{
	QStringList list;
	list = dev_info.split(" ");
	if (list.size() >= 4) {
		sn = list[2];
	}
}

void device_log_thread::set_file_name()
{
	QDir targetDir = devices_manager::Instance().log_path;
	QDateTime curDateTime = QDateTime::currentDateTime();
	QString time_str = curDateTime.toString("yyyy_MM_dd_hh_mm_ss");
	log_file.setFileName(targetDir.absolutePath() + QDir::separator() + "raw_" + sn+ "_" + time_str + ".bin");
}
