#include "device_record_thread.h"
#include <QDateTime>
#include "devices_manager.h"

device_record_thread::device_record_thread(QObject *parent, device_info_ptr info)
	: QThread(parent)
	, m_isStop(false)
	, log_size(0)
{
	m_info_ptr = info;
	m_pak_pos_list.clear();
	memset(&m_ins_pak, 0, sizeof(m_ins_pak));
	memset(&m_gnss_pak, 0, sizeof(m_gnss_pak));
}

device_record_thread::~device_record_thread()
{
	m_info_ptr = NULL;
}

void device_record_thread::run()
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
		QList<int> pak_pos_list;
		mutex.lock();
		if (log_byte_aray.size() == 0) {
			cond_wait.wait(&mutex);
		}
		data_array.swap(log_byte_aray);
		pak_pos_list.swap(m_pak_pos_list);
		mutex.unlock();

		int start_pos = 0;
		for (int i = 0; i < pak_pos_list.size(); i++) {
			int end_pos = pak_pos_list[i];
			int pak_length = end_pos - start_pos;
			QByteArray pak_data((const char*)data_array.data() + start_pos, pak_length);
			if (pak_data[0] == '$') {
				if (pak_data.startsWith("$GNGGA")) {
					emit sgnSendGGA(pak_data);
					//qDebug("%s \n", (const char*)pak_data.data());
					//qDebug("%s, %d ,%d\n", (const char*)pak_data.data(), start_pos, pak_length);
				}
				user_log_file.write(pak_data);
				log_size += pak_data.size();
			}
			else {
				//decode pak
				msg_packet_t pak = { 0 };
				memcpy(&pak, pak_data.data(), pak_data.size());
				//qDebug("%x,%x,%d \n", pak.msg_head,pak.msg_type,pak.msg_len);
				switch (pak.msg_type)
				{
				case em_RAW_IMU:
				{
					if (sizeof(raw_imu_t) == pak.msg_len) {
						memcpy(&m_raw_imu, pak.msg_data, pak.msg_len);

					}
				}
				break;
				case em_GNSS_SOL:
				{
					if (sizeof(gnss_sol_t) == pak.msg_len) {
						memcpy(&m_gnss_pak, pak.msg_data, pak.msg_len);
						emit sgnReceiveLogPak(em_GNSS_SOL);
					}
				}break;
				case em_INS_SOL:
				{
					if (sizeof(ins_sol_t) == pak.msg_len) {
						memcpy(&m_ins_pak, pak.msg_data, pak.msg_len);
						if (m_ins_pak.gps_millisecs % 1000 == 0) {
							emit sgnReceiveLogPak(em_INS_SOL);
						}
					}
				}break;
				default:
					break;
				}
				if (pak.msg_type == em_ROVER_RTCM) {
					rover_log_file.write((const char*)pak.msg_data, pak.msg_len);
					log_size += pak_data.size();
				}
				else {
					user_log_file.write(pak_data);
					log_size += pak_data.size();
				}
			}
			start_pos = end_pos;
		}
		//user_log_file.write(data_array);
		//log_size += data_array.size();
		emit devices_manager::Instance().sgnLogSize(m_info_ptr->get_ui_record_table_row(), log_size);
		data_array.clear();
	}
	user_log_file.close();
	rover_log_file.close();
}

void device_record_thread::stop()
{
	m_isStop = true;
	cond_wait.wakeAll();
}

void device_record_thread::log_data(const uint8_t * data, uint32_t len)
{
	if (user_log_file.isOpen()) {
		mutex.lock();
		log_byte_aray.append((const char*)data, len);
		m_pak_pos_list.append(log_byte_aray.size());//插入每次加入数据后的cache长度
		cond_wait.wakeAll();
		mutex.unlock();
	}
}

bool device_record_thread::open_record_files()
{
	QDir targetDir = m_info_ptr->get_record_path();
	QDateTime curDateTime = QDateTime::currentDateTime();
	QString time_str = curDateTime.toString("yyyy_MM_dd_hh_mm_ss");
	user_log_file.setFileName(targetDir.absolutePath() + QDir::separator() + "user_" + time_str + ".bin");
	rover_log_file.setFileName(targetDir.absolutePath() + QDir::separator() + "rtcm_rover_" + time_str + ".bin");
	user_log_file.open(QIODevice::WriteOnly);
	rover_log_file.open(QIODevice::WriteOnly);
	if (user_log_file.isOpen() && rover_log_file.isOpen()) {
		return true;
	}
	else {
		user_log_file.close();
		rover_log_file.close();
		return false;
	}
}
