#include "device_upgrade_thread.h"
#include <QtEndian>
#include "devices_manager.h"
#include "common_function.h"

const char core_name[2][4] = {
	"C0",
	"C1"
};
static uint8_t sta_sync[4] = ST9100_ETH_SYNC;

extern uint8_t  STA9100_Loader[];
extern uint32_t STA9100LoadSize;

device_upgrade_thread::device_upgrade_thread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, m_isRestarting(false)
	, m_wait_recv_count(0)
	, dest_mac_num(0)
	, dev_cmd_status(0)
	, ui_table_row(-1)
	, file_offset_addr(0)
	, ret_file_offset_addr(0)
	, upgrade_step(upgrade_rtk)
	, sta9100_last_recv_code(0)
{
	memset(&STA9100BinInfo, 0, sizeof(STA9100BinInfo));
	STA9100BinInfo.bootMode = 0x01;
	STA9100BinInfo.destinationAddress = 0x10000000;
	STA9100BinInfo.erase_nvm_u8 = 3;
}

device_upgrade_thread::~device_upgrade_thread()
{
	stop();
}

void device_upgrade_thread::run() 
{
	m_isStop = false;
	do {
		QList <sub_file_t>& sub_file_list = devices_manager::Instance().sub_file_list;
		if (sub_file_list[upgrade_rtk].file_switch ||
			sub_file_list[upgrade_ins].file_switch) {
			upgrade_app_process();
		}
		if (sub_file_list[upgrade_sdk].file_switch) {
			if (upgrade_sdk_process() == false) { break; }
		}
		if (sub_file_list[upgrade_imu].file_switch) {
			upgrade_imu_process();
		}
	} while (false);
	//finished
	emit devices_manager::Instance().sgnLog(QString::asprintf("The file is finished."));
	emit devices_manager::Instance().sgnThreadFinished();
}

void device_upgrade_thread::stop()
{
	mutex.lock();
	m_isStop = true;
	cond_wait.wakeAll();
	mutex.unlock();
}

void device_upgrade_thread::set_dest_mac(uint8_t mac[MAC_ADDRESS_LEN])
{
	memcpy(dest_mac, mac, MAC_ADDRESS_LEN);
	memcpy(&dest_mac_num, mac, MAC_ADDRESS_LEN);
}

void device_upgrade_thread::set_dev_info(const char * info)
{
	dev_info = info;
}

void device_upgrade_thread::set_ui_table_row(const int32_t row)
{
	ui_table_row = row;
}

QString device_upgrade_thread::get_mac_str()
{
	return QString::asprintf("%.2X:%.2X:%.2X:%.2X:%.2X:%.2X", dest_mac[0], dest_mac[1], dest_mac[2], dest_mac[3], dest_mac[4], dest_mac[5]);
}

QString device_upgrade_thread::get_dev_info()
{
	return dev_info;
}

int32_t device_upgrade_thread::get_ui_table_row()
{
	return ui_table_row;
}

void device_upgrade_thread::filled_SDK_bin_info()
{
	if (upgrade_step != upgrade_sdk) return;
	QByteArray& file_content = devices_manager::Instance().sub_file_list[upgrade_step].file_content;
	int file_size = file_content.size();
	STA9100BinInfo.binSize = file_size;
	int crc32 = 0;
	int read_len = 0;
	crc32 = calc_crc32(crc32, (uint8_t*)&STA9100BinInfo.binSize, 4);
	while (read_len < file_size) {
		QByteArray send_bytes = file_content.mid(read_len, 100);
		crc32 = calc_crc32(crc32, (uint8_t*)send_bytes.data(), send_bytes.size());
		read_len += send_bytes.size();
	}
	STA9100BinInfo.crc = crc32;
}

void device_upgrade_thread::recv_pack(msg_packet_t & msg_pak)
{
	switch (msg_pak.msg_type) {
	case IAP_CMD_GV:
		m_isRestarting = false;
		break;
	case IAP_CMD_JI:
	case IAP_CMD_JA:
	case SDK_CMD_JS:
	case SDK_CMD_JG:
	case IMU_CMD_JI_RET:
	case IMU_CMD_JA_RET:
	{
		mutex.lock();
		dev_cmd_status = msg_pak.msg_type;
		cond_wait.wakeAll();
		mutex.unlock();
		emit devices_manager::Instance().sgnChangeMode(ui_table_row, dev_cmd_status);
	}break;
	case IAP_CMD_CS:
	{
		mutex.lock();
		dev_cmd_status = msg_pak.msg_type;
		cond_wait.wakeAll();
		mutex.unlock();
	}break;
	case SDK_CMD_SENDSDK:
	{
		mutex.lock();
		dev_cmd_status = msg_pak.msg_type;
		if (m_wait_recv_count > 0) {
			m_wait_recv_count--;
		}		
		sta9100_last_recv_code = msg_pak.msg_data[0];
		emit devices_manager::Instance().sgnLog(QString::asprintf("reveive SDK size: %d ret[0]: 0x%2x", msg_pak.msg_len, sta9100_last_recv_code));
		cond_wait.wakeAll();
		mutex.unlock();
	}break;
	case IAP_CMD_WA:
	case IMU_CMD_WA_RET:
	{
		//ret_file_offset_addr = qFromBigEndian<uint32_t>(msg_pak.msg_data);
		mutex.lock();
		ret_file_offset_addr = file_offset_addr;
		dev_cmd_status = msg_pak.msg_type;
		cond_wait.wakeAll();
		mutex.unlock();
	}break;
	default:
		break;
	}
}

void device_upgrade_thread::send_get_version_cmd()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, IAP_CMD_GV, 0, NULL);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	//for (int i = 0; i < MAC_ADDRESS_LEN; i++)pak.dest_mac[i] = 0xff;
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_IAP_boot_cmd()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, IAP_CMD_JI, 0, NULL);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_IAP_app_cmd()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, IAP_CMD_JA, 0, NULL);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_IAP_set_core_cmd()
{
	if (upgrade_step > upgrade_ins) return;
	int file_size = devices_manager::Instance().sub_file_list[upgrade_step].file_content.size();
	const char* core = core_name[upgrade_step];
	app_packet_t pak = { 0 };
	uint8_t buffer[6] = { 0xff };
	memcpy(buffer, core, 2);
	qToBigEndian<uint32_t>(file_size, &buffer[2]);
	devices_manager::Instance().make_pack(pak, IAP_CMD_CS, 6, buffer);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
	emit devices_manager::Instance().sgnLog(QString::asprintf("Sending 0x%X Cmd CS to set %s and bin size %d 0x%08x-> 0x%02x|0x%02x|0x%02x|0x%02x "
		, pak.msg_load.msg_type, core, file_size, file_size, buffer[2], buffer[3], buffer[4], buffer[5]));
}

void device_upgrade_thread::send_SDK_boot_cmd()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, SDK_CMD_JS, 0, NULL);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_SDK_app_cmd()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, SDK_CMD_JG, 0, NULL);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_SDK_sync()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 4, sta_sync);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_SDK_change_baudrate()
{
	app_packet_t pak = { 0 };
	uint8_t buffer = ST9100_ETH_CHGBUAD;
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 1, &buffer);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_SDK_set_baudrate()
{
	app_packet_t pak = { 0 };
	uint8_t  buffer[4] = ST9100_BAUDRATE;
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 4, buffer);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_SDK_check_baudrate()
{
	app_packet_t pak = { 0 };
	uint8_t buffer = ST9100_ETH_CHKBAUD;
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 1, &buffer);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_SDK_check_ready()
{
	app_packet_t pak = { 0 };
	uint8_t buffer = ST9100_ETH_CHKREADY;
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 1, &buffer);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_SDK_boot_pre_info()
{
	app_packet_t pak = { 0 };
	uint8_t BootInfo[16] = ST9100_ETH_BOOTPREINFO;
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, ST9100_BOOTPREINFO_LEN, BootInfo);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);

	uint32_t crc32 = 0;
	uint32_t BootStartAddr = 0;
	uint8_t* pStaLoader = STA9100_Loader;
	crc32 = calc_crc32(crc32, (uint8_t*)&STA9100LoadSize, 4);
	crc32 = calc_crc32(crc32, (uint8_t*)&BootStartAddr, 4);
	crc32 = calc_crc32(crc32, pStaLoader, STA9100LoadSize);
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 4, (uint8_t*)&crc32);
	devices_manager::Instance().send_pack(pak);
	QThread::usleep(100);
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 4, (uint8_t*)&STA9100LoadSize);
	devices_manager::Instance().send_pack(pak);
	QThread::usleep(100);
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 4, (uint8_t*)&BootStartAddr);
	devices_manager::Instance().send_pack(pak);	
}

void device_upgrade_thread::send_SDK_boot_info()
{
	app_packet_t pak = { 0 };
	uint32_t bffer_current = 0;
	uint32_t pack_len = SDK_PACKET_LEN;
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	while (bffer_current < STA9100LoadSize) {
		if (STA9100LoadSize - bffer_current < SDK_PACKET_LEN) {
			pack_len = STA9100LoadSize - bffer_current;
		}
		devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, pack_len, &STA9100_Loader[bffer_current]);
		devices_manager::Instance().send_pack(pak);
		bffer_current += pack_len;
		QThread::usleep(100);
	}
}

void device_upgrade_thread::send_SDK_erase_cmd()
{
	app_packet_t pak = { 0 };
	uint8_t buffer = ST9100_ETH_EFLASH;
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, 1, &buffer);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_SDK_bin_info()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, SDK_CMD_SENDSDK, sizeof(stSTA9100BinInfo), (uint8_t*)&STA9100BinInfo);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_IMU_boot_cmd()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, IMU_CMD_JI, 0, NULL);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_IMU_app_cmd()
{
	app_packet_t pak = { 0 };
	devices_manager::Instance().make_pack(pak, IMU_CMD_JA, 0, NULL);
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	devices_manager::Instance().send_pack(pak);
}

void device_upgrade_thread::send_IAP_file()
{
	QByteArray& file_content = devices_manager::Instance().sub_file_list[upgrade_step].file_content;
	int file_size = file_content.size();
	if (file_offset_addr >= file_size) return;
	app_packet_t pak = { 0 };
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	uint8_t buffer[FILE_PACKET_LEN + 8] = { 0xff };// len(int) + off(int) + packet
	qToBigEndian<uint32_t>(file_offset_addr, buffer);
	qToBigEndian<uint32_t>(FILE_PACKET_LEN, buffer + sizeof(uint32_t));
	uint32_t need_read_len =  FILE_PACKET_LEN;
	if (file_size - file_offset_addr < FILE_PACKET_LEN) {
		need_read_len = file_size - file_offset_addr;
	}
	QByteArray send_bytes = file_content.mid(file_offset_addr, need_read_len);
	memcpy(&buffer[8], send_bytes.data(), send_bytes.size());
	devices_manager::Instance().make_pack(pak, IAP_CMD_WA, FILE_PACKET_LEN + 8, buffer);
	devices_manager::Instance().send_pack(pak);
	file_offset_addr += send_bytes.size();
	//UpdateProcess
	double percent = (double)file_offset_addr / (double)file_size * 10000;
	emit devices_manager::Instance().sgnUpdateProcess(ui_table_row, percent);
}

void device_upgrade_thread::send_SDK_file(int cmd)
{
	QByteArray& file_content = devices_manager::Instance().sub_file_list[upgrade_step].file_content;
	int file_size = file_content.size();
	if (file_offset_addr >= file_size) return;
	app_packet_t pak = { 0 };
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	uint8_t buffer[SDK_PACKET_LEN] = { 0xff };
	uint32_t need_read_len = SDK_PACKET_LEN;
	if (file_size - file_offset_addr < SDK_PACKET_LEN) {
		need_read_len = file_size - file_offset_addr;
	}
	QByteArray send_bytes = file_content.mid(file_offset_addr, need_read_len);
	memcpy(buffer, send_bytes.data(), send_bytes.size());
	devices_manager::Instance().make_pack(pak, cmd, send_bytes.size(), buffer);
	devices_manager::Instance().send_pack(pak);
	file_offset_addr += send_bytes.size();;
	//UpdateProcess
	double percent = (double)file_offset_addr / (double)file_size * 10000;
	emit devices_manager::Instance().sgnUpdateProcess(ui_table_row, percent);
}

void device_upgrade_thread::send_IMU_file()
{
	QByteArray& file_content = devices_manager::Instance().sub_file_list[upgrade_step].file_content;
	int file_size = file_content.size();
	if (file_offset_addr >= file_size) return;
	app_packet_t pak = { 0 };
	memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
	uint8_t buffer[FILE_PACKET_LEN + 5] = { 0xff };// len(int) + off(int) + packet
	qToBigEndian<uint32_t>(file_offset_addr, buffer);
	buffer[4] = IMU_PACKET_LEN & 0xff;
	uint32_t need_read_len = IMU_PACKET_LEN;
	if (file_size - file_offset_addr < IMU_PACKET_LEN) {
		need_read_len = file_size - file_offset_addr;
	}
	QByteArray send_bytes = file_content.mid(file_offset_addr, need_read_len);
	memcpy(&buffer[5], send_bytes.data(), send_bytes.size());
	devices_manager::Instance().make_pack(pak, IMU_CMD_WA, IMU_PACKET_LEN + 5, buffer);
	devices_manager::Instance().send_pack(pak);
	file_offset_addr += send_bytes.size();
	//UpdateProcess
	double percent = (double)file_offset_addr / (double)file_size * 10000;
	emit devices_manager::Instance().sgnUpdateProcess(ui_table_row, percent);
}

void device_upgrade_thread::waiting_restart()
{
	m_isRestarting = true;
	int loop_count = 0;
	emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "Restarting");
	while (m_isRestarting == true && !m_isStop) {
		QThread::sleep(1);
		loop_count++;
		if (loop_count >= 5) {
			send_get_version_cmd();
		}		
	}
	emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "Ready");
}

void device_upgrade_thread::step_IAP_jump_JI()
{
	send_IAP_boot_cmd();
	mutex.lock();
	while (dev_cmd_status != IAP_CMD_JI && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	waiting_restart();
}

void device_upgrade_thread::step_IAP_write_CS() {
	if (upgrade_step > upgrade_ins) return;
	send_IAP_set_core_cmd();
	emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "Start Write");
	mutex.lock();
	while (dev_cmd_status != IAP_CMD_CS && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
}

void device_upgrade_thread::step_IAP_write_file()
{
	if (upgrade_step > upgrade_ins) return;
	emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "Writting");
	QByteArray& file_content = devices_manager::Instance().sub_file_list[upgrade_step].file_content;
	int file_size = file_content.size();
	while (file_offset_addr < file_size) {
		if (m_isStop)break;
		mutex.lock();
		while (ret_file_offset_addr != file_offset_addr && !m_isStop) {
			cond_wait.wait(&mutex);
		}
		if (ret_file_offset_addr == file_offset_addr) {
			send_IAP_file();
		}
		mutex.unlock();
	}
}

void device_upgrade_thread::step_IAP_jump_JA()
{
	send_IAP_app_cmd();
	mutex.lock();
	while (dev_cmd_status != IAP_CMD_JA && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	waiting_restart();
}

void device_upgrade_thread::step_SDK_jump_JS()
{
	send_SDK_boot_cmd();
	mutex.lock();
	while (dev_cmd_status != SDK_CMD_JS && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	waiting_restart();
}

void device_upgrade_thread::step_SDK_jump_JG()
{
	send_SDK_app_cmd();
	mutex.lock();
	while (dev_cmd_status != SDK_CMD_JG && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	waiting_restart();
}

void device_upgrade_thread::step_SDK_sync()
{
	emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "Synchronizing");
	while (dev_cmd_status != SDK_CMD_SENDSDK && !m_isStop) {
		send_SDK_sync();
		QThread::msleep(100);
	}
	emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "Synchronized");
}

void device_upgrade_thread::step_SDK_set_baudrate()
{
	/*
	//send change baudrate
	emit devices_manager::Instance().sgnLog(QString::asprintf("send change baudrate."));
	send_SDK_change_baudrate();
	m_wait_recv_count = 1;
	mutex.lock();
	while (m_wait_recv_count > 0 && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	emit devices_manager::Instance().sgnLog(QString::asprintf("revc change baudrate."));
	QThread::msleep(1);
	//send set baudrate
	emit devices_manager::Instance().sgnLog(QString::asprintf("send set baudrate."));
	send_SDK_set_baudrate();
	m_wait_recv_count = 1;
	mutex.lock();
	while (m_wait_recv_count > 0 && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	emit devices_manager::Instance().sgnLog(QString::asprintf("revc set baudrate."));
	QThread::msleep(1);
	//send check baudrate
	emit devices_manager::Instance().sgnLog(QString::asprintf("send check baudrate."));
	send_SDK_check_baudrate();
	m_wait_recv_count = 1;
	mutex.lock();
	while (m_wait_recv_count > 0 && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	emit devices_manager::Instance().sgnLog(QString::asprintf("revc check baudrate."));
	QThread::msleep(1);
	*/
	//send check ready
	emit devices_manager::Instance().sgnLog(QString::asprintf("send check ready."));
	send_SDK_check_ready();
	m_wait_recv_count = 1;
	mutex.lock();
	while (m_wait_recv_count > 0 && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	emit devices_manager::Instance().sgnLog(QString::asprintf("revc check ready."));
	QThread::msleep(1);
}

void device_upgrade_thread::step_SDK_send_loader()
{
	emit devices_manager::Instance().sgnLog(QString::asprintf("send boot info."));
	send_SDK_boot_pre_info();
	QThread::msleep(1);
	send_SDK_boot_info();
	m_wait_recv_count = 1;
	mutex.lock();
	while (m_wait_recv_count > 0 && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	QThread::usleep(100);
	emit devices_manager::Instance().sgnLog(QString::asprintf("revc boot info."));
}

void device_upgrade_thread::step_SDK_start_write()
{
	//send erase cmd.
	emit devices_manager::Instance().sgnLog(QString::asprintf("send erase cmd."));
	send_SDK_erase_cmd();
	m_wait_recv_count = 1;
	mutex.lock();
	while (m_wait_recv_count > 0 && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	emit devices_manager::Instance().sgnLog(QString::asprintf("revc erase cmd."));
	QThread::sleep(5);
	//send start write
	filled_SDK_bin_info();
	emit devices_manager::Instance().sgnLog(QString::asprintf("send start write."));
	send_SDK_bin_info();
	m_wait_recv_count = 4;
	mutex.lock();
	while (m_wait_recv_count > 0 && !m_isStop) {
		cond_wait.wait(&mutex);
		emit devices_manager::Instance().sgnLog(QString::asprintf("revc start write %d.", m_wait_recv_count));
	}
	mutex.unlock();
}

void device_upgrade_thread::step_SDK_write_file()
{
	if (upgrade_step > upgrade_all) return;
	emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "Writting");
	QByteArray& file_content = devices_manager::Instance().sub_file_list[upgrade_step].file_content;
	int file_size = file_content.size();
	int loop_count = 0;
	while (file_offset_addr < file_size) {
		if (m_isStop)break;
		send_SDK_file(SDK_CMD_SENDSDK);
		QThread::msleep(10);
		loop_count++;
		if (loop_count == 5) {
			loop_count = 0;
			mutex.lock();
			m_wait_recv_count = 1;
			if (m_wait_recv_count > 0 && !m_isStop) {
				cond_wait.wait(&mutex);
			}
			mutex.unlock();
			QThread::msleep(1);
		}
	}
	m_wait_recv_count = 1;
	QThread::sleep(2);
	mutex.lock();
	if (m_wait_recv_count > 0 && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
}

void device_upgrade_thread::step_IMU_jump_JI()
{
	while (dev_cmd_status != IMU_CMD_JI_RET && !m_isStop) {
		send_IMU_boot_cmd();
		QThread::msleep(200);
	}
	QThread::msleep(1000);
	//waiting_restart();
}

void device_upgrade_thread::step_IMU_jump_JA()
{
	send_IMU_app_cmd();
	mutex.lock();
	while (dev_cmd_status != IMU_CMD_JA_RET && !m_isStop) {
		cond_wait.wait(&mutex);
	}
	mutex.unlock();
	waiting_restart();
}

void device_upgrade_thread::step_IMU_write_file()
{
	if (upgrade_step > upgrade_all) return;
	emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "Writting");
	QByteArray& file_content = devices_manager::Instance().sub_file_list[upgrade_step].file_content;
	int file_size = file_content.size();
	while (file_offset_addr < file_size) {
		if (m_isStop)break;
		mutex.lock();
		while (ret_file_offset_addr != file_offset_addr && !m_isStop) {
			cond_wait.wait(&mutex);
		}
		if (ret_file_offset_addr == file_offset_addr) {
			send_IMU_file();
		}
		mutex.unlock();
	}
}

bool device_upgrade_thread::upgrade_rtk_process()
{
	upgrade_step = upgrade_rtk;
	file_offset_addr = 0;
	ret_file_offset_addr = 0;
	emit devices_manager::Instance().sgnUpgradeStep(ui_table_row, "rtk");
	step_IAP_write_CS();
	step_IAP_write_file();
	return true;
}

bool device_upgrade_thread::upgrade_ins_process()
{
	upgrade_step = upgrade_ins;
	file_offset_addr = 0;
	ret_file_offset_addr = 0;
	emit devices_manager::Instance().sgnUpgradeStep(ui_table_row, "ins");
	step_IAP_write_CS();
	step_IAP_write_file();
	return true;
}

bool device_upgrade_thread::upgrade_app_process()
{
	QList <sub_file_t>& sub_file_list = devices_manager::Instance().sub_file_list;
	step_IAP_jump_JI();
	if (sub_file_list[upgrade_rtk].file_switch) {
		upgrade_rtk_process();
	}
	if (sub_file_list[upgrade_ins].file_switch) {
		upgrade_ins_process();
	}	
	step_IAP_jump_JA();
	return true;
}

bool device_upgrade_thread::upgrade_sdk_process()
{
	upgrade_step = upgrade_sdk;
	file_offset_addr = 0;
	ret_file_offset_addr = 0;
	emit devices_manager::Instance().sgnUpgradeStep(ui_table_row, "sdk");
	step_SDK_jump_JS();
	step_SDK_sync();
	for (int i = 0; i < 5; i++) {
		send_SDK_file(SDK_CMD_SENDJL);
		QThread::msleep(10);
	}
	QThread::sleep(4);
	step_SDK_set_baudrate();
	//QThread::sleep(1);
	step_SDK_send_loader();
	QThread::sleep(5);
	step_SDK_start_write();
	QThread::sleep(2);
	step_SDK_write_file();
	step_SDK_jump_JG();
	if (sta9100_last_recv_code == 0xcc) {
		emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "success");
		return true;
	}
	else {
		emit devices_manager::Instance().sgnUpdateStatus(ui_table_row, "falied");
		return false;
	}
}

bool device_upgrade_thread::upgrade_imu_process()
{
	upgrade_step = upgrade_imu;
	file_offset_addr = 0;
	ret_file_offset_addr = 0;
	emit devices_manager::Instance().sgnUpgradeStep(ui_table_row, "imu");
	step_IMU_jump_JI();
	step_IMU_write_file();
	step_IMU_jump_JA();
	return false;
}
