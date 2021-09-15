#include "devices_manager.h"
#include <mutex>
#include <QNetworkInterface>
#include <QtEndian>
#include <QDateTime>
#include "common_function.h"

devices_manager * devices_manager::m_instance = NULL;
std::once_flag      devices_manager::m_flag;

devices_manager::devices_manager(QObject *parent)
	: QObject(parent)
	, src_mac_num(0)
	, filter_log_pak(true)
	, filter_mac_flags(false)
	, log_flag(false)
{
	pcap_thread = new pcap_loop_thread(this);
	net_card_list.clear();
	net_mac_list.clear();
	upgrade_dev_list.clear();
	select_dev_list.clear();
	memset(src_mac, 0, MAC_ADDRESS_LEN);
	sub_file_list.clear();
	sub_file_list.append(sub_file_t("rtk_start:"));
	sub_file_list.append(sub_file_t("ins_start:"));
	sub_file_list.append(sub_file_t("sdk_start:",false));
	sub_file_list.append(sub_file_t("imu_start:"));
}

devices_manager::~devices_manager()
{
	stop_listen();
}

devices_manager& devices_manager::Instance()
{
	if (m_instance == NULL)
	{
		try
		{
			std::call_once(m_flag, createInstance);
		}
		catch (...)
		{
			qDebug("CreateInstance error\n");
		}
	}
	return *m_instance;
}

void devices_manager::createInstance()
{
	m_instance = new(std::nothrow) devices_manager(NULL);
	if (NULL == m_instance)
	{
		throw std::exception();
	}
}

void devices_manager::init_cmd()
{
	cmd_list.clear();
	cmd_list.append(cmd_info_t(IAP_CMD_JI, "0xaa01 IAP Jump to  Boot "));
	cmd_list.append(cmd_info_t(IAP_CMD_JA, "0xaa02 IAP Jump to  App"));
	//cmd_list.append(cmd_info_t(IAP_CMD_WA, "0xaa03 Write APP"));
	cmd_list.append(cmd_info_t(IAP_CMD_CS, "0xaa04 Set Write Core"));
	cmd_list.append(cmd_info_t(SDK_CMD_JS, "0xaa05 SDK Jump to  Boot"));
	cmd_list.append(cmd_info_t(SDK_CMD_JG, "0xaa06 SDK Jump to  App"));
	cmd_list.append(cmd_info_t(SDK_CMD_SENDSDK, "0xaa07 Send SDK"));
	cmd_list.append(cmd_info_t(IMU_CMD_JI, "0x4a49 IMU Jump to Boot"));
	cmd_list.append(cmd_info_t(IMU_CMD_JA, "0x4a41 IMU Jump to APP"));

	cmd_list.append(cmd_info_t(IAP_CMD_GV, "0xcc01 Get Version"));	
}

int devices_manager::search_cards()
{
	pcap_if_t * alldevs = NULL;
	pcap_if_t * dev = NULL;
	net_card_list.clear();
	char errbuf[PCAP_ERRBUF_SIZE];
	if (pcap_findalldevs(&alldevs, errbuf) == -1)
	{
		//onLog(QString::asprintf("Error in pcap_findalldevs: %s\n", errbuf));
		return -1;
	}
	int i = 0;
	for (dev = alldevs; dev; dev = dev->next)
	{
		if (dev->description && dev->addresses) {
			if (PCAP_IF_WIRELESS & dev->flags) continue;
			if (strstr(dev->description, "Virtual")) continue;
			if (strstr(dev->description, "VPN")) continue;
			net_card_t card = { 0 };
			strcpy(card.show_name, dev->description);
			strcpy(card.dev_name, dev->name);
			net_card_list.push_back(card);
		}
	}
	pcap_freealldevs(alldevs);
	return 0;
}

void devices_manager::search_macs()
{
	bool ok;
	net_mac_list.clear();
	QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
	for (int i = 0; i < interfaces.length(); i++) {
		if (interfaces[i].isValid() && interfaces[i].type() == QNetworkInterface::Ethernet) {
			net_mac_t mac_address = { 0 };
			mac_address.name_str = interfaces[i].humanReadableName();
			mac_address.address_str = interfaces[i].hardwareAddress();
			QStringList mac_num_list = mac_address.address_str.split(":");
		
			for (int i = 0; i < mac_num_list.size() && i < MAC_ADDRESS_LEN; i++) {
				int num = mac_num_list[i].toInt(&ok, 16);
				mac_address.mac[i] = (uint8_t)num;
			}
			net_mac_list.push_back(mac_address);
		}
	}
}

void devices_manager::search_devs_listen(int card_index, int mac_index)
{
	if (pcap_thread == NULL) return;
	if (!pcap_thread->isRunning()) {
		pcap_thread->set_dev_name(net_card_list[card_index].dev_name);
		memcpy(src_mac, net_mac_list[mac_index].mac, MAC_ADDRESS_LEN);
		memcpy(&src_mac_num, net_mac_list[mac_index].mac, MAC_ADDRESS_LEN);
		pcap_thread->start();
	}
}

void devices_manager::stop_listen()
{
	if (pcap_thread == NULL) return;
	if (pcap_thread->isRunning()) {
		pcap_thread->stop();
		pcap_thread->wait();
	}

	upgrade_devices::iterator it;
	for (it = upgrade_dev_list.begin(); it != upgrade_dev_list.end(); ++it) {
		device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
		if (upgrade_thread->isRunning()) {
			upgrade_thread->stop();
			upgrade_thread->wait();
		}
		delete upgrade_thread;
	}
	upgrade_dev_list.clear();

	log_devices::iterator it_log;
	for (it_log = log_dev_list.begin(); it_log != log_dev_list.end(); ++it_log) {
		device_log_thread* log_thread = (device_log_thread*)it_log.value();
		log_thread->stop();
		log_thread->wait();
		delete log_thread;
	}
	log_dev_list.clear();

	emit sgnClearDevices();
}

bool devices_manager::is_listening()
{
	return pcap_thread->isRunning();
}

void devices_manager::debug_pack(int index)
{
	if (index >= 0 && index < cmd_list.size()) {
		uint16_t cmd = cmd_list[index].cmd;
		upgrade_devices::iterator it;
		for (int i = 0; i < select_dev_list.size(); i++) {
			uint64_t dest_mac_num = select_dev_list[i];
			upgrade_devices::iterator it = upgrade_dev_list.find(dest_mac_num);
			if (it != upgrade_dev_list.end()) {
				device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
				switch (cmd)
				{
				case IAP_CMD_GV:
					upgrade_thread->send_get_version_cmd();
					break;
				case IAP_CMD_JI:
					upgrade_thread->send_IAP_boot_cmd();
					break;
				case IAP_CMD_JA:
					upgrade_thread->send_IAP_app_cmd();
					break;
				case IAP_CMD_CS:
					upgrade_thread->send_IAP_set_core_cmd();
					break;
				case SDK_CMD_JS:
					upgrade_thread->send_SDK_boot_cmd();
					break;
				case SDK_CMD_JG:
					upgrade_thread->send_SDK_app_cmd();
					break;
				case SDK_CMD_SENDSDK:
					upgrade_thread->send_SDK_sync();
					break;
				case IMU_CMD_JI:
					upgrade_thread->send_IMU_boot_cmd();
					break;
				case IMU_CMD_JA:
					upgrade_thread->send_IMU_app_cmd();
					break;
				default:
					break;
				}
			}
		}
	}
}

bool devices_manager::check_pak_crc(msg_packet_t & pak)
{
	uint16_t crc = calc_crc((uint8_t*)&pak.msg_type, pak.msg_len + 6);
	uint16_t read_crc = qFromBigEndian<uint16_t>(&pak.msg_data[pak.msg_len]);
	return (crc == read_crc);
}

bool devices_manager::fitler_pack(app_packet_t & pak)
{
	if (pak.load_len >= PACKET_MAX_SIZE) return false;
	if (pak.msg_load.msg_head != PACKET_MSG_HEADER) return false;
	if (pak.msg_load.msg_len >= PACKET_MAX_SIZE) return false;

	if (filter_log_pak) {
		switch (pak.msg_load.msg_type) {
		case 0x0a01:
		case 0x0a02:
		case 0x0a03:
		case 0x0a04:
		case 0x0a05:
		case 0x0a06:
			return false;
			break;
		default:
			return true;
		}
	}
	return true;
}

bool devices_manager::fitler_mac(app_packet_t & pak)
{
	bool ret = false;
	uint64_t dest_mac_num = 0;
	memcpy(&dest_mac_num, pak.src_mac, MAC_ADDRESS_LEN);
	if (filter_mac_flags) {
		bool find_it = false;
		for (int i = 0; i < select_dev_list.size(); i++) {
			if (dest_mac_num == select_dev_list[i]) {
				find_it = true;
				break;
			}
		}
		if (find_it) {
			ret = true;
		}
	}
	else {
		ret = true;
	}
	return ret;
}

void devices_manager::dispatch_pak(app_packet_t & pak)
{
	emit devices_manager::Instance().sgnLog(QString::asprintf("recv pack,src_mac: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X, dest_mac: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X, load_len:%3d, head:%04X, type:%04X msg_len:%3d",
		pak.src_mac[0], pak.src_mac[1], pak.src_mac[2], pak.src_mac[3], pak.src_mac[4], pak.src_mac[5],
		pak.dest_mac[0], pak.dest_mac[1], pak.dest_mac[2], pak.dest_mac[3], pak.dest_mac[4], pak.dest_mac[5],
		pak.load_len, pak.msg_load.msg_head, pak.msg_load.msg_type, pak.msg_load.msg_len));
	//run in pcap_loop_thread
	if (check_pak_crc(pak.msg_load) == false) return;
	switch (pak.msg_load.msg_type) {
	case IAP_CMD_GV:
	{
		append_device(pak);
	}break;
	default:
	{
		uint64_t dest_mac_num = 0;
		memcpy(&dest_mac_num, pak.src_mac, MAC_ADDRESS_LEN);
		if (dest_mac_num != 0) {
			upgrade_devices::iterator it = upgrade_dev_list.find(dest_mac_num);
			if (it != upgrade_dev_list.end()) {
				device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
				upgrade_thread->recv_pack(pak.msg_load);
			}
		}
	}break;
	}
}

void devices_manager::append_device(app_packet_t & pak)
{
	//run in pcap_loop_thread
	append_upgrade_device(pak);
	append_log_device(pak);
}

void devices_manager::append_upgrade_device(app_packet_t & pak)
{
	uint64_t dest_mac_num = 0;
	memcpy(&dest_mac_num, pak.src_mac, MAC_ADDRESS_LEN);
	if (dest_mac_num == 0) return;
	upgrade_devices::iterator it = upgrade_dev_list.find(dest_mac_num);
	if (it == upgrade_dev_list.end()) {
		device_upgrade_thread* upgrade_thread = new device_upgrade_thread(this);
		upgrade_thread->set_dest_mac(pak.src_mac);
		pak.msg_load.msg_data[pak.msg_load.msg_len] = 0;
		upgrade_thread->set_dev_info((const char*)pak.msg_load.msg_data);
		upgrade_dev_list.insert(dest_mac_num, upgrade_thread);
		emit sgnAddUpgradeDevice(QString::number(dest_mac_num));
	}
	else
	{
		uint64_t dest_mac_num = 0;
		memcpy(&dest_mac_num, pak.src_mac, MAC_ADDRESS_LEN);
		upgrade_devices::iterator it = upgrade_dev_list.find(dest_mac_num);
		if (it != upgrade_dev_list.end()) {
			device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
			pak.msg_load.msg_data[pak.msg_load.msg_len] = 0;
			upgrade_thread->recv_pack(pak.msg_load);
			upgrade_thread->set_dev_info((const char*)pak.msg_load.msg_data);
			emit sgnUpdateLogDevice(QString::number(dest_mac_num));
		}
	}
}

void devices_manager::append_log_device(app_packet_t & pak)
{
	uint64_t dest_mac_num = 0;
	memcpy(&dest_mac_num, pak.src_mac, MAC_ADDRESS_LEN);
	if (dest_mac_num == 0) return;
	log_devices::iterator it = log_dev_list.find(dest_mac_num);
	if (it == log_dev_list.end()) {
		device_log_thread* log_thread = new device_log_thread(this);
		log_thread->set_dest_mac(pak.src_mac);
		pak.msg_load.msg_data[pak.msg_load.msg_len] = 0;
		log_thread->set_dev_info((const char*)pak.msg_load.msg_data);
		log_dev_list.insert(dest_mac_num, log_thread);
		emit sgnAddLogDevice(QString::number(dest_mac_num));
	}
	else
	{
		uint64_t dest_mac_num = 0;
		memcpy(&dest_mac_num, pak.src_mac, MAC_ADDRESS_LEN);
		log_devices::iterator it = log_dev_list.find(dest_mac_num);
		if (it != log_dev_list.end()) {
			device_log_thread* log_thread = (device_log_thread*)it.value();
			pak.msg_load.msg_data[pak.msg_load.msg_len] = 0;
			log_thread->set_dev_info((const char*)pak.msg_load.msg_data);
			emit sgnUpdateLogDevice(QString::number(dest_mac_num));
		}
	}
}

void devices_manager::make_pack(app_packet_t & pak, uint16_t ntype, uint32_t nlen, uint8_t* buffer)
{
	memcpy(pak.src_mac, src_mac, MAC_ADDRESS_LEN);
	pak.msg_load.msg_head = PACKET_MSG_HEADER;
	pak.msg_load.msg_type = ntype;
	pak.msg_load.msg_len = nlen;
	if (nlen > 0 || buffer == NULL) {
		memcpy(pak.msg_load.msg_data, buffer, nlen);
	}
	uint16_t crc_calc = calc_crc((uint8_t*)&pak.msg_load.msg_type, nlen + 6);//6 = msg_type(2) + msg_len(4)
	qToBigEndian<uint16_t>(crc_calc, &pak.msg_load.msg_data[nlen]);
	pak.load_len = pak.msg_load.msg_len + 10;//10 = msg_head(2) + msg_type(2) + msg_len(4) + crc(2)
}

void devices_manager::pack_to_little_endian(app_packet_t & pak)
{
	pak.load_len = qToLittleEndian<uint16_t>(pak.load_len);
	pak.msg_load.msg_type = qToLittleEndian<uint16_t>(pak.msg_load.msg_type);
	pak.msg_load.msg_len = qToLittleEndian<uint32_t>(pak.msg_load.msg_len);
}

void devices_manager::pack_from_little_endian(app_packet_t & pak)
{
	pak.load_len = qFromLittleEndian<uint32_t>(pak.load_len);
	pak.msg_load.msg_type = qFromLittleEndian<uint32_t>(pak.msg_load.msg_type);
	pak.msg_load.msg_len = qFromLittleEndian<uint32_t>(pak.msg_load.msg_len);
}

void devices_manager::send_pack(app_packet_t & pak)
{
	//run in main_thread and device_upgrade_thread
	emit sgnLog(QString::asprintf("send pack,src_mac: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X, dest_mac: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X, load_len:%3d, head:%04X, type:%04X msg_len:%3d",
		pak.src_mac[0], pak.src_mac[1], pak.src_mac[2], pak.src_mac[3], pak.src_mac[4], pak.src_mac[5],
		pak.dest_mac[0], pak.dest_mac[1], pak.dest_mac[2], pak.dest_mac[3], pak.dest_mac[4], pak.dest_mac[5],
		pak.load_len, pak.msg_load.msg_head, pak.msg_load.msg_type, pak.msg_load.msg_len));
	pack_to_little_endian(pak);
	if (pcap_thread)pcap_thread->send_pack((u_char*)&pak, pak.load_len + PACKET_MAC_LEN);
}

void devices_manager::set_upgrade_file(QString name)
{
	file_name = name;
}

bool devices_manager::parse_upgrade_file()
{
	bool ret = false;
	if (file_name.isEmpty()) {
		emit sgnLog(QString::asprintf("Select upgrade file please."));
		return ret;
	}
	QFile upgrade_file(file_name);
	if (!upgrade_file.open(QIODevice::ReadOnly)) {
		emit sgnLog(QString::asprintf("Open upgrade file failed."));
		return ret;
	}
	int file_size = upgrade_file.size();
	emit sgnLog(QString::asprintf("Upgrade file size %d .", file_size));

	for (int i = 0; i < sub_file_list.size(); i++) {
		QByteArray sub_file_flag;
		char sub_file_size[4] = { 0 };
		sub_file_flag = upgrade_file.read(10);
		upgrade_file.read(sub_file_size, 4);
		int sub_part_size = qFromLittleEndian<int>(sub_file_size);
		if (sub_file_list[i].file_flag == sub_file_flag) {
			sub_file_list[i].file_content = upgrade_file.read(sub_part_size);
			emit sgnLog(QString(sub_file_flag) + QString::number(sub_part_size));
			ret = true;
		}
		else {
			emit sgnLog(QString("error:")+QString(sub_file_flag) + QString::number(sub_part_size));
			ret = false;
			break;
		}
	}
	upgrade_file.close();
	return ret;
}

void devices_manager::upgrade()
{
	if(!parse_upgrade_file()) return;

	for (int i = 0; i < select_dev_list.size(); i++) {
		uint64_t dest_mac_num = select_dev_list[i];
		upgrade_devices::iterator it = upgrade_dev_list.find(dest_mac_num);
		if (it != upgrade_dev_list.end()) {
			device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
			upgrade_thread->start();
		}
	}
}

bool devices_manager::is_upgrading()
{
	bool ret = false;
	upgrade_devices::iterator it;
	for (it = upgrade_dev_list.begin(); it != upgrade_dev_list.end(); ++it) {
		device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
		if (upgrade_thread->isRunning()) {
			ret = true;
			break;
		}
	}
	return ret;
}

void devices_manager::start_log_threads()
{
	if (log_flag) {
		make_log_path();
	}	
	log_devices::iterator it;
	for (it = log_dev_list.begin(); it != log_dev_list.end(); it++) {
		device_log_thread* log_thread = (device_log_thread*)it.value();
		if (log_flag) {
			if (!log_thread->isRunning()) {
				log_thread->start();
			}
		}
		else {			
			log_thread->stop();
		}
	}
}

void devices_manager::log_data(const uint8_t * data, uint32_t len)
{
	uint64_t dest_mac_num = 0;
	uint8_t dev_mac[MAC_ADDRESS_LEN] = { 0 };
	memcpy(dev_mac, data+ MAC_ADDRESS_LEN, MAC_ADDRESS_LEN);
	memcpy(&dest_mac_num, dev_mac, MAC_ADDRESS_LEN);
	log_devices::iterator it = log_dev_list.find(dest_mac_num);
	if (it != log_dev_list.end()) {
		device_log_thread* log_thread = (device_log_thread*)it.value();
		if (log_thread->isRunning()) {
			log_thread->log_data(data, len);
		}
	}
}

void devices_manager::make_log_path()
{
	QDateTime currentTime = QDateTime::currentDateTime();
	QString strData = currentTime.toString("yyyy-MM-dd_hh-mm-ss");
	//./Data/ins401_yyyy-MM-dd_hh-mm-ss/
	log_path = (QDir::currentPath() + QDir::separator() + "Data" + QDir::separator() + "ins401_" + strData + QDir::separator());
	if (!log_path.exists()) {
		log_path.mkpath(log_path.absolutePath());
	}
}

void devices_manager::broadcast_get_version_cmd()
{
	app_packet_t pak = { 0 };
	make_pack(pak, IAP_CMD_GV, 0, NULL);
	for (int i = 0; i < MAC_ADDRESS_LEN; i++)pak.dest_mac[i] = 0xff;
	send_pack(pak);
	emit sgnLog(QString::asprintf("Sending 0x%X Cmd, to Get Dst Mac. CRC = %02X%02X", pak.msg_load.msg_type, pak.msg_load.msg_data[0], pak.msg_load.msg_data[1]));
}