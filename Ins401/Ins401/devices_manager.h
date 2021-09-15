#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QFile>
#include <QDir>
#include <pcap.h>
#include "pcap_loop_thread.h"
#include "device_upgrade_thread.h"
#include "device_log_thread.h"

struct net_mac_t {
	QString name_str;
	QString address_str;
	uint8_t mac[MAC_ADDRESS_LEN];
};

struct cmd_info_t {
	uint16_t cmd;
	char cmd_name[256];
	cmd_info_t(uint16_t c, const char* n) {
		cmd = c;
		strcpy(cmd_name, n);
	}
};

struct sub_file_t {
	QString file_flag;
	QByteArray file_content;
	bool file_switch;
	sub_file_t(QString flag,bool checked = true) {
		file_flag = flag;
		file_content.clear();
		file_switch = checked;
	}
};

typedef QMap<uint64_t, device_upgrade_thread*> upgrade_devices;
typedef QMap<uint64_t, device_log_thread*> log_devices;

class devices_manager : public QObject
{
	Q_OBJECT

private:
	devices_manager(QObject *parent);
	~devices_manager();
public:
	static devices_manager& Instance();
	static void createInstance();
private:
	static devices_manager* m_instance;
	static std::once_flag m_flag;
public:
	void init_cmd();
	int search_cards();
	void search_macs();
	void search_devs_listen(int card_index, int mac_index);
	void stop_listen();
	bool is_listening();
	void broadcast_get_version_cmd();
	void debug_pack(int index);
	bool check_pak_crc(msg_packet_t& pak);
	bool fitler_pack(app_packet_t & pak);
	bool fitler_mac(app_packet_t & pak);
	void dispatch_pak(app_packet_t & pak);
	void append_device(app_packet_t& pak);
	void append_upgrade_device(app_packet_t& pak);
	void append_log_device(app_packet_t& pak);
	void make_pack(app_packet_t & pak, uint16_t ntype, uint32_t nlen, uint8_t * buffer);
	void pack_to_little_endian(app_packet_t & pak);
	void pack_from_little_endian(app_packet_t & pak);
	void send_pack(app_packet_t & pak);
	void set_upgrade_file(QString name);
	bool parse_upgrade_file();
	void upgrade();	
	bool is_upgrading();
	void start_log_threads();
	void log_data(const uint8_t* data, uint32_t len);
	void make_log_path();
private:
	pcap_loop_thread* pcap_thread;
	uint8_t		src_mac[MAC_ADDRESS_LEN];
public:
	QList<net_card_t> net_card_list;
	QList<net_mac_t> net_mac_list;
	QList<cmd_info_t> cmd_list;	
	QList<uint64_t> select_dev_list;
	upgrade_devices upgrade_dev_list;
	log_devices		log_dev_list;
	QList <sub_file_t> sub_file_list;
	uint64_t	src_mac_num;
	QString		file_name;
	bool		filter_log_pak;
	bool		filter_mac_flags;
	bool		log_flag;
	QDir		log_path;
signals:
	void sgnLog(QString log);
	void sgnAddUpgradeDevice(QString index);
	void sgnAddLogDevice(QString index);
	void sgnUpdateUpgradeDevice(QString index);
	void sgnUpdateLogDevice(QString index);
	void sgnClearDevices();
	void sgnChangeMode(int row, int mode);
	void sgnUpdateStatus(int row, QString status);
	void sgnUpdateProcess(int row, int process);
	void sgnUpgradeStep(int row, QString upgrade_step);
	void sgnLogSize(int row, int data_size);
	void sgnThreadFinished();
};
