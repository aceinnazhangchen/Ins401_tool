#pragma once

#include <QThread>
#include <QWaitCondition>
#include <QMutex>
#include "common_define.h"

class device_upgrade_thread : public QThread
{
	Q_OBJECT

public:
	device_upgrade_thread(QObject *parent);
	~device_upgrade_thread();
	void run();
	void stop();
	void set_dest_mac(uint8_t mac[MAC_ADDRESS_LEN]);
	void set_dev_info(const char* info);
	void set_ui_table_row(const int32_t row);
	QString get_mac_str();
	QString get_dev_info();
	int32_t get_ui_table_row();

	void filled_SDK_bin_info();

	void recv_pack(msg_packet_t& msg_pak);
	void send_get_version_cmd();
	void send_IAP_boot_cmd();
	void send_IAP_app_cmd();
	void send_IAP_set_core_cmd();
	void send_SDK_boot_cmd();
	void send_SDK_app_cmd();
	void send_SDK_sync();
	void send_SDK_change_baudrate();
	void send_SDK_set_baudrate();
	void send_SDK_check_baudrate();
	void send_SDK_check_ready();
	void send_SDK_boot_pre_info();
	void send_SDK_boot_info();
	void send_SDK_erase_cmd();
	void send_SDK_bin_info();
	void send_IMU_boot_cmd();
	void send_IMU_app_cmd();
	void send_IAP_file();
	void send_SDK_file(int cmd);
	void send_IMU_file();

	void waiting_restart();
	void step_IAP_jump_JI();	
	void step_IAP_write_CS();
	void step_IAP_write_file();
	void step_IAP_jump_JA();
	void step_SDK_jump_JS();
	void step_SDK_jump_JG();
	void step_SDK_sync();
	void step_SDK_set_baudrate();
	void step_SDK_send_loader();
	void step_SDK_start_write();
	void step_SDK_write_file();
	void step_IMU_jump_JI();
	void step_IMU_jump_JA();
	void step_IMU_write_file();

	bool upgrade_rtk_process();
	bool upgrade_ins_process();
	bool upgrade_app_process();
	bool upgrade_sdk_process();
	bool upgrade_imu_process();
private:
	bool m_isStop;
	bool m_isRestarting;
	int  m_send_sdk_count;
	int  m_recv_sdk_count;
	QMutex mutex;
	QWaitCondition cond_wait;
	uint16_t dev_cmd_status; //use cmd 
	uint64_t dest_mac_num;
	uint8_t dest_mac[MAC_ADDRESS_LEN];
	QString dev_info;
	int file_offset_addr;
	int ret_file_offset_addr;
	int32_t ui_table_row;
	emUpgradeStep upgrade_step;
	stSTA9100BinInfo STA9100BinInfo;
};
