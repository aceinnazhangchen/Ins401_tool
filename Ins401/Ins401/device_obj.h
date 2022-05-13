#pragma once

#include <QObject>
#include "device_info.h"
#include "device_record_thread.h"
#include "NtripClient.h"
#include "ntrip_record_thread.h"

class device_obj : public QObject
{
	Q_OBJECT

public:
	device_obj(QObject *parent);
	~device_obj();
	void start_record();
	void stop_record();
	bool is_recording();
	void set_dest_mac(uint8_t mac[MAC_ADDRESS_LEN]);
	void set_dev_info(const char* info);
	void makeRecordPath();
	void setNtripConfig(stNtripConfig& config);
	void recordConfig(QJsonObject & config);
	device_info_ptr get_info();
	device_record_thread_ptr get_recorder();
	ntrip_client_ptr get_ntrip();
private:
	device_info_ptr m_device_info_ptr;
	device_record_thread_ptr m_record_thread_ptr;
	ntrip_client_ptr m_base_ntrip_ptr;
	ntrip_record_thread_ptr m_ntrip_record_thread_ptr;
};

typedef std::shared_ptr<device_obj> device_obj_ptr;