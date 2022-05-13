#pragma once

#include <QThread>
#include <pcap.h>
#include <string>
#include <QList>
#include "common_define.h"

class pcap_loop_thread : public QThread
{
	Q_OBJECT

public:
	pcap_loop_thread(QObject *parent);
	~pcap_loop_thread();
	void run();
	void stop();
	void set_dev_name(char* name);
	void send_pack(u_char* buffer, uint32_t nlen);
protected:
	bool open_ethernet();	
	void recv_pack(pcap_pkthdr * header, const u_char * buffer);
private:
	bool		m_isStop;
	pcap_t*		adhandle;
	std::string card_name;
signals:
	void sgnReceiveData(QByteArray buffer);
};
