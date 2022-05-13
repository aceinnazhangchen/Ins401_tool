#pragma once

#include <QObject>
#include <QTcpSocket>
#include "common_define.h"

struct stNtripConfig {
	QString m_strIp;
	int m_nPort;
	QString m_sMountPoint;
	QString m_sUserName;
	QString m_sPassword;
};

class NtripClient : public QTcpSocket
{
	Q_OBJECT

public:
	NtripClient(QObject *parent);
	~NtripClient();
	void open();
	void close();
	bool isOpen();
	void setConfig(stNtripConfig& config);
	void setDestMac(uint8_t mac[MAC_ADDRESS_LEN]);
protected:
	void doConnect();
	QString getUserNameAndPasswordBase64();
	void sendNtripHead();
private:
	uint8_t dest_mac[MAC_ADDRESS_LEN];
	stNtripConfig m_config;
	bool m_isUserOpen;
	int m_nNetPackageNum;//how many packeage received
	int m_nDateSize;
public slots:
	void onClientConnected();
	void onClientDisconnect();
	void onClientReceiveData();
	void onSendGGA(QByteArray gga);
signals:
	void sgnUpdateUI(bool);
	void sgnUpdateDataSize(int);
	void sgnData(QByteArray);
};

typedef std::shared_ptr<NtripClient> ntrip_client_ptr;