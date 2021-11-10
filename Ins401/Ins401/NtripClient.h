#pragma once

#include <QObject>
#include <QTcpSocket>

class NtripClient : public QTcpSocket
{
	Q_OBJECT

public:
	NtripClient(QObject *parent);
	~NtripClient();
	void open();
	void close();
protected:
	void doConnect();
	QString getUserNameAndPasswordBase64();
	void sendNtripHead();
public:
	QString m_strIp;
	int m_nPort;
	QString m_sMountPoint;
	QString m_sUserName;
	QString m_sPassword;
private:
	bool m_isUserOpen;
	int m_nNetPackageNum;//how many packeage received
	int m_nDateSize;
	QByteArray m_receiveData;
public slots:
	void onClientConnected();
	void onClientDisconnect();
	void onClientReceiveData();

};
