#include "NtripClient.h"
#include "devices_manager.h"

NtripClient::NtripClient(QObject *parent)
	: QTcpSocket(parent)
	, m_isUserOpen(false)
	, m_nNetPackageNum(0)
	, m_nDateSize(0)
{
	m_receiveData.clear();
	connect(this, SIGNAL(connected()), this, SLOT(onClientConnected()));
	connect(this, SIGNAL(readyRead()), this, SLOT(onClientReceiveData()));
	connect(this, SIGNAL(disconnected()), this, SLOT(onClientDisconnect()));
}

NtripClient::~NtripClient()
{
}

void NtripClient::open()
{
	m_nNetPackageNum = 0;
	m_nDateSize = 0;
	m_receiveData.clear();
	doConnect();
	m_isUserOpen = true;
}

void NtripClient::close()
{
	m_isUserOpen = false;
	QTcpSocket::close();
}

void NtripClient::doConnect()
{
	connectToHost(m_strIp, m_nPort);//set server ip and port
}

QString NtripClient::getUserNameAndPasswordBase64()
{
	QString nameAndPassword = m_sUserName + ":" + m_sPassword;
	return nameAndPassword.toLocal8Bit().toBase64();
}

void NtripClient::onClientConnected()
{
	m_nNetPackageNum = 0;
	qDebug("Ntrip has connected %s:%d", qPrintable(m_strIp), m_nPort);
	sendNtripHead();
	//disable ui;
	emit devices_manager::Instance().sgnEnableBaseStationUI(false);
}

void NtripClient::onClientDisconnect()
{
	qDebug("Ntrip disconnected %s:%d", qPrintable(m_strIp), m_nPort);
	if (m_isUserOpen) {
		qDebug("Ntrip reconnect %s:%d ", qPrintable(m_strIp), m_nPort);
		doConnect();
	}
	//enable ui;
	emit devices_manager::Instance().sgnEnableBaseStationUI(true);
}

void NtripClient::onClientReceiveData()
{
	QByteArray byteArray = readAll();
	m_nDateSize += byteArray.size();
	m_nNetPackageNum++;
	qDebug("Ntrip address %s:%d receive %d byte", qPrintable(m_strIp), m_nPort, byteArray.size());
	if (m_nNetPackageNum != 1)//Skip first pagkage
	{
		//m_receiveData.append(byteArray);
	}
	emit devices_manager::Instance().sgnBaseStationDataSize(m_nDateSize);
}

void NtripClient::sendNtripHead()
{
	QString NtripData = QString("GET /%1 HTTP/1.1\r\nUser-Agent: NTRIP JS Client/0.2\r\nAuthorization: Basic %2\r\n\r\n").arg(m_sMountPoint, getUserNameAndPasswordBase64());
	qDebug("Ntrip send head %s", qPrintable(NtripData.trimmed()));
	write(NtripData.toUtf8());
}