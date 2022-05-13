#include "NtripClient.h"
#include "devices_manager.h"

NtripClient::NtripClient(QObject *parent)
	: QTcpSocket(parent)
	, m_isUserOpen(false)
	, m_nNetPackageNum(0)
	, m_nDateSize(0)
{
	memset(dest_mac, 0, MAC_ADDRESS_LEN);
	connect(this, SIGNAL(connected()), this, SLOT(onClientConnected()));
	connect(this, SIGNAL(readyRead()), this, SLOT(onClientReceiveData()));
	connect(this, SIGNAL(disconnected()), this, SLOT(onClientDisconnect()));
}

NtripClient::~NtripClient()
{	
	close();
	qDebug("NtripClient distroy");
}

void NtripClient::open()
{
	m_nNetPackageNum = 0;
	m_nDateSize = 0;
	doConnect();
	m_isUserOpen = true;
}

void NtripClient::close()
{
	m_isUserOpen = false;
	QTcpSocket::close();
}

bool NtripClient::isOpen()
{
	return m_isUserOpen;
}

void NtripClient::setConfig(stNtripConfig & config)
{
	m_config.m_strIp = config.m_strIp;
	m_config.m_nPort = config.m_nPort;
	m_config.m_sMountPoint = config.m_sMountPoint;
	m_config.m_sUserName = config.m_sUserName;
	m_config.m_sPassword = config.m_sPassword;
}

void NtripClient::doConnect()
{
	connectToHost(m_config.m_strIp, m_config.m_nPort);//set server ip and port
}

QString NtripClient::getUserNameAndPasswordBase64()
{
	QString nameAndPassword = m_config.m_sUserName + ":" + m_config.m_sPassword;
	return nameAndPassword.toLocal8Bit().toBase64();
}

void NtripClient::setDestMac(uint8_t mac[MAC_ADDRESS_LEN])
{
	memcpy(dest_mac, mac, MAC_ADDRESS_LEN);
}

void NtripClient::onClientConnected()
{
	m_nNetPackageNum = 0;
	qDebug("Ntrip has connected %s:%d", qPrintable(m_config.m_strIp), m_config.m_nPort);
	sendNtripHead();
	//disable ui;
	emit sgnUpdateUI(false);
}

void NtripClient::onClientDisconnect()
{
	m_nNetPackageNum = 0;
	qDebug("Ntrip disconnected %s:%d", qPrintable(m_config.m_strIp), m_config.m_nPort);
	if (m_isUserOpen) {
		qDebug("Ntrip reconnect %s:%d ", qPrintable(m_config.m_strIp), m_config.m_nPort);
		doConnect();
	}
	//enable ui;
	emit sgnUpdateUI(true);
}

void NtripClient::onClientReceiveData()
{
	QByteArray byteArray = readAll();
	m_nDateSize += byteArray.size();
	m_nNetPackageNum++;
	//qDebug("Ntrip address %s:%d receive %d byte", qPrintable(m_config.m_strIp), m_config.m_nPort, byteArray.size());
	if (m_nNetPackageNum != 1)//Skip first pagkage
	{
		int pos = 0;
		while (pos < byteArray.size()) {
			int length = byteArray.size() - pos;
			length = min(PACKET_MAX_SIZE-6, length);
			app_packet_t pak = { 0 };
			devices_manager::Instance().make_pack(pak, USER_CMD_RTCM_DATA, length, (uint8_t*)byteArray.data()+ pos);
			memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
			devices_manager::Instance().send_pack(pak);
			pos += length;
		}
		emit sgnData(byteArray);
	}
	emit sgnUpdateDataSize(m_nDateSize);
}

void NtripClient::onSendGGA(QByteArray gga)
{
	if (!isValid()) return;
	if (m_nNetPackageNum <= 1) return;
	write(gga);
}

void NtripClient::sendNtripHead()
{
	QString NtripData = QString("GET /%1 HTTP/1.1\r\nUser-Agent: NTRIP JS Client/0.2\r\nAuthorization: Basic %2\r\n\r\n").arg(m_config.m_sMountPoint, getUserNameAndPasswordBase64());
	qDebug("Ntrip send head %s", qPrintable(NtripData.trimmed()));
	write(NtripData.toUtf8());
}