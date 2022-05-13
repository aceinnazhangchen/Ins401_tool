#include "device_info.h"
#include <QDateTime>

device_info::device_info(QObject *parent)
	: QObject(parent)
	, dest_mac_num(0)
	, ui_record_table_row(0)
{
	memset(dest_mac, 0, MAC_ADDRESS_LEN);
	dev_info.clear();
}

device_info::~device_info()
{
}

void device_info::set_dest_mac(uint8_t mac[MAC_ADDRESS_LEN])
{
	memcpy(dest_mac, mac, MAC_ADDRESS_LEN);
	memcpy(&dest_mac_num, mac, MAC_ADDRESS_LEN);
}

void device_info::set_dev_info(const char * info)
{
	dev_info = info;
	reg_exp_device_info();
}

void device_info::set_ui_record_table_row(const int32_t row)
{
	ui_record_table_row = row;
}

QString device_info::get_mac_str()
{
	return QString::asprintf("%.2X:%.2X:%.2X:%.2X:%.2X:%.2X", dest_mac[0], dest_mac[1], dest_mac[2], dest_mac[3], dest_mac[4], dest_mac[5]);
}

uint8_t* device_info::get_mac()
{
	return dest_mac;
}

QString device_info::get_dev_info()
{
	return dev_info;
}

QString device_info::get_SN()
{
	return sn;
}

QString device_info::get_PN()
{
	return pn;
}

QString device_info::get_firmware_version()
{
	return firmware_version;
}

QString device_info::get_app_version()
{
	return app_version;
}

int32_t device_info::get_ui_record_table_row()
{
	return ui_record_table_row;
}

void device_info::reg_exp_device_info()
{
	int index = dev_info.indexOf("RTK_INS");
	app_version = dev_info.mid(index);
	QStringList list;
	list = dev_info.split(" ");
	if (list.size() >= 4) {
		pn = list[1];
		sn = list[2];
		firmware_version = list[7];
	}
}

void device_info::make_record_path()
{
	QDateTime currentTime = QDateTime::currentDateTime();
	QString strData = currentTime.toString("yyyyMMdd_hhmmss");
	//./Data/ins401_log_yyyyMMdd_hhmmss/
	record_path = (QDir::currentPath() + QDir::separator() + "Data" + QDir::separator() + "ins401_log_" + sn + "_" + strData + QDir::separator());
	if (!record_path.exists()) {
		record_path.mkpath(record_path.absolutePath());
	}
}

QDir device_info::get_record_path()
{
	return record_path;
}
