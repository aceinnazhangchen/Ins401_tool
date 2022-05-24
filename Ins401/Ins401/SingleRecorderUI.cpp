#include "SingleRecorderUI.h"
#include "ConfigFile.h"
#include <QtEndian>
#include "QCommonFuction.h"

SingleRecorderUI::SingleRecorderUI(QWidget *parent)
	: QWidget(parent)
	, m_DevMacId(0)
{
	memset(dest_mac, 0, MAC_ADDRESS_LEN);
	ui.setupUi(this);
	m_RtkSolWidget = new RtkSolWidget(this);
	m_InsSolWidget = new InsSolWidget(this);
	m_OrientationGuide = new OrientationGuide();
	ui.RtkLayout->addWidget(m_RtkSolWidget);
	ui.InsLayout->addWidget(m_InsSolWidget);	
	ui.tableWidget_arm_setting->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	ui.tableWidget_arm_setting->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 3; col++) {
			QTableWidgetItem *value_item = new QTableWidgetItem("0.0");
			ui.tableWidget_arm_setting->setItem(row, col, value_item);
		}
	}
	connect(ui.pushButton_get,SIGNAL(clicked()),this,SLOT(onGetConfigClicked()));
	connect(ui.pushButton_set,SIGNAL(clicked()),this,SLOT(onSetConfigClicked()));
	connect(ui.pushButton_save_ntrip,SIGNAL(clicked()),this,SLOT(onSaveNtripClient()));
	connect(ui.start_log_pushButton,SIGNAL(clicked()),this,SLOT(onStartLog()));
	connect(ui.pushButton_save_arms, SIGNAL(clicked()), this, SLOT(onSaveLeverArms()));
	connect(ui.pushButton_load_arms, SIGNAL(clicked()), this, SLOT(onLoadLeverArms()));
	connect(ui.pushButton_orientation, SIGNAL(clicked()), this, SLOT(onOrientationClicked()));
	connect(m_OrientationGuide, SIGNAL(sgnOrientationGuide()), this, SLOT(onOrientationGuide()));

	m_ParameterNames
		<< "gnss lever arm x"
		<< "gnss lever arm y"
		<< "gnss lever arm z"
		<< "vrp lever arm x"
		<< "vrp lever arm y"
		<< "vrp lever arm z"
		<< "user lever arm x"
		<< "user lever arm y"
		<< "user lever arm z"
		<< "rotation rbvx"
		<< "rotation rbvy"
		<< "rotation rbvz";
}

SingleRecorderUI::~SingleRecorderUI()
{
	if (m_device_ptr) {
		if (m_device_ptr->get_ntrip()) {
			disconnect(m_device_ptr->get_ntrip().get(), SIGNAL(sgnUpdateUI(bool)), this, SLOT(onUpdateNtripUI(bool)));
			disconnect(m_device_ptr->get_ntrip().get(), SIGNAL(sgnUpdateDataSize(int)), this, SLOT(onUpdateNtripDataSize(int)));
		}
		if (m_device_ptr->get_recorder()) {
			disconnect(m_device_ptr->get_recorder().get(), SIGNAL(sgnReceiveLogPak(int)), this, SLOT(onReceiveLogPak(int)));
		}
	}
	m_OrientationGuide->close();
	delete m_OrientationGuide;
}

void SingleRecorderUI::loadNtripConfig()
{
	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	readListConfig(configJson);
	QString SN = ui.lineEdit_SN->text();
	if (SN.isEmpty()) return;
	if (configJson[SN].isObject())
	{
		QJsonObject device = configJson[SN].toObject();
		if (device["base"].isObject()) {
			QJsonObject base = device["base"].toObject();
			readNtripConfig(base);
		}		
	}
}

void SingleRecorderUI::readListConfig(QJsonObject & config)
{
	ui.host_cmb->clear();
	ui.port_cmb->clear();
	ui.mountpoint_cmb->clear();
	ui.user_cmb->clear();
	ui.password_cmb->clear();
	QJsonArray hostArray = config["hostlist"].toArray();
	for (int i = 0; i < hostArray.size(); i++) {
		if (hostArray[i].toString().isEmpty()) continue;
		ui.host_cmb->addItem(hostArray[i].toString());
	}
	QJsonArray portArray = config["portlist"].toArray();
	for (int i = 0; i < portArray.size(); i++) {
		if (portArray[i].toString().isEmpty()) continue;
		ui.port_cmb->addItem(portArray[i].toString());
	}
	QJsonArray mountArray = config["mountlist"].toArray();
	for (int i = 0; i < mountArray.size(); i++) {
		if (mountArray[i].toString().isEmpty()) continue;
		ui.mountpoint_cmb->addItem(mountArray[i].toString());
	}
	QJsonArray userArray = config["userlist"].toArray();
	for (int i = 0; i < userArray.size(); i++) {
		if (userArray[i].toString().isEmpty()) continue;
		ui.user_cmb->addItem(userArray[i].toString());
	}
	QJsonArray passwordArray = config["passwordlist"].toArray();
	for (int i = 0; i < passwordArray.size(); i++) {
		if (passwordArray[i].toString().isEmpty()) continue;
		ui.password_cmb->addItem(passwordArray[i].toString());
	}
}

void SingleRecorderUI::readNtripConfig(QJsonObject & config)
{
	ui.host_cmb->setEditText(config["host"].toString().trimmed());
	ui.port_cmb->setEditText(config["port"].toString().trimmed());
	ui.mountpoint_cmb->setEditText(config["mountpoint"].toString().trimmed());
	ui.user_cmb->setEditText(config["username"].toString().trimmed());
	ui.password_cmb->setEditText(config["password"].toString().trimmed());
}

void SingleRecorderUI::saveNtripConfig()
{
	QString SN = ui.lineEdit_SN->text();
	if (SN.isEmpty()) return;

	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	writeListConfig(configJson);
	QJsonObject device;
	if (configJson[SN].isObject()) {
		device = configJson[SN].toObject();
	}
	QJsonObject base;
	writeNtripConfig(base);
	device.insert("base", base);
	configJson.insert(SN, device);
	ConfigFile::getInstance()->writeConfigFile();
	emit devices_manager::Instance().sgnLog("save config success!");
}

void SingleRecorderUI::writeListConfig(QJsonObject & config)
{
	QJsonArray hostArray = config["hostlist"].toArray();
	if (!hostArray.contains(ui.host_cmb->currentText().trimmed())) {
		hostArray.append(ui.host_cmb->currentText().trimmed());
		config.insert("hostlist", hostArray);
	}
	QJsonArray portArray = config["portlist"].toArray();
	if (!portArray.contains(ui.port_cmb->currentText().trimmed())) {
		portArray.append(ui.port_cmb->currentText().trimmed());
		config.insert("portlist", portArray);
	}
	QJsonArray mountArray = config["mountlist"].toArray();
	if (!mountArray.contains(ui.mountpoint_cmb->currentText().trimmed())) {
		mountArray.append(ui.mountpoint_cmb->currentText().trimmed());
		config.insert("mountlist", mountArray);
	}
	QJsonArray userArray = config["userlist"].toArray();
	if (!userArray.contains(ui.user_cmb->currentText().trimmed())) {
		userArray.append(ui.user_cmb->currentText().trimmed());
		config.insert("userlist", userArray);
	}
	QJsonArray passwordArray = config["passwordlist"].toArray();
	if (!passwordArray.contains(ui.password_cmb->currentText().trimmed())) {
		passwordArray.append(ui.password_cmb->currentText().trimmed());
		config.insert("passwordlist", passwordArray);
	}
}

void SingleRecorderUI::writeNtripConfig(QJsonObject & config)
{
	config.insert("host", ui.host_cmb->currentText().trimmed());
	config.insert("port", ui.port_cmb->currentText().trimmed());
	config.insert("mountpoint", ui.mountpoint_cmb->currentText().trimmed());
	config.insert("username", ui.user_cmb->currentText().trimmed());
	config.insert("password", ui.password_cmb->currentText().trimmed());
}

void SingleRecorderUI::readArmParameters(QJsonObject & config)
{
	for (int i = 1; i <= 12; i++) {
		float value = (float)config[m_ParameterNames[i - 1]].toDouble();
		SetConfigParameterOnUI(i, value);
	}
}

void SingleRecorderUI::writeArmParameters(QJsonObject & config)
{
	for (int i = 1; i <= 12; i++) {
		float value = GetConfigParameterOnUI(i);
		config.insert(m_ParameterNames[i-1], value);
	}
}

void SingleRecorderUI::SetDevMacId(uint64_t mac_id)
{
	m_DevMacId = mac_id;
	devices_list::iterator it = devices_manager::Instance().m_devices_list.find(mac_id);
	if (it != devices_manager::Instance().m_devices_list.end()) {
		device_obj_ptr device = (device_obj_ptr)it.value();
		SetDevice(device);
		ui.lineEdit_Mac->setText(device->get_info()->get_mac_str());
		ui.lineEdit_SN->setText(device->get_info()->get_SN());
		memcpy(dest_mac, device->get_info()->get_mac(), MAC_ADDRESS_LEN);
		onGetConfigClicked();
		loadNtripConfig();
	}
}

void SingleRecorderUI::SetConfigParameterOnUI(uint32_t sequence_id, float value)
{
	int row = 0, col = 0;
	switch (sequence_id) {
	case 1:row = 0; col = 0; break;
	case 2:row = 0; col = 1; break;
	case 3:row = 0; col = 2; break;
	case 4:row = 1; col = 0; break;
	case 5:row = 1; col = 1; break;
	case 6:row = 1; col = 2; break;
	case 7:row = 2; col = 0; break;
	case 8:row = 2; col = 1; break;
	case 9:row = 2; col = 2; break;
	case 10:row = 3; col = 0; break;
	case 11:row = 3; col = 1; break;
	case 12:row = 3; col = 2; break;
	}
	QTableWidgetItem* item = ui.tableWidget_arm_setting->item(row, col);
	if (item) {
		item->setText(QString::number(value));
	}
}

float SingleRecorderUI::GetConfigParameterOnUI(uint32_t sequence_id)
{
	float value = 0.0;
	int row = 0, col = 0;
	switch (sequence_id) {
	case 1:row = 0; col = 0; break;
	case 2:row = 0; col = 1; break;
	case 3:row = 0; col = 2; break;
	case 4:row = 1; col = 0; break;
	case 5:row = 1; col = 1; break;
	case 6:row = 1; col = 2; break;
	case 7:row = 2; col = 0; break;
	case 8:row = 2; col = 1; break;
	case 9:row = 2; col = 2; break;
	case 10:row = 3; col = 0; break;
	case 11:row = 3; col = 1; break;
	case 12:row = 3; col = 2; break;
	}
	QTableWidgetItem* item = ui.tableWidget_arm_setting->item(row, col);
	if (item) {
		QString value_str = item->text();
		value = value_str.toFloat();
	}
	return value;
}

void SingleRecorderUI::SetDevice(device_obj_ptr ptr)
{
	m_device_ptr = ptr;
	connect(m_device_ptr->get_ntrip().get(), SIGNAL(sgnUpdateUI(bool)), this, SLOT(onUpdateNtripUI(bool)), Qt::QueuedConnection);
	connect(m_device_ptr->get_ntrip().get(), SIGNAL(sgnUpdateDataSize(int)), this, SLOT(onUpdateNtripDataSize(int)), Qt::QueuedConnection);
	connect(m_device_ptr->get_recorder().get(), SIGNAL(sgnReceiveLogPak(int)), this, SLOT(onReceiveLogPak(int)), Qt::QueuedConnection);
}

void SingleRecorderUI::onGetConfigClicked()
{
	app_packet_t pak = { 0 };
	uint8_t buffer[8] = { 0 };
	config_parameter_t parameter = { 0 };
	parameter.value = 0;
	for (int i = 1; i <= 12; i++) {
		parameter.sequence_id = i;
		memcpy(buffer, &parameter, sizeof(config_parameter_t));
		devices_manager::Instance().make_pack(pak, USER_CMD_GET_CONFIG, 8, buffer);
		memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
		devices_manager::Instance().send_pack(pak);
	}
}

void SingleRecorderUI::onSetConfigClicked()
{
	app_packet_t pak = { 0 };
	uint8_t buffer[8] = { 0 };
	config_parameter_t parameter = { 0 };
	for (int i = 1; i <= 12; i++) {
		parameter.sequence_id = i;
		parameter.value = GetConfigParameterOnUI(i);
		memcpy(buffer, &parameter, sizeof(config_parameter_t));
		devices_manager::Instance().make_pack(pak, USER_CMD_SET_CONFIG, 8, buffer);
		memcpy(pak.dest_mac, dest_mac, MAC_ADDRESS_LEN);
		devices_manager::Instance().send_pack(pak);
	}
}

void SingleRecorderUI::onSaveNtripClient()
{
	saveNtripConfig();
}

void SingleRecorderUI::onSaveLeverArms()
{
	QString SN = ui.lineEdit_SN->text();
	if (SN.isEmpty()) return;

	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	QJsonObject device;
	if (configJson[SN].isObject()) {
		device = configJson[SN].toObject();
	}
	QJsonObject parameters;
	writeArmParameters(parameters);
	device.insert("arms", parameters);
	configJson.insert(SN, device);
	ConfigFile::getInstance()->writeConfigFile();
	emit devices_manager::Instance().sgnLog("save config success!");
}

void SingleRecorderUI::onLoadLeverArms()
{
	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	readListConfig(configJson);
	QString SN = ui.lineEdit_SN->text();
	if (SN.isEmpty()) return;
	if (configJson[SN].isObject())
	{
		QJsonObject device = configJson[SN].toObject();
		if (device["arms"].isObject()) {
			QJsonObject base = device["arms"].toObject();
			readArmParameters(base);
		}
	}
}

void SingleRecorderUI::onStartLog() {
	if (m_device_ptr == NULL) return;
	stNtripConfig config;
	config.m_strIp = ui.host_cmb->currentText();
	config.m_nPort = ui.port_cmb->currentText().toInt();
	config.m_sMountPoint = ui.mountpoint_cmb->currentText();
	config.m_sUserName = ui.user_cmb->currentText();
	config.m_sPassword = ui.password_cmb->currentText();
	m_device_ptr->setNtripConfig(config);

	if (m_device_ptr->is_recording()) {
		m_device_ptr->stop_record();
	}
	else {
		saveNtripConfig();
		m_device_ptr->makeRecordPath();
		QJsonObject arms;
		writeArmParameters(arms);
		m_device_ptr->recordConfig(arms);
		m_device_ptr->start_record();
	}
	if (m_device_ptr->is_recording()) {
		ui.start_log_pushButton->setText("Stop Log");
	}
	else {
		ui.start_log_pushButton->setText("Start Log");
	}
}

void SingleRecorderUI::onUpdateNtripUI(bool enable)
{
	ui.ntrip_edit_widget->setEnabled(enable);
}

void SingleRecorderUI::onUpdateNtripDataSize(int data_size)
{
	QString size_str = QCommonFuction::FormatBytes(data_size);
	ui.data_label->setText(size_str);
}

void SingleRecorderUI::onReceiveLogPak(int type)
{
	if (m_device_ptr == NULL) return;
	if (m_device_ptr->get_recorder() == NULL) return;
	switch (type)
	{
	case em_INS_SOL:{
		m_InsSolWidget->ShowInsSol(m_device_ptr->get_recorder()->m_ins_pak);
	}break;
	case em_GNSS_SOL: {
		m_RtkSolWidget->ShowRtkSol(m_device_ptr->get_recorder()->m_gnss_pak);
	}break;
	default:
		break;
	}
}

void SingleRecorderUI::onOrientationClicked()
{
	m_OrientationGuide->show();
}

void SingleRecorderUI::onOrientationGuide()
{
	SetConfigParameterOnUI(10, (float)m_OrientationGuide->rotation_rbv[0]);
	SetConfigParameterOnUI(11, (float)m_OrientationGuide->rotation_rbv[1]);
	SetConfigParameterOnUI(12, (float)m_OrientationGuide->rotation_rbv[2]);
}

