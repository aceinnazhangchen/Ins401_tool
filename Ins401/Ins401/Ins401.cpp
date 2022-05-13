#include "Ins401.h"
#include <qdebug.h>
#include <QNetworkInterface>
#include "devices_manager.h"
#include <QProgressBar>
#include "ConfigFile.h"
#include <QMetaType>
#include "QCommonFuction.h"

enum upgrade_table_column {
	col_check,
	col_mac,
	col_info,
	col_mode,
	col_status,
	col_upgrade,
	col_process,
};

Ins401::Ins401(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	setAcceptDrops(true);
	init_cmd();
	search_devs();
	m_RecorderMap.clear();
	search_macs();	
	ui.logtype_comboBox->setCurrentIndex(devices_manager::Instance().log_type);
	m_checkHeader = new SCheckBoxHeaderView(0, Qt::Horizontal, ui.devs_tableWidget);
	ui.devs_tableWidget->setHorizontalHeader(m_checkHeader);

	qRegisterMetaType<uint64_t>("uint64_t");
	connect(ui.listen_pushButton, SIGNAL(clicked()), this, SLOT(onListenClicked()));
	connect(ui.select_pushButton, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.search_pushButton, SIGNAL(clicked()), this, SLOT(onSearchClicked()));
	connect(ui.upgrade_pushButton, SIGNAL(clicked()), this, SLOT(onUpgradeClicked()));
	connect(ui.send_pushButton, SIGNAL(clicked()), this, SLOT(onSendClicked()));
	connect(ui.clear_pushButton, SIGNAL(clicked()), this, SLOT(onClearClicked()));
	//connect(ui.save_pushButton, SIGNAL(clicked()), this, SLOT(onSaveClicked()));	

	connect(ui.debug_checkBox, SIGNAL(toggled(bool)), this, SLOT(onDebugCheck(bool)));
	connect(ui.filter_mac_checkBox, SIGNAL(toggled(bool)), this, SLOT(onFilterMacCheck(bool)));
	connect(ui.devs_tableWidget, SIGNAL(cellChanged(int, int)), this, SLOT(onDevsTableChanged(int, int)));
	connect(m_checkHeader, SIGNAL(checkStausChange(bool)), this, SLOT(onCheckAll(bool)));
	connect(ui.logtype_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onLogTypeChanged(int)));

	connect(ui.log_devs_tableWidget, SIGNAL(cellDoubleClicked(int,int)), this, SLOT(onLogTableDoubleClicked(int, int)));

	connect(&devices_manager::Instance(), SIGNAL(sgnLog(QString)), this, SLOT(onLog(QString)),Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnAddUpgradeDevice(uint64_t)), this, SLOT(onAddUpgradeDevice(uint64_t)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnUpdateUpgradeDevice(uint64_t)), this, SLOT(onUpdateUpgradeDevice(uint64_t)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnAddLogDevice(uint64_t)), this, SLOT(onAddLogDevice(uint64_t)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnUpdateLogDevice(uint64_t)), this, SLOT(onUpdateLogDevice(uint64_t)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnClearDevices()), this, SLOT(onClearDevices()), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnChangeMode(int,int)), this, SLOT(onChangeMode(int, int)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnUpdateStatus(int, QString)), this, SLOT(onUpdateStatus(int, QString)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnUpdateProcess(int, int)), this, SLOT(onUpdateProcess(int, int)), Qt::QueuedConnection);	
	connect(&devices_manager::Instance(), SIGNAL(sgnUpgradeStep(int, QString)), this, SLOT(onUpgradeStep(int, QString)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnLogSize(int, int)), this, SLOT(onShowLogSize(int, int)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnThreadFinished()), this, SLOT(onCheckUpgradeFinished()), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnShowConfigParameterOnUI(uint64_t, int,float)), this, SLOT(onShowConfigParameterOnUI(uint64_t, int, float)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnPcapStarted()), this, SLOT(onPcapStarted()), Qt::QueuedConnection);
	
	ui.devs_tableWidget->horizontalHeader()->setStretchLastSection(true);
	ui.devs_tableWidget->setColumnWidth(0, 25);
	for (int i = 1; i < col_process; i++) {
		ui.devs_tableWidget->horizontalHeader()->setSectionResizeMode(i, QHeaderView::ResizeToContents);
	}

	ui.log_devs_tableWidget->horizontalHeader()->setStretchLastSection(true);
	for (int i = 0; i < 3; i++) {
		ui.log_devs_tableWidget->horizontalHeader()->setSectionResizeMode(i, QHeaderView::ResizeToContents);
	}

	loadConfig();

	chart_test = new QtCharts_Test();
	chart_test->hide();
	connect(ui.chart_pushButton, SIGNAL(clicked()), this, SLOT(onChartClicked()));
}

Ins401::~Ins401()
{
	m_RecorderMap.clear();
}

void Ins401::init_cmd()
{
	devices_manager::Instance().init_cmd();
	ui.cmd_comboBox->clear();
	foreach(const cmd_info_t &cmd, devices_manager::Instance().cmd_list) {
		ui.cmd_comboBox->addItem(cmd.cmd_name);
	}
}

void Ins401::search_devs()
{
	ui.devs_comboBox->clear();
	devices_manager::Instance().search_cards();
	foreach(const net_card_t &card, devices_manager::Instance().net_card_list) {
		ui.devs_comboBox->addItem(card.show_name);
	}
}

void Ins401::search_macs()
{
	ui.mac_comboBox->clear();
	devices_manager::Instance().search_macs();
	foreach(const net_mac_t &mac, devices_manager::Instance().net_mac_list) {
		ui.mac_comboBox->addItem(QString("%1 [%2]").arg(mac.name_str, mac.address_str));
	}
}

void Ins401::onLog(QString log)
{
	ui.log_plainTextEdit->appendPlainText(log);
}

void Ins401::saveConfig()
{
	//QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	//writeListConfig(configJson);
	//QJsonObject base;
	//writeConfig(base);
	//configJson.insert("base", base);
	//ConfigFile::getInstance()->writeConfigFile();
	//onLog("save config success!");
}

void Ins401::loadConfig()
{
	//QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	//readListConfig(configJson);
	//if (configJson["base"].isObject())
	//{
	//	QJsonObject base = configJson["base"].toObject();
	//	readConfig(base);
	//}
}

void Ins401::readListConfig(QJsonObject & config)
{
	//QJsonArray hostArray = config["hostlist"].toArray();
	//for (int i = 0; i < hostArray.size(); i++) {
	//	if (hostArray[i].toString().isEmpty()) continue;
	//	ui.host_cmb->addItem(hostArray[i].toString());
	//}
	//QJsonArray portArray = config["portlist"].toArray();
	//for (int i = 0; i < portArray.size(); i++) {
	//	if (portArray[i].toString().isEmpty()) continue;
	//	ui.port_cmb->addItem(portArray[i].toString());
	//}
	//QJsonArray mountArray = config["mountlist"].toArray();
	//for (int i = 0; i < mountArray.size(); i++) {
	//	if (mountArray[i].toString().isEmpty()) continue;
	//	ui.mountpoint_cmb->addItem(mountArray[i].toString());
	//}
	//QJsonArray userArray = config["userlist"].toArray();
	//for (int i = 0; i < userArray.size(); i++) {
	//	if (userArray[i].toString().isEmpty()) continue;
	//	ui.user_cmb->addItem(userArray[i].toString());
	//}
	//QJsonArray passwordArray = config["passwordlist"].toArray();
	//for (int i = 0; i < passwordArray.size(); i++) {
	//	if (passwordArray[i].toString().isEmpty()) continue;
	//	ui.password_cmb->addItem(passwordArray[i].toString());
	//}
}
void Ins401::writeListConfig(QJsonObject & config)
{
	//QJsonArray hostArray = config["hostlist"].toArray();
	//if (!hostArray.contains(ui.host_cmb->currentText().trimmed())) {
	//	hostArray.append(ui.host_cmb->currentText().trimmed());
	//	config.insert("hostlist", hostArray);
	//}
	//QJsonArray portArray = config["portlist"].toArray();
	//if (!portArray.contains(ui.port_cmb->currentText().trimmed())) {
	//	portArray.append(ui.port_cmb->currentText().trimmed());
	//	config.insert("portlist", portArray);
	//}
	//QJsonArray mountArray = config["mountlist"].toArray();
	//if (!mountArray.contains(ui.mountpoint_cmb->currentText().trimmed())) {
	//	mountArray.append(ui.mountpoint_cmb->currentText().trimmed());
	//	config.insert("mountlist", mountArray);
	//}
	//QJsonArray userArray = config["userlist"].toArray();
	//if (!userArray.contains(ui.user_cmb->currentText().trimmed())) {
	//	userArray.append(ui.user_cmb->currentText().trimmed());
	//	config.insert("userlist", userArray);
	//}
	//QJsonArray passwordArray = config["passwordlist"].toArray();
	//if (!passwordArray.contains(ui.password_cmb->currentText().trimmed())) {
	//	passwordArray.append(ui.password_cmb->currentText().trimmed());
	//	config.insert("passwordlist", passwordArray);
	//}
}

void Ins401::CloseDeviceRecordWidgets()
{
	for (MapRecorders::iterator it = m_RecorderMap.begin(); it != m_RecorderMap.end(); it++) {
		delete it.value();
	}
	m_RecorderMap.clear();
}

void Ins401::dragEnterEvent(QDragEnterEvent * event)
{
	event->acceptProposedAction();
}

void Ins401::dropEvent(QDropEvent * event)
{
	if (ui.file_lineEdit->isEnabled()) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.file_lineEdit->setText(name);
	}
}
void Ins401::check_devices()
{
	int count = ui.devs_tableWidget->rowCount();
	devices_manager::Instance().select_dev_list.clear();
	for (int i = 0; i < count; i++) {
		QTableWidgetItem *check_item = ui.devs_tableWidget->item(i, col_check);
		if (check_item->checkState() == Qt::Checked) {
			uint64_t mac_id = check_item->data(Qt::UserRole + 1).value<uint64_t>();
			devices_manager::Instance().select_dev_list.append(mac_id);
		}
	}
}

void Ins401::check_file()
{
	devices_manager::Instance().sub_file_list[upgrade_rtk].file_switch = ui.rtk_checkBox->isChecked();
	devices_manager::Instance().sub_file_list[upgrade_ins].file_switch = ui.ins_checkBox->isChecked();
	devices_manager::Instance().sub_file_list[upgrade_sdk].file_switch = ui.sdk_checkBox->isChecked();
	devices_manager::Instance().sub_file_list[upgrade_imu].file_switch = ui.imu_checkBox->isChecked();
}

void Ins401::upgrade_ui_set(bool enable)
{
	ui.upgrade_pushButton->setEnabled(enable);
	ui.search_pushButton->setEnabled(enable);	
	ui.file_lineEdit->setEnabled(enable);
	ui.select_pushButton->setEnabled(enable);
	ui.listen_pushButton->setEnabled(enable);
	//ui.rtk_checkBox->setEnabled(enable);
	//ui.ins_checkBox->setEnabled(enable);
	//ui.sdk_checkBox->setEnabled(enable);
	//ui.imu_checkBox->setEnabled(enable);
}

void Ins401::readConfig(QJsonObject & config)
{
	//ui.host_cmb->setEditText(config["host"].toString().trimmed());
	//ui.port_cmb->setEditText(config["port"].toString().trimmed());
	//ui.mountpoint_cmb->setEditText(config["mountpoint"].toString().trimmed());
	//ui.user_cmb->setEditText(config["username"].toString().trimmed());
	//ui.password_cmb->setEditText(config["password"].toString().trimmed());
}

void Ins401::writeConfig(QJsonObject & config)
{
	//config.insert("host", ui.host_cmb->currentText().trimmed());
	//config.insert("port", ui.port_cmb->currentText().trimmed());
	//config.insert("mountpoint", ui.mountpoint_cmb->currentText().trimmed());
	//config.insert("username", ui.user_cmb->currentText().trimmed());
	//config.insert("password", ui.password_cmb->currentText().trimmed());
}

void Ins401::onListenClicked()
{
	if(!devices_manager::Instance().is_listening()){
		int card_index = ui.devs_comboBox->currentIndex();
		int mac_index = ui.mac_comboBox->currentIndex();
		onLog(ui.mac_comboBox->currentText());
		devices_manager::Instance().search_devs_listen(card_index, mac_index);
	}
	else {
		devices_manager::Instance().stop_listen();		
	}
	//ÉèÖÃ°´Å¥×´Ì¬
	if (devices_manager::Instance().is_listening()) {
		ui.listen_pushButton->setText("Stop");
		ui.devs_comboBox->setDisabled(true);
		ui.mac_comboBox->setDisabled(true);
	}
	else {
		ui.listen_pushButton->setText("Listen");
		ui.devs_comboBox->setEnabled(true);
		ui.mac_comboBox->setEnabled(true);
	}
}

void Ins401::onSelectFileClicked()
{
	QString current_path = ".";
	QString file_name = ui.file_lineEdit->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("Data Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.file_lineEdit->setText(path);
}

void Ins401::onSearchClicked() {
	devices_manager::Instance().broadcast_get_version_cmd();
}

void Ins401::onSendClicked()
{
	check_devices();
	int index = ui.cmd_comboBox->currentIndex();
	devices_manager::Instance().debug_pack(index);
}

void Ins401::onClearClicked()
{
	ui.log_plainTextEdit->clear();
}

void Ins401::onSaveClicked()
{
	saveConfig();
}

void Ins401::onUpgradeClicked()
{
	if (devices_manager::Instance().is_upgrading()) {
		return;
	}
	check_file();
	check_devices();
	QString file_name = ui.file_lineEdit->text();
	devices_manager::Instance().set_upgrade_file(file_name);
	devices_manager::Instance().upgrade();
	if (devices_manager::Instance().is_upgrading()) {
		upgrade_ui_set(false);
	}
}

void Ins401::onAddUpgradeDevice(uint64_t index)
{
	upgrade_devices::iterator it = devices_manager::Instance().upgrade_dev_list.find(index);
	if (it != devices_manager::Instance().upgrade_dev_list.end()) {
		device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
		int rowIdx = ui.devs_tableWidget->rowCount();
		ui.devs_tableWidget->insertRow(rowIdx);
		upgrade_thread->set_ui_table_row(rowIdx);
		QTableWidgetItem *check_item = new QTableWidgetItem();
		QTableWidgetItem *mac_item = new QTableWidgetItem(upgrade_thread->get_mac_str());
		QTableWidgetItem *info_item = new QTableWidgetItem(upgrade_thread->get_dev_info());
		check_item->setData(Qt::UserRole + 1, index);
		check_item->setCheckState(Qt::Checked);
		ui.devs_tableWidget->setItem(rowIdx, col_check, check_item);
		ui.devs_tableWidget->setItem(rowIdx, col_mac, mac_item);
		ui.devs_tableWidget->setItem(rowIdx, col_info, info_item);
		ui.devs_tableWidget->setItem(rowIdx, col_mode, new QTableWidgetItem("Unknow"));
		ui.devs_tableWidget->setItem(rowIdx, col_status, new QTableWidgetItem("Ready"));
		ui.devs_tableWidget->setItem(rowIdx, col_upgrade, new QTableWidgetItem("None"));
		QProgressBar* probar = new QProgressBar();
		probar->setStyleSheet(
			"QProgressBar {border: 2px solid grey;   border-radius: 5px;"
			"background-color: #FFFFFF;"
			"text-align: center;}"
			"QProgressBar::chunk {background-color: rgb(0,250,0) ;}"
		);
		probar->setMaximum(10000);
		probar->setValue(0);
		ui.devs_tableWidget->setCellWidget(rowIdx, col_process, probar);
	}
}

void Ins401::onUpdateUpgradeDevice(uint64_t index)
{
	upgrade_devices::iterator it = devices_manager::Instance().upgrade_dev_list.find(index);
	if (it != devices_manager::Instance().upgrade_dev_list.end()) {
		device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
		QTableWidgetItem *item = ui.devs_tableWidget->item(upgrade_thread->get_ui_table_row(), col_info);
		item->setText(upgrade_thread->get_dev_info());		
	}
}

void Ins401::onAddLogDevice(uint64_t index)
{
	devices_list::iterator it = devices_manager::Instance().m_devices_list.find(index);
	if (it != devices_manager::Instance().m_devices_list.end()) {
		device_obj_ptr device = (device_obj_ptr)it.value();
		int rowIdx = ui.log_devs_tableWidget->rowCount();
		ui.log_devs_tableWidget->insertRow(rowIdx);
		device->get_info()->set_ui_record_table_row(rowIdx);
		QTableWidgetItem *mac_item = new QTableWidgetItem(device->get_info()->get_mac_str());
		mac_item->setData(Qt::UserRole + 1, index);
		QTableWidgetItem *info_item = new QTableWidgetItem(device->get_info()->get_dev_info());
		ui.log_devs_tableWidget->setItem(rowIdx, 0, mac_item);
		ui.log_devs_tableWidget->setItem(rowIdx, 1, info_item);
		ui.log_devs_tableWidget->setItem(rowIdx, 2, new QTableWidgetItem("0"));
	}
}

void Ins401::onUpdateLogDevice(uint64_t index)
{
	devices_list::iterator it = devices_manager::Instance().m_devices_list.find(index);
	if (it != devices_manager::Instance().m_devices_list.end()) {
		device_obj_ptr device = (device_obj_ptr)it.value();
		QTableWidgetItem *item = ui.devs_tableWidget->item(device->get_info()->get_ui_record_table_row(), col_info);
		item->setText(device->get_info()->get_dev_info());
	}
}

void Ins401::onClearDevices()
{
	ui.devs_tableWidget->clearContents();
	ui.devs_tableWidget->setRowCount(0);

	ui.log_devs_tableWidget->clearContents();
	ui.log_devs_tableWidget->setRowCount(0);

	CloseDeviceRecordWidgets();
}

void Ins401::onCheckAll(bool check)
{
	int rows_count = ui.devs_tableWidget->rowCount();
	for (int i = 0; i < rows_count; i++) {
		ui.devs_tableWidget->item(i, col_check)->setCheckState(check?Qt::Checked:Qt::Unchecked);
	}
}

void Ins401::onChangeMode(int row, int mode)
{
	if (row >= ui.devs_tableWidget->rowCount()) return;
	QString status_str = "Unknow";
	switch (mode) {
	case IAP_CMD_JI:
		status_str = "IAP Boot";
		break;
	case IAP_CMD_JA:
		status_str = "IAP APP";
		break;
	case SDK_CMD_JS:
		status_str = "SDK Boot";
		break;
	case SDK_CMD_JG:
		status_str = "SDK APP";
		break;
	case IMU_CMD_JI_RET:
		status_str = "IMU Boot";
		break;
	case IMU_CMD_JA_RET:
		status_str = "IMU APP";
		break;
	default:
		break;
	}
	ui.devs_tableWidget->item(row, col_mode)->setText(status_str);
}

void Ins401::onUpdateStatus(int row, QString status)
{
	if (row >= ui.devs_tableWidget->rowCount()) return;
	ui.devs_tableWidget->item(row, col_status)->setText(status);
}

void Ins401::onUpdateProcess(int row, int percent)
{
	if (row >= ui.devs_tableWidget->rowCount()) return;
	QProgressBar* progressBar = (QProgressBar*)ui.devs_tableWidget->cellWidget(row, col_process);
	if(progressBar == NULL) return;
	progressBar->setValue(percent);
	double dProgress = progressBar->value() * 100.0 / progressBar->maximum();
	progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
}

void Ins401::onUpgradeStep(int row, QString upgrade_step)
{
	if (row >= ui.devs_tableWidget->rowCount()) return;
	ui.devs_tableWidget->item(row, col_upgrade)->setText(upgrade_step);
}

void Ins401::onShowLogSize(int row, int data_size) {
	if (row >= ui.log_devs_tableWidget->rowCount()) return;
	QString size_str = QCommonFuction::FormatBytes(data_size);
	ui.log_devs_tableWidget->item(row, 2)->setText(size_str);
}

void Ins401::onDebugCheck(bool check)
{
	ui.send_pushButton->setEnabled(check);
}

void Ins401::onFilterMacCheck(bool check)
{
	if (!devices_manager::Instance().is_upgrading()) {
		check_devices();
	}
	devices_manager::Instance().filter_mac_flags = check;
}

void Ins401::onCheckUpgradeFinished()
{
	if (!devices_manager::Instance().is_upgrading()) {
		upgrade_ui_set(true);
	}
}

void Ins401::onDevsTableChanged(int row, int col)
{
	if (col == 0) {
		QTableWidgetItem* item = ui.devs_tableWidget->item(row, col);
		if (!devices_manager::Instance().is_upgrading()) {
			check_devices();
		}
	}	
}

void Ins401::onChartClicked()
{
	chart_test->show();
}

void Ins401::onLogTableDoubleClicked(int row, int column)
{
	QTableWidgetItem* item = ui.log_devs_tableWidget->item(row, 0);
	if (!item) return;
	uint64_t MacId = item->data(Qt::UserRole + 1).toULongLong();
	MapRecorders::iterator it = m_RecorderMap.find(MacId);
	if (it == m_RecorderMap.end()) {
		SingleRecorderUI* SingleRecorder = new SingleRecorderUI();
		SingleRecorder->SetDevMacId(MacId);
		m_RecorderMap.insert(MacId, SingleRecorder);
		SingleRecorder->show();
	}
	else {
		((SingleRecorderUI*)it.value())->show();
		((SingleRecorderUI*)it.value())->activateWindow();
	}
}

void Ins401::onShowConfigParameterOnUI(uint64_t index, int sequence_id, float value)
{
	MapRecorders::iterator it = m_RecorderMap.find(index);
	if (it != m_RecorderMap.end()) {
		((SingleRecorderUI*)it.value())->SetConfigParameterOnUI(sequence_id, value);
	}
}

void Ins401::onPcapStarted()
{
	onSearchClicked();
}

void Ins401::onLogTypeChanged(int index)
{
	devices_manager::Instance().log_type = index;
}
