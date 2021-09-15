#include "Ins401.h"
#include <qdebug.h>
#include <QNetworkInterface>
#include "devices_manager.h"
#include <QProgressBar>

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
	search_macs();	
	ui.filter_checkBox->setChecked(devices_manager::Instance().filter_log_pak);
	m_checkHeader = new SCheckBoxHeaderView(0, Qt::Horizontal, ui.devs_tableWidget);
	ui.devs_tableWidget->setHorizontalHeader(m_checkHeader);
	connect(ui.listen_pushButton, SIGNAL(clicked()), this, SLOT(onListenClicked()));
	connect(ui.select_pushButton, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.search_pushButton, SIGNAL(clicked()), this, SLOT(onSearchClicked()));
	connect(ui.upgrade_pushButton, SIGNAL(clicked()), this, SLOT(onUpgradeClicked()));
	connect(ui.send_pushButton, SIGNAL(clicked()), this, SLOT(onSendClicked()));
	connect(ui.clear_pushButton, SIGNAL(clicked()), this, SLOT(onClearClicked()));
	connect(ui.debug_checkBox, SIGNAL(toggled(bool)), this, SLOT(onDebugCheck(bool)));
	connect(ui.filter_checkBox, SIGNAL(toggled(bool)), this, SLOT(onFilterCheck(bool)));
	connect(ui.filter_mac_checkBox, SIGNAL(toggled(bool)), this, SLOT(onFilterMacCheck(bool)));
	connect(ui.devs_tableWidget, SIGNAL(cellChanged(int, int)), this, SLOT(onDevsTableChanged(int, int)));
	connect(ui.log_checkBox, SIGNAL(toggled(bool)), this, SLOT(onLogCheck(bool)));
	connect(m_checkHeader, SIGNAL(checkStausChange(bool)), this, SLOT(onCheckAll(bool)));

	connect(&devices_manager::Instance(), SIGNAL(sgnLog(QString)), this, SLOT(onLog(QString)),Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnAddUpgradeDevice(QString)), this, SLOT(onAddUpgradeDevice(QString)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnUpdateUpgradeDevice(QString)), this, SLOT(onUpdateUpgradeDevice(QString)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnAddLogDevice(QString)), this, SLOT(onAddLogDevice(QString)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnUpdateLogDevice(QString)), this, SLOT(onUpdateLogDevice(QString)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnClearDevices()), this, SLOT(onClearDevices()), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnChangeMode(int,int)), this, SLOT(onChangeMode(int, int)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnUpdateStatus(int, QString)), this, SLOT(onUpdateStatus(int, QString)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnUpdateProcess(int, int)), this, SLOT(onUpdateProcess(int, int)), Qt::QueuedConnection);	
	connect(&devices_manager::Instance(), SIGNAL(sgnUpgradeStep(int, QString)), this, SLOT(onUpgradeStep(int, QString)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnLogSize(int, int)), this, SLOT(onShowLogSize(int, int)), Qt::QueuedConnection);
	connect(&devices_manager::Instance(), SIGNAL(sgnThreadFinished()), this, SLOT(onCheckUpgradeFinished()), Qt::QueuedConnection);

	ui.devs_tableWidget->horizontalHeader()->setStretchLastSection(true);
	ui.devs_tableWidget->setColumnWidth(0, 25);
	for (int i = 1; i < col_process; i++) {
		ui.devs_tableWidget->horizontalHeader()->setSectionResizeMode(i, QHeaderView::ResizeToContents);
	}

	ui.log_devs_tableWidget->horizontalHeader()->setStretchLastSection(true);
	for (int i = 0; i < 3; i++) {
		ui.log_devs_tableWidget->horizontalHeader()->setSectionResizeMode(i, QHeaderView::ResizeToContents);
	}
}

Ins401::~Ins401()
{
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

void Ins401::onAddUpgradeDevice(QString index_str)
{
	uint64_t index = index_str.toULongLong();
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

void Ins401::onUpdateUpgradeDevice(QString index_str)
{
	uint64_t index = index_str.toULongLong();
	upgrade_devices::iterator it = devices_manager::Instance().upgrade_dev_list.find(index);
	if (it != devices_manager::Instance().upgrade_dev_list.end()) {
		device_upgrade_thread* upgrade_thread = (device_upgrade_thread*)it.value();
		QTableWidgetItem *item = ui.devs_tableWidget->item(upgrade_thread->get_ui_table_row(), col_info);
		item->setText(upgrade_thread->get_dev_info());		
	}
}

void Ins401::onAddLogDevice(QString index_str)
{
	uint64_t index = index_str.toULongLong();
	log_devices::iterator it = devices_manager::Instance().log_dev_list.find(index);
	if (it != devices_manager::Instance().log_dev_list.end()) {
		device_log_thread* log_thread = (device_log_thread*)it.value();
		int rowIdx = ui.log_devs_tableWidget->rowCount();
		ui.log_devs_tableWidget->insertRow(rowIdx);
		log_thread->set_ui_table_row(rowIdx);
		QTableWidgetItem *mac_item = new QTableWidgetItem(log_thread->get_mac_str());
		QTableWidgetItem *info_item = new QTableWidgetItem(log_thread->get_dev_info());
		ui.log_devs_tableWidget->setItem(rowIdx, 0, mac_item);
		ui.log_devs_tableWidget->setItem(rowIdx, 1, info_item);
		ui.log_devs_tableWidget->setItem(rowIdx, 2, new QTableWidgetItem("0"));
	}
}

void Ins401::onUpdateLogDevice(QString index_str)
{
	uint64_t index = index_str.toULongLong();
	log_devices::iterator it = devices_manager::Instance().log_dev_list.find(index);
	if (it != devices_manager::Instance().log_dev_list.end()) {
		device_log_thread* log_thread = (device_log_thread*)it.value();
		QTableWidgetItem *item = ui.devs_tableWidget->item(log_thread->get_ui_table_row(), col_info);
		item->setText(log_thread->get_dev_info());
	}
}

void Ins401::onClearDevices()
{
	ui.devs_tableWidget->clearContents();
	ui.devs_tableWidget->setRowCount(0);

	ui.log_devs_tableWidget->clearContents();
	ui.log_devs_tableWidget->setRowCount(0);
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

void Ins401::onShowLogSize(int row, int percent) {
	if (row >= ui.log_devs_tableWidget->rowCount()) return;
	ui.log_devs_tableWidget->item(row, 2)->setText(QString::number(percent));

}

void Ins401::onDebugCheck(bool check)
{
	ui.send_pushButton->setEnabled(check);
}

void Ins401::onFilterCheck(bool check)
{
	devices_manager::Instance().filter_log_pak = check;
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

void Ins401::onLogCheck(bool check) {
	devices_manager::Instance().log_flag = check;
	devices_manager::Instance().start_log_threads();
}

