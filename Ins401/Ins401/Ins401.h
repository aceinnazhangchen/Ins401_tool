#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Ins401.h"
#include <pcap.h>
#include "pcap_loop_thread.h"
#include <QFileDialog>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include "SCheckBoxHeaderView.h"

class Ins401 : public QMainWindow
{
    Q_OBJECT

public:
    Ins401(QWidget *parent = Q_NULLPTR);
	~Ins401();
	void init_cmd();
	void search_devs();
	void search_macs();
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
	void check_devices();
	void check_file();
	void upgrade_ui_set(bool enable);
public slots:
	void onLog(QString log);
	void onListenClicked();
	void onSelectFileClicked();
	void onSearchClicked();
	void onSendClicked();
	void onClearClicked();
	void onUpgradeClicked();
	void onAddDevice(QString index_str);
	void onUpdateDevice(QString index_str);
	void onClearDevices();
	void onAddRowClicked();
	void onCheckAll(bool check);
	void onChangeMode(int row, int mode);
	void onUpdateStatus(int row, QString status);
	void onUpdateProcess(int row, int percent);
	void onUpgradeStep(int row, QString upgrade_step);
	void onDebugCheck(bool check);
	void onFilterCheck(bool check);
	void onFilterMacCheck(bool check);
	void onCheckUpgradeFinished();
	void onDevsTableChanged(int row, int col);
private:
    Ui::Ins401Class ui;
	SCheckBoxHeaderView* m_checkHeader;
};
