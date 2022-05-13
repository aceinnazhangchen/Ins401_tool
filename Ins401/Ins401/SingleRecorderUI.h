#pragma once

#include <QWidget>
#include "ui_SingleRecorderUI.h"
#include "common_define.h"
#include "NtripClient.h"
#include "InsSolWidget.h"
#include "RtkSolWidget.h"
#include "devices_manager.h"
#include "device_obj.h"
#include "OrientationGuide.h"

class SingleRecorderUI : public QWidget
{
	Q_OBJECT

public:
	SingleRecorderUI(QWidget *parent = Q_NULLPTR);
	~SingleRecorderUI();
	void loadNtripConfig();
	void readListConfig(QJsonObject & config);
	void readNtripConfig(QJsonObject & config);
	void saveNtripConfig();
	void writeListConfig(QJsonObject & config);
	void writeNtripConfig(QJsonObject & config);
	void readArmParameters(QJsonObject & config);
	void writeArmParameters(QJsonObject & config);
	void SetDevMacId(uint64_t mac_id);
	void SetConfigParameterOnUI(uint32_t sequence_id, float value);
	float GetConfigParameterOnUI(uint32_t sequence_id);
	void SetDevice(device_obj_ptr ptr);
public slots:
	void onGetConfigClicked();
	void onSetConfigClicked();
	void onSaveNtripClient();
	void onSaveLeverArms();
	void onLoadLeverArms();
	void onStartLog();
	void onUpdateNtripUI(bool enable);
	void onUpdateNtripDataSize(int data_size);
	void onReceiveLogPak(int);
	void onOrientationClicked();
	void onOrientationGuide();
private:
	Ui::SingleRecorderUI ui;
	uint64_t m_DevMacId;
	uint8_t dest_mac[MAC_ADDRESS_LEN];
	InsSolWidget* m_InsSolWidget;
	RtkSolWidget* m_RtkSolWidget;
	device_obj_ptr m_device_ptr;
	QStringList m_ParameterNames;
	OrientationGuide* m_OrientationGuide;
};
