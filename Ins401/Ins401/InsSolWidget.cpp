#include "InsSolWidget.h"

InsSolWidget::InsSolWidget(QWidget *parent)
	: QWidget(parent)
	, mLastRtkType(0)
	, mLastInsStatus(0)
{
	ui.setupUi(this);
}

InsSolWidget::~InsSolWidget()
{
}

void InsSolWidget::ShowInsSol(ins_sol_t& ins) {
	ui.lineEdit_time->setText(QString::number((double)ins.gps_millisecs / 1000.0f, 'f', 2));
	ui.lineEdit_ins_status->setText(QString::number(ins.ins_status));
	ui.lineEdit_rtk_type->setText(QString::number(ins.ins_position_type));
	ui.lineEdit_latiude->setText(QString::number(ins.latitude, 'f'));
	ui.lineEdit_longitude->setText(QString::number(ins.longitude, 'f'));
	ui.lineEdit_height->setText(QString::number(ins.height, 'f', 2));
	ui.lineEdit_north->setText(QString::number(ins.north_velocity, 'f', 2));
	ui.lineEdit_east->setText(QString::number(ins.east_velocity, 'f', 2));
	ui.lineEdit_up->setText(QString::number(ins.up_velocity, 'f', 2));
	ui.lineEdit_roll->setText(QString::number(ins.roll, 'f', 2));
	ui.lineEdit_pitch->setText(QString::number(ins.pitch, 'f', 2));
	ui.lineEdit_yaw->setText(QString::number(ins.heading, 'f', 2));

	if (mLastRtkType == ins.ins_position_type && mLastInsStatus == ins.ins_status) {
		return;
	}
	if (ins.ins_status == 1) {
		ui.lineEdit_ins_status->setStyleSheet("color:white;background:red");
	}
	else if (ins.ins_status == 2) {
		ui.lineEdit_ins_status->setStyleSheet("color:white;background:purple");
	}
	else if (ins.ins_status == 3) {
		ui.lineEdit_ins_status->setStyleSheet("color:white;background:blue");
	}
	else if (ins.ins_status == 5) {
		ui.lineEdit_ins_status->setStyleSheet("color:white;background:orange");
	}
	else if (ins.ins_status == 4) {
		ui.lineEdit_ins_status->setStyleSheet("color:white;background:green");
	}
	else {
		ui.lineEdit_ins_status->setStyleSheet("color:black;background:white");
	}
	mLastInsStatus = ins.ins_status;

	if (ins.ins_position_type == 1) {
		ui.lineEdit_rtk_type->setStyleSheet("color:white;background:red");
	}
	else if (ins.ins_position_type == 2) {
		ui.lineEdit_rtk_type->setStyleSheet("color:white;background:purple");
	}
	else if (ins.ins_position_type == 5) {
		ui.lineEdit_rtk_type->setStyleSheet("color:white;background:orange");
	}
	else if (ins.ins_position_type == 4) {
		ui.lineEdit_rtk_type->setStyleSheet("color:white;background:green");
	}
	else {
		ui.lineEdit_rtk_type->setStyleSheet("color:black;background:white");
	}
	mLastRtkType = ins.ins_position_type;
}