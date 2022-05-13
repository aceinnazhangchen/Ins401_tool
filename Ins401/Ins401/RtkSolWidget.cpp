#include "RtkSolWidget.h"

RtkSolWidget::RtkSolWidget(QWidget *parent)
	: QWidget(parent)
	, mLastRtkType(0)
{
	ui.setupUi(this);
}

RtkSolWidget::~RtkSolWidget()
{
}

void RtkSolWidget::ShowRtkSol(gnss_sol_t& gnss){
	ui.lineEdit_time->setText(QString::number((double)gnss.gps_millisecs / 1000.0f, 'f', 2));
	ui.lineEdit_rtk_type->setText(QString::number(gnss.position_type));
	ui.lineEdit_satellites->setText(QString::asprintf("%d/%d",gnss.numberOfSVs_in_solution,gnss.numberOfSVs));
	ui.lineEdit_latiude->setText(QString::number(gnss.latitude, 'f'));
	ui.lineEdit_longitude->setText(QString::number(gnss.longitude, 'f'));
	ui.lineEdit_height->setText(QString::number(gnss.height, 'f', 2));
	ui.lineEdit_north->setText(QString::number(gnss.north_vel, 'f', 2));
	ui.lineEdit_east->setText(QString::number(gnss.east_vel, 'f', 2));
	ui.lineEdit_up->setText(QString::number(gnss.up_vel, 'f', 2));

	if (mLastRtkType == gnss.position_type) {
		return;
	}
	if (gnss.position_type == 1) {
		ui.lineEdit_rtk_type->setStyleSheet("color:white;background:red");
	}
	else if (gnss.position_type == 2) {
		ui.lineEdit_rtk_type->setStyleSheet("color:white;background:purple");
	}
	else if (gnss.position_type == 5) {
		ui.lineEdit_rtk_type->setStyleSheet("color:white;background:orange");
	}
	else if (gnss.position_type == 4) {
		ui.lineEdit_rtk_type->setStyleSheet("color:white;background:green");
	}
	else {
		ui.lineEdit_rtk_type->setStyleSheet("color:black;background:white");
	}
	mLastRtkType = gnss.position_type;
}