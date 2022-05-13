#pragma once

#include <QWidget>
#include "ui_InsSolWidget.h"
#include "Ins401_Message.h"

class InsSolWidget : public QWidget
{
	Q_OBJECT

public:
	InsSolWidget(QWidget *parent = Q_NULLPTR);
	~InsSolWidget();

	void ShowInsSol(ins_sol_t& ins);

private:
	Ui::InsSolWidget ui;
	int mLastRtkType;
	int mLastInsStatus;
};
