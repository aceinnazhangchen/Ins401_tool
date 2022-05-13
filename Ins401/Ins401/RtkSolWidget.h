#pragma once

#include <QWidget>
#include "ui_RtkSolWidget.h"
#include "Ins401_Message.h"

class RtkSolWidget : public QWidget
{
	Q_OBJECT

public:
	RtkSolWidget(QWidget *parent = Q_NULLPTR);
	~RtkSolWidget();
	void ShowRtkSol(gnss_sol_t & gnss);
private:
	Ui::RtkSolWidget ui;
	int mLastRtkType;
};
