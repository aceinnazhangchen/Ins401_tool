#pragma once

#include <QWidget>
#include "ui_OrientationGuide.h"

enum emOrientation {
	emForward,
	emBackward,
	emRight,
	emLeft,
	emDown,
	emUp,
};

class OrientationGuide : public QWidget
{
	Q_OBJECT

public:
	OrientationGuide(QWidget *parent = Q_NULLPTR);
	~OrientationGuide();
public slots:
	void onCancelClicked();
	void onOkClicked();
private:
	Ui::OrientationGuide ui;
public:
	int rotation_rbv[3];
signals:
	void sgnOrientationGuide();
};
