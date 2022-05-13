#include "OrientationGuide.h"
#include <QMessageBox>

OrientationGuide::OrientationGuide(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	ui.comboBox_Right->setCurrentIndex(emRight);
	ui.comboBox_Down->setCurrentIndex(emDown);
	connect(ui.pushButton_Cancel, SIGNAL(clicked()), this, SLOT(onCancelClicked()));
	connect(ui.pushButton_OK, SIGNAL(clicked()), this, SLOT(onOkClicked()));
	memset(rotation_rbv, 0, sizeof(int) * 3);
}

OrientationGuide::~OrientationGuide()
{
}

void OrientationGuide::onCancelClicked()
{
	close();
}

void OrientationGuide::onOkClicked()
{
	int RtkForward = ui.comboBox_Forward->currentIndex();
	int RtkRight = ui.comboBox_Right->currentIndex();
	int RtkDown = ui.comboBox_Down->currentIndex();
	//Froward
	if (RtkForward == emForward && RtkRight == emRight && RtkDown == emDown) {
		rotation_rbv[0] = 0; rotation_rbv[1] = 0; rotation_rbv[2] = 0;
	}
	else if (RtkForward == emForward && RtkRight == emLeft && RtkDown == emUp) {
		rotation_rbv[0] = 180; rotation_rbv[1] = 0; rotation_rbv[2] = 0;
	}
	else if (RtkForward == emForward && RtkRight == emDown && RtkDown == emLeft) {
		rotation_rbv[0] = -90; rotation_rbv[1] = 0; rotation_rbv[2] = 0;
	}
	else if (RtkForward == emForward && RtkRight == emUp && RtkDown == emRight) {
		rotation_rbv[0] = 90; rotation_rbv[1] = 0; rotation_rbv[2] = 0;
	}
	//Backward
	else if (RtkForward == emBackward && RtkRight == emLeft && RtkDown == emDown) {
		rotation_rbv[0] = 0; rotation_rbv[1] = 0; rotation_rbv[2] = 180;
	}
	else if (RtkForward == emBackward && RtkRight == emRight && RtkDown == emUp) {
		rotation_rbv[0] = 0; rotation_rbv[1] = 180; rotation_rbv[2] = 0;
	}
	else if (RtkForward == emBackward && RtkRight == emUp && RtkDown == emLeft) {
		rotation_rbv[0] = -90; rotation_rbv[1] = 0; rotation_rbv[2] = 180;
	}
	else if (RtkForward == emBackward && RtkRight == emDown && RtkDown == emRight) {
		rotation_rbv[0] = 90; rotation_rbv[1] = 0; rotation_rbv[2] = 180;
	}
	//Right
	else if (RtkForward == emRight && RtkRight == emBackward && RtkDown == emDown) {
		rotation_rbv[0] = 0; rotation_rbv[1] = 0; rotation_rbv[2] = -90;
	}
	else if (RtkForward == emRight && RtkRight == emForward && RtkDown == emUp) {
		rotation_rbv[0] = 180; rotation_rbv[1] = 0; rotation_rbv[2] = -90;
	}
	else if (RtkForward == emRight && RtkRight == emUp && RtkDown == emBackward) {
		rotation_rbv[0] = 90; rotation_rbv[1] = 90; rotation_rbv[2] = 0;
	}
	else if (RtkForward == emRight && RtkRight == emDown && RtkDown == emForward) {
		rotation_rbv[0] = 0; rotation_rbv[1] = -90; rotation_rbv[2] = -90;
	}
	//Left
	else if (RtkForward == emLeft && RtkRight == emForward && RtkDown == emDown) {
		rotation_rbv[0] = 0; rotation_rbv[1] = 0; rotation_rbv[2] = 90;
	}
	else if (RtkForward == emLeft && RtkRight == emBackward && RtkDown == emUp) {
		rotation_rbv[0] = 180; rotation_rbv[1] = 0; rotation_rbv[2] = -90;
	}
	else if (RtkForward == emLeft && RtkRight == emUp && RtkDown == emForward) {
		rotation_rbv[0] = 0; rotation_rbv[1] = -90; rotation_rbv[2] = 90;
	}
	else if (RtkForward == emLeft && RtkRight == emDown && RtkDown == emBackward) {
		rotation_rbv[0] = -90; rotation_rbv[1] = 90; rotation_rbv[2] = 0;
	}
	//Up
	else if (RtkForward == emUp && RtkRight == emForward && RtkDown == emLeft) {
		rotation_rbv[0] = -90; rotation_rbv[1] = 0; rotation_rbv[2] = 90;
	}
	else if (RtkForward == emUp && RtkRight == emBackward && RtkDown == emRight) {
		rotation_rbv[0] = 90; rotation_rbv[1] = 0; rotation_rbv[2] = -90;
	}
	else if (RtkForward == emUp && RtkRight == emRight && RtkDown == emForward) {
		rotation_rbv[0] = 0; rotation_rbv[1] = -90; rotation_rbv[2] = 0;
	}
	else if (RtkForward == emUp && RtkRight == emLeft && RtkDown == emBackward) {
		rotation_rbv[0] = 0; rotation_rbv[1] = 90; rotation_rbv[2] = 180;
	}
	//Down
	else if (RtkForward == emDown && RtkRight == emForward && RtkDown == emRight) {
		rotation_rbv[0] = 90; rotation_rbv[1] = 0; rotation_rbv[2] = 90;
	}
	else if (RtkForward == emDown && RtkRight == emBackward && RtkDown == emLeft) {
		rotation_rbv[0] = -90; rotation_rbv[1] = 0; rotation_rbv[2] = -90;
	}
	else if (RtkForward == emDown && RtkRight == emLeft && RtkDown == emForward) {
		rotation_rbv[0] = 0; rotation_rbv[1] = -90; rotation_rbv[2] = 180;
	}
	else if (RtkForward == emDown && RtkRight == emRight && RtkDown == emBackward) {
		rotation_rbv[0] = 0; rotation_rbv[1] = 90; rotation_rbv[2] = 0;
	}
	else {
		QMessageBox::StandardButton result = QMessageBox::information(this, "Error", "You maybe select the error axis!");
		return;
	}
	emit sgnOrientationGuide();
	close();
}