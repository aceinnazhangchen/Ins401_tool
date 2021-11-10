#include "QtCharts_Test.h"
#include <QDateTime>
#include <QDebug>

#define AXIS_X_RANGE 100

QtCharts_Test::QtCharts_Test(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	m_chart_x = new Seriesline_Chart(this, "axisX", QColor("red"));
	m_chart_y = new Seriesline_Chart(this, "axisY", QColor("green"));
	m_chart_z = new Seriesline_Chart(this, "axisZ", QColor("blue"));
	ui.verticalLayout->addWidget(m_chart_x);
	ui.verticalLayout->addWidget(m_chart_y);
	ui.verticalLayout->addWidget(m_chart_z);

	qsrand(QDateTime::currentDateTime().toTime_t());
	m_timer = new QTimer(this);
	m_timer->setSingleShot(false);
	m_timer->stop();
	connect(m_timer, SIGNAL(timeout()), this, SLOT(onTimerTimeout()));
	m_timer->setInterval(100);
	m_timer->start();

}

QtCharts_Test::~QtCharts_Test()
{
	m_timer->stop();
	delete m_timer;
}

void QtCharts_Test::onTimerTimeout() {

	double x = double(qrand() % 100) / 10.0 - 5;
	ui.x_lineEdit->setText(QString::number(x));
	m_chart_x->Append(x);
	double y = double(qrand() % 100) / 10.0 - 5;
	ui.y_lineEdit->setText(QString::number(y));
	m_chart_y->Append(y);
	double z = double(qrand() % 100) / 10.0 - 5;
	ui.z_lineEdit->setText(QString::number(z));
	m_chart_z->Append(z);
}