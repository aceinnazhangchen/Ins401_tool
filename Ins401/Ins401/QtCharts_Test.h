#pragma once

#include <QWidget>
#include <QChart>
#include <QChartView>
#include <QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QTimer>
#include <QList>
QT_CHARTS_USE_NAMESPACE
#include "ui_QtCharts_Test.h"
#include "Seriesline_Chart.h"

class QtCharts_Test : public QWidget
{
	Q_OBJECT

public:
	QtCharts_Test(QWidget *parent = Q_NULLPTR);
	~QtCharts_Test();
private:
	Ui::QtCharts_Test ui;
	QTimer* m_timer;
	Seriesline_Chart* m_chart_x;
	Seriesline_Chart* m_chart_y;
	Seriesline_Chart* m_chart_z;
public slots:
	void onTimerTimeout();
};
