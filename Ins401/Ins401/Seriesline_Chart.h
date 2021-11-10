#pragma once

#include <QWidget>
#include <QChart>
#include <QChartView>
#include <QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QList>
QT_CHARTS_USE_NAMESPACE
#include "ui_Seriesline_Chart.h"

class Seriesline_Chart : public QWidget
{
	Q_OBJECT

public:
	Seriesline_Chart(QWidget *parent = Q_NULLPTR, QString name = "test", QColor color = QColor("red"));
	~Seriesline_Chart();
	void Append(double y);
private:
	Ui::Seriesline_Chart ui;
	QChart* chart;
	QLineSeries* series;
	QValueAxis *axisX;
	QValueAxis *axisY;
	QTimer *m_timer;
	QList<double> y_list;
};
