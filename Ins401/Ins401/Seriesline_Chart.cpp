#include "Seriesline_Chart.h"

#define CHART_AXIS_X_RANGE 100

Seriesline_Chart::Seriesline_Chart(QWidget *parent,QString name, QColor color)
	: QWidget(parent)
{
	ui.setupUi(this);
	chart = new QChart();
	chart->setMargins(QMargins(0, 0, 0, 0));
	chart->legend()->setAlignment(Qt::AlignRight);
	//chart->setAnimationOptions(QChart::SeriesAnimations);//动画效果
	//chart->setTheme(QChart::ChartThemeDark);//主题
	axisX = new QValueAxis(this);
	axisY = new QValueAxis(this);
	axisX->setMinorTickCount(5);
	axisX->setLabelFormat("%d");
	axisX->setRange(0, CHART_AXIS_X_RANGE);
	axisY->setRange(-5.0, 5.0);

	series = new QLineSeries(this);
	series->setPointsVisible(true);
	series->setName(name);
	series->setColor(color);
	series->setUseOpenGL(true);

	chart->addSeries(series);
	chart->setAxisX(axisX, series);
	chart->setAxisY(axisY, series);
	for (int i = 0; i < CHART_AXIS_X_RANGE; i++)
	{
		y_list.append(0);
	}
	//chart->createDefaultAxes();
	ui.chart_view->setChart(chart);
}

Seriesline_Chart::~Seriesline_Chart()
{
}

void Seriesline_Chart::Append(double y)
{
	y_list.append(y);
	if (y_list.length() > CHART_AXIS_X_RANGE)
		y_list.removeFirst();
	if (isVisible()) {
		QList<QPointF> points;
		points.clear();
		for (int i = 0; i < y_list.length(); i++)
		{
			points.append(QPointF(i, y_list.at(i)));
		}
		series->replace(points);
	}
}
