#pragma once

#include <QtGui>
#include <QPainter>
#include <QHeaderView>
#include <QStyleOptionButton>
#include <QStyle>

class SCheckBoxHeaderView : public QHeaderView
{
	Q_OBJECT
private:
	bool isChecked;
	int m_checkColIdx;
public:
	SCheckBoxHeaderView(int checkColumnIndex, Qt::Orientation orientation, QWidget * parent = 0);
signals:
	void checkStausChange(bool);
protected:
	void paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const;
	void mousePressEvent(QMouseEvent *event);
};