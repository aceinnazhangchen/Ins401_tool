#include "SCheckBoxHeaderView.h"

SCheckBoxHeaderView::SCheckBoxHeaderView(int checkColumnIndex, Qt::Orientation orientation, QWidget * parent)
	:QHeaderView(orientation, parent) 
{
	m_checkColIdx = checkColumnIndex;
	isChecked = true;
}


void SCheckBoxHeaderView::paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const
{
	painter->save();
	QHeaderView::paintSection(painter, rect, logicalIndex);
	painter->restore();
	if (logicalIndex == m_checkColIdx) {
		QStyleOptionButton option;
		if (this->isEnabled()) {
			option.state |= QStyle::State_Enabled;
		}		
		int width = 10;
		for (int i = 0; i < logicalIndex; ++i)
			width += sectionSize(i);
		option.rect = QRect(3, 5, 21, 21);
		if (isChecked)
			option.state |= QStyle::State_On;
		else
			option.state |= QStyle::State_Off;
		this->style()->drawControl(QStyle::CE_CheckBox, &option, painter);
	}
}

void SCheckBoxHeaderView::mousePressEvent(QMouseEvent *event) {
	if (visualIndexAt(event->pos().x()) == m_checkColIdx) {
		isChecked = !isChecked;
		this->updateSection(m_checkColIdx);
		emit checkStausChange(isChecked);
	}
	QHeaderView::mousePressEvent(event);
}