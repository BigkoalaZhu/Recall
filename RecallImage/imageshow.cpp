#include "imageshow.h"
#include <QPaintEvent>
#include <QPainter>

imageshow::imageshow(QObject *parent)
{
	image = NULL;
}

imageshow::~imageshow()
{

}


void imageshow::Loadimage(QString filename)
{
	image = new QImage(filename);
	update();
}

void imageshow::paintEvent(QPaintEvent *event)
{
	if (image == NULL)
		return;
	QPainter paint(this);
	QRect source(0.0, 0.0, image->width(), image->height());
	QRect target;

	if(image->height()*width()/image->width() < height())
	{
		int offset = height() - image->height()*width()/image->width();
		target = QRect(0.0, offset/2, width(), image->height()*width()/image->width());
	}
	else
	{
		int offset = width() - image->width()*height()/image->height();
		target = QRect(offset/2, 0.0, image->width()*height()/image->height(), height());
	}

	paint.drawImage(target, *image, source);
}
