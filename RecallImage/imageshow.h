#ifndef IMAGESHOW_H
#define IMAGESHOW_H

#include <QWidget>

class imageshow : public QWidget
{
	Q_OBJECT

public:
	imageshow(QObject *parent);
	~imageshow();

	void Loadimage(QString filename);

private:
	void paintEvent(QPaintEvent *);

	QImage * image;
};

#endif // IMAGESHOW_H
