#ifndef RECALLIMAGE_H
#define RECALLIMAGE_H

#include <QtWidgets/QMainWindow>
#include "ui_recallimage.h"

#include "BasicImage.h"
#include "PoseModels.h"

class RecallImage : public QMainWindow
{
	Q_OBJECT

public:
	RecallImage(QWidget *parent = 0);
	~RecallImage();

private:
	Ui::RecallImageClass ui;
	BasicImage BI;
	PoseModels *PM;

	int CannyThreshold;
	int superNum;
	double superCompact;

	QString Recall_path;

public slots:
	void Loadimage();
	void EdgeDetect();
	void CannyChanged(int);
	void SLICSuperPixel();
	void SLICSuperNum(QString);
	void SLICSuperCompact(QString);

	void Load3dShape();
	void LoadRecalls();
	void GenerateRecalls();
};

#endif // RECALLIMAGE_H
