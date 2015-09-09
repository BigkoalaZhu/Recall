#include "recallimage.h"
#include <QFileDialog>

RecallImage::RecallImage(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	CannyThreshold = 50;
	superNum = 600;
	superCompact = 30;
}

RecallImage::~RecallImage()
{

}

void RecallImage::Loadimage()
{
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setNameFilter(tr("PNG File (*.png)"));

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
		BI.LoadImage(filename);
		ui.InputImage->Loadimage(filename);
	}
}

void RecallImage::EdgeDetect()
{
	BI.EdgeDetection(CannyThreshold);
	ui.MatchingDisplay->Loadimage("edges.png");
	ui.CannyThreshold->setEnabled(true);
}

void RecallImage::CannyChanged(int value)
{
	CannyThreshold = value;
	EdgeDetect();
}

void RecallImage::SLICSuperPixel()
{
	BI.SLICSuperPixel(superNum,superCompact);
	ui.MatchingDisplay->Loadimage("cutted.png");
}

void RecallImage::SLICSuperNum(QString num)
{
	superNum = num.toInt();
}
	
void RecallImage::SLICSuperCompact(QString num)
{
	superCompact = num.toDouble();
}

void RecallImage::Load3dShape()
{
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setNameFilter(tr("OFF File (*.off)"));

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
		QString labelname = filename;
		QString skelname = filename;
		QString weightname = filename;
		QString edgesname = filename;
		labelname.replace(QString(".off"), QString(".seg"));
		skelname.replace(QString(".off"), QString(".tgf"));
		weightname.replace(QString(".off"), QString(".dmat"));
		edgesname.replace(QString(".off"), QString(".txt"));

		PM = new PoseModels(filename.toStdString().c_str(), labelname.toStdString().c_str(),
			skelname.toStdString().c_str(), weightname.toStdString().c_str(), edgesname.toStdString().c_str());
		ui.actionGenerate_recalls->setEnabled(true);
	}
}

void RecallImage::LoadRecalls()
{
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::Directory);

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
		Recall_path = filename;
	}
}
	
void RecallImage::GenerateRecalls()
{
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::Directory);

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
		for (int i = 0; i < 10; i++)
		{
			PM->randomPose();
			PM->GenerateRecalls((filename+"/recall_"+QString::number(i)+"_").toStdString().c_str());
		}
		Recall_path = filename;
	}
}