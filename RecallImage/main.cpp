#include "recallimage.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	RecallImage w;
	w.show();
	return a.exec();
}
