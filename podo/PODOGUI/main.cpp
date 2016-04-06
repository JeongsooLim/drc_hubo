#include "GUIMainWindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GUIMainWindow w;
    w.show();

    return a.exec();
}
